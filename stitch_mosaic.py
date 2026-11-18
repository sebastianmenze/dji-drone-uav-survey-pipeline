"""Panorama-style stitched orthomosaic: GPS start positions + feature matching
+ global least-squares alignment + gain compensation + feathered blending.

Each photo gets a 2-D similarity transform (shift, rotation, scale) from image
to map coordinates, written as  ground = t + c * (u + i w)  with complex t, c
(u = right, w = up in pixels from the image centre). That is linear, so all
photos are aligned at once: SIFT matches between overlapping photos say
"these two pixels are the same ground point", weak priors keep each photo near
its GPS position / height / heading. Brightness differences are removed with
per-photo, per-channel gains, and seams are feathered.

Still a flat-ground model per photo: relief inside a photo (trees, steep
slopes) can leave some ghosting, but edges no longer show as steps.

Needs OpenCV + rasterio. If they are not installed it re-runs itself inside the
OpenDroneMap Docker image automatically (python stitch_mosaic.py ODM_DIR ...).
Manual equivalent:
  docker run --rm -v C:/Users/.../dji-survey-pipeline:/work -v C:/survey:/data -w /work --entrypoint python3 \
      opendronemap/odm stitch_mosaic.py /data/odm --res 0.05

PROJECT_DIR must contain images/ and cameras_init.csv (from quick_mosaic.py).
Writes stitched_mosaic.tif (GeoTIFF RGBA), stitched_mosaic.png, cameras_aligned.csv.
"""
import argparse
import csv
import math
import time
from pathlib import Path

try:
    import cv2
    import rasterio
except ImportError:          # not installed locally -> run inside the OpenDroneMap image
    if __name__ == '__main__':
        from docker_util import relaunch
        relaunch(__file__, name='stitch_mosaic')
    raise
import numpy as np
import scipy.sparse as sp
from PIL import Image
from rasterio.transform import from_origin
from scipy.sparse.linalg import lsqr

W, H = 6144, 4096      # replaced by the photo size from cameras_init.csv


def load(path, scale):
    im = Image.open(path)
    im.draft('RGB', (W // scale, H // scale))
    return np.asarray(im.convert('RGB'))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('project')
    ap.add_argument('--res', type=float, default=0.05, help='output pixel size, m')
    ap.add_argument('--scale', type=int, default=4, help='image decode reduction (1,2,4,8)')
    ap.add_argument('--features', type=int, default=3000)
    ap.add_argument('--pair-dist', type=float, default=0.75,
                    help='match photos whose centres are closer than this x footprint height')
    ap.add_argument('--gps-sigma', type=float, default=3.0, help='m')
    ap.add_argument('--scale-sigma', type=float, default=0.03, help='relative')
    ap.add_argument('--yaw-sigma', type=float, default=3.0, help='deg')
    ap.add_argument('--match-sigma', type=float, default=0.3, help='m')
    ap.add_argument('--pair-points', type=float, default=4,
                    help='each photo pair counts as this many independent points (stops matches outvoting GPS)')
    ap.add_argument('--no-render', action='store_true', help='align and report only')
    ap.add_argument('--render-scale', type=int, default=0,
                    help='decode reduction for rendering (0 = auto from --res; 1 = full resolution)')
    ap.add_argument('--render-only', action='store_true', help='reuse alignment.npz from a previous run')
    ap.add_argument('--tile', type=int, default=4096, help='render tile size, px')
    ap.add_argument('--cache', type=int, default=24, help='decoded photos kept in memory')
    ap.add_argument('--jpeg-quality', type=int, default=90)
    a = ap.parse_args()
    cv2.setNumThreads(4)

    proj = Path(a.project)
    cams = list(csv.DictReader(open(proj / 'cameras_init.csv')))
    global W, H
    if cams[0].get('w'):
        W, H = int(cams[0]['w']), int(cams[0]['h'])
    n = len(cams)
    E = np.array([float(c['E']) for c in cams])
    N = np.array([float(c['N']) for c in cams])
    yaw = np.radians([float(c['yaw_deg']) for c in cams])
    agl = np.array([float(c['agl']) for c in cams])
    epsg = int(cams[0]['epsg'])
    fpx = float(cams[0]['f_px'])
    sw, sh = W // a.scale, H // a.scale
    f_s = fpx / a.scale
    # initial similarity: ground = t + c*(u + i w);  right = e^{-i yaw}, fwd = i e^{-i yaw}
    t0 = E + 1j * N
    if a.render_only:
        z = np.load(proj / 'alignment.npz')
        assert list(z['files']) == [cm['file'] for cm in cams], 'photo list changed - rerun alignment'
        render(a, proj, cams, z['t'], z['c'], z['gain'], int(z['epsg']), time.time())
        return
    NORM = sw / 2                            # image coords normalised to about [-1, 1]
    c0 = (agl / f_s) * NORM * np.exp(-1j * yaw)

    import pickle
    files = [cm['file'] for cm in cams]
    mfile = proj / 'matches.pkl'
    links = None
    tic = time.time()
    if mfile.exists():
        cached = pickle.load(open(mfile, 'rb'))
        if cached['files'] == files and cached['scale'] == a.scale:
            links = cached['links']
            print('reusing matches from', mfile, flush=True)
    if links is None:
        # ---------------------------------------------------------- features
        tic = time.time()
        sift = cv2.SIFT_create(nfeatures=a.features)
        kps, descs, cols = [], [], []
        for k, c in enumerate(cams):
            img = load(proj / 'images' / c['file'], a.scale)
            g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            kp, d = sift.detectAndCompute(g, None)
            pts = np.array([p.pt for p in kp], np.float32).reshape(-1, 2)
            kps.append(pts)
            descs.append(d if d is not None else np.zeros((0, 128), np.float32))
            blur = cv2.blur(img, (5, 5)).astype(np.float32)
            ij = np.clip(pts.round().astype(int), 0, [sw - 1, sh - 1])
            cols.append(blur[ij[:, 1], ij[:, 0]] + 1.0)
            if (k + 1) % 100 == 0:
                print(f'  features {k + 1}/{n} ({time.time() - tic:.0f} s)', flush=True)

        # ---------------------------------------------------------- pairs + matching
        foot_h = agl.mean() * sh / f_s           # along-track footprint, m
        P = np.c_[E, N]
        pairs = [(i, j) for i in range(n) for j in range(i + 1, n)
                 if np.hypot(*(P[i] - P[j])) < a.pair_dist * foot_h]
        print(f'{n} photos, {len(pairs)} candidate pairs', flush=True)
        flann = cv2.FlannBasedMatcher({'algorithm': 1, 'trees': 4}, {'checks': 64})
        links = []          # (i, j, pts_i, pts_j, gain_obs[3])
        for m, (i, j) in enumerate(pairs):
            if len(descs[i]) < 10 or len(descs[j]) < 10:
                continue
            mm = flann.knnMatch(descs[i], descs[j], k=2)
            good = [x[0] for x in mm if len(x) == 2 and x[0].distance < 0.75 * x[1].distance]
            if len(good) < 15:
                continue
            qi = np.array([kps[i][x.queryIdx] for x in good])
            qj = np.array([kps[j][x.trainIdx] for x in good])
            M, inl = cv2.estimateAffinePartial2D(qi, qj, method=cv2.RANSAC,
                                                 ransacReprojThreshold=4.0, maxIters=3000)
            if M is None or inl.sum() < 15:
                continue
            sel = np.flatnonzero(inl.ravel())
            gi = np.log(np.median(cols[i][[good[s].queryIdx for s in sel]], axis=0))
            gj = np.log(np.median(cols[j][[good[s].trainIdx for s in sel]], axis=0))
            if len(sel) > 60:
                sel = sel[np.linspace(0, len(sel) - 1, 60).astype(int)]
            links.append((i, j, qi[sel], qj[sel], gj - gi))
            if (m + 1) % 500 == 0:
                print(f'  matched {m + 1}/{len(pairs)} pairs, {len(links)} linked '
                      f'({time.time() - tic:.0f} s)', flush=True)
        pickle.dump({'files': files, 'scale': a.scale, 'links': links}, open(mfile, 'wb'))
    print(f'{len(links)} linked pairs', flush=True)
    linked = np.zeros(n, bool)
    for i, j, *_ in links:
        linked[i] = linked[j] = True
    if (~linked).any():
        print(f'  {(~linked).sum()} photos without matches stay at their GPS position')

    # ---------------------------------------------------------- global alignment
    def uv(pts):        # pixel -> complex image coordinate (u right, w up)
        return ((pts[:, 0] - sw / 2) + 1j * (sh / 2 - pts[:, 1])) / NORM

    def solve(weights):
        rows, colsA, vals, rhs = [], [], [], []
        r = 0

        def add(entries, b, w):
            nonlocal r
            # complex equation -> two real rows; unknowns per photo: tx, ty, a, b
            for re_im in (0, 1):
                for idx, coef in entries:
                    base = 4 * idx
                    # coef * t  (coef real) or coef * c  (coef complex, multiplies c)
                    kind, z = coef
                    if kind == 't':
                        if re_im == 0:
                            rows.append(r); colsA.append(base); vals.append(w * z)
                        else:
                            rows.append(r); colsA.append(base + 1); vals.append(w * z)
                    else:       # z * c,  c = a + i b:  re = zr a - zi b, im = zi a + zr b
                        zr, zi = z.real, z.imag
                        if re_im == 0:
                            rows.extend([r, r]); colsA.extend([base + 2, base + 3]); vals.extend([w * zr, -w * zi])
                        else:
                            rows.extend([r, r]); colsA.extend([base + 2, base + 3]); vals.extend([w * zi, w * zr])
                rhs.append(w * (b.real if re_im == 0 else b.imag))
                r += 1

        wm = 1 / a.match_sigma
        for (i, j, pi, pj, _), wl in zip(links, weights):
            wp = wm * math.sqrt(a.pair_points / len(pi))
            for zi_, zj_ in zip(uv(pi), uv(pj)):
                # unknowns are corrections dt, dc to the GPS start values
                b = -(t0[i] + c0[i] * zi_ - t0[j] - c0[j] * zj_)
                add([(i, ('t', 1.0)), (i, ('c', zi_)), (j, ('t', -1.0)), (j, ('c', -zj_))],
                    b, wp * wl)
        for k in range(n):
            add([(k, ('t', 1.0))], 0j, 1 / a.gps_sigma)
            # prior on c: scale and heading near initial values
            sig_c = abs(c0[k]) * math.hypot(a.scale_sigma, math.radians(a.yaw_sigma)) / math.sqrt(2)
            add([(k, ('c', 1.0 + 0j))], 0j, 1 / sig_c)
        A = sp.csr_matrix((vals, (rows, colsA)), shape=(r, 4 * n))
        x = lsqr(A, np.array(rhs), atol=1e-12, btol=1e-12, iter_lim=20000)[0]
        return t0 + x[0::4] + 1j * x[1::4], c0 + x[2::4] + 1j * x[3::4]

    def residuals(t, c):
        return [np.abs(t[i] + c[i] * uv(pi) - t[j] - c[j] * uv(pj)) for i, j, pi, pj, _ in links]

    r_init = np.concatenate(residuals(t0, c0))
    c_gps = c0.copy()
    weights = np.ones(len(links))
    for it in range(5):
        t, c = solve(weights)
        res = residuals(t, c)
        med = np.array([np.median(x) for x in res])
        cut = 3 * np.median(med) + 0.05
        weights = np.where(med < cut, 1.0, 0.0)          # drop pairs that disagree
        # self-calibration: one shared scale / heading correction for all photos
        # (height-above-ground and compass/gimbal-yaw biases), priors then centre on it
        if it < 4:
            q = c[linked] / c0[linked]
            c0 = c0 * (np.median(np.abs(q)) * np.exp(1j * np.median(np.angle(q))))
    g = np.median(c0 / c_gps)
    print(f'shared correction: scale x{abs(g):.3f} (height above ground x{abs(g):.3f}), '
          f'heading {math.degrees(np.angle(g)) * -1:+.1f} deg', flush=True)
    r_fin = np.concatenate([x for x, wl in zip(res, weights) if wl > 0])
    shift = np.abs(t - t0)
    print(f'seam mismatch: GPS only median {np.median(r_init):.2f} m (95% {np.percentile(r_init, 95):.2f}) '
          f'-> aligned median {np.median(r_fin):.2f} m (95% {np.percentile(r_fin, 95):.2f}); '
          f'{int((weights == 0).sum())} pairs rejected', flush=True)
    print(f'photo position corrections: median {np.median(shift):.1f} m, max {shift.max():.1f} m', flush=True)
    sr = np.abs(c) / np.abs(c0)
    dr = np.degrees(np.angle(c / c0))
    print(f'scale vs GPS/DEM height: {np.percentile(sr, 5):.3f}-{np.percentile(sr, 95):.3f} (5-95%), '
          f'heading change {np.percentile(dr, 5):.1f} to {np.percentile(dr, 95):.1f} deg', flush=True)

    # ---------------------------------------------------------- gain compensation
    G = np.zeros((n, 3))
    for ch in range(3):
        rows_, cols_, vals_, b_ = [], [], [], []
        r = 0
        for (i, j, _, _, d), wl in zip(links, weights):
            if wl == 0:
                continue
            rows_ += [r, r]; cols_ += [i, j]; vals_ += [1.0, -1.0]; b_.append(d[ch]); r += 1
        for k in range(n):
            rows_.append(r); cols_.append(k); vals_.append(0.1); b_.append(0.0); r += 1
        A = sp.csr_matrix((vals_, (rows_, cols_)), shape=(r, n))
        G[:, ch] = lsqr(A, np.array(b_))[0]
    gain = np.exp(G)
    print(f'brightness gains {gain.min():.2f}-{gain.max():.2f}', flush=True)

    with open(proj / 'cameras_aligned.csv', 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['file', 'E', 'N', 'scale_m_per_px', 'rot_deg', 'shift_m', 'gain_r', 'gain_g', 'gain_b'])
        for k, cm in enumerate(cams):
            wr.writerow([cm['file'], f'{t[k].real:.3f}', f'{t[k].imag:.3f}', f'{abs(c[k]) / NORM:.5f}',
                         f'{math.degrees(-np.angle(c[k])):.2f}', f'{shift[k]:.2f}',
                         *(f'{g:.3f}' for g in gain[k])])
    np.savez(proj / 'alignment.npz', t=t, c=c, gain=gain, epsg=epsg,
             files=np.array([cm['file'] for cm in cams]))
    if not a.no_render:
        render(a, proj, cams, t, c, gain, epsg, tic)


def render(a, proj, cams, t, c, gain, epsg, tic):
    """Tiled feathered rendering straight into a JPEG-compressed BigTIFF.
    t, c map normalised image coords z = (u + i w) / (width/2) to map coords."""
    from rasterio.enums import ColorInterp, Resampling
    from rasterio.windows import Window
    res = a.res
    gsd = np.abs(c).min() / (W / 2)                        # finest native m/px at full size
    rs = a.render_scale or max([s for s in (1, 2, 4, 8) if s * gsd <= res * 1.01] or [1])
    rw, rh = W // rs, H // rs
    half = rw / 2
    corners = np.array([-rw / 2 + 1j * rh / 2, rw / 2 + 1j * rh / 2,
                        rw / 2 - 1j * rh / 2, -rw / 2 - 1j * rh / 2]) / half
    gc = t[:, None] + c[:, None] * corners[None, :]
    x0 = math.floor(gc.real.min() / res) * res
    y1 = math.ceil(gc.imag.max() / res) * res
    nx = int(math.ceil((gc.real.max() - x0) / res))
    ny = int(math.ceil((y1 - gc.imag.min()) / res))
    pc0 = np.floor((gc.real.min(1) - x0) / res).astype(int)
    pc1 = np.ceil((gc.real.max(1) - x0) / res).astype(int)
    pr0 = np.floor((y1 - gc.imag.max(1)) / res).astype(int)
    pr1 = np.ceil((y1 - gc.imag.min(1)) / res).astype(int)
    print(f'rendering {nx} x {ny} px ({nx * ny / 1e9:.2f} Gpx) at {res * 100:g} cm from photos '
          f'decoded at 1/{rs} ({gsd * rs * 100:.2f} cm native), EPSG:{epsg}', flush=True)

    # feather weight inside each image: 0 at the edge, 1 in the centre, squared
    yy, xx = np.mgrid[0:rh, 0:rw].astype(np.float32)
    feather = np.minimum(np.minimum(xx, rw - 1 - xx) / (rw / 2), np.minimum(yy, rh - 1 - yy) / (rh / 2))
    feather = (np.clip(feather, 0, 1) ** 2).astype(np.float32)
    del xx, yy

    cache, order = {}, []

    def get(k):
        if k not in cache:
            cache[k] = load(proj / 'images' / cams[k]['file'], rs)
            order.append(k)
            if len(order) > a.cache:
                cache.pop(order.pop(0))
        return cache[k]

    step = max(1, math.ceil(max(nx, ny) / 3000))
    T = max(step, (a.tile // step) * step)
    prev = np.zeros((math.ceil(ny / step), math.ceil(nx / step), 4), np.uint8)
    tif = proj / 'stitched_mosaic.tif'
    profile = dict(driver='GTiff', width=nx, height=ny, count=3, dtype='uint8',
                   crs=f'EPSG:{epsg}', transform=from_origin(x0, y1, res, res),
                   tiled=True, blockxsize=512, blockysize=512, compress='JPEG',
                   photometric='YCBCR', jpeg_quality=a.jpeg_quality, BIGTIFF='YES')
    ntiles = math.ceil(ny / T) * math.ceil(nx / T)
    done = 0
    flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
    with rasterio.Env(GDAL_TIFF_INTERNAL_MASK=True):
        with rasterio.open(tif, 'w', **profile) as dst:
            dst.colorinterp = [ColorInterp.red, ColorInterp.green, ColorInterp.blue]
            for ty in range(0, ny, T):
                for tx in range(0, nx, T):
                    th, tw = min(T, ny - ty), min(T, nx - tx)
                    ks = np.flatnonzero((pc1 > tx) & (pc0 < tx + tw) & (pr1 > ty) & (pr0 < ty + th))
                    done += 1
                    if not len(ks):
                        continue
                    acc = np.zeros((th, tw, 3), np.float32)
                    wsum = np.zeros((th, tw), np.float32)
                    for k in ks:
                        wc0, wc1 = max(tx, pc0[k]) - tx, min(tx + tw, pc1[k]) - tx
                        wr0, wr1 = max(ty, pr0[k]) - ty, min(ty + th, pr1[k]) - ty
                        # output pixel -> source pixel is affine (inverse similarity)
                        d = 1 / c[k]
                        g0 = (x0 + (tx + wc0 + 0.5) * res) + 1j * (y1 - (ty + wr0 + 0.5) * res)
                        K = d * (g0 - t[k])
                        M = np.array([[half * res * d.real, half * res * d.imag, half * K.real + rw / 2 - 0.5],
                                      [-half * res * d.imag, half * res * d.real, rh / 2 - half * K.imag - 0.5]])
                        size = (int(wc1 - wc0), int(wr1 - wr0))
                        wgt = cv2.warpAffine(feather, M, size, flags=flags, borderMode=cv2.BORDER_CONSTANT)
                        if not wgt.any():
                            continue
                        img = cv2.warpAffine(get(k), M, size, flags=flags, borderMode=cv2.BORDER_CONSTANT)
                        acc[wr0:wr1, wc0:wc1] += img.astype(np.float32) * (wgt[..., None] / gain[k].astype(np.float32))
                        wsum[wr0:wr1, wc0:wc1] += wgt
                    valid = wsum > 1e-4
                    np.divide(acc, wsum[..., None], out=acc, where=valid[..., None])
                    np.clip(acc, 0, 255, out=acc)
                    rgb = acc.astype(np.uint8)
                    rgb[~valid] = 0
                    del acc
                    win = Window(tx, ty, tw, th)
                    dst.write(np.moveaxis(rgb, -1, 0), window=win)
                    dst.write_mask(np.where(valid, 255, 0).astype(np.uint8), window=win)
                    pv = prev[ty // step:ty // step + math.ceil(th / step),
                              tx // step:tx // step + math.ceil(tw / step)]
                    pv[..., :3] = rgb[::step, ::step]
                    pv[..., 3] = np.where(valid[::step, ::step], 255, 0)
                    print(f'  tile {done}/{ntiles} ({len(ks)} photos, {time.time() - tic:.0f} s)', flush=True)
    print('building overviews ...', flush=True)
    with rasterio.Env(GDAL_TIFF_INTERNAL_MASK=True, COMPRESS_OVERVIEW='JPEG',
                      PHOTOMETRIC_OVERVIEW='YCBCR', JPEG_QUALITY_OVERVIEW=str(a.jpeg_quality)):
        with rasterio.open(tif, 'r+') as dst:
            dst.build_overviews([f for f in (2, 4, 8, 16, 32, 64, 128) if max(nx, ny) / f > 256],
                                Resampling.average)
    print('wrote', tif, f'({tif.stat().st_size / 1e9:.2f} GB)', flush=True)
    im = Image.fromarray(prev, 'RGBA')
    bg = Image.new('RGBA', im.size, (255, 255, 255, 255))
    bg.alpha_composite(im)
    bg.convert('RGB').save(proj / 'stitched_mosaic.png')
    print('wrote', proj / 'stitched_mosaic.png', f'(total {time.time() - tic:.0f} s)', flush=True)


if __name__ == '__main__':
    main()
