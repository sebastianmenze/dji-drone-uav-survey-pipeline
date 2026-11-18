"""Quick-look orthomosaic without photogrammetry ("direct georeferencing").

Each nadir photo is placed on the ground using only its GPS position, heading
(drone yaw by default; on the Mavic 4 Pro the XMP gimbal yaw was ~14 deg off)
and height above ground; nothing is matched between images. The ground
under each photo is assumed flat. Heights above ground come from
    height above takeoff (photo XMP) + takeoff ground elevation - ground
    elevation under the photo (DEM),
or a constant --agl. Where photos overlap, the pixel from the photo whose
centre is nearest wins (least lens/relief distortion).

Expect seams/offsets of a few metres (GPS + compass error, relief). Good for
coverage and sharpness checks in the field, not for measuring.

    python quick_mosaic.py PROJECT_DIR --kml AREA.kml [--res 0.05]

PROJECT_DIR is the folder made by prep_photos.py (images/ + photos.csv).
Writes quick_mosaic.tif (GeoTIFF, UTM, RGBA) and quick_mosaic.png (preview).
"""
import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
import tifffile
from PIL import Image
from pyproj import Transformer

from dji_fly_survey import Terrain, camera_for_image, read_points, read_polygon


def utm_epsg(lat, lon):
    zone = int((lon + 180) // 6) + 1
    if 56 <= lat < 64 and 3 <= lon < 12:          # Norway exception (zone 32V)
        zone = 32
    elif 72 <= lat < 84 and 0 <= lon < 42:        # Svalbard exceptions
        zone = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
    return (32600 if lat >= 0 else 32700) + zone


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('project')
    ap.add_argument('--kml', help='survey KML (takeoff point named "start")')
    ap.add_argument('--res', type=float, default=0.05, help='output pixel size, m')
    ap.add_argument('--agl', type=float, help='constant height above ground instead of DEM')
    ap.add_argument('--dem', default='open-meteo', help='open-meteo | kartverket')
    ap.add_argument('--takeoff-elev', type=float, help='ground elevation at takeoff, m')
    ap.add_argument('--crop', type=float, default=1.0,
                    help='use only the central fraction of each photo (e.g. 0.8)')
    ap.add_argument('--yaw', choices=['flight', 'gimbal'], default='flight',
                    help='image heading from FlightYawDegree (default) or GimbalYawDegree')
    a = ap.parse_args()

    proj = Path(a.project)
    rows = [r for r in csv.DictReader(open(proj / 'photos.csv')) if r['keep'] == 'True']
    lat = np.array([float(r['GpsLatitude']) for r in rows])
    lon = np.array([float(r['GpsLongitude']) for r in rows])
    rel = np.array([float(r['RelativeAltitude']) for r in rows])
    ykey = 'FlightYawDegree' if a.yaw == 'flight' and rows[0].get('FlightYawDegree') else 'GimbalYawDegree'
    yaw = np.radians([float(r[ykey]) for r in rows])

    # height above ground per photo
    if a.agl:
        agl = np.full(len(rows), a.agl)
        print(f'constant height above ground {a.agl} m')
    else:
        root = Path(__file__).resolve().parent
        terr = Terrain(a.dem, root / 'elevation_cache' / f'{a.dem}.json')
        if a.takeoff_elev is not None:
            z0 = a.takeoff_elev
        else:
            pts = read_points(a.kml) if a.kml else []
            st = [p for p in pts if p[0].lower().startswith(('start', 'takeoff', 'home'))] or pts
            if not st:
                raise SystemExit('need --kml with a start point, --takeoff-elev or --agl')
            z0 = terr.heights([st[0][1:]])[0]
        zg = np.array(terr.heights(list(zip(lat, lon))))
        terr.save()
        agl = rel + z0 - zg
        print(f'takeoff ground {z0:.0f} m, ground under photos {zg.min():.0f}-{zg.max():.0f} m, '
              f'height above ground {agl.min():.0f}-{agl.max():.0f} m')

    epsg = utm_epsg(lat.mean(), lon.mean())
    tr = Transformer.from_crs(4326, epsg, always_xy=True)
    E, N = tr.transform(lon, lat)
    E, N = np.asarray(E), np.asarray(N)

    W, H = int(rows[0].get('width') or 6144), int(rows[0].get('height') or 4096)
    fpx, camname = camera_for_image(W, H, rows[0].get('f35mm'))     # focal length, px
    print(f'camera: {camname}, {W}x{H}, focal length {fpx:.0f} px, heading from {ykey}')
    half_d = agl * math.hypot(W, H) / 2 / fpx * a.crop                # footprint half-diagonal, m

    with open(proj / 'cameras_init.csv', 'w', newline='') as f:   # used by stitch_mosaic.py
        w = csv.writer(f)
        w.writerow(['file', 'E', 'N', 'yaw_deg', 'agl', 'epsg', 'f_px', 'w', 'h'])
        for k, r in enumerate(rows):
            w.writerow([r['file'], f'{E[k]:.3f}', f'{N[k]:.3f}', f'{math.degrees(yaw[k]):.2f}',
                        f'{agl[k]:.2f}', epsg, f'{fpx:.1f}', W, H])

    res = a.res
    x0 =math.floor((E - half_d).min() / res) * res
    y1 = math.ceil((N + half_d).max() / res) * res
    nx = int(math.ceil(((E + half_d).max() - x0) / res))
    ny = int(math.ceil((y1 - (N - half_d).min()) / res))
    print(f'{len(rows)} photos -> {nx} x {ny} px at {res * 100:.0f} cm, EPSG:{epsg}')
    rgb = np.zeros((ny, nx, 3), np.uint8)
    best = np.full((ny, nx), np.inf, np.float32)

    t0 = time.time()
    for k, r in enumerate(rows):
        gsd_img = agl[k] / fpx
        scale = max(1, min(8, int(res / gsd_img)))       # decode JPEG at reduced size
        im = Image.open(proj / 'images' / r['file'])
        im.draft('RGB', (W // scale, H // scale))
        img = np.asarray(im.convert('RGB'))
        sh, sw = img.shape[:2]
        f_s = fpx * sw / W

        # output window around the photo
        c0 = max(0, int((E[k] - half_d[k] - x0) / res))
        c1 = min(nx, int((E[k] + half_d[k] - x0) / res) + 1)
        r0 = max(0, int((y1 - N[k] - half_d[k]) / res))
        r1 = min(ny, int((y1 - N[k] + half_d[k]) / res) + 1)
        xs = x0 + (np.arange(c0, c1) + 0.5) * res - E[k]
        ys = y1 - (np.arange(r0, r1) + 0.5) * res - N[k]
        de, dn = np.meshgrid(xs, ys)
        # ground offset -> camera frame (image top = gimbal yaw direction)
        fwd = dn * math.cos(yaw[k]) + de * math.sin(yaw[k])
        rgt = de * math.cos(yaw[k]) - dn * math.sin(yaw[k])
        col = sw / 2 + rgt / agl[k] * f_s
        row = sh / 2 - fwd / agl[k] * f_s
        ok = ((col >= sw * (1 - a.crop) / 2) & (col < sw * (1 + a.crop) / 2 - 1) &
              (row >= sh * (1 - a.crop) / 2) & (row < sh * (1 + a.crop) / 2 - 1))
        d = (de * de + dn * dn).astype(np.float32)
        sub = best[r0:r1, c0:c1]
        ok &= d < sub
        if ok.any():
            sub[ok] = d[ok]
            rgb[r0:r1, c0:c1][ok] = img[row[ok].astype(int), col[ok].astype(int)]
        if (k + 1) % 50 == 0:
            print(f'  {k + 1}/{len(rows)} photos ({time.time() - t0:.0f} s)')

    alpha = np.where(np.isfinite(best), 255, 0).astype(np.uint8)
    out = proj / 'quick_mosaic.tif'
    geokeys = (1, 1, 0, 3,
               1024, 0, 1, 1,        # GTModelType = projected
               1025, 0, 1, 1,        # GTRasterType = PixelIsArea
               3072, 0, 1, epsg)     # ProjectedCSType
    tifffile.imwrite(out, np.dstack([rgb, alpha]), photometric='rgb', extrasamples=['unassalpha'],
                     tile=(512, 512), compression='zlib',
                     extratags=[(33550, 12, 3, (res, res, 0.0)),
                                (33922, 12, 6, (0.0, 0.0, 0.0, x0, y1, 0.0)),
                                (34735, 3, len(geokeys), geokeys)])
    print('wrote', out)

    # small preview with the survey outline
    step = max(1, int(max(nx, ny) / 3000))
    prev = Image.fromarray(np.dstack([rgb, alpha])[::step, ::step])
    bg = Image.new('RGBA', prev.size, (255, 255, 255, 255))
    bg.alpha_composite(prev)
    if a.kml:
        from PIL import ImageDraw
        ring = read_polygon(a.kml)
        px, py = tr.transform([p[1] for p in ring], [p[0] for p in ring])
        pts = [((x - x0) / res / step, (y1 - y) / res / step) for x, y in zip(px, py)]
        ImageDraw.Draw(bg).line(pts + pts[:1], fill=(255, 0, 0, 255), width=3)
    bg.convert('RGB').save(proj / 'quick_mosaic.png')
    print('wrote', proj / 'quick_mosaic.png')


if __name__ == '__main__':
    main()
