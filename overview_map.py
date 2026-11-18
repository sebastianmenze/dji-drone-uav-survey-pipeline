"""Overview map PNG: orthophoto + flight track + photo positions (+ plan, area).

    python overview_map.py PROJECT_DIR [--kml AREA.kml] [--plan PLAN_waypoints.csv]
                           [--ortho some.tif] [--out map.png]

PROJECT_DIR is the folder made by prep_photos.py (photos.csv). The flight
track is drawn through the GPS positions of ALL photos on the card in time
order (the SD card has no flight log); photos used for processing are blue,
dropped ones (transit, oblique, outside the area) are red. --plan adds the
planned waypoints from dji_fly_survey.py (*_waypoints.csv) as a dashed line.

Orthophoto: the first that exists of odm_orthophoto/odm_orthophoto.tif,
stitched_mosaic.tif, quick_mosaic.tif (or --ortho). Needs rasterio; without it
the script re-runs itself in the OpenDroneMap Docker image.
"""
import argparse
import csv
import math
from pathlib import Path

try:
    import rasterio
    from rasterio.enums import Resampling
except ImportError:
    if __name__ == '__main__':
        from docker_util import relaunch
        relaunch(__file__, name='overview_map')
    raise
import numpy as np
from pyproj import Transformer

from dji_fly_survey import read_points, read_polygon


def utm_epsg(lat, lon):
    zone = int((lon + 180) // 6) + 1
    if 56 <= lat < 64 and 3 <= lon < 12:          # Norway exception (zone 32V)
        zone = 32
    elif 72 <= lat < 84 and 0 <= lon < 42:        # Svalbard exceptions
        zone = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
    return (32600 if lat >= 0 else 32700) + zone


def nice_length(x):
    p = 10 ** math.floor(math.log10(x))
    return max(m * p for m in (1, 2, 5) if m * p <= x)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('project')
    ap.add_argument('--ortho', help='GeoTIFF to show (default: best available in PROJECT_DIR)')
    ap.add_argument('--kml', help='survey area KML (polygon + takeoff point)')
    ap.add_argument('--plan', help='*_waypoints.csv from dji_fly_survey.py')
    ap.add_argument('--out', help='PNG path (default PROJECT_DIR/overview_map.png)')
    ap.add_argument('--size', type=int, default=3000, help='longest side of the image, px')
    ap.add_argument('--dpi', type=int, default=150)
    ap.add_argument('--no-track', action='store_true')
    a = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    proj = Path(a.project)
    rows = list(csv.DictReader(open(proj / 'photos.csv')))
    rows = [r for r in rows if r['GpsLatitude'] not in ('', 'None')]
    rows.sort(key=lambda r: (r['time'], r['file']))
    if a.ortho:
        ortho = Path(a.ortho)
    else:
        cands = [proj / 'odm_orthophoto' / 'odm_orthophoto.tif', proj / 'stitched_mosaic.tif',
                 proj / 'quick_mosaic.tif']
        ortho = next((p for p in cands if p.exists()), None)

    fig, ax = plt.subplots(figsize=(10, 10))
    if ortho:
        with rasterio.open(ortho) as src:
            crs = src.crs
            f = max(1, max(src.width, src.height) / a.size)
            shape = (int(src.height / f), int(src.width / f))
            bands = min(src.count, 4)
            img = src.read(list(range(1, bands + 1)), out_shape=(bands,) + shape,
                           resampling=Resampling.average)
            img = np.moveaxis(img, 0, -1)
            if bands == 3:
                m = src.read_masks(1, out_shape=shape)
                img = np.dstack([img, m])
            b = src.bounds
        ax.imshow(img, extent=(b.left, b.right, b.bottom, b.top), zorder=1)
        epsg = crs.to_epsg()
        title = f'{ortho.name} (EPSG:{epsg})'
    else:
        epsg = utm_epsg(float(rows[0]['GpsLatitude']), float(rows[0]['GpsLongitude']))
        title = f'no orthophoto yet (EPSG:{epsg})'
    tr = Transformer.from_crs(4326, epsg, always_xy=True)

    lon = np.array([float(r['GpsLongitude']) for r in rows])
    lat = np.array([float(r['GpsLatitude']) for r in rows])
    E, N = map(np.asarray, tr.transform(lon, lat))
    keep = np.array([r['keep'] == 'True' for r in rows])

    if a.kml:
        ring = read_polygon(a.kml)
        px, py = tr.transform([p[1] for p in ring] + [ring[0][1]], [p[0] for p in ring] + [ring[0][0]])
        ax.plot(px, py, '-', color='gold', lw=2, label='survey area', zorder=3)
        pts = read_points(a.kml)
        st = [p for p in pts if p[0].lower().startswith(('start', 'takeoff', 'home'))]
        if st:
            x, y = tr.transform(st[0][2], st[0][1])
            ax.plot(x, y, '*', ms=18, color='yellow', mec='k', label='takeoff (plan)', zorder=6)
    if a.plan:
        wp = list(csv.DictReader(open(a.plan)))
        x, y = tr.transform([float(w['lon']) for w in wp], [float(w['lat']) for w in wp])
        ax.plot(x, y, '--', color='magenta', lw=1, alpha=0.7, label='planned track', zorder=2)
    if not a.no_track:
        ax.plot(E, N, '-', color='orange', lw=1, alpha=0.9, label='flight track (photo GPS)', zorder=4)
    ax.scatter(E[~keep], N[~keep], s=10, c='red', marker='x', lw=1,
               label=f'dropped photos ({(~keep).sum()})', zorder=5)
    ax.scatter(E[keep], N[keep], s=12, c='deepskyblue', edgecolors='k', linewidths=0.3,
               label=f'survey photos ({keep.sum()})', zorder=5)
    ax.plot(E[0], N[0], 'o', ms=9, color='lime', mec='k', label='first photo', zorder=6)
    ax.plot(E[-1], N[-1], 's', ms=8, color='magenta', mec='k', label='last photo', zorder=6)

    # view: survey photos + ortho, not the whole transit
    xs = list(E[keep] if keep.any() else E)
    ys = list(N[keep] if keep.any() else N)
    if ortho:
        xs += [b.left, b.right]
        ys += [b.bottom, b.top]
    pad = 0.05 * max(max(xs) - min(xs), max(ys) - min(ys))
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect('equal')
    ax.set_facecolor('#dddddd')

    # scale bar + north arrow
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    L = nice_length((x1 - x0) / 5)
    bx, by = x0 + 0.04 * (x1 - x0), y0 + 0.04 * (y1 - y0)
    ax.plot([bx, bx + L], [by, by], '-', color='k', lw=6, solid_capstyle='butt', zorder=7)
    ax.plot([bx, bx + L], [by, by], '-', color='w', lw=3, solid_capstyle='butt', zorder=8)
    ax.text(bx + L / 2, by + 0.015 * (y1 - y0), f'{L:g} m', ha='center', va='bottom', fontsize=11,
            fontweight='bold', color='k', zorder=8,
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1))
    ax.annotate('N', xy=(x1 - 0.05 * (x1 - x0), y1 - 0.04 * (y1 - y0)),
                xytext=(x1 - 0.05 * (x1 - x0), y1 - 0.13 * (y1 - y0)),
                ha='center', fontsize=14, fontweight='bold', zorder=8,
                arrowprops=dict(arrowstyle='-|>', lw=2, color='k'))

    ax.ticklabel_format(useOffset=False, style='plain')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    t0, t1 = rows[0]['time'], rows[-1]['time']
    ax.set_title(f'{proj.name}: {len(rows)} photos, {t0} - {t1[-8:]}\n{title}', fontsize=11)
    ax.legend(loc='upper left', fontsize=9, framealpha=0.85)
    out = Path(a.out) if a.out else proj / 'overview_map.png'
    fig.savefig(out, dpi=a.dpi, bbox_inches='tight')
    print('wrote', out)


if __name__ == '__main__':
    main()
