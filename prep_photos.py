"""Sort DJI survey photos into survey-line shots and transit shots, then stage
the survey shots for OpenDroneMap (<project>/images, hard links, no copying).
Works for Mavic 3 / 3 Classic / 3 Pro and Mavic 4 Pro photos (DJI XMP tags).

    python prep_photos.py PHOTO_DIR --kml AREA.kml --project ODM_DIR

A photo is kept when the gimbal points (nearly) straight down, the drone is
inside the survey polygon (+ buffer), and it is not on the ground / climbing
out near takeoff. Writes photos.csv and photos.png into the project folder.
"""
import argparse
import csv
import os
import re
import shutil
from pathlib import Path

from PIL import Image

from dji_fly_survey import LocalFrame, read_polygon

XMP_KEYS = ('GpsLatitude', 'GpsLongitude', 'AbsoluteAltitude', 'RelativeAltitude',
            'GimbalPitchDegree', 'GimbalYawDegree', 'FlightYawDegree', 'FlightXSpeed',
            'FlightYSpeed', 'ShutterType')


def read_meta(path):
    with open(path, 'rb') as f:
        head = f.read(300_000)
    m = re.search(rb'<x:xmpmeta.*?</x:xmpmeta>', head, re.S)
    meta = {}
    if m:
        for k, v in re.findall(rb'drone-dji:(\w+)="([^"]*)"', m.group(0)):
            meta[k.decode()] = v.decode()
    out = {'file': path.name}
    for k in XMP_KEYS:
        v = meta.get(k)
        try:
            out[k] = float(v)
        except (TypeError, ValueError):
            out[k] = v
    im = Image.open(path)
    ex = im.getexif()
    out['time'] = ex.get(306, '')
    out['model'] = str(ex.get(272, '')).strip('\x00 ')
    out['width'], out['height'] = im.size
    out['f35mm'] = ex.get_ifd(0x8769).get(0xA405, '')      # FocalLengthIn35mmFilm
    return out


def inside(pt, poly):
    x, y = pt
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def dist_to_poly(pt, poly):
    x, y = pt
    best = 1e18
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1)))
        best = min(best, ((x - x1 - t * dx) ** 2 + (y - y1 - t * dy) ** 2) ** .5)
    return 0.0 if inside(pt, poly) else best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('photos')
    ap.add_argument('--kml', required=True)
    ap.add_argument('--project', required=True)
    ap.add_argument('--buffer', type=float, default=30, help='m outside polygon still kept')
    ap.add_argument('--min-pitch', type=float, default=-85, help='keep gimbal pitch <= this')
    ap.add_argument('--min-rel-alt', type=float, default=15, help='m above takeoff')
    a = ap.parse_args()

    # DJI SD cards: DCIM/DJI_001/..., several folders per flight day -> search recursively
    files = sorted((p for p in Path(a.photos).rglob('*') if p.suffix.lower() in ('.jpg', '.jpeg')),
                   key=lambda p: p.name)
    names = [p.name for p in files]
    if len(set(names)) != len(names):
        raise SystemExit('duplicate photo file names in different folders - stage them separately')
    ring = read_polygon(a.kml)
    fr = LocalFrame(*ring[0])
    poly = [fr.xy(la, lo) for la, lo in ring]

    rows = []
    for p in files:
        r = read_meta(p)
        r['path'], r['folder'] = p, str(p.parent)
        if not isinstance(r.get('GpsLatitude'), float):
            r['keep'], r['why'] = False, 'no GPS'
        else:
            r['x'], r['y'] = fr.xy(r['GpsLatitude'], r['GpsLongitude'])
            d = dist_to_poly((r['x'], r['y']), poly)
            pitch = r.get('GimbalPitchDegree')
            why = []
            if not isinstance(pitch, float) or pitch > a.min_pitch:
                why.append(f'pitch {pitch}')
            if d > a.buffer:
                why.append(f'{d:.0f} m outside area')
            if (r.get('RelativeAltitude') or 0) < a.min_rel_alt:
                why.append(f'rel alt {r.get("RelativeAltitude")}')
            r['keep'], r['why'] = not why, '; '.join(why)
        rows.append(r)

    proj = Path(a.project)
    img = proj / 'images'
    if img.exists():
        shutil.rmtree(img)
    img.mkdir(parents=True)
    for r in rows:
        if r['keep']:
            src = r['path']
            try:
                os.link(src, img / r['file'])
            except OSError:
                shutil.copy2(src, img / r['file'])

    cols = ['file', 'time', 'keep', 'why'] + list(XMP_KEYS) + ['model', 'width', 'height', 'f35mm', 'folder']
    with open(proj / 'photos.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, cols, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    kept = [r for r in rows if r['keep']]
    print(f'{len(files)} photos, {len(kept)} kept for processing, {len(files) - len(kept)} dropped')
    shut = {r.get('ShutterType') for r in rows}
    print('shutter type:', ', '.join(map(str, shut)))
    print('camera:', ', '.join(sorted({f"{r['model']} {r['width']}x{r['height']}" for r in rows})))
    if kept:
        ra = [r['RelativeAltitude'] for r in kept]
        print(f'height above takeoff of kept photos: {min(ra):.0f}-{max(ra):.0f} m')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8))
    px, py = zip(*(poly + poly[:1]))
    ax.plot(px, py, 'k-', lw=1, label='survey area')
    for keep, col, lab in ((False, 'tab:red', 'dropped'), (True, 'tab:blue', 'kept')):
        s = [r for r in rows if r['keep'] == keep and 'x' in r]
        if s:
            ax.scatter([r['x'] for r in s], [r['y'] for r in s], s=6, c=col, label=f'{lab} ({len(s)})')
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.legend()
    ax.set_title('Photo positions')
    fig.savefig(proj / 'photos.png', dpi=120, bbox_inches='tight')
    print('wrote', proj / 'photos.csv', 'and photos.png')


if __name__ == '__main__':
    main()
