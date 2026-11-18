#!/usr/bin/env python3
"""
dji_fly_survey.py - terrain-following photogrammetry transects for DJI Fly
                    (DJI Mavic 4 Pro by default; Mavic 3 / 3 Classic / 3 Pro supported)

Turns a Google Earth polygon (KML/KMZ) into DJI Fly waypoint missions (WPML KMZ)
that fly parallel transects ("lawnmower") at a constant height above ground.

Notes:
  * Mavic 4 Pro camera model (100 MP / 25 MP, 72 deg FOV, 3:2 images).
  * DJI Fly IGNORES photo actions between waypoints (interval/distance triggers),
    so the default 'timed' mode flies the transects at the speed that makes the
    camera's Timed Shot interval (2 s, 3 s, ...) give the requested overlap. You
    start Timed Shot manually when the mission starts. The optional 'waypoint'
    mode puts a waypoint + takePhoto action at every photo position instead.
  * No DEM needed (default --terrain none): constant height above takeoff.
    Optional terrain following from a DEM (Kartverket in Norway, Open-Meteo/Copernicus
    elsewhere): extra pass-through waypoints are inserted only where the ground
    changes, so AGL and GSD stay constant on slopes. Heights in the KMZ are
    relative to the TAKEOFF point, so the takeoff location matters.
  * Transect direction: auto (fewest turns), contour (along the slope), or a
    fixed bearing. Exact line/polygon clipping (concave polygons ok).
  * Splits long surveys into several KMZ files by battery time.
  * Writes a Google Earth 3D preview KML, waypoint CSV, summary and plot.

Example (Floyen test):
  python dji_fly_survey.py "floyen-test/floyen transect.kml" --agl 80 \
      --like path/to/dummy_from_controller.kmz --plot

Python >= 3.8, no required packages (matplotlib optional for the plot).
Elevation lookups need internet; results are cached next to the output.
"""

import argparse
import csv
import json
import math
import re
import sys
import time
import urllib.parse
import urllib.request
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

# --------------------------------------------------------------------------
# Cameras
# --------------------------------------------------------------------------
# dfov = diagonal field of view (DJI spec). w/h = image size in pixels.
# Timed Shot intervals (s) per DJI spec; the 64 GB Mavic 4 Pro has longer minimums.
CAMERAS = {
    'm4p-25mp': dict(name='Mavic 4 Pro Hasselblad 25 MP', w=6144, h=4096, dfov=72.0,
                     intervals={'64': [2, 3, 5, 7, 10, 15, 20, 30, 60],
                                '512': [1, 2, 3, 5, 7, 10, 15, 20, 30, 60]}),
    'm4p-100mp': dict(name='Mavic 4 Pro Hasselblad 100 MP', w=12288, h=8192, dfov=72.0,
                      intervals={'64': [10, 15, 20, 30, 60],
                                 '512': [2, 3, 5, 7, 10, 15, 20, 30, 60]}),
    'm3p-20mp': dict(name='Mavic 3 Pro Hasselblad 20 MP', w=5280, h=3956, dfov=84.0,
                     intervals={'64': [2, 3, 5, 7, 10, 15, 20, 30, 60],
                                '512': [2, 3, 5, 7, 10, 15, 20, 30, 60]}),
    # Mavic 3 / Mavic 3 Classic: same 4/3 Hasselblad main camera as the 3 Pro
    'm3-20mp': dict(name='Mavic 3 / 3 Classic Hasselblad 20 MP', w=5280, h=3956, dfov=84.0,
                    intervals={'64': [2, 3, 5, 7, 10, 15, 20, 30, 60],
                               '512': [2, 3, 5, 7, 10, 15, 20, 30, 60]}),
}


def camera_for_image(w, h, f35=None):
    """Focal length in pixels for a photo of size w x h (landscape).
    Known DJI cameras are recognised by their image size; otherwise the EXIF
    35 mm-equivalent focal length is used. Returns (f_px, description)."""
    w, h = max(w, h), min(w, h)
    diag = math.hypot(w, h)
    for key, c in CAMERAS.items():
        if (c['w'], c['h']) == (w, h):
            return diag / 2 / math.tan(math.radians(c['dfov'] / 2)), c['name']
    if f35:
        return float(f35) * diag / math.hypot(36, 24), f'unknown camera, {f35} mm (35 mm equiv.)'
    raise ValueError(f'unknown camera with {w}x{h} photos and no 35 mm focal length in EXIF')


class Camera:
    def __init__(self, key):
        c = CAMERAS[key]
        self.key, self.name, self.w, self.h = key, c['name'], c['w'], c['h']
        diag = math.hypot(self.w, self.h)
        t = math.tan(math.radians(c['dfov'] / 2))
        self.tan_h = t * self.w / diag      # half-angle tangent, long side
        self.tan_v = t * self.h / diag      # half-angle tangent, short side
        self.intervals = c['intervals']

    def footprint(self, agl):
        """(across-track, along-track) ground footprint in m. With the nose
        following the wayline the long image side is across-track."""
        return 2 * agl * self.tan_h, 2 * agl * self.tan_v

    def gsd_cm(self, agl):
        return 100 * self.footprint(agl)[0] / self.w

    def agl_for_gsd(self, gsd_cm):
        return gsd_cm / 100 * self.w / (2 * self.tan_h)


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
class LocalFrame:
    """Simple local tangent-plane projection (fine for areas of a few km)."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        p = math.radians(lat0)
        self.my = 111132.954 - 559.822 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
        e2 = 0.00669437999014
        self.mx = math.pi / 180 * 6378137.0 * math.cos(p) / math.sqrt(1 - e2 * math.sin(p) ** 2)

    def xy(self, lat, lon):
        return (lon - self.lon0) * self.mx, (lat - self.lat0) * self.my

    def ll(self, x, y):
        return self.lat0 + y / self.my, self.lon0 + x / self.mx


def read_polygon(path):
    """First polygon outer ring from a KML or KMZ -> [(lat, lon), ...]."""
    path = Path(path)
    if path.suffix.lower() == '.kmz':
        with zipfile.ZipFile(path) as z:
            name = next(n for n in z.namelist() if n.lower().endswith('.kml'))
            root = ET.fromstring(z.read(name))
    else:
        root = ET.parse(path).getroot()
    coords_el = None
    for el in root.iter():
        if el.tag.endswith('Polygon'):
            for sub in el.iter():
                if sub.tag.endswith('outerBoundaryIs'):
                    coords_el = next(c for c in sub.iter() if c.tag.endswith('coordinates'))
                    break
            break
    if coords_el is None:
        raise ValueError(f'No Polygon found in {path}')
    pts = []
    for tok in coords_el.text.split():
        parts = tok.split(',')
        if len(parts) >= 2:
            pts.append((float(parts[1]), float(parts[0])))
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        raise ValueError('Polygon needs at least 3 vertices')
    return pts


def read_points(path):
    """Point placemarks in a KML/KMZ -> [(name, lat, lon)]."""
    path = Path(path)
    if path.suffix.lower() == '.kmz':
        with zipfile.ZipFile(path) as z:
            name = next(n for n in z.namelist() if n.lower().endswith('.kml'))
            root = ET.fromstring(z.read(name))
    else:
        root = ET.parse(path).getroot()
    out = []
    for pm in root.iter():
        if not pm.tag.endswith('Placemark'):
            continue
        nm = next((e.text or '' for e in pm if e.tag.endswith('name')), '')
        for pt in pm.iter():
            if pt.tag.endswith('Point'):
                c = next((e.text for e in pt.iter() if e.tag.endswith('coordinates')), None)
                if c:
                    v = c.strip().split(',')
                    out.append((nm.strip(), float(v[1]), float(v[0])))
    return out


def polygon_area(xy):
    return abs(sum(xy[i][0] * xy[i - 1][1] - xy[i - 1][0] * xy[i][1] for i in range(len(xy)))) / 2


def basis(bearing_deg):
    """Unit vectors: d along the transects (compass bearing), n across them."""
    b = math.radians(bearing_deg)
    return (math.sin(b), math.cos(b)), (math.cos(b), -math.sin(b))


def to_uv(xy, bearing):
    d, n = basis(bearing)
    return [(x * d[0] + y * d[1], x * n[0] + y * n[1]) for x, y in xy]


def from_uv(u, v, bearing):
    d, n = basis(bearing)
    return u * d[0] + v * n[0], u * d[1] + v * n[1]


def crossings(uv, v):
    out = []
    m = len(uv)
    for i in range(m):
        (ua, va), (ub, vb) = uv[i], uv[(i + 1) % m]
        if va == vb:
            continue
        if min(va, vb) <= v <= max(va, vb):
            out.append(ua + (v - va) / (vb - va) * (ub - ua))
    return out


def make_lines(poly_xy, bearing, spacing, extend):
    """Parallel transects covering the polygon. Returns [(v, u_min, u_max)]."""
    uv = to_uv(poly_xy, bearing)
    vmin = min(p[1] for p in uv)
    vmax = max(p[1] for p in uv)
    width = vmax - vmin
    n = max(1, math.ceil(width / spacing - 1e-9)) + 1
    v0 = vmin - ((n - 1) * spacing - width) / 2
    lines = []
    for k in range(n):
        v = v0 + k * spacing
        us = []
        # Use the whole strip this line images so no polygon corner is missed
        for t in (-0.5, -0.25, 0.0, 0.25, 0.5):
            us += crossings(uv, v + t * spacing)
        if us:
            lines.append((v, min(us) - extend, max(us) + extend))
    return lines


def dist3(a, b):
    return math.sqrt((a['x'] - b['x']) ** 2 + (a['y'] - b['y']) ** 2 + (a['alt'] - b['alt']) ** 2)


# --------------------------------------------------------------------------
# Terrain
# --------------------------------------------------------------------------
class Terrain:
    """Ground elevation lookups with an on-disk cache.
    kartverket : Norwegian national height model (best available, ~1 m in most areas)
    open-meteo : Copernicus GLO-90 (global, 90 m)
    none       : flat ground (heights become plain height above takeoff)"""

    UA = {'User-Agent': 'dji-fly-survey/2.0 (python urllib)'}

    def __init__(self, source, cache_file):
        self.source = source
        self.cache_file = Path(cache_file)
        self.cache = {}
        if self.cache_file.exists():
            try:
                self.cache = {tuple(map(float, k.split(','))): v
                              for k, v in json.loads(self.cache_file.read_text()).items()}
            except Exception:
                self.cache = {}

    @staticmethod
    def key(lat, lon):
        return (round(lat, 6), round(lon, 6))

    def save(self):
        if self.source == 'none':
            return
        self.cache_file.parent.mkdir(parents=True, exist_ok=True)
        self.cache_file.write_text(json.dumps({f'{k[0]},{k[1]}': v for k, v in self.cache.items()}))

    def _get_json(self, url):
        last = None
        for attempt in range(4):
            try:
                req = urllib.request.Request(url, headers=self.UA)
                with urllib.request.urlopen(req, timeout=30) as r:
                    return json.loads(r.read().decode())
            except Exception as e:  # network hiccup / service busy
                last = e
                busy = '429' in str(e)
                if busy:
                    print('  (elevation service rate limit - waiting)', flush=True)
                time.sleep((20 if busy else 1.5) * (attempt + 1))
        raise RuntimeError(f'Elevation request failed: {last}')

    def _kartverket(self, pts):
        q = urllib.parse.urlencode({
            'koordsys': 4258, 'geojson': 'false',
            'punkter': json.dumps([[lon, lat] for lat, lon in pts], separators=(',', ':'))})
        data = self._get_json('https://ws.geonorge.no/hoydedata/v1/punkt?' + q)
        if 'punkter' not in data:
            raise RuntimeError(f'Kartverket: {data}')
        return [p.get('z') for p in data['punkter']]

    def _openmeteo(self, pts):
        q = urllib.parse.urlencode({
            'latitude': ','.join(f'{lat:.6f}' for lat, _ in pts),
            'longitude': ','.join(f'{lon:.6f}' for _, lon in pts)})
        data = self._get_json('https://api.open-meteo.com/v1/elevation?' + q)
        return data['elevation']

    def heights(self, latlons):
        if self.source == 'none':
            return [0.0] * len(latlons)
        if self.source == 'open-meteo':
            return self._grid_heights(latlons)
        return self._point_heights(latlons)

    def _grid_heights(self, latlons):
        """The global DEM is 90 m, so fetch a fixed ~30 m lattice once and
        interpolate bilinearly (few requests, avoids rate limits)."""
        dlat = 0.0003
        dlon = round(dlat / math.cos(math.radians(latlons[0][0])), 6)
        nodes, cells = set(), []
        for lat, lon in latlons:
            i, j = math.floor(lat / dlat), math.floor(lon / dlon)
            cells.append((i, j, lat / dlat - i, lon / dlon - j))
            for a in (0, 1):
                for b in (0, 1):
                    nodes.add((round((i + a) * dlat, 6), round((j + b) * dlon, 6)))
        self._point_heights(sorted(nodes))
        out = []
        for i, j, fi, fj in cells:
            z = [[self.cache[(round((i + a) * dlat, 6), round((j + b) * dlon, 6))] for b in (0, 1)] for a in (0, 1)]
            out.append(z[0][0] * (1 - fi) * (1 - fj) + z[0][1] * (1 - fi) * fj
                       + z[1][0] * fi * (1 - fj) + z[1][1] * fi * fj)
        return out

    def _point_heights(self, latlons):
        keys = [self.key(*p) for p in latlons]
        todo = sorted({k for k in keys if k not in self.cache})
        if todo:
            print(f'  Fetching {len(todo)} ground elevations from {self.source} ...', flush=True)
        batch = 50 if self.source == 'kartverket' else 100
        for i in range(0, len(todo), batch):
            chunk = todo[i:i + batch]
            z = self._kartverket(chunk) if self.source == 'kartverket' else self._openmeteo(chunk)
            missing = [c for c, h in zip(chunk, z) if h is None]
            for c, h in zip(chunk, z):
                if h is not None:
                    self.cache[c] = float(h)
            if missing:  # e.g. outside Kartverket coverage -> global fallback
                for c, h in zip(missing, self._openmeteo(missing)):
                    self.cache[c] = float(h)
            if self.source == 'open-meteo':
                time.sleep(1.0)
                self.save()
        if todo:
            self.save()
        return [self.cache[k] for k in keys]


class MeasuredTerrain:
    """Ground heights from your own measurements.
    CSV  : columns lat, lon, elev (header optional, ';' or ',' or tab separated). Heights are
           interpolated linearly between points (scipy) or by inverse-distance weighting.
           Outside the measured points the nearest point is used.
    TIFF : GeoTIFF DSM/DEM (needs tifffile + pyproj). Nodata cells -> highest valid sample.
    All heights must use the same datum as the takeoff elevation (e.g. all from the same GPS)."""

    def __init__(self, path, frame):
        self.path, self.fr = Path(path), frame
        if not self.path.exists():
            sys.exit(f'--terrain: file not found: {path}')
        self.outside = 0
        if self.path.suffix.lower() in ('.tif', '.tiff'):
            self._load_tiff()
        else:
            self._load_csv()

    def _load_csv(self):
        rows = []
        for line in self.path.read_text(encoding='utf-8-sig').splitlines():
            parts = [p.strip() for p in re.split(r'[;,\t]', line) if p.strip()]
            try:
                lat, lon, z = (float(p) for p in parts[:3])
            except (ValueError, TypeError):
                continue  # header / comment
            rows.append((lat, lon, z))
        if len(rows) < 3:
            sys.exit(f'--terrain: need >= 3 lat,lon,elev rows in {self.path}')
        self.kind = 'points'
        self.pts = [self.fr.xy(lat, lon) for lat, lon, _ in rows]
        self.z = [z for _, _, z in rows]
        print(f'  Measured heights: {len(rows)} points, {min(self.z):.1f}-{max(self.z):.1f} m')
        try:
            from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
            self._lin = LinearNDInterpolator(self.pts, self.z)
            self._near = NearestNDInterpolator(self.pts, self.z)
        except Exception:
            self._lin = None

    def _load_tiff(self):
        import tifffile
        from pyproj import CRS, Transformer
        with tifffile.TiffFile(self.path) as tf:
            page = tf.pages[0]
            self.arr = page.asarray()
            if self.arr.ndim == 3:
                self.arr = self.arr[..., 0] if self.arr.shape[-1] < self.arr.shape[0] else self.arr[0]
            tags = page.tags
            scale = tags['ModelPixelScaleTag'].value
            tie = tags['ModelTiepointTag'].value
            geo = tags['GeoKeyDirectoryTag'].value
            nod = tags.get('GDAL_NODATA')
        keys = {geo[i]: geo[i + 3] for i in range(4, len(geo), 4)}
        epsg = keys.get(3072) or keys.get(2048) or 4326
        self.x0, self.y0 = tie[3] - tie[0] * scale[0], tie[4] + tie[1] * scale[1]
        self.dx, self.dy = scale[0], scale[1]
        self.nodata = float(nod.value.strip('\x00')) if nod else None
        self.tr = Transformer.from_crs(4326, CRS.from_epsg(epsg), always_xy=True)
        self.kind = 'raster'
        print(f'  Measured DSM: {self.path.name} {self.arr.shape[1]}x{self.arr.shape[0]} px, '
              f'{self.dx:.2f} m cells, EPSG:{epsg}')

    def heights(self, latlons):
        out = []
        if self.kind == 'raster':
            ny, nx = self.arr.shape
            for lat, lon in latlons:
                X, Y = self.tr.transform(lon, lat)
                c = (X - self.x0) / self.dx - 0.5
                r = (self.y0 - Y) / self.dy - 0.5
                ci, ri = int(math.floor(c)), int(math.floor(r))
                vals = []
                for rr, cc, w in ((ri, ci, (1 - (r - ri)) * (1 - (c - ci))), (ri, ci + 1, (1 - (r - ri)) * (c - ci)),
                                  (ri + 1, ci, (r - ri) * (1 - (c - ci))), (ri + 1, ci + 1, (r - ri) * (c - ci))):
                    if 0 <= rr < ny and 0 <= cc < nx:
                        v = float(self.arr[rr, cc])
                        if math.isfinite(v) and (self.nodata is None or v != self.nodata) and v > -1000:
                            vals.append((v, w))
                if vals and sum(w for _, w in vals) > 0:
                    out.append(sum(v * w for v, w in vals) / sum(w for _, w in vals))
                else:
                    out.append(None)
            valid = [v for v in out if v is not None]
            if not valid:
                sys.exit('--terrain: the DSM does not cover the survey area')
            self.outside += len(out) - len(valid)
            top = max(valid)
            return [top if v is None else v for v in out]
        xy = [self.fr.xy(lat, lon) for lat, lon in latlons]
        if self._lin is not None:
            lin = self._lin(xy)
            near = self._near(xy)
            for a, b in zip(lin, near):
                if math.isfinite(a):
                    out.append(float(a))
                else:
                    out.append(float(b)); self.outside += 1
            return out
        for x, y in xy:  # inverse distance weighting fallback
            ws = []
            for (px, py), z in zip(self.pts, self.z):
                d2 = (x - px) ** 2 + (y - py) ** 2
                if d2 < 0.01:
                    ws = [(1.0, z)]; break
                ws.append((1 / d2, z))
            out.append(sum(w * z for w, z in ws) / sum(w for w, _ in ws))
        return out

    def save(self):
        pass


def fit_plane(xs, ys, zs):
    """Least-squares z = a*x + b*y + c. Returns (a, b, c)."""
    n = len(xs)
    sx, sy, sz = sum(xs), sum(ys), sum(zs)
    sxx = sum(x * x for x in xs); syy = sum(y * y for y in ys); sxy = sum(x * y for x, y in zip(xs, ys))
    sxz = sum(x * z for x, z in zip(xs, zs)); syz = sum(y * z for y, z in zip(ys, zs))
    A = [[sxx, sxy, sx, sxz], [sxy, syy, sy, syz], [sx, sy, n, sz]]
    for i in range(3):  # Gauss-Jordan
        p = max(range(i, 3), key=lambda r: abs(A[r][i]))
        A[i], A[p] = A[p], A[i]
        for r in range(3):
            if r != i:
                f = A[r][i] / A[i][i]
                A[r] = [a - f * b for a, b in zip(A[r], A[i])]
    return A[0][3] / A[0][0], A[1][3] / A[1][1], A[2][3] / A[2][2]


def simplify_profile(s, req, tol):
    """Douglas-Peucker on the required-altitude profile, then lift segments so
    the flown (linearly interpolated) altitude is never more than `tol` below
    the requirement. Returns (kept indices, altitudes at kept indices)."""
    n = len(s)
    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        worst, wk = -1, None
        for k in range(i + 1, j):
            h = req[i] + (req[j] - req[i]) * (s[k] - s[i]) / (s[j] - s[i])
            e = abs(req[k] - h)
            if e > worst:
                worst, wk = e, k
        if worst > tol:
            keep.add(wk)
            stack += [(i, wk), (wk, j)]
    idx = sorted(keep)
    alt = {i: req[i] for i in idx}
    for a, b in zip(idx[:-1], idx[1:]):
        deficit = max(req[k] - (alt[a] + (alt[b] - alt[a]) * (s[k] - s[a]) / (s[b] - s[a]))
                      for k in range(a, b + 1))
        if deficit > tol:
            alt[a] += deficit - tol
            alt[b] += deficit - tol
    return idx, [alt[i] for i in idx]


def interp(s, xs, ys):
    if s <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if s <= xs[i]:
            t = (s - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


# --------------------------------------------------------------------------
# DJI Fly WPML writer
# --------------------------------------------------------------------------
def read_drone_info(kmz):
    """droneEnumValue / droneSubEnumValue from a mission made in DJI Fly."""
    with zipfile.ZipFile(kmz) as z:
        name = next(n for n in z.namelist() if n.endswith('template.kml') or n.endswith('waylines.wpml'))
        txt = z.read(name).decode('utf-8', 'replace')
    e = re.search(r'<wpml:droneEnumValue>(\d+)<', txt)
    s = re.search(r'<wpml:droneSubEnumValue>(\d+)<', txt)
    if not e:
        raise ValueError(f'No droneEnumValue in {kmz}')
    return int(e.group(1)), int(s.group(1)) if s else 0


def inspect_kmz(kmz):
    with zipfile.ZipFile(kmz) as z:
        for n in z.namelist():
            if n.endswith('/'):
                continue
            txt = z.read(n).decode('utf-8', 'replace')
            print(f'== {n}  ({len(txt)} chars)')
            for tag in ('droneEnumValue', 'droneSubEnumValue', 'payloadEnumValue', 'executeHeightMode',
                        'globalTransitionalSpeed', 'autoFlightSpeed', 'finishAction'):
                m = re.search(rf'<wpml:{tag}>([^<]*)<', txt)
                if m:
                    print(f'   {tag}: {m.group(1)}')
            print(f'   waypoints: {txt.count("<Placemark>")}')
            funcs = sorted(set(re.findall(r'<wpml:actionActuatorFunc>([^<]*)<', txt)))
            if funcs:
                print(f'   actions: {", ".join(funcs)}')
                for f in funcs:
                    m = re.search(rf'<wpml:actionActuatorFunc>{f}</wpml:actionActuatorFunc>\s*'
                                  r'<wpml:actionActuatorFuncParam>(.*?)</wpml:actionActuatorFuncParam>',
                                  txt, re.S)
                    if m:
                        params = re.findall(r'<wpml:(\w+)>([^<]*)<', m.group(1))
                        print(f'     {f}: ' + ', '.join(f'{k}={v}' for k, v in params))


def _mission_config(drone, trans_speed):
    return f"""    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{trans_speed:g}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{drone[0]}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>{drone[1]}</wpml:droneSubEnumValue>
      </wpml:droneInfo>
    </wpml:missionConfig>
"""


HEAD = ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">\n'
        '  <Document>\n')
TAIL = '  </Document>\n</kml>\n'


def write_kmz(path, wps, speed, trans_speed, drone, pitch):
    """wps: list of dicts with lat, lon, rel (height above takeoff), turn ('stop'|'pass'), photo (bool)."""
    now = str(int(time.time() * 1000))
    template = (HEAD + '    <wpml:author>fly</wpml:author>\n'
                f'    <wpml:createTime>{now}</wpml:createTime>\n'
                f'    <wpml:updateTime>{now}</wpml:updateTime>\n'
                + _mission_config(drone, trans_speed) + TAIL)

    group_id, action_id = 1, 1
    pm = []
    for i, w in enumerate(wps):
        turn = ('toPointAndStopWithContinuityCurvature' if w['turn'] == 'stop'
                else 'toPointAndPassWithContinuityCurvature')
        groups = []

        def group(start, end, body):
            nonlocal group_id
            g = f"""        <wpml:actionGroup>
          <wpml:actionGroupId>{group_id}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{start}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{end}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>{'sequence' if 'takePhoto' in body else 'parallel'}</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
          </wpml:actionTrigger>
{body}        </wpml:actionGroup>
"""
            group_id += 1
            return g

        def action(func, params):
            nonlocal action_id
            p = ''.join(f'              <wpml:{k}>{v}</wpml:{k}>\n' for k, v in params)
            a = f"""          <wpml:action>
            <wpml:actionId>{action_id}</wpml:actionId>
            <wpml:actionActuatorFunc>{func}</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
{p}            </wpml:actionActuatorFuncParam>
          </wpml:action>
"""
            action_id += 1
            return a

        if i == 0:
            groups.append(group(0, 0, action('gimbalRotate', [
                ('gimbalHeadingYawBase', 'aircraft'), ('gimbalRotateMode', 'absoluteAngle'),
                ('gimbalPitchRotateEnable', 1), ('gimbalPitchRotateAngle', pitch),
                ('gimbalRollRotateEnable', 0), ('gimbalRollRotateAngle', 0),
                ('gimbalYawRotateEnable', 0), ('gimbalYawRotateAngle', 0),
                ('gimbalRotateTimeEnable', 0), ('gimbalRotateTime', 0),
                ('payloadPositionIndex', 0)])))
        if w.get('photo'):
            groups.append(group(i, i, action('takePhoto', [
                ('payloadPositionIndex', 0), ('useGlobalPayloadLensIndex', 0)])))
        if i < len(wps) - 1:  # keep the gimbal nadir on the leg to the next point (as DJI Fly writes it)
            groups.append(group(i, i + 1, action('gimbalEvenlyRotate', [
                ('gimbalPitchRotateAngle', pitch), ('payloadPositionIndex', 0)])))

        pm.append(f"""      <Placemark>
        <Point>
          <coordinates>
            {w['lon']:.10f},{w['lat']:.10f}
          </coordinates>
        </Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{w['rel_int']}</wpml:executeHeight>
        <wpml:waypointSpeed>{speed:g}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingAngleEnable>{1 if i == 0 else 0}</wpml:waypointHeadingAngleEnable>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>{turn}</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>
{''.join(groups)}      </Placemark>
""")

    waylines = (HEAD + _mission_config(drone, trans_speed) + f"""    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>0</wpml:distance>
      <wpml:duration>0</wpml:duration>
      <wpml:autoFlightSpeed>{speed:g}</wpml:autoFlightSpeed>
{''.join(pm)}    </Folder>
""" + TAIL)

    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('wpmz/template.kml', template)
        z.writestr('wpmz/waylines.wpml', waylines)
    return Path(path).stat().st_size


def write_preview_kml(path, name, poly_ll, parts, takeoff, z_takeoff):
    """Google Earth 3D preview, altitudes absolute (DEM datum ~ mean sea level)."""
    colors = ['ff0000ff', 'ffff7f00', 'ff00c000', 'ff00a5ff', 'ffc000c0', 'ffc0c000']
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
           f'<name>{name} preview</name>',
           '<Style id="poly"><LineStyle><color>ff00ffff</color><width>2</width></LineStyle>'
           '<PolyStyle><color>2000ffff</color></PolyStyle></Style>']
    for i, c in enumerate(colors):
        out.append(f'<Style id="p{i}"><LineStyle><color>{c}</color><width>3</width></LineStyle></Style>')
    ring = ' '.join(f'{lon:.7f},{lat:.7f},0' for lat, lon in poly_ll + poly_ll[:1])
    out.append('<Placemark><name>Survey area</name><styleUrl>#poly</styleUrl><Polygon><tessellate>1</tessellate>'
               f'<outerBoundaryIs><LinearRing><coordinates>{ring}</coordinates></LinearRing></outerBoundaryIs>'
               '</Polygon></Placemark>')
    out.append(f'<Placemark><name>TAKEOFF ({z_takeoff:.0f} m)</name><Point><coordinates>'
               f'{takeoff[1]:.7f},{takeoff[0]:.7f},0</coordinates></Point></Placemark>')
    for pi, wps in enumerate(parts):
        coords = ' '.join(f"{w['lon']:.7f},{w['lat']:.7f},{w['alt']:.1f}" for w in wps)
        out.append(f'<Folder><name>Part {pi + 1}</name>'
                   f'<Placemark><name>Part {pi + 1} flight path</name><styleUrl>#p{pi % len(colors)}</styleUrl>'
                   f'<LineString><extrude>1</extrude><altitudeMode>absolute</altitudeMode>'
                   f'<coordinates>{coords}</coordinates></LineString></Placemark>')
        for i, w in enumerate(wps):
            out.append(f"<Placemark><name>{i}</name><description>{w['rel_int']} m above takeoff, "
                       f"{w['agl']:.0f} m AGL</description><Point><altitudeMode>absolute</altitudeMode>"
                       f"<coordinates>{w['lon']:.7f},{w['lat']:.7f},{w['alt']:.1f}</coordinates></Point></Placemark>")
        out.append('</Folder>')
    out.append('</Document></kml>')
    Path(path).write_text('\n'.join(out), encoding='utf-8')


# --------------------------------------------------------------------------
# Planner
# --------------------------------------------------------------------------
def choose_speed(spacing, allowed, max_speed, forced_interval):
    if forced_interval:
        iv = forced_interval
        if iv not in allowed:
            print(f'  WARNING: {iv} s is not a Timed Shot interval for this camera/storage {allowed}')
    else:
        ok = [i for i in allowed if spacing / i <= max_speed]
        iv = min(ok) if ok else max(allowed)
    speed = math.floor(spacing / iv * 10) / 10  # round down -> overlap >= target
    return iv, max(speed, 0.5)


def build(args):
    cam = Camera(args.camera)
    poly_ll = read_polygon(args.input)
    lat0 = sum(p[0] for p in poly_ll) / len(poly_ll)
    lon0 = sum(p[1] for p in poly_ll) / len(poly_ll)
    fr = LocalFrame(lat0, lon0)
    poly_xy = [fr.xy(*p) for p in poly_ll]

    agl = args.agl if args.agl else cam.agl_for_gsd(args.gsd)
    across, along = cam.footprint(agl)
    line_spacing = across * (1 - args.side_overlap / 100)
    photo_spacing = along * (1 - args.forward_overlap / 100)
    gsd = cam.gsd_cm(agl)

    if args.trigger == 'timed':
        interval, speed = choose_speed(photo_spacing, cam.intervals[args.storage], args.max_speed, args.interval)
    else:
        interval, speed = None, min(args.max_speed, args.speed or args.max_speed)
    eff_fwd = 100 * (1 - (speed * interval if interval else photo_spacing) / along)

    src = args.terrain
    if src == 'auto':
        src = 'kartverket' if (57.5 < lat0 < 71.5 and 4.0 < lon0 < 31.5) else 'open-meteo'
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    if src in ('none', 'kartverket', 'open-meteo'):
        terr = Terrain(src, Path(__file__).resolve().parent / 'elevation_cache' / f'{src}.json')
    else:
        terr = MeasuredTerrain(src, fr)
        src = f'measured ({Path(args.terrain).name})'

    # ---- transect direction
    turn_cost_m = 8 * speed  # ~8 s per line for stop/turn/accelerate
    xs = [p[0] for p in poly_xy]; ys = [p[1] for p in poly_xy]
    slope_info = None
    if src != 'none':
        gx, gy = [], []
        for i in range(7):
            for j in range(7):
                gx.append(min(xs) + (max(xs) - min(xs)) * i / 6)
                gy.append(min(ys) + (max(ys) - min(ys)) * j / 6)
        gz = terr.heights([fr.ll(x, y) for x, y in zip(gx, gy)])
        a, b, _ = fit_plane(gx, gy, gz)
        slope_info = dict(pct=100 * math.hypot(a, b), downslope=math.degrees(math.atan2(-a, -b)) % 360,
                          contour=math.degrees(math.atan2(-b, a)) % 180, zmin=min(gz), zmax=max(gz))

    def cost(bearing):
        ls = make_lines(poly_xy, bearing, line_spacing, 0)
        return sum(u1 - u0 for _, u0, u1 in ls) + len(ls) * (turn_cost_m + line_spacing)

    if args.direction == 'auto':
        bearing = min(range(0, 180), key=cost)
    elif args.direction == 'contour':
        if not slope_info:
            sys.exit('--direction contour needs terrain data')
        bearing = slope_info['contour']
    else:
        bearing = float(args.direction) % 180

    extend = along * 0.5 if args.edge_extend is None else args.edge_extend
    lines = make_lines(poly_xy, bearing, line_spacing, extend)

    # ---- order (boustrophedon) and start point
    if args.takeoff:
        tparts = [float(v) for v in args.takeoff.split(',')]
        takeoff_xy = fr.xy(tparts[0], tparts[1])
    else:
        tparts = None
        cx = {'s': min(ys), 'n': max(ys)}; cy = {'w': min(xs), 'e': max(xs)}
        c = args.start_corner
        takeoff_xy = (cy[c[1]], cx[c[0]])
    best = None
    for rev_lines in (False, True):
        for rev_first in (False, True):
            ordered = list(reversed(lines)) if rev_lines else list(lines)
            first = from_uv(ordered[0][2] if rev_first else ordered[0][1], ordered[0][0], bearing)
            dd = math.dist(first, takeoff_xy)
            if best is None or dd < best[0]:
                best = (dd, ordered, rev_first)
    _, ordered, rev_first = best

    # ---- terrain sampling: centre of each line and both strip edges
    step = args.terrain_step
    half = line_spacing / 2
    line_samples = []
    query = []
    for k, (v, u0, u1) in enumerate(ordered):
        forward = (k % 2 == 0) != rev_first
        ua, ub = (u0, u1) if forward else (u1, u0)
        L = abs(ub - ua)
        n = max(2, math.ceil(L / step) + 1)
        s = [L * i / (n - 1) for i in range(n)]
        us = [ua + (ub - ua) * t / L for t in s]
        pts = {o: [fr.ll(*from_uv(u, v + o, bearing)) for u in us] for o in (-half, 0.0, half)}
        line_samples.append(dict(v=v, s=s, us=us, pts=pts))
        for o in pts:
            query += pts[o]
    z_all = terr.heights(query)
    pos = 0
    for ls in line_samples:
        n = len(ls['s'])
        z = {}
        for o in (-half, 0.0, half):
            z[o] = z_all[pos:pos + n]; pos += n
        ls['ground'] = z[0.0]
        ls['zoff'] = z
        strip = [max(z[o][i] for o in z) for i in range(n)]
        ls['strip'] = [max(strip[max(0, i - 1):i + 2]) for i in range(n)]

    if tparts and len(tparts) >= 3:
        z_takeoff = tparts[2]
    elif tparts:
        z_takeoff = terr.heights([(tparts[0], tparts[1])])[0]
    else:
        z_takeoff = line_samples[0]['ground'][0]
    takeoff_ll = fr.ll(*takeoff_xy)

    # ---- per-line waypoints
    lines_wps = []
    for ls in line_samples:
        s = ls['s']
        if src == 'none':
            idx, alts = [0, len(s) - 1], [agl, agl]
        else:
            ref = ls['strip'] if args.clearance == 'strip' else ls['ground']
            req = [g + agl for g in ref]
            idx, alts = simplify_profile(s, req, args.terrain_tol)
        ls['flight'] = [interp(t, [s[i] for i in idx], alts) for t in s]
        if args.trigger == 'waypoint':
            L = s[-1]
            n = max(2, math.ceil(L / photo_spacing) + 1)
            stations = [L * i / (n - 1) for i in range(n)]
            wl = [dict(s=t, alt=interp(t, s, ls['flight']), turn='stop', photo=True) for t in stations]
        else:
            wl = [dict(s=s[i], alt=a, turn='stop' if i in (0, len(s) - 1) else 'pass', photo=False)
                  for i, a in zip(idx, alts)]
        for w in wl:
            u = interp(w['s'], s, ls['us'])
            w['x'], w['y'] = from_uv(u, ls['v'], bearing)
            w['lat'], w['lon'] = fr.ll(w['x'], w['y'])
            w['ground'] = interp(w['s'], s, ls['ground'])
        lines_wps.append(wl)

    # ---- timing and battery split
    stop_s = 8 if args.trigger == 'timed' else 3
    climb_rate = 4.0

    def transit_time(a_xy, b_xy, dz):
        return math.dist(a_xy, b_xy) / args.transit_speed + abs(dz) / climb_rate

    def part_time(wl_lines):
        flat = [w for wl in wl_lines for w in wl]
        t = sum(dist3(a, b) for a, b in zip(flat[:-1], flat[1:])) / speed
        t += stop_s * sum(1 for w in flat if w['turn'] == 'stop')
        t += transit_time(takeoff_xy, (flat[0]['x'], flat[0]['y']), flat[0]['alt'] - z_takeoff)
        t += transit_time((flat[-1]['x'], flat[-1]['y']), takeoff_xy, flat[-1]['alt'] - z_takeoff)
        return t

    def split(limit_s):
        parts, cur = [], []
        for wl in lines_wps:
            if cur and limit_s and part_time(cur + [wl]) > limit_s:
                parts.append(cur); cur = []
            cur.append(wl)
        parts.append(cur)
        return parts

    limit = args.max_flight_time * 60 if args.max_flight_time else 0
    parts = split(limit)
    if len(parts) > 1:  # balance the parts: lowest limit that still gives the same number of parts
        lo, hi = part_time(lines_wps) / len(parts), limit
        for _ in range(30):
            mid = (lo + hi) / 2
            if len(split(mid)) > len(parts):
                lo = mid
            else:
                hi = mid
        parts = split(hi)

    return dict(cam=cam, poly_ll=poly_ll, poly_xy=poly_xy, fr=fr, agl=agl, gsd=gsd, across=across,
                along=along, line_spacing=line_spacing, photo_spacing=photo_spacing, interval=interval,
                speed=speed, eff_fwd=eff_fwd, src=src, bearing=bearing, slope=slope_info,
                line_samples=line_samples, parts=parts, part_time=part_time, z_takeoff=z_takeoff,
                takeoff_ll=takeoff_ll, takeoff_xy=takeoff_xy, takeoff_given=bool(tparts), extend=extend, terr=terr)


def terrain_points(P):
    tx, ty, tz, seen = [], [], [], set()
    for ls in P['line_samples']:
        for o, zs in ls['zoff'].items():
            for u, z in zip(ls['us'], zs):
                x, y = from_uv(u, ls['v'] + o, P['bearing'])
                k = (round(x), round(y))
                if k not in seen:
                    seen.add(k); tx.append(x); ty.append(y); tz.append(z)
    return tx, ty, tz


def plot3d(P, path):
    """Large 3D views of terrain + flight plan (two viewing directions)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    tx, ty, tz = terrain_points(P)
    cols = plt.rcParams['axes.prop_cycle'].by_key()['color']
    xs = tx + [w['x'] for part in P['parts'] for wl in part for w in wl]
    ys = ty + [w['y'] for part in P['parts'] for wl in part for w in wl]
    xr, yr = max(xs) - min(xs), max(ys) - min(ys)
    zr = max(max(w['alt'] for part in P['parts'] for wl in part for w in wl), max(tz)) - min(tz)
    exag = max(1.0, round(0.3 * max(xr, yr) / max(zr, 1), 1))  # make relief readable
    fig = plt.figure(figsize=(20, 9))
    for k, azim in enumerate((-125, 55)):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        ax.plot_trisurf(tx, ty, tz, cmap='terrain', linewidth=0, alpha=0.8, antialiased=True)
        for pi, part in enumerate(P['parts']):
            flat = [w for wl in part for w in wl]
            c = cols[pi % len(cols)]
            ax.plot([w['x'] for w in flat], [w['y'] for w in flat], [w['alt'] for w in flat],
                    '-', color=c, lw=1.6, label=f'Part {pi + 1}' if len(P['parts']) > 1 else 'Flight path')
            for w in flat:
                if w['turn'] == 'stop':  # drop lines show height above ground at line ends
                    ax.plot([w['x']] * 2, [w['y']] * 2, [w['ground'], w['alt']], color=c, lw=0.5, alpha=0.6)
            ax.text(flat[0]['x'], flat[0]['y'], flat[0]['alt'] + 5, f'P{pi + 1}', color=c, fontsize=9,
                    fontweight='bold')
        ax.scatter(*P['takeoff_xy'], P['z_takeoff'], marker='*', s=300, color='gold', edgecolor='k',
                   label='Takeoff', depthshade=False)
        ax.set_box_aspect((xr, yr, zr * exag))
        ax.view_init(elev=28, azim=azim)
        ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.set_zlabel('Elevation (m)')
        ax.set_title(f'View from {"south-west" if k == 0 else "north-east"} (vertical x{exag:g})')
        if k == 0:
            ax.legend(fontsize=8, loc='upper left')
    fig.suptitle(f"3D flight plan: {P['agl']:.0f} m above ground, GSD {P['gsd']:.2f} cm, "
                 f"{len(P['parts'])} part(s), takeoff ground {P['z_takeoff']:.0f} m", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(f'  3D plot: {path}')


def plot(P, path, show):
    try:
        import matplotlib
        if not show:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('  (matplotlib not installed - skipping plot)')
        return
    px = [p[0] for p in P['poly_xy']] + [P['poly_xy'][0][0]]
    py = [p[1] for p in P['poly_xy']] + [P['poly_xy'][0][1]]
    if P['src'] == 'none':
        fig, ax = plt.subplots(figsize=(10, 8)); ax2 = ax3 = None
        ax.fill(px, py, color='#2ca02c', alpha=0.12)
    else:
        fig = plt.figure(figsize=(22, 8))
        ax = fig.add_subplot(1, 3, 1)
        ax3 = fig.add_subplot(1, 3, 2, projection='3d')
        ax2 = fig.add_subplot(1, 3, 3)
        # terrain from all DEM samples (line centres + strip edges)
        tx, ty, tz = [], [], []
        seen = set()
        for ls in P['line_samples']:
            for o, zs in ls['zoff'].items():
                for u, z in zip(ls['us'], zs):
                    x, y = from_uv(u, ls['v'] + o, P['bearing'])
                    k = (round(x), round(y))
                    if k not in seen:
                        seen.add(k); tx.append(x); ty.append(y); tz.append(z)
        cf = ax.tricontourf(tx, ty, tz, levels=20, cmap='terrain', alpha=0.85, zorder=0)
        ax.tricontour(tx, ty, tz, levels=20, colors='k', linewidths=0.3, alpha=0.4, zorder=0)
        fig.colorbar(cf, ax=ax, shrink=0.7, label='Ground elevation (m)')
        ax3.plot_trisurf(tx, ty, tz, cmap='terrain', linewidth=0, alpha=0.75)
    ax.plot(px, py, color='#2ca02c' if P['src'] == 'none' else 'k', lw=2, label='Survey polygon')
    cols = plt.rcParams['axes.prop_cycle'].by_key()['color']
    dist0 = 0
    for pi, part in enumerate(P['parts']):
        flat = [w for wl in part for w in wl]
        c = cols[pi % len(cols)]
        ax.plot([w['x'] for w in flat], [w['y'] for w in flat], '-', color=c, lw=1.6,
                label=f'Part {pi + 1}' if len(P['parts']) > 1 else 'Flight path')
        st = [w for w in flat if w['turn'] == 'stop']
        mid = [w for w in flat if w['turn'] != 'stop']
        ax.scatter([w['x'] for w in st], [w['y'] for w in st], s=22, color=c, edgecolor='k', zorder=3)
        if mid:
            ax.scatter([w['x'] for w in mid], [w['y'] for w in mid], s=10, marker='^', color=c, zorder=3)
        ax.annotate(f'P{pi + 1} start', (flat[0]['x'], flat[0]['y']), fontsize=8,
                    xytext=(4, 4), textcoords='offset points')
        if ax3 is not None:
            ax3.plot([w['x'] for w in flat], [w['y'] for w in flat], [w['alt'] for w in flat],
                     '-', color=c, lw=1.5, zorder=10)
    ax.scatter(*P['takeoff_xy'], marker='*', s=250, color='gold', edgecolor='k', zorder=5, label='Takeoff')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
    ax.set_title(f"{P['cam'].name}: GSD {P['gsd']:.2f} cm, {P['agl']:.0f} m AGL, lines {P['bearing']:.0f}°")
    ax.legend(fontsize=8, loc='best')
    if ax3 is not None:
        ax3.scatter(*P['takeoff_xy'], P['z_takeoff'], marker='*', s=200, color='gold', edgecolor='k')
        ax3.set_xlabel('East (m)'); ax3.set_ylabel('North (m)'); ax3.set_zlabel('Elevation (m)')
        ax3.set_title('3D view: terrain and flight path')
        ax3.view_init(elev=30, azim=-120)
        try:
            xr = max(px) - min(px); yr = max(py) - min(py)
            ax3.set_box_aspect((xr, yr, max(0.25 * max(xr, yr), 1)))
        except Exception:
            pass

    # profile along the mission (transects only)
    if ax2 is None:
        fig.tight_layout(); fig.savefig(path, dpi=150); print(f'  Plot: {path}')
        if show:
            plt.show()
        plt.close(fig)
        return
    for ls in P['line_samples']:
        s = [dist0 + t for t in ls['s']]
        ax2.plot(s, ls['ground'], color='saddlebrown', lw=1)
        ax2.plot(s, ls['strip'], color='saddlebrown', lw=0.6, ls=':')
        ax2.plot(s, ls['flight'], color='tab:blue', lw=1.2)
        dist0 = s[-1] + 10
    ax2.axhline(P['z_takeoff'], color='gold', lw=1, ls='--', label=f"Takeoff ground {P['z_takeoff']:.0f} m")
    ax2.plot([], [], color='saddlebrown', label='Ground (line centre)')
    ax2.plot([], [], color='saddlebrown', ls=':', label='Highest ground in strip')
    ax2.plot([], [], color='tab:blue', label='Flight altitude')
    ax2.set_xlabel('Distance along transects (m, turns removed)'); ax2.set_ylabel('Elevation (m)')
    ax2.set_title('Terrain following profile'); ax2.grid(alpha=0.3); ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f'  Plot: {path}')
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', nargs='?', help='KML/KMZ with the survey polygon (Google Earth)')
    ap.add_argument('-o', '--out', help='Output folder (default: <input folder>/<name>_mission)')
    ap.add_argument('--name', help='Mission name / file prefix (default: from input file name)')
    g = ap.add_argument_group('camera and overlap')
    g.add_argument('--camera', choices=sorted(CAMERAS), default='m4p-25mp',
                   help='Camera/photo mode (default m4p-25mp: Mavic 4 Pro 25 MP; m3-20mp / m3p-20mp for Mavic 3)')
    g.add_argument('--storage', choices=['64', '512'], default='64',
                   help='Mavic 4 Pro storage version, sets the shortest Timed Shot interval (default 64)')
    g.add_argument('--agl', type=float, help='Flight height above ground, m (overrides --gsd)')
    g.add_argument('--gsd', type=float, default=1.5, help='Target GSD in cm/pixel (default 1.5)')
    g.add_argument('--forward-overlap', type=float, default=80, help='Forward overlap %% (default 80)')
    g.add_argument('--side-overlap', type=float, default=70, help='Side overlap %% (default 70)')
    g.add_argument('--gimbal-pitch', type=int, default=-90, help='Gimbal pitch (default -90 = nadir)')
    g = ap.add_argument_group('triggering and speed')
    g.add_argument('--trigger', choices=['timed', 'waypoint'], default='timed',
                   help="timed: fly at constant speed, you start Timed Shot manually (default). "
                        "waypoint: stop and takePhoto at every photo position (many waypoints)")
    g.add_argument('--interval', type=float, help='Force Timed Shot interval in s (default: auto)')
    g.add_argument('--max-speed', type=float, default=8.0, help='Max transect speed m/s (default 8)')
    g.add_argument('--speed', type=float, help='Speed for --trigger waypoint (default --max-speed)')
    g.add_argument('--transit-speed', type=float, default=10.0,
                   help='Speed flying to the first waypoint, m/s (default 10)')
    g = ap.add_argument_group('layout')
    g.add_argument('--direction', default='auto',
                   help="Transect bearing: 'auto' (fewest/shortest lines), 'contour' (along the slope), "
                        "or degrees from north, e.g. 90 for east-west lines")
    g.add_argument('--edge-extend', type=float, help='Extend lines past the polygon by this many m '
                   '(default: half the along-track footprint)')
    g.add_argument('--takeoff', help="Takeoff point 'lat,lon' or 'lat,lon,ground_elev', or 'lowest' (lowest ground "
                   "on the tracks; needs --terrain). STRONGLY recommended: "
                   'all heights are relative to it. Default: the mission start point')
    g.add_argument('--start-corner', choices=['sw', 'se', 'nw', 'ne'], default='sw',
                   help='Start corner if --takeoff is not given (default sw)')
    g.add_argument('--max-flight-time', type=float, default=25,
                   help='Split into parts longer than this many minutes incl. transit (default 25, 0=off)')
    g = ap.add_argument_group('terrain')
    g.add_argument('--terrain', default='none',
                   help='none (default): constant height above takeoff. '
                        'FILE: your measured ground heights - CSV with lat,lon,elev columns '
                        '(GPS walk, spot heights) or a GeoTIFF DSM/DEM (e.g. WebODM from a scout '
                        'flight). auto/kartverket/open-meteo: online DEM')
    g.add_argument('--terrain-step', type=float, default=10.0, help='DEM sampling step along lines, m')
    g.add_argument('--terrain-tol', type=float, default=4.0,
                   help='Allowed AGL deviation before a terrain waypoint is added, m (default 4)')
    g.add_argument('--clearance', choices=['strip', 'centre'], default='strip',
                   help='Follow highest ground in the imaged strip (safer, default) or under the line')
    g = ap.add_argument_group('DJI Fly output')
    g.add_argument('--like', help='A KMZ made by DJI Fly on YOUR controller/drone; its droneEnumValue is copied')
    g.add_argument('--drone-enum', type=int, default=68, help='droneEnumValue if --like is not given (68)')
    g.add_argument('--drone-sub-enum', type=int, default=0)
    g.add_argument('--plot', action='store_true', help='Show the plot window (a PNG is always saved)')
    g.add_argument('--no-plot', action='store_true', help='Do not make the PNG')
    ap.add_argument('--inspect', help='Print drone enum and actions of a DJI Fly KMZ and exit')
    args = ap.parse_args()

    if args.inspect:
        inspect_kmz(args.inspect)
        return
    if not args.input:
        ap.error('input polygon required')
    inp = Path(args.input)
    args.name = args.name or re.sub(r'[^A-Za-z0-9_-]+', '_', inp.stem).strip('_')
    args.out = args.out or str(inp.parent / f'{args.name}_mission')
    if args.like:
        drone = read_drone_info(args.like)
        drone_note = f'copied from {Path(args.like).name}'
    else:
        drone = (args.drone_enum, args.drone_sub_enum)
        drone_note = 'DEFAULT - use --like with a DJI Fly mission made on your own drone'

    print(f'Reading {inp}')
    if not args.takeoff:
        pts = read_points(inp)
        if pts:
            pick = next((p for p in pts if re.search(r'start|take ?off|home|launch', p[0], re.I)), pts[0])
            args.takeoff = f'{pick[1]:.7f},{pick[2]:.7f}'
            print(f"  Takeoff from KML point '{pick[0]}': {args.takeoff}")
    if args.takeoff == 'lowest':
        if args.terrain == 'none':
            sys.exit("--takeoff lowest needs terrain data (--terrain ...)")
        args.takeoff = None
        P0 = build(args)  # elevations are cached, so the second pass is cheap
        lo = min(((g, ls['s'][i], ls) for ls in P0['line_samples'] for i, g in enumerate(ls['ground'])),
                 key=lambda t: t[0])
        u = interp(lo[1], lo[2]['s'], lo[2]['us'])
        lat, lon = P0['fr'].ll(*from_uv(u, lo[2]['v'], P0['bearing']))
        args.takeoff = f'{lat:.6f},{lon:.6f}'
        print(f'  Takeoff set to lowest ground on the tracks: {args.takeoff} ({lo[0]:.0f} m)')
    P = build(args)
    out = Path(args.out)
    cam = P['cam']

    # ---- heights relative to takeoff + safety checks
    warnings = []
    all_rel, all_agl = [], []
    for part in P['parts']:
        for wl in part:
            for w in wl:
                w['rel'] = w['alt'] - P['z_takeoff']
                w['rel_int'] = math.ceil(w['rel'])
                w['agl'] = w['alt'] - w['ground']
                all_rel.append(w['rel_int']); all_agl.append(w['agl'])
    flown_agl = [f - g for ls in P['line_samples'] for f, g in zip(ls['flight'], ls['ground'])]
    clear = [f - g for ls in P['line_samples'] for f, g in zip(ls['flight'], ls['strip'])]
    if P['src'] != 'none' and min(all_rel) < 5:
        warnings.append(f'Lowest waypoint is {min(all_rel)} m above takeoff - take off from lower ground '
                        '(DJI Fly may refuse heights below takeoff).')
    if max(flown_agl) > 120:
        warnings.append(f'Max height above ground along the path is {max(flown_agl):.0f} m (> 120 m EU/Norway limit).')
    if max(all_rel) > 120:
        warnings.append(f'Max height above takeoff is {max(all_rel)} m: raise the altitude limit in DJI Fly '
                        f'(Safety > Max altitude) above {max(all_rel)} m or the mission will be capped.')
    if not P['takeoff_given'] and P['src'] != 'none':
        warnings.append('No --takeoff given: heights assume you take off at the mission start point '
                        f"({P['takeoff_ll'][0]:.6f}, {P['takeoff_ll'][1]:.6f}, ground {P['z_takeoff']:.0f} m). "
                        'Taking off from higher ground makes the drone fly LOWER than planned.')
    if P['src'] == 'none':
        warnings.append('No DEM: heights are above the TAKEOFF point. Ground (and trees/rocks) higher than '
                        'takeoff reduces AGL and GSD; take off from the highest point or add margin to --agl.')
    if getattr(P['terr'], 'outside', 0):
        warnings.append(f"{P['terr'].outside} terrain samples lie outside your measured heights; the nearest "
                        'measurement (points) or the highest DSM value (raster) was used there. Measure the whole area.')
    if P['src'] != 'none':
        warnings.append('AGL is above bare ground/DEM. Add the canopy height (trees, buildings) to --agl.')
    if not args.like:
        warnings.append(f'droneEnumValue {drone[0]} is a default; pass --like <dummy mission from your '
                        'controller> so DJI Fly accepts the file for your drone.')

    # ---- outputs
    kmzs = []
    flat_parts = []
    for pi, part in enumerate(P['parts']):
        flat = [w for wl in part for w in wl]
        flat_parts.append(flat)
        suffix = f'_part{pi + 1}' if len(P['parts']) > 1 else ''
        path = out / f'{args.name}{suffix}.kmz'
        size = write_kmz(path, flat, P['speed'], args.transit_speed, drone, args.gimbal_pitch)
        kmzs.append((path, flat, P['part_time'](part), size, len(part)))
        if len(flat) > 200:
            warnings.append(f'{path.name} has {len(flat)} waypoints; DJI Fly on some controllers fails '
                            'to load >~200. Use --max-flight-time to make smaller parts.')

    write_preview_kml(out / f'{args.name}_preview.kml', args.name, P['poly_ll'], flat_parts,
                      P['takeoff_ll'], P['z_takeoff'])
    with open(out / f'{args.name}_waypoints.csv', 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['part', 'index', 'lat', 'lon', 'ground_m', 'alt_m', 'height_above_takeoff_m', 'agl_m',
                     'turn', 'photo'])
        for pi, flat in enumerate(flat_parts):
            for i, w in enumerate(flat):
                wr.writerow([pi + 1, i, f"{w['lat']:.7f}", f"{w['lon']:.7f}", f"{w['ground']:.1f}",
                             f"{w['alt']:.1f}", w['rel_int'], f"{w['agl']:.1f}", w['turn'], int(w['photo'])])

    area_ha = polygon_area(P['poly_xy']) / 1e4
    trans_len = sum(ls['s'][-1] for ls in P['line_samples'])
    n_photos = (trans_len / (P['speed'] * P['interval']) if P['interval']
                else sum(1 for fl in flat_parts for w in fl if w['photo']))
    blur_t = 0.5 * P['gsd'] / 100 / P['speed']
    L = []
    L.append(f"Mission: {args.name}   ({time.strftime('%Y-%m-%d %H:%M')})")
    L.append(f"Camera: {cam.name} ({cam.w}x{cam.h})   area {area_ha:.1f} ha")
    L.append(f"Height above ground: {P['agl']:.0f} m  ->  GSD {P['gsd']:.2f} cm/px, "
             f"footprint {P['across']:.0f} x {P['along']:.0f} m (across x along)")
    L.append(f"Overlap: side {args.side_overlap:.0f}% (line spacing {P['line_spacing']:.1f} m), "
             f"forward {P['eff_fwd']:.0f}% (photo every {P['photo_spacing']:.1f} m)")
    L.append(f"Transects: {len(P['line_samples'])} lines, bearing {P['bearing']:.0f} deg, "
             f"{trans_len / 1000:.2f} km, extended {P['extend']:.0f} m past the polygon")
    if P['slope']:
        s = P['slope']
        L.append(f"Terrain ({P['src']}): {s['zmin']:.0f}-{s['zmax']:.0f} m, mean slope {s['pct']:.0f}% "
                 f"facing {s['downslope']:.0f} deg (contour bearing {s['contour']:.0f} deg)")
        L.append(f"Along the path: AGL {min(flown_agl):.0f}-{max(flown_agl):.0f} m; "
                 f"min clearance over highest ground in strip {min(clear):.0f} m")
    if P['src'] != 'none':
        L.append(f"Takeoff ground elevation used: {P['z_takeoff']:.0f} m at "
                 f"{P['takeoff_ll'][0]:.6f}, {P['takeoff_ll'][1]:.6f}")
    else:
        L.append(f"Terrain: none (flat). Planned takeoff/start near {P['takeoff_ll'][0]:.6f}, "
                 f"{P['takeoff_ll'][1]:.6f}")
    L.append(f"Heights above takeoff in KMZ: {min(all_rel)} to {max(all_rel)} m")
    if args.trigger == 'timed':
        L.append(f"SPEED {P['speed']:.1f} m/s  +  TIMED SHOT every {P['interval']:g} s  "
                 f"(~{n_photos:.0f} photos on transects, more during turns)")
    else:
        L.append(f"Stop-and-shoot at {n_photos} waypoints, speed {P['speed']:.1f} m/s")
    L.append(f"Motion blur <= 0.5 px needs shutter 1/{math.ceil(1 / blur_t)} s or faster")
    L.append(f"droneEnumValue {drone[0]}/{drone[1]} ({drone_note})")
    L.append('')
    for i, (path, flat, t, size, nl) in enumerate(kmzs):
        L.append(f"  {path.name}: {nl} lines, {len(flat)} waypoints, ~{t / 60:.0f} min incl. transit, {size / 1024:.0f} kB")
    if warnings:
        L.append('')
        L += ['WARNING: ' + w for w in warnings]
    L.append('')
    L.append('Field procedure (DJI Fly):')
    L.append('  1. On the controller, create a dummy waypoint mission (2 points) per part and save it.')
    L.append('  2. USB to PC: Android/data/dji.go.v5/files/waypoint/<UUID>/<UUID>.kmz')
    L.append('     replace that file with a part KMZ, renamed to exactly <UUID>.kmz.')
    L.append('  3. Restart DJI Fly, open the mission and check the path and heights.')
    if args.trigger == 'timed':
        L.append(f"  4. Camera: Photo > Timed Shot {P['interval']:g} s, {cam.name.split()[-2]} MP, S or M mode "
                 f'with shutter >= 1/{math.ceil(1 / blur_t)} s, focus at infinity (MF), JPEG or JPEG+RAW.')
        L.append('  5. TAKE OFF FROM THE PLANNED POINT, start the mission, start Timed Shot when the drone')
        L.append('     reaches the first waypoint, stop it after the last one.')
    else:
        L.append('  4. Camera in single-shot photo mode, shutter fixed as above. Take off from the planned point.')
    txt = '\n'.join(L)
    (out / f'{args.name}_summary.txt').write_text(txt + '\n', encoding='utf-8')
    print()
    print(txt)
    print(f'\nOutputs in {out}')
    if not args.no_plot:
        plot(P, out / f'{args.name}_plan.png', args.plot)
        if P['src'] != 'none':
            plot3d(P, out / f'{args.name}_3d.png')


if __name__ == '__main__':
    main()
