"""Run OpenDroneMap (Docker) on a project made by prep_photos.py.

    python run_odm.py PROJECT_DIR                 # fast orthophoto, 1 cm
    python run_odm.py PROJECT_DIR --mode full     # + dense point cloud, DSM/DTM, 3-D mesh
    python run_odm.py PROJECT_DIR --dry-run       # only print the docker command
    python run_odm.py PROJECT_DIR -- --gps-accuracy 3   # extra ODM options after --

PROJECT_DIR must contain images/ (from prep_photos.py). Results go to
PROJECT_DIR/odm_orthophoto/odm_orthophoto.tif (+ odm_dem/, odm_texturing/ in
full mode). Afterwards zoom pyramids (.ovr) are built so QGIS opens the
orthophoto fast.

Settings that worked for 530 Mavic 4 Pro 25 MP photos with 29 GB Docker RAM:
  * more than --split-above photos -> process in overlapping blocks
    (--split), otherwise texturing runs out of memory;
  * rolling-shutter correction only WITHOUT split (with split OpenSfM crashes
    in create_tracks: IndexError);
  * a restart after a crash needs --clean (opensfm/ and submodels/ keep stale
    features even with --rerun-all).

Docker Desktop must be running. On Windows, Docker could not write to D: or
network drives here - keep projects on C:.
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

IMAGE = os.environ.get('ODM_IMAGE', 'opendronemap/odm')
CLEAN = ['opensfm', 'submodels', 'odm_meshing', 'odm_texturing', 'odm_georeferencing',
         'odm_orthophoto', 'odm_dem', 'odm_filterpoints', 'odm_report', 'mve', 'openmvs',
         'entwine_pointcloud']


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('project')
    ap.add_argument('--mode', choices=['fast', 'full'], default='fast',
                    help='fast = orthophoto only (no 3-D); full = point cloud, DSM/DTM, mesh, orthophoto')
    ap.add_argument('--res', type=float, default=1.0, help='orthophoto (and DEM) resolution, cm/pixel')
    ap.add_argument('--split-above', type=int, default=250,
                    help='use split-merge when there are more photos than this (default 250)')
    ap.add_argument('--split', type=int, default=180, help='photos per block when splitting')
    ap.add_argument('--split-overlap', type=float, default=60, help='block overlap, m')
    ap.add_argument('--rolling-shutter', choices=['auto', 'on', 'off'], default='auto',
                    help='auto = on when not splitting (Mavic 3/4 use an electronic shutter)')
    ap.add_argument('--pc-quality', default='medium', help='full mode point cloud quality')
    ap.add_argument('--cpus', type=int, default=min(8, os.cpu_count() or 4))
    ap.add_argument('--clean', action='store_true', help='delete results of a previous/crashed run first')
    ap.add_argument('--no-overviews', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    argv = sys.argv[1:]
    extra = argv[argv.index('--') + 1:] if '--' in argv else []
    a = ap.parse_args(argv[:argv.index('--')] if '--' in argv else argv)

    proj = Path(a.project).resolve()
    imgs = [p for p in (proj / 'images').glob('*') if p.suffix.lower() in ('.jpg', '.jpeg')]
    if not imgs:
        sys.exit(f'no photos in {proj / "images"} - run prep_photos.py first')
    n = len(imgs)

    if a.clean:
        for d in CLEAN:
            if (proj / d).exists():
                print('removing', proj / d)
                shutil.rmtree(proj / d)

    opts = ['--orthophoto-resolution', f'{a.res:g}', '--feature-quality', 'high',
            '--max-concurrency', str(a.cpus)]
    if a.mode == 'fast':
        opts.append('--fast-orthophoto')
    else:
        opts += ['--dsm', '--dtm', '--dem-resolution', f'{max(a.res, 2):g}',
                 '--pc-quality', a.pc_quality, '--pc-las']
    split = n > a.split_above
    if split:
        opts += ['--split', str(a.split), '--split-overlap', f'{a.split_overlap:g}']
    rs = a.rolling_shutter == 'on' or (a.rolling_shutter == 'auto' and not split)
    if rs and split:
        print('WARNING: rolling shutter + split crashed OpenSfM before (create_tracks IndexError)')
    if rs:
        opts.append('--rolling-shutter')

    name = 'odm_' + ''.join(c if c.isalnum() else '_' for c in proj.name)
    cmd = ['docker', 'run', '--rm', '--name', name,
           '-v', f'{proj.parent}:/datasets', IMAGE,
           '--project-path', '/datasets', proj.name] + opts + extra
    print(f'{n} photos, mode {a.mode}, {"split into blocks of " + str(a.split) if split else "no split"}, '
          f'rolling shutter {"on" if rs else "off"}')
    print(' '.join(cmd), flush=True)
    if a.dry_run:
        return
    env = dict(os.environ, MSYS_NO_PATHCONV='1')
    rc = subprocess.call(cmd, env=env)
    if rc:
        sys.exit(f'ODM failed (exit {rc}). Fix the cause, then rerun with --clean.')

    ortho = proj / 'odm_orthophoto' / 'odm_orthophoto.tif'
    print('orthophoto:', ortho if ortho.exists() else 'NOT FOUND')
    if ortho.exists() and not a.no_overviews:
        print('building zoom pyramids (.ovr) ...', flush=True)
        subprocess.call(['docker', 'run', '--rm', '-v', f'{ortho.parent}:/data', '-w', '/data',
                         '--entrypoint', '/code/SuperBuild/install/bin/gdaladdo', IMAGE,
                         '-ro', '-r', 'average',
                         '--config', 'COMPRESS_OVERVIEW', 'JPEG',
                         '--config', 'JPEG_QUALITY_OVERVIEW', '90',
                         '--config', 'INTERLEAVE_OVERVIEW', 'PIXEL',
                         '--config', 'BIGTIFF_OVERVIEW', 'YES',
                         '--config', 'GDAL_CACHEMAX', '4096',
                         ortho.name, '2', '4', '8', '16', '32', '64', '128', '256'], env=env)
    dup = proj / 'odm_orthophoto' / 'odm_orthophoto.original.tif'
    if dup.exists():
        print(f'note: {dup.name} ({dup.stat().st_size / 1e9:.1f} GB) is an uncropped copy and can be deleted')


if __name__ == '__main__':
    main()
