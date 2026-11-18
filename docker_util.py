"""Run a script of this repo inside the OpenDroneMap Docker image.

stitch_mosaic.py and overview_map.py need OpenCV / rasterio / GDAL. Instead of
installing those on Windows, they re-launch themselves inside the
opendronemap/odm image (which has them) when the import fails locally.

Every command-line argument that is an existing file or folder (or a new file
in an existing folder) is mounted into the container and rewritten to the
container path, so you can call the scripts with normal Windows paths.
"""
import os
import subprocess
import sys
from pathlib import Path

IMAGE = os.environ.get('ODM_IMAGE', 'opendronemap/odm')
REPO = Path(__file__).resolve().parent


def in_container():
    return Path('/.dockerenv').exists() or Path('/code/SuperBuild').exists()


def relaunch(script, argv=None, name=None):
    """Run `python3 script argv...` inside the ODM image and exit with its code."""
    if in_container():
        raise SystemExit('missing Python package inside the container - unexpected')
    argv = list(sys.argv[1:] if argv is None else argv)
    mounts = {str(REPO): '/work'}
    out = []
    for arg in argv:
        p = Path(arg)
        if arg.startswith('-') or not (p.exists() or (p.parent.exists() and p.suffix)):
            out.append(arg)
            continue
        p = p.resolve()
        host = p.parent if p.parent != p else p
        if str(host) not in mounts:
            mounts[str(host)] = f'/m{len(mounts)}'
        out.append(f'{mounts[str(host)]}/{p.name}')
    cmd = ['docker', 'run', '--rm']
    if name:
        cmd += ['--name', name]
    for h, c in mounts.items():
        cmd += ['-v', f'{h}:{c}']
    cmd += ['-w', '/work', '--entrypoint', 'python3', IMAGE, f'/work/{Path(script).name}'] + out
    print('running in Docker:', ' '.join(cmd), flush=True)
    env = dict(os.environ, MSYS_NO_PATHCONV='1')
    sys.exit(subprocess.call(cmd, env=env))
