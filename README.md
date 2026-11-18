# dji-survey-pipeline

From a polygon drawn in **Google Earth** to **DJI Fly waypoint transects**, and
from the **SD-card photos** to a **georeferenced orthophoto (GeoTIFF)**.
The pipeline is made for the DJI **Mavic 4 Pro** and the **Mavic 3 / 3 Classic / 3 Pro**.

```
Google Earth polygon (.kml)
  └─ dji_fly_survey.py  ──► DJI Fly mission .kmz (one per battery) + plan plot + 3-D preview .kml
        fly it, copy the SD card
SD-card photos (DCIM/...)
  └─ prep_photos.py     ──► project/images (survey shots only) + photos.csv/png
       ├─ quick_mosaic.py   ──► quick_mosaic.tif   (GPS only, minutes, seams of a few m)
       ├─ stitch_mosaic.py  ──► stitched_mosaic.tif (feature matching, flat ground, no 3-D)
       └─ run_odm.py        ──► odm_orthophoto.tif  (OpenDroneMap photogrammetry, best)
  └─ overview_map.py    ──► overview_map.png (orthophoto + flight track + photo positions)
```

Tested on a Mavic 4 Pro survey (Fløyen, Bergen: 530 × 25 MP photos → 1 cm
orthophoto). The Mavic 3 camera models are in the code, but the Mavic 3 has
not been flown with this pipeline yet.

## Install

* **Python ≥ 3.9** with `pip install -r requirements.txt` (numpy, pillow,
  pyproj, tifffile, matplotlib). The planner alone needs only the standard library.
* **Docker Desktop**, for OpenDroneMap: `docker pull opendronemap/odm`.
  `stitch_mosaic.py` and `overview_map.py` need OpenCV, rasterio and GDAL. If
  these are missing locally, the scripts **re-run themselves inside the ODM
  image** (see `docker_util.py`), so nothing else has to be installed.
* For viewing, use **QGIS** (LTR). See [Viewing the results](#viewing-the-results).

Keep the projects on **C:**. Docker Desktop on Windows could not write to D:
or to network drives.

## 1. Plan: Google Earth → DJI Fly transects

1. In Google Earth Pro or Web, draw a **polygon** around the area. Optionally
   add a **placemark named `start`** where you will take off.
2. Save both into one **KML** (or KMZ), for example `area.kml`.
3. In DJI Fly on **your** controller, make a tiny dummy waypoint mission
   (2 points) and save it. Copy its KMZ off the controller
   (`Android/data/dji.go.v5/files/waypoint/<uuid>/<uuid>.kmz`). It tells the
   script your drone's model code (`--like`).
4. Run:

```bash
# Mavic 4 Pro, 25 MP, 50 m above ground, terrain following (Norway: Kartverket DEM)
python dji_fly_survey.py area.kml --camera m4p-25mp --agl 50 \
    --forward-overlap 75 --side-overlap 65 --terrain kartverket \
    --like dummy.kmz --max-flight-time 25

# Mavic 3 / 3 Classic (or m3p-20mp for the 3 Pro), target 1.5 cm GSD, no DEM
python dji_fly_survey.py area.kml --camera m3-20mp --gsd 1.5 --like dummy.kmz
```

Outputs go to `<name>_mission/`:

* `*_partN.kmz`: one mission per battery.
* `*_summary.txt`: the flight checklist, with speed, Timed Shot interval and heights.
* `*_waypoints.csv`
* `*_plan.png`
* `*_preview.kml`: 3-D check in Google Earth.

`--inspect some.kmz` shows what a DJI Fly mission contains.

### Getting the missions into DJI Fly

DJI Fly has no import button. The usual trick:

1. Make one dummy mission per part on the controller.
2. Overwrite each dummy's KMZ with the planned one, **keeping the dummy's
   uuid file name**.
3. Restart DJI Fly.

### Important flight facts (read `*_summary.txt`)

* DJI Fly ignores photo actions between waypoints. The default `--trigger timed`
  flies at the speed where the camera's **Timed Shot** interval gives the
  requested overlap. **Start Timed Shot yourself** when the mission starts.
* Heights in the KMZ are relative to the **takeoff point**. Take off where the
  plan says. With `--terrain none`, ground higher than takeoff reduces the clearance.
* **Obstacle sensing does not keep the height above ground.** Only the DEM-based
  terrain following does. Add the tree and building height to `--agl`. Keep it ≤ 120 m.
* Both the Mavic 3 and the Mavic 4 have an electronic (rolling) shutter. Use
  a fast shutter (≤ 1/1000 s) and moderate speed.
* Norway/EU (open category, A1–A3), quick reminders:
  * Flyver registration and ID on the drone.
  * Remote-pilot certificate.
  * Check the airspace and NOTAMs (Avinor/ninox map).
  * Max 120 m.
  * Keep VLOS.
  * Stay clear of people.

## 2. Process: SD card → orthophoto

Copy the card (or point at it). The scripts search `DCIM/` sub-folders recursively.

```bash
# 1. pick the survey photos (nadir, inside the area + 30 m, above 15 m) and stage them
python prep_photos.py E:/DCIM --kml area.kml --project C:/survey/site1

# 2a. quick look in minutes, GPS + heading + DEM only (5 cm)
python quick_mosaic.py C:/survey/site1 --kml area.kml --res 0.05

# 2b. photogrammetry with OpenDroneMap: fast orthophoto, 1 cm (about 2 h for 530 photos)
python run_odm.py C:/survey/site1 --res 1
#     full 3-D: dense point cloud, DSM/DTM, textured mesh (several hours, more RAM)
python run_odm.py C:/survey/site1 --mode full --res 2

# 2c. (optional) panorama-style stitch without 3-D (needs quick_mosaic.py output first)
python stitch_mosaic.py C:/survey/site1 --res 0.01

# 3. overview map: orthophoto + flight track + photo positions (+ planned lines)
python overview_map.py C:/survey/site1 --kml area.kml --plan area_mission/area_waypoints.csv
```

Main outputs in the project folder:

| file | what |
|---|---|
| `photos.csv`, `photos.png` | every photo with GPS, gimbal, camera; kept or dropped and why |
| `quick_mosaic.tif` | GPS-only mosaic in UTM, RGBA GeoTIFF |
| `odm_orthophoto/odm_orthophoto.tif` (+ `.ovr`) | ODM orthophoto, zoom pyramids built automatically |
| `odm_dem/dsm.tif`, `dtm.tif` | surface and terrain models (`--mode full`) |
| `odm_texturing/odm_textured_model_geo.obj` | 3-D mesh (`--mode full`) |
| `odm_georeferencing/odm_georeferenced_model.laz` | point cloud (sparse in fast mode) |
| `stitched_mosaic.tif` | feature-matched stitch, JPEG GeoTIFF with overviews |
| `overview_map.png` | map with track and photo positions |

### Which method?

| | time (530 × 25 MP) | quality |
|---|---|---|
| `quick_mosaic.py` | ~5 min | seams/offsets of a few m. Good for a field check of coverage and sharpness |
| `stitch_mosaic.py` | ~1–2 h at 1 cm | seamless on flat ground. Relief blurs, colours are flatter |
| `run_odm.py` (fast) | ~2 h | **best**: sharp, true 1 cm, correct over relief |
| `run_odm.py --mode full` | 3–6 h | adds the DSM, DTM, point cloud and 3-D mesh |

All products rely only on the drone GPS, so the **absolute position is good
to about 2–5 m**. For better accuracy, use ground control points (ODM
`gcp_list.txt`) or an RTK drone.

### Lessons learnt (Mavic 4 Pro, Fløyen, Nov 2026)

* **XMP `GimbalYawDegree` was about 14° off** the true image heading;
  `FlightYawDegree` was right. `quick_mosaic.py` therefore uses the flight yaw
  (`--yaw gimbal` switches back). `stitch_mosaic.py` self-calibrates heading and scale.
* **The true height above ground was about 0.8 × the DEM-derived value.** The
  90 m Copernicus DEM is too coarse on a summit. Use `--dem kartverket` in Norway.
* **ODM ran out of memory when texturing** 530 full-resolution 25 MP photos with
  29 GB of Docker RAM. `run_odm.py` therefore splits into overlapping blocks
  above 250 photos.
* **`--split` together with `--rolling-shutter` crashes OpenSfM** (create_tracks
  IndexError). `run_odm.py` turns rolling-shutter correction off when splitting.
* **After a crash, rerun with `--clean`.** `--rerun-all` does not remove the
  stale `opensfm/` and `submodels/` features.
* Give Docker Desktop as much RAM as you can (Settings → Resources / `.wslconfig`).

## Viewing the results

* **QGIS:** drag the `.tif` in. Different UTM zones are reprojected on the fly.
  Add a background map with XYZ Tiles → OpenStreetMap, or Kartverket WMTS
  `https://cache.kartverket.no/v1/wmts/1.0.0/WMTSCapabilities.xml`
  ("Norge i bilder").
* **3-D in QGIS:** open View → 3D Map Views → New. Set Terrain to DEM, using
  `odm_dem/dsm.tif` (full mode) or Kartverket DTM 1 m (hoydedata.no). The
  orthophoto drapes over it. A `.laz` point cloud can also be dragged in.
* **The mesh** (`.obj`) opens in MeshLab, Blender or Windows 3D Viewer.

## Files

| file | purpose |
|---|---|
| `dji_fly_survey.py` | planner: KML polygon → DJI Fly WPML KMZ missions (cameras, terrain, battery split) |
| `prep_photos.py` | reads DJI XMP/EXIF, keeps the survey shots, stages `images/` as hard links |
| `quick_mosaic.py` | direct-georeferenced flat-ground mosaic; writes `cameras_init.csv` |
| `stitch_mosaic.py` | SIFT matching + global similarity alignment + gains + feathered tiled render |
| `run_odm.py` | OpenDroneMap Docker run with settings that work, plus zoom pyramids |
| `overview_map.py` | PNG map of orthophoto, flight track, photo positions, plan and area |
| `docker_util.py` | runs a script inside the ODM image when OpenCV/rasterio are missing |
| `examples/floyen_transect.kml` | example area + takeoff point |
| `examples/measured_heights_template.csv` | own height measurements for `--terrain` |

## License

MIT, see `LICENSE`.
