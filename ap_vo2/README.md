# ap_vo2 - Map-Matching VPS Node

Standalone ROS 2 package. No dependency on any other NGPS package.

## What it does

Every `match_interval_ms` milliseconds:
1. Crops a `roi_size x roi_size` ROI from the centre of the latest camera frame
2. Runs AKAZE detect + describe on the ROI
3. Brute-force Hamming match against pre-computed reference tile descriptors (computed once at startup)
4. USAC_MAGSAC homography estimation + inlier count gate
5. Maps live ROI centre pixel through H -> reference tile pixel
6. Applies GDAL affine geotransform: `E = GT[0] + col*GT[1] + row*GT[2]`, `N = GT[3] + col*GT[4] + row*GT[5]`
7. Subtracts UTM origin: `x = E - E0`, `y = N - N0`

## Outputs

| Topic | Type | Description |
|---|---|---|
| `~/fix/local_xy` | `geometry_msgs/PointStamped` | Local (x, y) in metres |
| `~/fix/odometry` | `nav_msgs/Odometry` | Same position with covariance, for EKF fusion |

## Reference tile requirements

- GeoTIFF projected in **UTM** (e.g. EPSG:32633 for UTM zone 33N)
- Single-band greyscale **or** 3-band RGB (band 1 is used / bands are converted to grey)
- Must have a valid `GeoTransform` (standard for any orthorectified GeoTIFF)
- Web Mercator (EPSG:3857) tiles must be reprojected first:
  ```bash
  gdalwarp -t_srs EPSG:32633 input_3857.tif output_utm.tif
  ```

## Quick start

```bash
# build
colcon build --packages-select ap_vo2

# run
ros2 launch ap_vo2 map_match.launch.py \
  ref_tile_path:=/absolute/path/to/map.tif
```

## Key parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_topic` | `/camera/image_raw` | Downward-facing camera |
| `ref_tile_path` | `""` | Path to UTM GeoTIFF |
| `match_interval_ms` | `200.0` | Fix rate (~5 Hz) |
| `roi_size` | `512` | ROI side length (px) |
| `min_inliers` | `12` | RANSAC inlier gate |
| `origin_easting` | `0.0` | UTM E0 (0 = tile top-left) |
| `origin_northing` | `0.0` | UTM N0 (0 = tile top-left) |
| `position_covariance_xy` | `2.0` | m^2 placed on published covariance |

## Dependencies

- `opencv-python` (>= 4.7 for `cv2.USAC_MAGSAC`)
- `gdal` / `python3-gdal` (`from osgeo import gdal`)
- Standard ROS 2 Humble Python deps (`rclpy`, `cv_bridge`, etc.)
