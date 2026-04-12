#!/usr/bin/env python3
"""
SYSTEM_PLAN.md Component 1 — build pixel_utm_map.npy from a georeferenced GeoTIFF in UTM.

Output: numpy array float32 of shape (H, W, 2) — [:,:,0] = Easting, [:,:,1] = Northing.

Requires: pip install rasterio (or use GDAL python bindings).

Example:
  python3 generate_utm_map.py --geotiff ./satellite_georef.tif --out ./deploy/pixel_utm_map.npy
"""
from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate pixel_utm_map.npy from UTM GeoTIFF")
    parser.add_argument("--geotiff", required=True, help="Path to warped UTM GeoTIFF")
    parser.add_argument("--out", required=True, help="Output .npy path")
    args = parser.parse_args()

    try:
        import numpy as np
        import rasterio
        from rasterio.transform import xy as transform_xy
    except ImportError:
        print("Install dependencies: pip install numpy rasterio", file=sys.stderr)
        raise SystemExit(1)

    with rasterio.open(args.geotiff) as ds:
        if ds.crs is None:
            print("GeoTIFF has no CRS; run gdalwarp to UTM first.", file=sys.stderr)
            return 1
        t = ds.transform
        h, w = ds.height, ds.width
        rows, cols = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
        xe, yn = transform_xy(t, rows, cols, offset="center")
        east = np.asarray(xe, dtype=np.float64)
        north = np.asarray(yn, dtype=np.float64)

    out = np.stack([east, north], axis=-1).astype(np.float32)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    np.save(args.out, out)
    print(f"Wrote {out.shape} to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
