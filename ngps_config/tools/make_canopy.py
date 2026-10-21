#!/usr/bin/env python3
"""Build an optional canopy-height layer (height of the visible surface above bare ground) around a site.

For offline evaluation only: visual-odometry scale depends on height above what the camera sees, which over
forest is the treetops, not the GSI dem5a bare ground. Nothing in the flight stack reads this file yet.

    python3 -I ngps_config/tools/make_canopy.py --name nagano1 --lat 36.323136 --lon 138.618168 \
        --raw-dir /some/scratch/canopy_raw

Needs rasterio (GDAL) and matplotlib, e.g. in a venv:  uv venv v && uv pip install --python v/bin/python
rasterio matplotlib  (do not install into the system/Jetson Python). Only windowed HTTP range reads of
the source COGs are made (a few MB), no login is needed.

Sources (all anonymous, CC BY 4.0):
  meta  Meta/WRI High Resolution Canopy Height Maps v1 (alsgedi_global_v6_float), ~1 m, EPSG:3857, uint8 m,
        s3://dataforgood-fb-data/forests/v1/ (AWS Open Data). Cloud mask from msk/<quadkey>.tif.msk
        (255 = cloud/invalid, inferred from the data; the readme states the opposite).
        Attribution: Tolan et al. 2024, "Very high resolution canopy height maps from RGB imagery using
        self-supervised vision transformer and convolutional decoder trained on aerial lidar", Remote Sensing
        of Environment 300:113888. Data (c) Meta and World Resources Institute, CC BY 4.0.
  eth   ETH Global Canopy Height 2020, 10 m, EPSG:4326, uint8 m (255 = nodata), plus per-pixel std dev.
        Attribution: Lang, Jetz, Schindler, Wegner 2023, "A high-resolution canopy height model of the
        Earth", Nature Ecology & Evolution 7:1778-1789. Data doi:10.3929/ethz-b-000609802, CC BY 4.0.
JAXA AW3D30 (a DSM, would need DSM - dem5a) is not used: JAXA requires user registration to download it.

Output <out-dir>/canopy.npz (row 0 = north, pixel centres on a regular lat/lon grid):
  height_m      float32 (H, W) canopy height above ground [m] from the primary source, NaN = unknown
  lat, lon      float64 1-D pixel-centre axes (lat descending)
  bounds        float64 [min_lon, min_lat, max_lon, max_lat] of the outer pixel edges
  source        str, primary source description;  attribution, licence: str
  eth_height_m, eth_sd_m   float32 (H, W) ETH 10 m layer nearest-resampled onto the same grid (cross-check)
  summary_json  str, the same summary that is written to canopy.json
plus canopy.json (provenance + summary) and canopy_preview.png.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

os.environ.setdefault('GDAL_DISABLE_READDIR_ON_OPEN', 'EMPTY_DIR')
os.environ.setdefault('CPL_VSIL_CURL_USE_HEAD', 'NO')  # libdrive (ETH) does not answer HEAD usefully
os.environ.setdefault('GDAL_HTTP_MAX_RETRY', '4')
os.environ.setdefault('GDAL_HTTP_RETRY_DELAY', '2')

import rasterio  # noqa: E402
from rasterio.transform import from_origin  # noqa: E402
from rasterio.warp import Resampling, reproject, transform_bounds  # noqa: E402
from rasterio.windows import from_bounds  # noqa: E402

META_BASE = 'https://dataforgood-fb-data.s3.amazonaws.com/forests/v1/alsgedi_global_v6_float'
ETH_BASE = ('https://libdrive.ethz.ch/index.php/s/cO8or7iOe5dT2Rt/download?path=%2F3deg_cogs'
            '&files=ETH_GlobalCanopyHeight_10m_2020_{tile}_{kind}.tif')
META_SRC = 'Meta/WRI High Resolution Canopy Height Maps v1 (alsgedi_global_v6_float), ~1 m, CHM (canopy above ground)'
META_ATTR = ('Tolan et al. 2024, Remote Sens. Environ. 300:113888; data (c) Meta and World Resources Institute, '
             'via AWS Open Data s3://dataforgood-fb-data/forests/v1/')
ETH_SRC = 'ETH Global Canopy Height 2020 (Lang et al.), 10 m'
ETH_ATTR = 'Lang, Jetz, Schindler, Wegner 2023, Nat. Ecol. Evol. 7:1778-1789; data doi:10.3929/ethz-b-000609802'
LICENCE = 'CC BY 4.0'


def m_per_deg(lat):
    p = math.radians(lat)
    return (111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p),
            111412.84 * math.cos(p) - 93.5 * math.cos(3 * p))


def quadkey(lat, lon, z):
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    return ''.join(str(((x >> i) & 1) + 2 * ((y >> i) & 1)) for i in range(z - 1, -1, -1))


def out_grid(lat0, lon0, half_m, res_m):
    mlat, mlon = m_per_deg(lat0)
    n = int(round(2 * half_m / res_m))
    dlat, dlon = res_m / mlat, res_m / mlon
    west, north = lon0 - n / 2 * dlon, lat0 + n / 2 * dlat
    transform = from_origin(west, north, dlon, dlat)
    lon = west + (np.arange(n) + 0.5) * dlon
    lat = north - (np.arange(n) + 0.5) * dlat
    bounds = np.array([west, north - n * dlat, west + n * dlon, north])
    return transform, (n, n), lat, lon, bounds


def read_crop(url, bounds_ll, margin_px=8, raw_path=None, mask_url=None):
    """Windowed read of `url` covering lon/lat bounds; returns (float32 array with NaN, transform, crs)."""
    with rasterio.open('/vsicurl/' + url) as ds:
        b = transform_bounds('EPSG:4326', ds.crs, *bounds_ll, densify_pts=21)
        win = from_bounds(*b, transform=ds.transform).round_offsets().round_lengths()
        win = rasterio.windows.Window(win.col_off - margin_px, win.row_off - margin_px,
                                      win.width + 2 * margin_px, win.height + 2 * margin_px)
        win = win.intersection(rasterio.windows.Window(0, 0, ds.width, ds.height))
        raw = ds.read(1, window=win)
        tr, crs, nodata, prof = ds.window_transform(win), ds.crs, ds.nodata, ds.profile
    valid = np.ones(raw.shape, bool) if nodata is None else raw != nodata
    if mask_url:
        # Same pixel grid, no georeferencing. The readme says False = NODATA, but in practice the mask is 0 almost
        # everywhere and 255 only on small patches where the CHM is flat 0 (cloud gaps), so 255 = invalid here.
        with rasterio.open('/vsicurl/' + mask_url) as mk:
            valid &= mk.read(1, window=win) == 0
    if raw_path:
        prof.update(width=raw.shape[1], height=raw.shape[0], transform=tr, tiled=False, compress='deflate')
        for k in ('blockxsize', 'blockysize'):
            prof.pop(k, None)
        with rasterio.open(raw_path, 'w', **prof) as dst:
            dst.write(raw, 1)
        if mask_url:
            np.save(raw_path + '.valid.npy', valid)
    arr = raw.astype(np.float32)
    arr[~valid] = np.nan
    return arr, tr, crs


def warp(arr, src_tr, src_crs, dst_tr, shape, resampling):
    out = np.full(shape, np.nan, np.float32)
    reproject(arr, out, src_transform=src_tr, src_crs=src_crs, src_nodata=np.nan, dst_transform=dst_tr,
              dst_crs='EPSG:4326', dst_nodata=np.nan, resampling=resampling)
    return out


def stats(a):
    v = a[np.isfinite(a)]
    if v.size == 0:
        return {'median': None, 'p90': None, 'max': None, 'valid_frac': 0.0}
    return {'median': round(float(np.median(v)), 2), 'p90': round(float(np.percentile(v, 90)), 2),
            'max': round(float(v.max()), 2), 'mean': round(float(v.mean()), 2),
            'frac_over_5m': round(float((v > 5).mean()), 3), 'valid_frac': round(v.size / a.size, 3)}


def summarise(h, res_m):
    n = h.shape[0]
    c = n // 2  # grid is even-sized and centred, so the centre point is the corner of pixels c-1/c
    out = {'centre_m': round(float(np.nanmean(h[c - 1:c + 1, c - 1:c + 1])), 2)
           if np.isfinite(h[c - 1:c + 1, c - 1:c + 1]).any() else None}
    for half in (5, 50, 150, 400):
        k = max(1, int(round(half / res_m)))
        out[f'box_pm{half}m'] = stats(h[max(0, c - k):c + k, max(0, c - k):c + k])
    return out


def preview(path, layers, lat0, lon0, half_m, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Rectangle
    ramp = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']  # one-hue sequential
    cmap = LinearSegmentedColormap.from_list('seq_blue', ramp)
    cmap.set_bad('#d9d8d4')
    vmax = max(5.0, float(np.nanpercentile(np.concatenate([a[np.isfinite(a)] for _, a in layers]), 99)))
    fig, axes = plt.subplots(1, len(layers), figsize=(6.2 * len(layers), 6.0), squeeze=False,
                             constrained_layout=True)
    ext = [-half_m, half_m, -half_m, half_m]
    for ax, (name, a) in zip(axes[0], layers):
        im = ax.imshow(a, cmap=cmap, vmin=0, vmax=vmax, extent=ext, origin='upper', interpolation='nearest')
        for r in (50, 150):
            ax.add_patch(Rectangle((-r, -r), 2 * r, 2 * r, fill=False, ec='#1f1f1d', lw=1.0, ls='--'))
            ax.text(-r + 4, r - 4, f'±{r} m', color='#1f1f1d', fontsize=8, va='top',
                    bbox=dict(fc='white', ec='none', alpha=0.7, pad=1))
        ax.plot(0, 0, marker='+', ms=14, mew=3.5, color='white')
        ax.plot(0, 0, marker='+', ms=12, mew=1.8, color='#d03b3b')
        ax.set_title(name, fontsize=10, color='#1f1f1d')
        ax.set_xlabel('east of centre [m]', color='#5f5e5a')
        ax.set_ylabel('north of centre [m]', color='#5f5e5a')
        ax.tick_params(colors='#5f5e5a', labelsize=8)
        for s in ax.spines.values():
            s.set_color('#bdbcb7')
    cb = fig.colorbar(im, ax=axes[0].tolist(), shrink=0.85, pad=0.01)
    cb.set_label('canopy height above ground [m]  (gray = no data)', color='#5f5e5a')
    fig.suptitle(f'{title}  centre {lat0:.6f}, {lon0:.6f} (red +)', fontsize=11, color='#1f1f1d')
    fig.savefig(path, dpi=110, facecolor='white')
    plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True)
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--half-size-m', type=float, default=400.0, help='half side of the square [m]')
    ap.add_argument('--res-m', type=float, default=1.0, help='output grid spacing [m]')
    ap.add_argument('--sources', default='meta,eth', help='comma list, first available is primary')
    ap.add_argument('--raw-dir', help='if given, save the raw source crops (native grid) here')
    ap.add_argument('--out-dir', help='default ngps_config/terrain/<name>')
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.join(os.path.dirname(here), 'terrain', args.name)
    os.makedirs(out_dir, exist_ok=True)
    if args.raw_dir:
        os.makedirs(args.raw_dir, exist_ok=True)
    raw = (lambda f: os.path.join(args.raw_dir, f)) if args.raw_dir else (lambda f: None)
    dst_tr, shape, lat, lon, bounds = out_grid(args.lat, args.lon, args.half_size_m, args.res_m)
    bll = tuple(bounds)
    layers, meta = {}, {}
    wanted = [s.strip() for s in args.sources.split(',') if s.strip()]

    if 'meta' in wanted:
        try:
            qks = {quadkey(la, lo, 9) for la in (bll[1], bll[3]) for lo in (bll[0], bll[2])}
            if len(qks) != 1:
                raise RuntimeError(f'area spans several Meta tiles {sorted(qks)}; mosaicking not implemented')
            qk = qks.pop()
            a, tr, crs = read_crop(f'{META_BASE}/chm/{qk}.tif', bll, raw_path=raw(f'meta_chm_{qk}_crop.tif'),
                                   mask_url=f'{META_BASE}/msk/{qk}.tif.msk')
            layers['meta'] = warp(a, tr, crs, dst_tr, shape, Resampling.bilinear)
            meta['meta'] = {'source': META_SRC, 'attribution': META_ATTR, 'licence': LICENCE,
                            'url': f'{META_BASE}/chm/{qk}.tif', 'quadkey_z9': qk,
                            'native_res': '1.194 m web-mercator (~0.96 m ground here)', 'resampling': 'bilinear'}
            try:  # imagery epoch; raster value is a decimal year offset (e.g. 18.9 -> late 2018), unverified
                with rasterio.open(f'/vsicurl/{META_BASE}/CHM_acquisition_date.tif') as d:
                    meta['meta']['acquisition_date_raw'] = float(next(d.sample([(args.lon, args.lat)]))[0])
            except Exception as exc:  # noqa: BLE001
                print(f'meta acquisition date unavailable: {exc}', file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            print(f'meta failed: {exc}', file=sys.stderr)

    if 'eth' in wanted:
        try:
            tile = f'N{int(math.floor(args.lat / 3) * 3):02d}E{int(math.floor(args.lon / 3) * 3):03d}'
            for kind, key in (('Map', 'eth'), ('Map_SD', 'eth_sd')):
                url = ETH_BASE.format(tile=tile, kind=kind)
                a, tr, crs = read_crop(url, bll, raw_path=raw(f'eth_{tile}_{kind}_crop.tif'))
                layers[key] = warp(a, tr, crs, dst_tr, shape, Resampling.nearest)
            meta['eth'] = {'source': ETH_SRC, 'attribution': ETH_ATTR, 'licence': LICENCE,
                           'url': ETH_BASE.format(tile=tile, kind='Map'), 'tile': tile,
                           'native_res': '1/12000 deg (~7.5 x 9.3 m here)', 'resampling': 'nearest'}
        except Exception as exc:  # noqa: BLE001
            print(f'eth failed: {exc}', file=sys.stderr)

    primary = next((s for s in wanted if s in layers), None)
    if primary is None:
        sys.exit('no canopy source could be read')
    summary = {'name': args.name, 'centre_lat': args.lat, 'centre_lon': args.lon, 'half_size_m': args.half_size_m,
               'res_m': args.res_m, 'shape': list(shape), 'bounds': [round(float(v), 8) for v in bounds],
               'primary': primary, 'sources': meta,
               'note': 'canopy height above bare ground (not AMSL); add to GSI dem5a ground for surface height',
               'stats': {k: summarise(v, args.res_m) for k, v in layers.items() if k != 'eth_sd'}}
    if 'eth_sd' in layers:
        summary['stats']['eth_sd_pm150m_median'] = stats(
            layers['eth_sd'][shape[0] // 2 - 150:shape[0] // 2 + 150, shape[1] // 2 - 150:shape[1] // 2 + 150]
        )['median']
    sj = json.dumps(summary, indent=1, ensure_ascii=False)
    nan = np.full(shape, np.nan, np.float32)
    np.savez_compressed(os.path.join(out_dir, 'canopy.npz'), height_m=layers[primary].astype(np.float32),
                        lat=lat, lon=lon, bounds=bounds, source=np.array(meta[primary]['source']),
                        attribution=np.array(meta[primary]['attribution']), licence=np.array(LICENCE),
                        eth_height_m=layers.get('eth', nan), eth_sd_m=layers.get('eth_sd', nan),
                        summary_json=np.array(sj))
    with open(os.path.join(out_dir, 'canopy.json'), 'w') as f:
        f.write(sj + '\n')
    names = {'meta': 'Meta/WRI CHM ~1 m', 'eth': 'ETH 2020 CHM 10 m'}
    preview(os.path.join(out_dir, 'canopy_preview.png'), [(names[k], layers[k]) for k in ('meta', 'eth') if k in layers],
            args.lat, args.lon, args.half_size_m, f'{args.name} canopy height')
    print(sj)


if __name__ == '__main__':
    main()
