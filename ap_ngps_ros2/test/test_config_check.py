#!/usr/bin/env python3
"""Config rules that would have stopped flight 2 (no NGPS output) and the HFOV mismatch."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ap_ngps_ros2 import camera, config_check as cc  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / 'config'
LIVE = ['fusion_nagano0.yaml', 'ngps_config_nagano0.yaml', 'ngps_config_nagano_gsi2018.yaml', 'ngps_config.yaml']


def errors(findings):
    return [t for lvl, t in findings if lvl == cc.ERROR]


@pytest.mark.parametrize('name', LIVE)
def test_shipped_configs_pass(name):
    assert errors(cc.check_file(str(CONFIG / name))) == []


GOOD_NGPS = {'publish_ap_dds_tf': True, 'altitude_topic': '/ap/pose/filtered', 'altitude_source': 'pose'}


def test_flight2_config_is_rejected():
    bad = dict(GOOD_NGPS, publish_ap_dds_tf=False, altitude_topic='/mavros/global_position/rel_alt',
               altitude_source='float64')
    errs = ' | '.join(errors(cc.check_params('ngps_localization_node', bad)))
    assert 'MAVROS' in errs and 'publish_ap_dds_tf' in errs and 'altitude_source' in errs


def test_publish_ap_dds_tf_defaults_to_off_so_must_be_set():
    params = dict(GOOD_NGPS)
    del params['publish_ap_dds_tf']
    assert any('publish_ap_dds_tf' in e for e in errors(cc.check_params('ngps_localization_node', params)))


@pytest.mark.parametrize('source', ['geopose', 'navsat'])
def test_amsl_altitude_is_rejected(source):
    params = dict(GOOD_NGPS, altitude_topic='/ap/geopose/filtered', altitude_source=source)
    assert any('AMSL' in e for e in errors(cc.check_params('ngps_localization_node', params)))


def test_mavros_anywhere_in_fusion_config_is_rejected():
    errs = errors(cc.check_params('vo_ngps_fusion_node', {'pose_topic': '/mavros/local_position/pose'}))
    assert any('MAVROS' in e for e in errs)


@pytest.mark.parametrize('node,key', [('ngps_localization_node', 'kernel_hfov_deg'),
                                      ('vo_ngps_fusion_node', 'hfov_deg')])
def test_hfov_must_match_measured_camera(node, key):
    base = GOOD_NGPS if node == 'ngps_localization_node' else {}
    assert errors(cc.check_params(node, dict(base, **{key: camera.HFOV_DEG}))) == []
    assert any(key in e for e in errors(cc.check_params(node, dict(base, **{key: 71.8}))))


def test_slow_keepalive_is_rejected():
    assert errors(cc.check_params('vo_ngps_fusion_node', {'tf_rate_hz': 2.0}))


def test_wrong_map_scale_is_rejected():
    params = {'metadata_path': str(CONFIG.parents[1] / 'ngps_config/tiff/nagano0/metadata.json'),
              'mosaic_meters_per_pixel': 0.48112}
    if not Path(params['metadata_path']).exists():
        pytest.skip('map not checked out')
    assert any('mosaic_meters_per_pixel' in e for e in errors(cc.check_params('vo_ngps_fusion_node', params)))


@pytest.mark.parametrize('backend,ok', [('opencv', True), ('vpi_cuda', True), ('cuda', False), ('VPI', False)])
def test_vo_backend_must_be_known(backend, ok):
    errs = errors(cc.check_params('vo_ngps_fusion_node', {'vo_backend': backend}))
    assert (not any('vo_backend' in e for e in errs)) == ok


@pytest.mark.parametrize('fpl,ok', [(500, True), (1000, True), (50, False), (5000, False)])
def test_vpi_features_per_level_range(fpl, ok):
    errs = errors(cc.check_params('vo_ngps_fusion_node', {'vpi_features_per_level': fpl}))
    assert (not any('vpi_features_per_level' in e for e in errs)) == ok


@pytest.mark.parametrize('params,ok', [
    ({}, True), ({'camera_source': 'topic'}, True), ({'camera_source': 'rtsp'}, True),
    ({'camera_source': 'direct'}, False), ({'camera_source': 'rtsp', 'rtsp_url': 'http://x'}, False),
    ({'camera_source': 'rtsp', 'rtsp_codec': 'mjpeg'}, False)])
def test_camera_source_settings(params, ok):
    errs = errors(cc.check_params('vo_ngps_fusion_node', params))
    assert (not any(k in e for e in errs for k in ('camera_source', 'rtsp_'))) == ok


@pytest.mark.parametrize('offset,ok', [([-2.35, 1.19], True), ([0.0, 0.0], True), ([30.0, 0.0], False), ([1.0], False)])
def test_map_offset_must_be_plausible(tmp_path, offset, ok):
    meta = tmp_path / 'metadata.json'
    meta.write_text(__import__('json').dumps({'bounds': '138.63,36.32,138.64,36.33', 'offset_en_m': offset}))
    errs = errors(cc.check_params('vo_ngps_fusion_node', {'metadata_path': str(meta)}))
    assert (not any('offset_en_m' in e for e in errs)) == ok


@pytest.mark.parametrize('pm,ok', [(0.0, True), (200.0, True), (400.0, True), (-10.0, False), (900.0, False)])
def test_predict_ms_range(pm, ok):
    errs = errors(cc.check_params('vo_ngps_fusion_node', {'predict_ms': pm}))
    assert (not any('predict_ms' in e for e in errs)) == ok
