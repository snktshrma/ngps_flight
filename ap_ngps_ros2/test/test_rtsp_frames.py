"""The direct camera pipeline must keep every latency fix the recorder's camera node has."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
rf = pytest.importorskip('ap_ngps_ros2.rtsp_frames')


def test_pipeline_is_low_latency_and_hardware_scaled():
    d = rf.build_description('rtsp://192.168.144.25:8554/video1')
    for must in ('latency=0', 'drop-on-latency=true', 'do-retransmission=false', 'rtph264depay name=depay',
                 'nvv4l2decoder disable-dpb=true', 'leaky=downstream', 'nvvidconv',
                 'format=BGRx,width=640,height=360', 'max-buffers=1 drop=true sync=false'):
        assert must in d, must
    # the leaky queue must sit between the decoder and the scaler
    assert d.index('nvv4l2decoder') < d.index('leaky=downstream') < d.index('nvvidconv')
    assert 'jpeg' not in d


def test_h265_and_tcp_options():
    d = rf.build_description('rtsp://h/x', codec='h265', protocols='tcp')
    assert 'rtph265depay name=depay ! h265parse' in d and 'protocols=tcp' in d
