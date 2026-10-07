"""flight_ui reads flight_log.sh's check table; pin the format it depends on."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import flight_ui  # noqa: E402

SAMPLE = """link: dds   listening for 10 s ...

  ok  /camera/image_raw/compressed    GO       29.0 Hz   camera frames
  ok  /ap/time                        GO       81.9 Hz   aligns bag with dataflash log
  ok  /ap/tf rate                     GO       10.0 Hz   max gap  105 ms (VisOdom times out at 300 ms)
  !!  flight controller               NO-GO  no /ap/* topics from the FC (see hint below)
  ok  disk                            GO     25.4 GB free  (~0.6 h at 11.3 MB/s)

  hint: DDS_ENABLE=1 ...

  NO-GO  (1 blocking, 0 warnings)
"""


def test_parse_rows_and_result():
    rows, result = flight_ui.parse_check(SAMPLE)
    assert result == 'NO-GO'
    assert [r['item'] for r in rows] == ['/camera/image_raw/compressed', '/ap/time', '/ap/tf rate',
                                         'flight controller', 'disk']
    assert rows[3]['status'] == 'NO-GO' and rows[0]['status'] == 'GO'


def test_fc_reboot_prompt_only_when_fc_missing():
    rows, _ = flight_ui.parse_check(SAMPLE)
    assert flight_ui.fc_prompt(rows)
    ok_rows, res = flight_ui.parse_check(SAMPLE.replace('  !!  flight controller               NO-GO  no /ap/* topics from the FC (see hint below)\n', '').replace('NO-GO  (1 blocking, 0 warnings)', 'GO'))
    assert res == 'GO' and flight_ui.fc_prompt(ok_rows) is None
