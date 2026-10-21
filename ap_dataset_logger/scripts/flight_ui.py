#!/usr/bin/env python3
"""Minimal web UI for flight logging: start, status, stop, FC reboot prompt, log download.

    flight_ui.py [--port 8020]      then open http://<jetson>:8020

A thin wrapper around flight_log.sh (start/check/stop/status) and fc_log_check.download(); it adds no
logic of its own. No authentication: run it only on the drone's private networks.
"""
import argparse
import html
import json
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote, unquote

HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHT_LOG = os.path.join(HERE, 'flight_log.sh')
WS = os.path.expanduser('~/ngps_ws')
# Site map: nagano1 = 36.323136,138.618168 (from 2026-10-07); nagano0 = the first site. --fusion overrides.
FUSION_CFG = os.path.join(WS, 'src/ngps_flight/ap_ngps_ros2/config/fusion_nagano1.yaml')
BAGS = os.path.join(WS, 'src/ngps_flight/bags')
UI_LOG = os.path.join(BAGS, 'flight_ui_action.log')
FC = 'tcp:192.168.144.15:5760'
ROW = re.compile(r'^  (\S+) (.+?)\s{2,}(GO|WARN|NO-GO)\s{2,}(.*)$')

state = {'busy': None, 'last_check': [], 'check_result': None, 'check_time': None,
         'message': '', 'download': None}
lock = threading.Lock()


def sh(args, timeout=None):
    p = subprocess.run([FLIGHT_LOG] + args, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout + p.stderr


def parse_check(text):
    rows = [{'item': m.group(2).strip(), 'status': m.group(3), 'detail': m.group(4).strip()}
            for m in map(ROW.match, text.splitlines()) if m]
    result = None
    for line in reversed(text.splitlines()):
        s = line.strip()
        if s.startswith('NO-GO'):
            result = 'NO-GO'
        elif s.startswith('GO WITH WARNINGS'):
            result = 'WARN'
        elif s == 'GO':
            result = 'GO'
        if result:
            break
    return rows, result


def fc_prompt(rows):
    for r in rows:
        if r['status'] == 'NO-GO' and (r['item'] == 'flight controller' or r['item'].startswith('/ap/time')):
            return 'FC not connected over DDS: reboot the FC'
    return None


def store_check(text):
    rows, result = parse_check(text)
    if rows:
        state.update(last_check=rows, check_result=result, check_time=time.strftime('%H:%M:%S'))


def run_action(name, fn):
    with lock:
        if state['busy']:
            return False
        state['busy'] = name
        state['message'] = ''

    def work():
        try:
            fn()
        except BaseException as exc:      # incl. SystemExit from fc_log_check.download
            state['message'] = f'{name} failed: {exc}'
        finally:
            state['busy'] = None
    threading.Thread(target=work, daemon=True).start()
    return True


def do_start():
    args = ['start', '--fc', 'udp:2019', '--fusion', FUSION_CFG, '--name', 'flight', '--archive',
            '--check-timeout', '300']
    with open(UI_LOG, 'w') as log:
        p = subprocess.Popen([FLIGHT_LOG] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        buf = ''
        for line in p.stdout:          # stream: the check table updates while start re-checks
            log.write(line)
            log.flush()
            buf += line
            if line.strip() in ('GO',) or line.strip().startswith(('NO-GO', 'GO WITH WARNINGS')):
                store_check(buf)
                buf = ''
        p.wait()
    state['message'] = 'started: GO' if p.returncode == 0 else 'start failed or NO-GO: everything stopped'


def do_check():
    _, out = sh(['check'], timeout=120)
    store_check(out)


def do_stop():
    _, out = sh(['stop'], timeout=180)
    bag = re.search(r'bag: (\S+)\s+\((\S+)\)', out)
    clean = 'closed cleanly' in out
    state['message'] = (f'stopped. bag {os.path.basename(bag.group(1).rstrip("/"))} ({bag.group(2)})'
                        f'{"" if clean else " NOT closed cleanly"}') if bag else 'stopped.'
    state.update(last_check=[], check_result=None)


def do_reboot_fc():
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(FC, source_system=240, source_component=190)
    if not m.wait_heartbeat(timeout=15):
        raise RuntimeError('no heartbeat from the FC')
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    while hb is not None and (hb.get_srcComponent() != 1 or hb.type == mavutil.mavlink.MAV_TYPE_GCS):
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if hb is None:
        raise RuntimeError('no autopilot heartbeat')
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError('FC is ARMED: not rebooting')
    m.mav.command_long_send(m.target_system, m.target_component, 246, 0, 1, 0, 0, 0, 0, 0, 0)
    state['message'] = 'FC reboot sent; press Check in ~30 s'


def do_download():
    sys.path.insert(0, HERE)
    import fc_log_check
    state['download'] = 'waiting for FC (must be disarmed)...'
    path = fc_log_check.download('flight')
    state['download'] = None
    state['message'] = f'log saved: {os.path.basename(path)}'


def running():
    try:
        _, out = sh(['status'], timeout=20)
    except subprocess.TimeoutExpired:
        return None, 'status timed out'
    lines = out.splitlines()
    container = lines[0].replace('container: ', '') if lines else '?'
    return any(l.strip() == 'running' for l in lines), container


def logs():
    try:
        files = [f for f in os.listdir(BAGS) if f.endswith('.BIN') and not f.startswith('.')]
    except OSError:
        return []
    files.sort(key=lambda f: os.path.getmtime(os.path.join(BAGS, f)), reverse=True)
    return [{'name': f, 'mb': round(os.path.getsize(os.path.join(BAGS, f)) / 1e6, 1),
             'time': time.strftime('%m-%d %H:%M', time.localtime(os.path.getmtime(os.path.join(BAGS, f))))}
            for f in files[:8]]


def status():
    run, container = running()
    st = os.statvfs(BAGS)
    rows = state['last_check']
    return {'running': run, 'container': container, 'busy': state['busy'], 'message': state['message'],
            'check': rows, 'check_result': state['check_result'], 'check_time': state['check_time'],
            'prompt': fc_prompt(rows), 'download': state['download'],
            'disk_gb': round(st.f_bavail * st.f_frsize / 1e9, 1), 'logs': logs()}


ACTIONS = {'start': do_start, 'check': do_check, 'stop': do_stop, 'reboot_fc': do_reboot_fc,
           'download': do_download}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='application/json'):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/':
            return self._send(200, PAGE.replace('{{SITE}}', html.escape(os.path.basename(FUSION_CFG))),
                              'text/html; charset=utf-8')
        if self.path == '/api/status':
            return self._send(200, json.dumps(status()))
        if self.path.startswith('/logs/'):
            name = os.path.basename(unquote(self.path[6:]))
            path = os.path.join(BAGS, name)
            if not name.endswith('.BIN') or not os.path.isfile(path):
                return self._send(404, '{"error": "not found"}')
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Disposition', f'attachment; filename="{quote(name)}"')
            self.send_header('Content-Length', str(os.path.getsize(path)))
            self.end_headers()
            with open(path, 'rb') as f:
                while chunk := f.read(1 << 20):
                    self.wfile.write(chunk)
            return
        self._send(404, '{"error": "not found"}')

    def do_POST(self):
        name = self.path.rsplit('/', 1)[-1]
        if not self.path.startswith('/api/') or name not in ACTIONS:
            return self._send(404, '{"error": "not found"}')
        ok = run_action(name, ACTIONS[name])
        self._send(200 if ok else 409, json.dumps({'ok': ok, 'busy': state['busy']}))

    def log_message(self, *args):
        pass


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Flight Logger</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#16181d;--muted:#667085;--line:#e4e7ec;--go:#12805c;--warn:#b54708;--bad:#c01048;--btn:#1f6fd1}
@media (prefers-color-scheme:dark){:root{--bg:#0f1115;--card:#181b21;--ink:#e8eaee;--muted:#98a2b3;--line:#2a2f38;--go:#3ccb7f;--warn:#f79009;--bad:#f63d68;--btn:#4e8ff0}}
*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,sans-serif}
main{max-width:640px;margin:0 auto;padding:16px}
h1{font-size:18px;margin:4px 0 14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.pill{display:inline-block;padding:6px 14px;border-radius:999px;font-weight:700;font-size:16px;color:#fff;background:var(--muted)}
.GO{background:var(--go)}.WARN{background:var(--warn)}.NO-GO{background:var(--bad)}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
button{flex:1;min-width:120px;padding:12px;border:0;border-radius:10px;font-size:15px;font-weight:600;color:#fff;background:var(--btn);cursor:pointer}
button.stop{background:var(--bad)}button.sec{background:var(--muted)}button:disabled{opacity:.4;cursor:default}
.banner{border-left:4px solid var(--warn);padding:10px 12px;border-radius:8px;background:var(--card);margin-bottom:12px;display:flex;gap:10px;align-items:center;justify-content:space-between}
.muted{color:var(--muted);font-size:13px}
table{width:100%;border-collapse:collapse;font-size:13px}td{padding:5px 4px;border-top:1px solid var(--line);vertical-align:top}
td.s{font-weight:700;white-space:nowrap}td.s.GO{color:var(--go);background:none}td.s.WARN{color:var(--warn);background:none}td.s.NO-GO{color:var(--bad);background:none}
details summary{cursor:pointer;font-weight:600}
a{color:var(--btn)}ul{list-style:none;padding:0;margin:8px 0 0}li{padding:6px 0;border-top:1px solid var(--line);display:flex;justify-content:space-between;gap:8px}
li a{word-break:break-all}li .muted{white-space:nowrap}
</style></head><body><main>
<h1>Flight Logger</h1>
<div class="muted">config: {{SITE}}</div>
<div id="prompt" class="banner" hidden><span id="ptext"></span><button id="breboot" class="stop" style="flex:0 0 auto">Reboot FC</button></div>
<div class="card">
  <span id="pill" class="pill">…</span> <span id="sub" class="muted"></span>
  <div id="msg" class="muted" style="margin-top:8px"></div>
  <div class="row">
    <button id="bstart">Start</button>
    <button id="bcheck" class="sec">Check</button>
    <button id="bstop" class="stop">Stop</button>
  </div>
</div>
<div class="card"><details id="det"><summary>Details <span id="ctime" class="muted"></span></summary>
  <table id="tbl"></table></details></div>
<div class="card"><b>Flight controller log</b>
  <div class="row"><button id="bdl" class="sec">Download latest log</button></div>
  <div id="dl" class="muted" style="margin-top:8px"></div><ul id="logs"></ul></div>
<div class="muted" id="foot"></div>
</main><script>
const $=id=>document.getElementById(id);
async function act(n,confirmText){if(confirmText&&!confirm(confirmText))return;
  const r=await fetch('/api/'+n,{method:'POST'});if(r.status==409)alert('Busy: wait for the current action');refresh();}
$('bstart').onclick=()=>act('start');$('bcheck').onclick=()=>act('check');
$('bstop').onclick=()=>act('stop','Stop logging and close the bag?');
$('breboot').onclick=()=>act('reboot_fc','Reboot the flight controller? (only if disarmed)');
$('bdl').onclick=()=>act('download');
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
async function refresh(){let s;try{s=await(await fetch('/api/status')).json()}catch(e){$('pill').textContent='OFFLINE';$('pill').className='pill NO-GO';return}
  const busy=s.busy, res=s.check_result;
  let label='STOPPED',cls='';
  if(busy==='start'){label='STARTING…'}else if(s.running){label=res==='GO'?'ALL OK':res==='WARN'?'OK (warnings)':res==='NO-GO'?'NOT OK':'RUNNING';cls=res||''}
  $('pill').textContent=label;$('pill').className='pill '+cls;
  $('sub').textContent=(busy?busy+'…  ':'')+'disk '+s.disk_gb+' GB';
  $('msg').textContent=s.message||'';
  $('bstart').disabled=!!busy||s.running;$('bstop').disabled=!!busy||!s.running;$('bcheck').disabled=!!busy||!s.running;$('bdl').disabled=!!busy;
  $('prompt').hidden=!s.prompt;$('ptext').textContent=s.prompt||'';$('breboot').disabled=!!busy;
  if(s.check_result==='NO-GO'&&!$('det').open)$('det').open=true;
  $('ctime').textContent=s.check_time?'· checked '+s.check_time:'';
  $('tbl').innerHTML=s.check.map(r=>`<tr><td class="s ${r.status}">${r.status}</td><td>${esc(r.item)}</td><td class="muted">${esc(r.detail)}</td></tr>`).join('')||'<tr><td class="muted">No check yet</td></tr>';
  $('dl').textContent=s.download||'';
  $('logs').innerHTML=s.logs.map(l=>`<li><a href="/logs/${encodeURIComponent(l.name)}">${esc(l.name)}</a><span class="muted">${l.mb} MB · ${l.time}</span></li>`).join('');
  $('foot').textContent='container '+s.container;}
refresh();setInterval(refresh,2000);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8020)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--fusion', default=FUSION_CFG, help='fusion node config (picks the site map)')
    a = ap.parse_args()
    globals()['FUSION_CFG'] = os.path.abspath(a.fusion)
    print(f'fusion config: {FUSION_CFG}', flush=True)
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f'flight UI on http://{a.host}:{a.port}', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
