#!/usr/bin/env python3
"""Local-only web front end for the protected manual ONNX test runner."""
import argparse
import fcntl
import json
import math
import os
import pty
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


WORKSPACE = Path('/home/unitree/h1_nav2_ros2_ws')
RUNNER = Path(__file__).with_name('run_acceptance.py')
MAX_LOG_LINES = 120


def validate_trial(payload):
    if not isinstance(payload, dict):
        raise ValueError('请求格式错误')
    motion = payload.get('motion')
    if motion not in ('forward', 'backward', 'turn_left', 'turn_right'):
        raise ValueError('运动方向无效')
    try:
        speed = float(payload.get('speed'))
        duration = float(payload.get('duration'))
    except (TypeError, ValueError):
        raise ValueError('速度和时长必须是数字')
    limit = 0.1 if motion.startswith('turn_') else 0.12
    if not math.isfinite(speed) or not 0.01 <= speed <= limit:
        raise ValueError(f'速度必须在 0.01 到 {limit:g} 之间')
    if not math.isfinite(duration) or not 0.5 <= duration <= 3.0:
        raise ValueError('单次时长必须在 0.5 到 3 秒之间')
    confirmations = payload.get('confirmations')
    if not isinstance(confirmations, dict) or any(confirmations.get(key) is not True for key in (
        'debug_mode', 'exclusive_control', 'supported_upright'
    )):
        raise ValueError('请先完成并勾选全部现场确认')
    return motion, speed, duration


class TrialController:
    def __init__(self, workspace=WORKSPACE, runner=RUNNER):
        self.workspace = Path(workspace)
        self.runner = Path(runner)
        self.lock = threading.RLock()
        self.process = None
        self.master_fd = None
        self.state = 'idle'
        self.log = []
        self.trial = None
        self.exit_code = None

    def status(self):
        with self.lock:
            if self.process is not None and self.process.poll() is not None and self.state not in ('complete', 'failed'):
                self.exit_code = self.process.returncode
                self.state = 'complete' if self.exit_code == 0 else 'failed'
            return {'state': self.state, 'log': list(self.log[-MAX_LOG_LINES:]),
                    'trial': self.trial, 'exit_code': self.exit_code,
                    'active': self.process is not None and self.process.poll() is None}

    def start(self, payload):
        motion, speed, duration = validate_trial(payload)
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise ValueError('当前已有测试会话')
            if not self.runner.is_file():
                raise ValueError('测试 runner 不存在')
            slave_fd = None
            master_fd, slave_fd = pty.openpty()
            command = [sys.executable, str(self.runner), '--execute',
                       '--confirm-debug-mode', '--confirm-exclusive-control',
                       '--continuous-walk-trial', '--joint-command-mode', 'position_pd',
                       '--protection-profile', 'train_20261106', '--confirm-supported-upright',
                       '--manual-motion', motion, '--manual-speed', str(speed),
                       '--manual-pulse-seconds', str(duration), '--brief-summary', '--web-repeat-trials']
            try:
                self.process = subprocess.Popen(command, cwd=self.workspace, stdin=slave_fd,
                                                stdout=slave_fd, stderr=slave_fd,
                                                start_new_session=True, close_fds=True)
            except OSError:
                os.close(master_fd)
                raise
            finally:
                if slave_fd is not None:
                    os.close(slave_fd)
            os.set_blocking(master_fd, False)
            self.master_fd = master_fd
            self.trial = {'motion': motion, 'speed': speed, 'duration': duration,
                          'linear_unit': 'm/s' if motion in ('forward', 'backward') else 'rad/s'}
            self.exit_code = None
            self.log = []
            self.state = 'starting'
            threading.Thread(target=self._read_output, daemon=True).start()
            return self.status()

    def _read_output(self):
        # Capture this session, so an old reader cannot consume a new PTY.
        with self.lock:
            fd, process = self.master_fd, self.process
        pending = ''
        try:
            while True:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    data = b''
                except OSError:
                    break
                if data:
                    pending += data.decode(errors='replace').replace('\r', '')
                    while '\n' in pending:
                        line, pending = pending.split('\n', 1)
                        with self.lock:
                            if self.process is not process:
                                return
                            self._output_line(line)
                if process.poll() is not None:
                    break
                threading.Event().wait(.05)
        finally:
            os.close(fd)
            with self.lock:
                if self.process is process:
                    self.master_fd = None
                    self.status()

    def _output_line(self, line):
        self.log.append(line)
        self.log = self.log[-MAX_LOG_LINES:]
        prefix = 'MANUAL_WEB_EVENT '
        if line.startswith(prefix):
            try:
                event = json.loads(line[len(prefix):])
            except ValueError:
                return
            if event.get('state') in ('ready_next', 'running', 'awaiting_revoke'):
                # A late idle marker cannot reopen a session after revoke.
                if self.state != 'revoking':
                    self.state = event['state']
        elif 'Type CONFIRM to continue:' in line and self.state == 'starting':
            self.state = 'awaiting_confirmation'
        elif ('Manual velocity canceled:' in line or
              'Training guard entered Passive.' in line or
              'Release is blocked; output remains authorized in Hold' in line):
            self.state = 'awaiting_revoke'

    def next_trial(self, payload):
        motion, speed, duration = validate_trial(payload)
        command = 'TRIAL ' + json.dumps({'motion': motion, 'speed': speed, 'duration': duration})
        with self.lock:
            self._send(command, 'ready_next', 'running')
            self.trial = {'motion': motion, 'speed': speed, 'duration': duration,
                          'linear_unit': 'rad/s' if motion.startswith('turn_') else 'm/s'}
            return self.status()

    def confirm(self):
        return self._send('CONFIRM', 'awaiting_confirmation', 'confirmed')

    def revoke(self):
        with self.lock:
            required = 'ready_next' if self.state == 'ready_next' else 'awaiting_revoke'
            return self._send('REVOKE', required, 'revoking')

    def _send(self, value, required_state, next_state):
        with self.lock:
            if self.process is None or self.process.poll() is not None or self.state != required_state:
                raise ValueError('当前阶段不接受此确认')
            os.write(self.master_fd, (value + '\n').encode())
            self.state = next_state
            return self.status()


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ONNX 手动测试</title><style>
:root{font-family:Inter,"Noto Sans SC","Microsoft YaHei",sans-serif;color:#20272e;background:#f2f5f6;font-size:14px;--line:#d6dfe2;--muted:#607078;--teal:#087f78;--red:#b42318;--white:#fff}*{box-sizing:border-box}body{margin:0}header{height:58px;background:#fff;border-bottom:1px solid var(--line);display:flex;align-items:center;padding:0 22px;gap:12px}header strong{font-size:17px}header span{color:var(--muted)}main{max-width:860px;margin:24px auto;padding:0 16px;display:grid;gap:16px}.section{background:#fff;border:1px solid var(--line);border-radius:6px;padding:18px}.section h2{font-size:16px;margin:0 0 16px}.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.motion{height:54px;border:1px solid var(--line);border-radius:5px;background:#fff;color:#20272e;cursor:pointer}.motion[aria-pressed=true]{border:2px solid var(--teal);background:#effaf8;color:#075e59;font-weight:650}.row{display:grid;grid-template-columns:150px 1fr 90px;align-items:center;gap:16px;margin:18px 0}.row label{font-weight:600}.row output{text-align:right;font-variant-numeric:tabular-nums}.row input{width:100%;accent-color:var(--teal)}.checks{display:grid;gap:10px}.checks label{display:flex;gap:9px;align-items:flex-start;line-height:1.45}.checks input{margin-top:3px;accent-color:var(--teal)}.notice{border-left:3px solid #d17b00;background:#fff9eb;padding:10px 12px;color:#594516;line-height:1.5;margin-top:14px}.actions{display:flex;gap:10px;margin-top:18px;flex-wrap:wrap}button.action{min-height:42px;border:0;border-radius:5px;padding:0 16px;background:var(--teal);color:#fff;font-weight:650;cursor:pointer}button.action.secondary{background:#e7eff0;color:#25373a}button.action.danger{background:var(--red)}button:disabled{opacity:.45;cursor:not-allowed}.status{display:flex;align-items:center;gap:10px}.dot{width:9px;height:9px;border-radius:50%;background:#809098}.dot.live{background:#07866e}.dot.wait{background:#d17b00}.details{color:var(--muted);margin-top:7px}.log{height:180px;overflow:auto;background:#172126;color:#e1ecec;border-radius:4px;padding:12px;font:12px/1.5 ui-monospace,monospace;white-space:pre-wrap;word-break:break-word}.small{font-size:12px;color:var(--muted);line-height:1.5}.hidden{display:none}@media(max-width:620px){main{margin:12px auto}.section{padding:14px}.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.row{grid-template-columns:1fr 70px;gap:8px}.row input{grid-column:1/-1;grid-row:2}.row output{grid-column:2;grid-row:1}}
</style></head><body><header><strong>ONNX 手动测试</strong><span>本机测试会话</span></header><main>
<section class="section"><h2>运动参数</h2><div class="grid" id="motions"><button class="motion" data-motion="forward" aria-pressed="true">前进</button><button class="motion" data-motion="backward" aria-pressed="false">后退</button><button class="motion" data-motion="turn_left" aria-pressed="false">左转</button><button class="motion" data-motion="turn_right" aria-pressed="false">右转</button></div>
<div class="row"><label for="speed">速度</label><input id="speed" type="range" min="0.01" max="0.12" step="0.01" value="0.12"><output id="speedValue">0.12 m/s</output></div>
<div class="row"><label for="duration">持续时间</label><input id="duration" type="range" min="0.5" max="3" step="0.5" value="0.5"><output id="durationValue">0.5 s</output></div>
<div class="notice">每次动作结束自动归零并保持 ArmHold，可修改参数后执行下一次。结束整场测试时才撤权。前进与左转有历史实测记录；后退与右转尚未现场验证。速度上限为线速度 0.12 m/s、角速度 0.1 rad/s，时长 0.5–3 秒。</div></section>
<section class="section"><h2>现场准入确认</h2><div class="checks"><label><input id="debug" type="checkbox">已按设备流程确认调试模式</label><label><input id="exclusive" type="checkbox">已确认当前为独占控制，其他控制发布者不会同时发令</label><label><input id="support" type="checkbox">机器人姿态、支撑和急停条件已现场确认，操作者能持续观察机器人</label></div><p class="small">这些确认由现场操作者负责。软件仍会执行状态、CRC、速度、租约和策略输出检查；无法从网页证明机械支撑或控制权。</p><div class="actions"><button class="action" id="start">准备测试</button></div></section>
<section class="section"><h2>测试状态</h2><div class="status"><i class="dot" id="dot"></i><strong id="state">尚未开始</strong></div><div class="details" id="details">准备测试后，runner 会先完成策略接管和姿态阶段，再等待现场确认。</div><div class="actions"><button class="action secondary hidden" id="confirm">确认姿态并继续</button><button class="action hidden" id="next">执行下一次</button><button class="action danger hidden" id="revoke">结束测试：已支撑，确认撤权</button></div></section>
<section class="section"><h2>运行记录</h2><div class="log" id="log">等待测试。</div></section></main>
<script>
const $=id=>document.getElementById(id);let motion="forward",state="idle",limit=.12;
function updateRange(){const turn=motion.startsWith("turn_");limit=turn?.1:.12;$("speed").max=limit;$("speed").value=Math.min(Number($("speed").value),limit);$("speedValue").textContent=Number($("speed").value).toFixed(2)+(turn?" rad/s":" m/s");}
document.querySelectorAll(".motion").forEach(button=>button.addEventListener("click",()=>{motion=button.dataset.motion;document.querySelectorAll(".motion").forEach(item=>item.setAttribute("aria-pressed",String(item===button)));updateRange();}));
$("speed").addEventListener("input",updateRange);$("duration").addEventListener("input",()=>$("durationValue").textContent=Number($("duration").value).toFixed(1)+" s");
async function post(path,data={}){const r=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(data)});const j=await r.json();if(!r.ok||!j.success)throw Error(j.message||"请求失败");return j;}
$("start").onclick=async()=>{try{$("start").disabled=true;await post("/api/start",{motion,speed:Number($("speed").value),duration:Number($("duration").value),confirmations:{debug_mode:$("debug").checked,exclusive_control:$("exclusive").checked,supported_upright:$("support").checked}});await refresh();}catch(e){alert(e.message);$("start").disabled=false;}};
$("confirm").onclick=async()=>{try{await post("/api/confirm");await refresh();}catch(e){alert(e.message);}};$("revoke").onclick=async()=>{try{await post("/api/revoke");await refresh();}catch(e){alert(e.message);}};
$("next").onclick=async()=>{try{$("next").disabled=true;await post("/api/next",{motion,speed:Number($("speed").value),duration:Number($("duration").value),confirmations:{debug_mode:$("debug").checked,exclusive_control:$("exclusive").checked,supported_upright:$("support").checked}});await refresh();}catch(e){alert(e.message);}finally{$("next").disabled=false;}};
const labels={ready_next:"零速 ArmHold，可执行下一次",running:"正在执行单次动作",idle:"尚未开始",starting:"启动与准备中",awaiting_confirmation:"等待姿态确认",confirmed:"正在执行单次速度测试",awaiting_revoke:"归零保持中，等待支撑交接",revoking:"正在确认撤权",complete:"测试流程结束",failed:"测试流程异常退出"};
async function refresh(){try{const r=await fetch("/api/status");const d=await r.json();state=d.state;$("state").textContent=labels[state]||state;$("dot").className="dot "+(state.startsWith("awaiting")?"wait":d.active?"live":"");$("confirm").classList.toggle("hidden",state!=="awaiting_confirmation");$("revoke").classList.toggle("hidden",!["awaiting_revoke","ready_next"].includes(state));$("next").classList.toggle("hidden",state!=="ready_next");$("start").disabled=d.active;$("details").textContent=d.trial?`${d.trial.motion} · ${d.trial.speed} ${d.trial.linear_unit} · ${d.trial.duration} s · exit=${d.exit_code??"运行中"}`:"准备测试后，runner 会先完成策略接管和姿态阶段，再等待现场确认。";$("log").textContent=(d.log||[]).join("\n")||"等待测试。";$("log").scrollTop=$("log").scrollHeight;}catch(_){} }
updateRange();setInterval(refresh,500);refresh();
</script></body></html>"""


def make_handler(controller):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'PolicyManualTest/1.0'

        def _json(self, value, status=200):
            raw = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _local_request(self):
            host = self.headers.get('Host', '').split(':', 1)[0].strip('[]').lower()
            origin = self.headers.get('Origin')
            return host in ('127.0.0.1', 'localhost', '::1') and (not origin or
                urlparse(origin).hostname in ('127.0.0.1', 'localhost', '::1'))

        def do_GET(self):
            if not self._local_request():
                self.send_error(403)
                return
            if self.path == '/':
                raw = HTML.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            elif self.path == '/api/status':
                self._json(controller.status())
            else:
                self.send_error(404)

        def do_POST(self):
            if not self._local_request():
                self._json({'success': False, 'message': '仅允许本机访问'}, 403)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('请求内容长度无效')
                payload = json.loads(self.rfile.read(length))
                if self.path == '/api/start':
                    result = controller.start(payload)
                elif self.path == '/api/next':
                    result = controller.next_trial(payload)
                elif self.path == '/api/confirm':
                    result = controller.confirm()
                elif self.path == '/api/revoke':
                    result = controller.revoke()
                else:
                    self._json({'success': False, 'message': '接口不存在'}, 404)
                    return
                self._json({'success': True, **result})
            except (ValueError, json.JSONDecodeError) as exc:
                self._json({'success': False, 'message': str(exc)}, 400)

        def log_message(self, _format, *_args):
            return
    return Handler


def main():
    parser = argparse.ArgumentParser(description='Local ONNX manual test web console')
    parser.add_argument('--host', default='127.0.0.1', choices=('127.0.0.1', 'localhost'))
    parser.add_argument('--port', type=int, default=8781)
    args = parser.parse_args()
    controller = TrialController()
    lock_path = Path.home() / '.ros' / 'h1_policy_manual_test_web.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = lock_path.open('w')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another ONNX manual test web server is already running')
    server = ThreadingHTTPServer((args.host, args.port), make_handler(controller))
    print(f'ONNX manual test web: http://{args.host}:{args.port}', flush=True)
    print('For remote browser use an SSH local port forward; the server only binds loopback.', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()


if __name__ == '__main__':
    main()
