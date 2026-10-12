"""Exercise the browser log poller in Node without importing the ROS GUI module."""
import re
import subprocess
from pathlib import Path

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py'


def js_function(source, name):
    match = re.search(rf'    (?:async )?function {name}\s*\(', source)
    assert match, f'missing JavaScript function {name}'
    start = source.index('{', match.start())
    depth = 0
    for pos in range(start, len(source)):
        if source[pos] == '{':
            depth += 1
        elif source[pos] == '}':
            depth -= 1
            if depth == 0:
                return source[match.start():pos + 1].strip()
    raise AssertionError(f'unclosed JavaScript function {name}')


@pytest.fixture(scope='module')
def js_runtime():
    source = SOURCE.read_text(encoding='utf-8')
    declarations = re.search(r'    let lastLogSeq = 0;\s*let logsAutoFollow = true;\s*let logsSelecting = false;\s*let logsPollInFlight = false;\s*const MAX_LOG_TEXT_LENGTH = 200000;', source)
    assert declarations
    functions = '\n'.join(js_function(source, name) for name in
                          ('appendLogText', 'appendLog', 'logsNearBottom',
                           'logsHasSelection', 'shouldAutoFollowLogs',
                           'followLogsIfAllowed', 'pollLogs'))
    return declarations.group(0).strip() + '\n' + functions + '\n' + r'''
      globalThis.configure = options => {
        lastLogSeq = options.lastLogSeq || 0;
        logsAutoFollow = true;
        logsSelecting = false;
        logsPollInFlight = false;
        globalThis.calls = 0;
        globalThis.followCalls = 0;
        globalThis.logs = {scrollHeight: 0, scrollTop: 0, clientHeight: 100,
          _text: '', writes: 0,
          get textContent() { return this._text; },
          set textContent(value) { this.writes++; this._text = value; }};
        globalThis.$ = id => logs;
        globalThis.shouldAutoFollowLogs = () => false;
        globalThis.followLogsIfAllowed = () => { followCalls++; };
        globalThis.api = options.api;
      };
      globalThis.state = () => ({text: logs.textContent, writes: logs.writes,
        lastLogSeq, inFlight: logsPollInFlight, calls, followCalls});
    '''


def run_js(js_runtime, body):
    script = js_runtime + '\n' + body
    result = subprocess.run(['node', '-e', script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_delayed_poll_guard_and_duplicate_sequence_batch_append(js_runtime):
    run_js(js_runtime, r'''
      (async () => {
        let resolve;
        configure({api: () => { calls++; return new Promise(r => { resolve = r; }); }});
        const first = pollLogs();
        const second = pollLogs();
        await second;
        if (calls !== 1) throw new Error(`expected one API call, got ${calls}`);
        resolve({entries: [
          {seq: 1, time: 't1', source: 'nav', message: 'one'},
          {seq: 1, time: 't1', source: 'nav', message: 'duplicate'},
          {seq: 2, time: 't2', source: 'http', message: 'hidden'},
          {seq: 3, time: 't3', source: 'nav', message: 'three'}
        ]});
        await first;
        const s = state();
        if (s.text !== '[t1] [nav] one\n[t3] [nav] three\n') throw new Error(s.text);
        if (s.writes !== 1) throw new Error(`expected one DOM write, got ${s.writes}`);
        if (s.lastLogSeq !== 3 || s.inFlight) throw new Error(JSON.stringify(s));
      })().catch(e => { console.error(e); process.exitCode = 1; });
    ''')


def test_log_limit_and_append_log_use_bounded_helper(js_runtime):
    run_js(js_runtime, r'''
      configure({api: async () => ({entries: []})});
      appendLogText(logs, 'old line\n' + 'x'.repeat(199990));
      appendLogText(logs, 'new line\n');
      if (logs.textContent.length > MAX_LOG_TEXT_LENGTH) throw new Error('log cap exceeded');
      if (logs.textContent.includes('old line')) throw new Error('old complete line was retained');
      const before = logs.writes;
      appendLog('web', 'direct append');
      if (logs.writes !== before + 1) throw new Error('appendLog did not use one bounded write');
      if (logs.textContent.length > MAX_LOG_TEXT_LENGTH) throw new Error('appendLog exceeded cap');
    ''')


def test_failed_poll_releases_guard_for_next_attempt(js_runtime):
    run_js(js_runtime, r'''
      (async () => {
        configure({api: async () => { calls++; if (calls === 1) throw new Error('offline'); return {entries: []}; }});
        try { await pollLogs(); } catch (_) {}
        if (state().inFlight) throw new Error('guard remained set after failure');
        await pollLogs();
        if (calls !== 2 || state().inFlight) throw new Error('retry did not run cleanly');
      })().catch(e => { console.error(e); process.exitCode = 1; });
    ''')
