"""Optional integration smoke test against an already-running local DEMO only."""
import argparse
import json
import threading
import time
import urllib.request

BASE = 'http://127.0.0.1:5001/api/'
TOKEN = ''


def api(path, data=None):
    request = urllib.request.Request(BASE+path, data=None if data is None else json.dumps(data).encode(), headers={'Content-Type':'application/json', 'X-Control-Token':TOKEN})
    with urllib.request.urlopen(request, timeout=8) as response:
        return json.load(response)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:5001')
    parser.add_argument('--script', action='append', default=[])
    args = parser.parse_args()
    BASE = args.url.rstrip('/') + '/api/'
    config = api('config')
    if config['live']:
        raise SystemExit('Refusing: this smoke test only runs in DEMO')
    TOKEN = config['token']
    stop = threading.Event()
    def heartbeat():
        while not stop.wait(.3):
            api('heartbeat',{})
    threading.Thread(target=heartbeat,daemon=True).start()
    try:
        available = api('scripts')['scripts']
        known = {script['name'] for script in available if script['supported']}
        if set(args.script) - known:
            raise ValueError('Unknown or unsupported test script: ' + str(set(args.script) - known))
        for script in available:
            if not script['supported'] or (args.script and script['name'] not in args.script):
                continue
            api('enable',{})
            api('scripts/run',{'name':script['name'], 'side':'both'})
            deadline = time.monotonic()+2.5
            while time.monotonic() < deadline:
                time.sleep(.1)
                info = api('scripts')
                output = '\n'.join(info['output'])
                if 'Traceback' in output or '错误:' in output:
                    raise AssertionError(script['name']+'\n'+output[-2000:])
                if not info['running']:
                    raise AssertionError('Unexpected early exit: '+script['name']+'\n'+output[-1000:])
            api('scripts/stop',{})
            after=api('state')
            assert not after['enabled'] and not after['script_running']
            before=after['positions']
            time.sleep(.15)
            assert api('state')['positions']==before, 'motion continued after stop'
            print('PASS start / telemetry / revoke / stop:',script['name'],flush=True)
        print('DEMO_SCRIPT_SMOKE_PASS',flush=True)
    finally:
        api('stop',{})
        stop.set()
