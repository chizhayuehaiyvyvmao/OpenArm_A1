import assert from 'node:assert/strict';
import { io } from '../frontend/src/robot/OpenarmTransport.js';

// Prevent connection bootstrap: all requests and dialogs below are mocked.
const microtask = globalThis.queueMicrotask;
globalThis.queueMicrotask = () => {};
const transport = io('http://localhost:5002');
globalThis.queueMicrotask = microtask;
transport.bootstrap = {live:true};
transport.config = {arms:['left','right']};
let enabled = false;
let connected = true;
let fault = '';
let accept = true;
let calls = [];
globalThis.window = {confirm: () => {
    calls.push('confirm');
    // Simulate a user taking >2 seconds: existing control expires in the dialog.
    enabled = false;
    return accept;
}};
transport.request = async (path,data) => {
    calls.push(path);
    if (path === 'state') return {connected,enabled,fault};
    if (path === 'enable') { assert.equal(data.confirm,'ENABLE');enabled=true; }
    if (path === 'scripts/run') {
        assert.ok(enabled,'script launched after watchdog revoked control');
        assert.equal(data.side,'both');
        assert.equal(data.confirm,'RUN');
    }
    return {success:true};
};
for (const initiallyEnabled of [false,true]) {
    calls=[];enabled=initiallyEnabled;
    await transport.runScript('3_dual_sin_trajectory_control.py');
    assert.deepEqual(calls,['confirm','state','enable','scripts/run']);
}
calls=[];accept=false;
await assert.rejects(transport.runScript('example.py'),/操作已取消/);
assert.deepEqual(calls,['confirm']);
calls=[];accept=true;fault='反馈故障';
await assert.rejects(transport.runScript('example.py'),/反馈故障/);
assert.deepEqual(calls,['confirm','state']);
console.log('SCRIPT_CONFIRMATION_WATCHDOG_ORDER_PASS');
