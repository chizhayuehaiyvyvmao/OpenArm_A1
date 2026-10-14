import assert from 'node:assert/strict';
import { ScriptControlUI } from '../frontend/src/ui/ScriptControlUI.js';

// Exercise the polling regression using a minimal existing-panel DOM contract.
const output = {textContent:'', scrollTop:0, scrollHeight:0};
const ui = new ScriptControlUI(null, {
    isConnected: () => true,
    socket: {runScript: async () => {throw new Error('测试：目标超出限位');}},
});
ui.panel = {querySelector: selector => selector === '#script-output' ? output : null};
ui.syncOutputPolling = () => {};
ui.scripts = [{file:'unsupported.py', supported:false, reason:'该例程需要终端'}];
ui.selectedScript = 'unsupported.py';
ui.previewName = 'unsupported.py';
ui.preview = 'source preview';
globalThis.fetch = async path => path.endsWith('/log')
    ? {ok:true, text:async()=>''}
    : {json:async()=>({running:false,current_script:null})};
for (let i=0;i<3;i++) await ui.updateOutput();
assert.match(output.textContent,/该例程需要终端/);
assert.match(output.textContent,/source preview/);
ui.scripts = [{file:'supported.py',supported:true}];
ui.selectedScript = 'supported.py';
await ui.runScript();
assert.match(output.textContent,/目标超出限位/);
await ui.updateOutput();
assert.match(output.textContent,/目标超出限位/);
console.log('SCRIPT_UI_ERROR_AND_PREVIEW_POLLING_PASS');
