/** Original panel event contract over the OpenArm HTTP service. No layout code. */
export function io(url) { return new OpenarmTransport(url); }

class OpenarmTransport {
    constructor(url) {
        this.handlers = new Map();
        this.closed = false;
        this.queue = Promise.resolve();
        this.keys = new Set();
        this.keyBusy = false;
        this.url = url;
        queueMicrotask(() => this.open().catch(error => this.fire('connect_error', error)));
    }
    on(name, callback) {
        if (!this.handlers.has(name)) this.handlers.set(name, new Set());
        this.handlers.get(name).add(callback);
        return this;
    }
    off(name) { this.handlers.delete(name); return this; }
    fire(name, value) { for (const callback of this.handlers.get(name) || []) callback(value); }
    async request(path, data) {
        const response = await fetch('/api/' + path, {
            method: data === undefined ? 'GET' : 'POST',
            headers: {'Content-Type':'application/json', 'X-Control-Token':this.token || ''},
            body: data === undefined ? undefined : JSON.stringify(data),
            signal: AbortSignal.timeout(8000)
        });
        const result = await response.json();
        if (!response.ok || result.success === false) throw new Error(result.error || 'Request failed');
        return result;
    }
    async open() {
        const target = new URL(this.url, window.location.href);
        if (!['localhost','127.0.0.1'].includes(target.hostname)) throw new Error('OpenArm 服务仅允许本机访问');
        this.bootstrap = await this.request('config');
        this.token = this.bootstrap.token;
        let state = await this.request('state');
        if (!state.connected) {
            let side = 'both';
            if (this.bootstrap.live) {
                side = window.prompt('连接并仅读取真机状态。输入 left（左臂）、right（右臂）或 both（双臂）：', 'both');
                if (!['left','right','both'].includes(side)) throw new Error('连接已取消');
            }
            await this.request('connect', {side,confirm:'CONNECT'});
        }
        this.config = await this.request('legacy/config');
        this.fire('connect');
        this.fire('config', this.config);
        this.heartbeat = setInterval(() => this.request('heartbeat',{}).catch(error => this.report(error)),500);
        this.keyboard = setInterval(() => this.processKeys(),100);
        await this.poll();
    }
    report(error) { this.fire('operation_error',error); }
    async ensureEnabled({confirmed = false} = {}) {
        const state = await this.request('state');
        if (!state.connected || state.fault) throw new Error(state.fault || 'Robot disconnected');
        if (!state.enabled) {
            if (this.bootstrap.live && !confirmed && !window.confirm('将启用真机控制并执行所选操作。请确认机械臂周围安全。')) throw new Error('操作已取消');
            await this.request('enable',{confirm:'ENABLE'});
        }
    }
    async poll() {
        if (this.closed) return;
        try {
            const state = await this.request('legacy/state');
            const previous = this.state;
            this.state = state;
            this.fire('robot_state',state);
            if (previous?.control_mode !== state.control_mode) this.fire('mode_changed',{mode:state.control_mode});
            const points = JSON.stringify(state.waypoints);
            if (points !== this.lastPoints) {this.lastPoints=points;this.fire('waypoints_updated',{waypoints:state.waypoints});}
            if (previous?.trajectory_running && !state.trajectory_running) this.fire('trajectory_complete',{});
            if (state.fault && state.fault !== previous?.fault) this.report(new Error(state.fault));
            if (!state.connected) throw new Error(state.fault || 'Robot disconnected');
        } catch(error) {
            this.report(error);
            this.closeLocal();
            return;
        }
        this.pollTimer=setTimeout(() => this.poll(),100);
    }
    emit(event, data={}) {
        if (event === 'key_down') {
            this.keys.add(data.key);
            this.ensureEnabled().catch(error=>{this.keys.delete(data.key);this.report(error);});
            return;
        }
        if (event === 'key_up') {this.keys.delete(data.key);return;}
        if (event === 'command') {
            if (data.action === 'print_pose') {console.log('[OpenArm]',this.state?.positions);return;}
            if (data.action === 'zero_ft') {this.report(new Error('OpenArm 未接入六维力传感器'));return;}
            event = data.action;
        }
        if (event === 'stop' || event === 'stop_trajectory') {
            this.keys.clear();
            this.request('stop',{}).catch(error=>this.report(error));
            return;
        }
        this.queue=this.queue.then(async()=>{
            if (this.closed) return;
            if (!['add_waypoint','clear_waypoints','update_waypoint_duration'].includes(event)) await this.ensureEnabled();
            if (event === 'move_all' && typeof data.gripper === 'object' && data.gripper !== null) {
                data={...data,grippers:data.gripper};delete data.gripper;
            }
            const result=await this.request('legacy/'+event,data);
            if (result.mode) this.fire('mode_changed',{mode:result.mode});
            if (result.waypoints) this.fire('waypoints_updated',{waypoints:result.waypoints});
        }).catch(error=>{this.report(error);if(event==='run_trajectory')this.fire('trajectory_error',{error:error.message});});
    }
    async processKeys() {
        if(this.closed||this.keyBusy||!this.keys.size||!this.state?.enabled||this.state.script_running||this.state.trajectory_running)return;
        this.keyBusy=true;
        try {
            const positions=this.state.target_positions.slice();
            const changes=[['w','s'],['a','d'],['q','e'],['i','k'],['j','l'],['u','o']];
            const joints=this.config.joints.filter(x=>x.kind!=='gripper');
            changes.forEach(([positive,negative],index)=>{
                const delta=(Number(this.keys.has(positive))-Number(this.keys.has(negative)))*.01;
                if(delta)positions[index]=Math.min(joints[index].max,Math.max(joints[index].min,positions[index]+delta));
            });
            const grips={};
            const g=this.config.joints.find(x=>x.kind==='gripper');
            if(g&&(this.keys.has('z')||this.keys.has('x')))grips[g.name]=Math.max(g.min,Math.min(g.max,this.state.gripper_positions[g.name]+(this.keys.has('z')?.03:-.03)));
            await this.request('legacy/move_all',{positions,grippers:grips});
        }catch(error){this.keys.clear();this.report(error);}finally{this.keyBusy=false;}
    }
    async runScript(name) {
        // Native dialogs pause heartbeat timers. Finish confirmation BEFORE
        // enabling, then re-read state in case the watchdog stopped existing control.
        if(this.bootstrap.live&&!window.confirm('将启用真机控制并运行 SDK 脚本，可能连续运动并操作夹爪，确认执行？'))throw new Error('操作已取消');
        await this.ensureEnabled({confirmed: this.bootstrap.live});
        const side=this.config.arms.length===2?'both':this.config.arms[0];
        return this.request('scripts/run',{name,side,confirm:'RUN'});
    }
    async stopScript() {return this.request('scripts/stop',{});}
    closeLocal() {
        this.closed=true;
        clearTimeout(this.pollTimer);clearInterval(this.heartbeat);clearInterval(this.keyboard);
        this.keys.clear();
        this.fire('disconnect','closed');
    }
    disconnect() {
        this.closeLocal();
        return this.request('disconnect',{}).catch(error=>this.report(error));
    }
}
