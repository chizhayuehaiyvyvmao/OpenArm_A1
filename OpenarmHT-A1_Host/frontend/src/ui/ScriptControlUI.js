/**
 * ScriptControlUI - Run SDK example scripts from the web UI
 */
import { robotConnection as robotConnectionInstance } from '../robot/RobotConnection.js';

const SCRIPT_PANEL_OFFSET_X = 100;

export class ScriptControlUI {
    constructor(panelManager = null, robotConnection = robotConnectionInstance) {
        this.panel = null;
        this.panelManager = panelManager;
        this.robotConnection = robotConnection;
        this.scripts = [];
        this.running = false;
        this.currentScript = null;
        this.selectedScript = '';
        this.outputPollTimer = null;
        this.connectionPollTimer = null;
        this.scriptsDir = '';
        this.actionError = '';
        this.preview = '';
        this.previewName = '';
    }

    createUI() {
        this.createPanel();
        this.createTopBarButton();
        this.loadScripts();
        this.startConnectionPolling();
    }

    /**
     * Add a "Scripts" button to the top control bar.
     * Retries if the element isn't ready yet.
     */
    createTopBarButton(attempt = 0) {
        const topBar = document.getElementById('top-control-bar');
        if (!topBar) {
            if (attempt < 20) {
                setTimeout(() => this.createTopBarButton(attempt + 1), 200);
            }
            return;
        }

        // Check if already created
        if (document.getElementById('script-control-btn')) return;

        const divider = document.createElement('div');
        divider.className = 'control-bar-divider';

        this.topBarBtn = document.createElement('button');
        this.topBarBtn.id = 'script-control-btn';
        this.topBarBtn.className = 'tool-button';
        this.topBarBtn.innerHTML = '&#9654; Scripts';
        this.topBarBtn.title = 'Run SDK Example Scripts';
        this.topBarBtn.addEventListener('click', () => this.togglePanel());

        topBar.appendChild(divider);
        topBar.appendChild(this.topBarBtn);
    }

    /**
     * Create the floating script control panel
     */
    createPanel() {
        this.panel = document.createElement('div');
        this.panel.id = 'floating-script-panel';
        this.panel.className = 'floating-panel hidden';
        this.panel.innerHTML = `
            <div class="floating-panel-header">
                <span>SDK Example Scripts</span>
                <button class="panel-close-btn" title="Close">&times;</button>
            </div>
            <div class="floating-panel-content">
                <div class="script-panel-layout">
                    <div class="script-control-pane">
                        <div class="script-status">
                            <span class="status-dot-small disconnected"></span>
                            <span id="script-status-text">Idle</span>
                        </div>

                        <div class="script-form">
                            <div class="script-list-header">
                                <div class="script-list-label" id="script-list-label">Select Script</div>
                                <button id="script-refresh-btn" class="script-refresh-btn" type="button" title="Refresh script list">Refresh</button>
                            </div>
                            <div id="script-list" class="script-list" role="listbox" aria-label="Select Script">
                                <div class="script-list-empty">Loading scripts...</div>
                            </div>
                        </div>

                        <div class="script-buttons">
                            <button id="script-run-btn" class="control-button" disabled>&#9654; Run</button>
                            <button id="script-stop-btn" class="control-button danger" disabled>&#9632; Stop</button>
                        </div>
                    </div>

                    <div class="script-output-pane">
                        <div class="script-section-title">Output</div>
                        <pre id="script-output">No script output.</pre>
                    </div>
                </div>
            </div>
        `;

        document.body.appendChild(this.panel);
        this.bindEvents();
        this.panelManager?.registerPanel('floating-script-panel');
    }

    bindEvents() {
        const closeBtn = this.panel.querySelector('.panel-close-btn');
        closeBtn?.addEventListener('click', () => this.hidePanel());

        const runBtn = this.panel.querySelector('#script-run-btn');
        runBtn?.addEventListener('click', () => this.runScript());

        const stopBtn = this.panel.querySelector('#script-stop-btn');
        stopBtn?.addEventListener('click', () => this.stopScript());

        const refreshBtn = this.panel.querySelector('#script-refresh-btn');
        refreshBtn?.addEventListener('click', () => this.loadScripts());
    }

    /**
     * Load available scripts from backend
     */
    async loadScripts() {
        try {
            const resp = await fetch('/api/scripts', { cache: 'no-store' });
            const data = await resp.json();
            this.scripts = data.scripts || [];
            this.running = data.running || false;
            this.currentScript = data.current_script || null;
            this.scriptsDir = data.scripts_dir || '';
            if (this.currentScript) {
                this.selectedScript = this.currentScript;
            } else if (!this.scripts.some(script => script.file === this.selectedScript)) {
                this.selectedScript = '';
            }

            this.updateScriptList();
            this.updateStatus();
            this.updateOutput();
            this.syncOutputPolling();
        } catch (e) {
            console.warn('[ScriptControlUI] Failed to load scripts:', e);
            // Retry after connection
            setTimeout(() => this.loadScripts(), 2000);
        }
    }

    updateScriptList() {
        const list = this.panel?.querySelector('#script-list');
        if (!list) return;

        const label = this.panel?.querySelector('#script-list-label');
        if (label) {
            label.textContent = this.scriptsDir
                ? `Select Script (${this.scripts.length}) - ${this.scriptsDir}`
                : `Select Script (${this.scripts.length})`;
        }

        list.innerHTML = '';

        if (this.scripts.length === 0) {
            const empty = document.createElement('div');
            empty.className = 'script-list-empty';
            empty.textContent = 'No scripts found';
            list.appendChild(empty);
            this.updateButtons();
            return;
        }

        this.scripts.forEach(script => {
            const item = document.createElement('button');
            item.type = 'button';
            item.className = 'script-list-item';
            item.setAttribute('role', 'option');
            item.setAttribute('data-script', script.file);
            item.setAttribute('aria-selected', (script.file === this.selectedScript).toString());
            item.disabled = this.running;

            const title = document.createElement('span');
            title.className = 'script-list-title';
            title.textContent = script.file + (script.supported ? '' : ' [需终端]');
            item.title = script.supported ? 'Run through OpenArm backend' : 'Source preview only; not adapted for web execution';

            item.appendChild(title);

            item.addEventListener('click', () => {
                if (this.running) return;
                this.selectedScript = script.file;
                this.actionError = '';
                this.preview = '';
                this.previewName = script.file;
                this.updateScriptList();
                if (!script.supported) {
                    fetch('/api/scripts/source?name=' + encodeURIComponent(script.file)).then(r => r.json()).then(data => {
                        if (this.selectedScript !== script.file) return;
                        this.preview = data.source || data.error || '';
                        this.updateOutput();
                    });
                }
            });

            list.appendChild(item);
        });

        this.updateButtons();
    }

    async runScript() {
        if (!this.isRobotConnected() || !this.selectedScript) return;

        const script = this.selectedScript;
        this.actionError = '';
        this.panel.querySelector('#script-output').textContent = '正在启动例程…';
        try {
            const data = await this.robotConnection.socket.runScript(script);
            if (data.success) {
                this.running = true;
                this.currentScript = script;
                this.updateStatus();
                this.updateButtons();
                this.startOutputPolling();
            }
        } catch (e) {
            this.actionError = e.message;
            this.panel.querySelector('#script-output').textContent = '操作失败：' + e.message;
            this.robotConnection.onError?.(e);
        }
    }

    async stopScript() {
        if (!this.isRobotConnected()) return;

        try {
            const data = await this.robotConnection.socket.stopScript();
            if (data.success) {
                this.running = false;
                this.currentScript = null;
                this.updateStatus();
                this.updateButtons();
                this.updateOutput();
                this.syncOutputPolling();
            }
        } catch (e) {
            this.actionError = e.message;
            this.panel.querySelector('#script-output').textContent = '操作失败：' + e.message;
            this.robotConnection.onError?.(e);
        }
    }

    updateStatus() {
        const dot = this.panel?.querySelector('.status-dot-small');
        const text = this.panel?.querySelector('#script-status-text');
        if (dot) {
            dot.className = `status-dot-small ${this.running ? 'connected' : 'disconnected'}`;
        }
        if (text) {
            text.textContent = this.running
                ? `Running: ${this.currentScript || ''}`
                : 'Idle';
        }

        // Update top bar button
        if (this.topBarBtn) {
            if (this.running) {
                this.topBarBtn.innerHTML = '&#9632; Scripts';
            } else {
                this.topBarBtn.innerHTML = '&#9654; Scripts';
            }
        }
    }

    updateButtons() {
        const runBtn = this.panel?.querySelector('#script-run-btn');
        const stopBtn = this.panel?.querySelector('#script-stop-btn');
        const list = this.panel?.querySelector('#script-list');
        const connected = this.isRobotConnected();

        if (runBtn) runBtn.disabled = !connected || this.running || !this.selectedScript || !this.scripts.find(s => s.file === this.selectedScript)?.supported;
        if (stopBtn) stopBtn.disabled = !connected || !this.running;
        if (list) {
            list.classList.toggle('disabled', this.running);
            list.querySelectorAll('.script-list-item').forEach(item => {
                item.disabled = this.running;
            });
        }
    }

    isRobotConnected() {
        return Boolean(this.robotConnection?.isConnected?.());
    }

    async updateOutput() {
        const output = this.panel?.querySelector('#script-output');
        if (!output) return;

        try {
            const logResp = await fetch('/api/scripts/log', { cache: 'no-store' });
            const text = await logResp.text();
            if (!logResp.ok || text.trimStart().startsWith('<!doctype') || text.trimStart().startsWith('<html')) {
                output.textContent = 'Failed to load script log: /api/scripts/log did not reach the backend. Restart backend/frontend and check Vite proxy.';
                return;
            }

            const selected = this.scripts.find(s => s.file === this.selectedScript);
            if (this.actionError) {
                output.textContent = '操作失败：' + this.actionError;
            } else if (selected && !selected.supported && !this.running) {
                output.textContent = (selected.reason || '此例程尚未适配网页执行，请在终端运行。') + '\n\n' + (this.previewName === selected.file ? this.preview : '');
            } else {
                output.textContent = text.trim().length > 0 ? text : 'No script output.';
            }
            output.scrollTop = output.scrollHeight;

            const statusResp = await fetch('/api/scripts');
            const data = await statusResp.json();
            const wasRunning = this.running;
            this.running = Boolean(data.running);
            this.currentScript = data.current_script || (this.running ? this.currentScript : null);

            if (wasRunning !== this.running) {
                this.updateStatus();
                this.updateButtons();
            }

            this.syncOutputPolling();
        } catch (error) {
            output.textContent = `Failed to load script output: ${error.message}`;
        }
    }

    startOutputPolling() {
        if (this.outputPollTimer) return;
        this.outputPollTimer = setInterval(() => this.updateOutput(), 500);
        this.updateOutput();
    }

    stopOutputPolling() {
        if (!this.outputPollTimer) return;
        clearInterval(this.outputPollTimer);
        this.outputPollTimer = null;
    }

    startConnectionPolling() {
        if (this.connectionPollTimer) return;

        let lastConnected = this.isRobotConnected();
        this.connectionPollTimer = setInterval(() => {
            const connected = this.isRobotConnected();
            if (connected !== lastConnected) {
                lastConnected = connected;
                this.updateButtons();
            }
        }, 500);
        this.updateButtons();
    }

    syncOutputPolling() {
        if (this.running || (this.panel && !this.panel.classList.contains('hidden'))) {
            this.startOutputPolling();
        } else {
            this.stopOutputPolling();
        }
    }

    togglePanel() {
        if (this.panel) {
            const visible = this.panelManager
                ? this.panelManager.togglePanel('floating-script-panel', 'flex', {
                    anchorEl: this.topBarBtn,
                    align: 'right',
                    offsetX: SCRIPT_PANEL_OFFSET_X
                })
                : !this.panel.classList.toggle('hidden');
            if (visible) {
                this.topBarBtn?.classList.add('active');
                this.loadScripts();
                this.startOutputPolling();
            } else {
                this.topBarBtn?.classList.remove('active');
                this.syncOutputPolling();
            }
        }
    }

    showPanel() {
        if (this.panel) {
            if (this.panelManager) {
                this.panelManager.showPanel('floating-script-panel', 'flex', {
                    anchorEl: this.topBarBtn,
                    align: 'right',
                    offsetX: SCRIPT_PANEL_OFFSET_X
                });
            } else {
                this.panel.classList.remove('hidden');
            }
            this.topBarBtn?.classList.add('active');
            this.loadScripts();
            this.startOutputPolling();
        }
    }

    hidePanel() {
        if (this.panel) {
            if (this.panelManager) {
                this.panelManager.hidePanel('floating-script-panel');
            } else {
                this.panel.classList.add('hidden');
            }
            this.topBarBtn?.classList.remove('active');
            this.syncOutputPolling();
        }
    }
}
