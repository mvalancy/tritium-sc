// Created by Matthew Valancy
// Copyright 2026 Valpatel Software LLC
// Licensed under AGPL-3.0 — see LICENSE for details.
//
// Training Data Dashboard — shows model status, training stats,
// accuracy over time, recent feedback entries, and retrain button.

export function createTrainingDashboard() {
    const panel = document.createElement('div');
    panel.className = 'panel training-dashboard';
    panel.innerHTML = `
        <div class="panel-header">
            <h3>ML Training Dashboard</h3>
            <div class="panel-controls">
                <button class="btn-retrain" title="Retrain correlation model">RETRAIN</button>
                <button class="btn-refresh" title="Refresh status">REFRESH</button>
            </div>
        </div>
        <div class="training-content">
            <div class="model-status-section">
                <h4>Model Status</h4>
                <div class="status-grid">
                    <div class="stat-card">
                        <span class="stat-label">Status</span>
                        <span class="stat-value model-trained">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Accuracy</span>
                        <span class="stat-value model-accuracy">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Training Examples</span>
                        <span class="stat-value model-count">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Last Trained</span>
                        <span class="stat-value model-last-trained">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">sklearn Available</span>
                        <span class="stat-value sklearn-status">--</span>
                    </div>
                </div>
            </div>

            <div class="training-data-section">
                <h4>Training Data</h4>
                <div class="status-grid">
                    <div class="stat-card">
                        <span class="stat-label">Correlation Decisions</span>
                        <span class="stat-value corr-total">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Confirmed Outcomes</span>
                        <span class="stat-value corr-confirmed">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Classifications</span>
                        <span class="stat-value class-total">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Operator Feedback</span>
                        <span class="stat-value feedback-total">--</span>
                    </div>
                    <div class="stat-card">
                        <span class="stat-label">Feedback Accuracy</span>
                        <span class="stat-value feedback-accuracy">--</span>
                    </div>
                </div>
            </div>

            <div class="retrain-log-section">
                <h4>Retrain Log</h4>
                <div class="retrain-log"></div>
            </div>
        </div>
    `;

    // Style
    const style = document.createElement('style');
    style.textContent = `
        .training-dashboard {
            color: #c0c0c0;
            font-family: 'JetBrains Mono', 'Fira Code', monospace;
        }
        .training-dashboard .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px 12px;
            border-bottom: 1px solid #1a1a2e;
        }
        .training-dashboard .panel-header h3 {
            margin: 0;
            color: #00f0ff;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .training-dashboard .panel-controls {
            display: flex;
            gap: 8px;
        }
        .training-dashboard .btn-retrain,
        .training-dashboard .btn-refresh {
            background: #12121a;
            border: 1px solid #00f0ff;
            color: #00f0ff;
            padding: 4px 12px;
            font-size: 11px;
            cursor: pointer;
            font-family: inherit;
            text-transform: uppercase;
        }
        .training-dashboard .btn-retrain:hover {
            background: #00f0ff;
            color: #0a0a0f;
        }
        .training-dashboard .btn-refresh:hover {
            background: #1a1a2e;
        }
        .training-dashboard .training-content {
            padding: 12px;
        }
        .training-dashboard h4 {
            color: #ff2a6d;
            font-size: 12px;
            margin: 12px 0 8px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .training-dashboard .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
            gap: 8px;
        }
        .training-dashboard .stat-card {
            background: #0e0e14;
            border: 1px solid #1a1a2e;
            padding: 8px;
            display: flex;
            flex-direction: column;
        }
        .training-dashboard .stat-label {
            font-size: 10px;
            color: #666;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .training-dashboard .stat-value {
            font-size: 16px;
            color: #05ffa1;
            margin-top: 4px;
        }
        .training-dashboard .retrain-log {
            background: #0e0e14;
            border: 1px solid #1a1a2e;
            padding: 8px;
            max-height: 200px;
            overflow-y: auto;
            font-size: 11px;
        }
        .training-dashboard .log-entry {
            padding: 4px 0;
            border-bottom: 1px solid #12121a;
        }
        .training-dashboard .log-entry.success { color: #05ffa1; }
        .training-dashboard .log-entry.error { color: #ff2a6d; }
        .training-dashboard .log-entry.info { color: #00f0ff; }
    `;
    panel.appendChild(style);

    // Wire up buttons
    const btnRetrain = panel.querySelector('.btn-retrain');
    const btnRefresh = panel.querySelector('.btn-refresh');
    const logEl = panel.querySelector('.retrain-log');

    btnRetrain.addEventListener('click', async () => {
        addLogEntry(logEl, 'Triggering model retrain...', 'info');
        btnRetrain.disabled = true;
        btnRetrain.textContent = 'TRAINING...';
        try {
            const resp = await fetch('/api/intelligence/retrain', { method: 'POST' });
            const data = await resp.json();
            if (data.success) {
                addLogEntry(logEl,
                    `Retrain complete: accuracy=${(data.accuracy * 100).toFixed(1)}% n=${data.training_count}`,
                    'success');
            } else {
                addLogEntry(logEl, `Retrain failed: ${data.error || 'unknown error'}`, 'error');
            }
        } catch (err) {
            addLogEntry(logEl, `Retrain error: ${err.message}`, 'error');
        }
        btnRetrain.disabled = false;
        btnRetrain.textContent = 'RETRAIN';
        refreshStatus(panel);
    });

    btnRefresh.addEventListener('click', () => refreshStatus(panel));

    // Initial load
    refreshStatus(panel);

    return panel;
}

async function refreshStatus(panel) {
    try {
        const resp = await fetch('/api/intelligence/model/status');
        const data = await resp.json();

        // Model status
        const trainedEl = panel.querySelector('.model-trained');
        trainedEl.textContent = data.trained ? 'TRAINED' : 'NOT TRAINED';
        trainedEl.style.color = data.trained ? '#05ffa1' : '#ff2a6d';

        panel.querySelector('.model-accuracy').textContent =
            data.accuracy > 0 ? (data.accuracy * 100).toFixed(1) + '%' : '--';

        panel.querySelector('.model-count').textContent =
            data.training_count > 0 ? data.training_count.toString() : '0';

        panel.querySelector('.model-last-trained').textContent =
            data.last_trained_iso || 'never';

        const sklearnEl = panel.querySelector('.sklearn-status');
        sklearnEl.textContent = data.sklearn_available ? 'YES' : 'NO';
        sklearnEl.style.color = data.sklearn_available ? '#05ffa1' : '#fcee0a';

        // Training data stats
        const stats = data.training_data_stats || {};
        const corr = stats.correlation || {};
        const cls = stats.classification || {};
        const fb = stats.feedback || {};

        panel.querySelector('.corr-total').textContent = (corr.total || 0).toString();
        panel.querySelector('.corr-confirmed').textContent = (corr.confirmed || 0).toString();
        panel.querySelector('.class-total').textContent = (cls.total || 0).toString();
        panel.querySelector('.feedback-total').textContent = (fb.total || 0).toString();
        panel.querySelector('.feedback-accuracy').textContent =
            fb.accuracy > 0 ? (fb.accuracy * 100).toFixed(1) + '%' : '--';

    } catch (err) {
        console.warn('Failed to fetch training status:', err);
    }
}

function addLogEntry(logEl, message, type) {
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    const ts = new Date().toLocaleTimeString();
    entry.textContent = `[${ts}] ${message}`;
    logEl.prepend(entry);
    // Keep last 50 entries
    while (logEl.children.length > 50) {
        logEl.removeChild(logEl.lastChild);
    }
}
