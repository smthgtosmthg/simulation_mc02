/* ═══════════════════════════════════════════════════════════════
   Drone Simulation Dashboard — JavaScript
   Polls the Python backend every 2s, updates charts & tables.
   ═══════════════════════════════════════════════════════════════ */

const API = window.location.origin + '/api';
const POLL_MS = 2000;
const MAX_HISTORY = 120;       // keep last 120 ticks on charts

// ─── Colors ───
const C = {
    wifi:      'rgba(34,211,238,1)',
    wifiBg:    'rgba(34,211,238,.15)',
    fiveg:     'rgba(167,139,250,1)',
    fivegBg:   'rgba(167,139,250,.15)',
    green:     'rgba(52,211,153,1)',
    greenBg:   'rgba(52,211,153,.15)',
    red:       'rgba(248,113,113,1)',
    orange:    'rgba(251,146,60,1)',
    orangeBg:  'rgba(251,146,60,.12)',
    yellow:    'rgba(251,191,36,1)',
    grid:      'rgba(255,255,255,.06)',
    text:      'rgba(160,174,192,.7)',
};

// ─── Chart.js global defaults ───
Chart.defaults.color = C.text;
Chart.defaults.borderColor = C.grid;
Chart.defaults.font.family = "'Inter', sans-serif";
Chart.defaults.font.size = 11;
Chart.defaults.elements.point.radius = 2;
Chart.defaults.elements.point.hoverRadius = 5;
Chart.defaults.animation.duration = 400;

// Pair-specific colors (up to 6 pairs)
const PAIR_COLORS = [
    { wifi: '#22d3ee', fiveg: '#a78bfa' },
    { wifi: '#34d399', fiveg: '#f472b6' },
    { wifi: '#fbbf24', fiveg: '#fb923c' },
    { wifi: '#60a5fa', fiveg: '#e879f9' },
    { wifi: '#a3e635', fiveg: '#f87171' },
    { wifi: '#2dd4bf', fiveg: '#c084fc' },
];

// ─── State ───
let activeScenario = 'both';   // 'both' | 'wifi' | '5g'

let state = {
    tick: 0,
    history: {
        labels: [],
        rssiWifi: {},     // { "0↔1": [values] }
        rssi5g: {},       // { "D0": [values] }
        latWifi: {},
        lat5g: {},
    },
    currentWifi: null,
    current5g: null,
    connected: false,
};

// ─── Scenario Filter ───
function applyFilter(scenario) {
    activeScenario = scenario;
    // Show/hide elements with data-vis attribute
    document.querySelectorAll('[data-vis]').forEach(el => {
        const vis = el.getAttribute('data-vis').split(/\s+/);
        if (vis.includes(scenario)) {
            el.style.display = '';
            el.classList.remove('hidden-by-filter');
        } else {
            el.style.display = 'none';
            el.classList.add('hidden-by-filter');
        }
    });
    // Re-render charts with filtered datasets
    updateTimeCharts();
    if (state.currentWifi || state.current5g) {
        updateComparisonCharts(state.currentWifi, state.current5g);
        drawMap(state.currentWifi, state.current5g, state.livePositions || {});
    }
}

// ─────────────────────────────────────────────────────────────
// CHARTS
// ─────────────────────────────────────────────────────────────

function makeTimeChart(canvasId, yLabel, unit = '') {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            plugins: {
                legend: { display: true, position: 'top', labels: { usePointStyle: true, pointStyle: 'circle', boxWidth: 6, padding: 14, font: { size: 11 } } },
                tooltip: {
                    backgroundColor: 'rgba(17,22,32,.92)',
                    borderColor: 'rgba(255,255,255,.1)', borderWidth: 1,
                    titleFont: { weight: 600 },
                    callbacks: { label: (c) => `${c.dataset.label}: ${c.parsed.y?.toFixed(2)}${unit}` }
                }
            },
            scales: {
                x: { grid: { display: false }, ticks: { maxTicksLimit: 12, maxRotation: 0 } },
                y: { grid: { color: C.grid }, title: { display: true, text: yLabel, font: { size: 11 } } }
            }
        }
    });
}

function makeBarChart(canvasId, yLabel) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
        type: 'bar',
        data: { labels: [], datasets: [] },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: { display: true, position: 'top', labels: { usePointStyle: true, pointStyle: 'rect', boxWidth: 10, padding: 14 } },
                tooltip: { backgroundColor: 'rgba(17,22,32,.92)', borderColor: 'rgba(255,255,255,.1)', borderWidth: 1 }
            },
            scales: {
                x: { grid: { display: false } },
                y: { grid: { color: C.grid }, title: { display: true, text: yLabel, font: { size: 11 } } }
            }
        }
    });
}

const chartRSSI     = makeTimeChart('chartRSSI',    'RSSI (dBm)',    ' dBm');
const chartLatency  = makeTimeChart('chartLatency',  'Latence (ms)',  ' ms');
const chartCompRSSI = makeBarChart('chartCompareRSSI',    'RSSI (dBm)');
const chartCompLat  = makeBarChart('chartCompareLatency', 'Latence (ms)');
const chartDist     = makeBarChart('chartDistance',       'Distance (m)');

// ─────────────────────────────────────────────────────────────
// MAP CANVAS
// ─────────────────────────────────────────────────────────────

const mapCanvas = document.getElementById('canvasMap');
const mapCtx    = mapCanvas.getContext('2d');

function resizeMap() {
    const rect = mapCanvas.parentElement.getBoundingClientRect();
    mapCanvas.width  = rect.width - 16;
    mapCanvas.height = Math.max(rect.height - 16, 260);
}
resizeMap();
window.addEventListener('resize', resizeMap);

const DRONE_COLORS = ['#22d3ee', '#34d399', '#a78bfa', '#fbbf24', '#f472b6', '#fb923c'];
const GNB_POS = { x: 0, y: 0, z: 6 };

function drawMap(wifiData, fivegData, data_positions) {
    const W = mapCanvas.width, H = mapCanvas.height;
    mapCtx.clearRect(0, 0, W, H);

    // Warehouse bounds approx: x ∈ [-8, 8], y ∈ [-5, 5]
    const scale = Math.min(W / 20, H / 14);
    const cx = W / 2, cy = H / 2;
    const toX = (x) => cx + x * scale;
    const toY = (y) => cy - y * scale;  // invert Y

    // Grid
    mapCtx.strokeStyle = 'rgba(255,255,255,.04)';
    mapCtx.lineWidth = 1;
    for (let gx = -8; gx <= 8; gx += 2) {
        mapCtx.beginPath(); mapCtx.moveTo(toX(gx), 0); mapCtx.lineTo(toX(gx), H); mapCtx.stroke();
    }
    for (let gy = -5; gy <= 5; gy += 2) {
        mapCtx.beginPath(); mapCtx.moveTo(0, toY(gy)); mapCtx.lineTo(W, toY(gy)); mapCtx.stroke();
    }

    // Warehouse outline
    mapCtx.strokeStyle = 'rgba(255,255,255,.1)';
    mapCtx.lineWidth = 2;
    mapCtx.strokeRect(toX(-7), toY(4.5), 14 * scale, 9 * scale);

    // Collect positions — prioritize live positions from API
    let positions = {};

    // 1. Start with live positions (always fresh from positions CSV)
    if (data_positions) {
        Object.entries(data_positions).forEach(([id, pos]) => {
            positions[parseInt(id)] = pos;
        });
    }

    // 2. Override/add from wifi pair data
    if (wifiData && wifiData.pairs) {
        wifiData.pairs.forEach(p => {
            if (p.pos_a) positions[p.drone_a] = p.pos_a;
            if (p.pos_b) positions[p.drone_b] = p.pos_b;
        });
    }
    // 3. Override/add from 5G drone data
    if (fivegData && fivegData.drones) {
        fivegData.drones.forEach(d => {
            positions[d.id] = { x: d.x, y: d.y, z: d.z };
        });
    }

    // Draw gNB
    if (fivegData) {
        const gnb = fivegData.gnb || GNB_POS;
        const gx = toX(gnb.x), gy = toY(gnb.y);
        mapCtx.fillStyle = 'rgba(167,139,250,.3)';
        mapCtx.beginPath(); mapCtx.arc(gx, gy, 18, 0, Math.PI * 2); mapCtx.fill();
        mapCtx.fillStyle = '#a78bfa';
        mapCtx.beginPath(); mapCtx.arc(gx, gy, 6, 0, Math.PI * 2); mapCtx.fill();
        mapCtx.fillStyle = '#a78bfa';
        mapCtx.font = '600 11px Inter';
        mapCtx.textAlign = 'center';
        mapCtx.fillText('gNB', gx, gy - 14);
    }

    // Draw links between pairs (WiFi)
    if (wifiData && wifiData.pairs) {
        wifiData.pairs.forEach((p, i) => {
            const a = positions[p.drone_a], b = positions[p.drone_b];
            if (!a || !b) return;
            const blocked = p.rssi === 'blocked' || p.rssi === 'error';
            mapCtx.strokeStyle = blocked ? 'rgba(248,113,113,.3)' : 'rgba(34,211,238,.2)';
            mapCtx.lineWidth = blocked ? 1 : 2;
            mapCtx.setLineDash(blocked ? [4, 4] : []);
            mapCtx.beginPath();
            mapCtx.moveTo(toX(a.x), toY(a.y));
            mapCtx.lineTo(toX(b.x), toY(b.y));
            mapCtx.stroke();
            mapCtx.setLineDash([]);
        });
    }

    // Draw links to gNB (5G)
    if (fivegData && fivegData.drones) {
        const gnb = fivegData.gnb || GNB_POS;
        fivegData.drones.forEach(d => {
            const blocked = d.rssi === null || d.rssi === 'BLOCKED';
            mapCtx.strokeStyle = blocked ? 'rgba(248,113,113,.2)' : 'rgba(167,139,250,.15)';
            mapCtx.lineWidth = 1;
            mapCtx.setLineDash([3, 3]);
            mapCtx.beginPath();
            mapCtx.moveTo(toX(d.x), toY(d.y));
            mapCtx.lineTo(toX(gnb.x), toY(gnb.y));
            mapCtx.stroke();
            mapCtx.setLineDash([]);
        });
    }

    // Draw drones
    const droneIds = Object.keys(positions).map(Number).sort();
    droneIds.forEach((did, idx) => {
        const p = positions[did];
        const px = toX(p.x), py = toY(p.y);
        const col = DRONE_COLORS[idx % DRONE_COLORS.length];

        // Glow
        const grad = mapCtx.createRadialGradient(px, py, 0, px, py, 22);
        grad.addColorStop(0, col + '33');
        grad.addColorStop(1, col + '00');
        mapCtx.fillStyle = grad;
        mapCtx.beginPath(); mapCtx.arc(px, py, 22, 0, Math.PI * 2); mapCtx.fill();

        // Dot
        mapCtx.fillStyle = col;
        mapCtx.beginPath(); mapCtx.arc(px, py, 6, 0, Math.PI * 2); mapCtx.fill();

        // Label
        mapCtx.fillStyle = '#fff';
        mapCtx.font = '600 11px Inter';
        mapCtx.textAlign = 'center';
        mapCtx.fillText(`D${did}`, px, py - 12);

        // Altitude
        mapCtx.fillStyle = 'rgba(255,255,255,.4)';
        mapCtx.font = '10px JetBrains Mono';
        mapCtx.fillText(`z=${p.z?.toFixed(1) || '?'}`, px, py + 18);
    });
}

// ─────────────────────────────────────────────────────────────
// QUALITY BADGE
// ─────────────────────────────────────────────────────────────

function qualityBadge(rssi) {
    if (rssi === null || rssi === 'blocked' || rssi === 'error' || rssi === 'BLOCKED') {
        return '<span class="quality blocked">BLOCKED</span>';
    }
    const v = parseFloat(rssi);
    if (isNaN(v)) return '<span class="quality blocked">N/A</span>';
    if (v > -50) return '<span class="quality excellent">Excellent</span>';
    if (v > -60) return '<span class="quality good">Good</span>';
    if (v > -70) return '<span class="quality fair">Fair</span>';
    return '<span class="quality weak">Weak</span>';
}

// ─────────────────────────────────────────────────────────────
// UPDATE FUNCTIONS
// ─────────────────────────────────────────────────────────────

function updateKPIs(wifi, fiveg) {
    // Drone count
    let droneSet = new Set();
    if (wifi?.pairs) wifi.pairs.forEach(p => { droneSet.add(p.drone_a); droneSet.add(p.drone_b); });
    if (fiveg?.drones) fiveg.drones.forEach(d => droneSet.add(d.id));
    document.getElementById('valDrones').textContent = droneSet.size || '0';

    // WiFi avg RSSI
    if (wifi?.pairs?.length) {
        const valid = wifi.pairs.filter(p => p.rssi !== 'blocked' && p.rssi !== 'error').map(p => parseFloat(p.rssi));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valRssiWifi').textContent = avg !== null ? avg.toFixed(1) + ' dBm' : '-- dBm';
    }

    // WiFi avg latency
    if (wifi?.pairs?.length) {
        const valid = wifi.pairs.filter(p => p.latency !== '---').map(p => parseFloat(p.latency));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valLatWifi').textContent = avg !== null ? avg.toFixed(2) + ' ms' : '-- ms';
    }

    // 5G avg RSSI
    if (fiveg?.drones?.length) {
        const valid = fiveg.drones.filter(d => d.rssi !== null && d.rssi !== 'BLOCKED').map(d => parseFloat(d.rssi));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valRssi5g').textContent = avg !== null ? avg.toFixed(1) + ' dBm' : '-- dBm';
    }

    // 5G avg latency
    if (fiveg?.pairs?.length) {
        const valid = fiveg.pairs.map(p => parseFloat(p.latency));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valLat5g').textContent = avg !== null ? avg.toFixed(2) + ' ms' : '-- ms';
    }

    document.getElementById('valTick').textContent = state.tick;
}

function updateWifiTable(wifi) {
    const tbody = document.querySelector('#tableWifi tbody');
    if (!wifi?.pairs?.length) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--text-2)">En attente...</td></tr>';
        return;
    }
    tbody.innerHTML = wifi.pairs.map(p => {
        const rssiStr = (p.rssi === 'blocked' || p.rssi === 'error')
            ? '<span style="color:var(--accent-red)">BLOCKED</span>'
            : parseFloat(p.rssi).toFixed(1) + ' dBm';
        const latStr = p.latency === '---' ? '---' : parseFloat(p.latency).toFixed(2) + ' ms';
        return `<tr>
            <td>D${p.drone_a} ↔ D${p.drone_b}</td>
            <td>${rssiStr}</td>
            <td>${latStr}</td>
            <td>${parseFloat(p.distance).toFixed(1)} m</td>
            <td>${qualityBadge(p.rssi)}</td>
        </tr>`;
    }).join('');
}

function update5gTable(fiveg) {
    const tbody = document.querySelector('#table5g tbody');
    if (!fiveg?.drones?.length) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--text-2)">En attente...</td></tr>';
        return;
    }

    // Drone → gNB rows
    let rows = fiveg.drones.map(d => {
        const rssiStr = (d.rssi === null || d.rssi === 'BLOCKED')
            ? '<span style="color:var(--accent-red)">BLOCKED</span>'
            : parseFloat(d.rssi).toFixed(1) + ' dBm';
        // Find latency from pairs
        let lat = '--', jit = '--';
        if (fiveg.pairs) {
            const p = fiveg.pairs.find(p => p.drone_a === d.id || p.drone_b === d.id);
            if (p) { lat = parseFloat(p.latency).toFixed(2) + ' ms'; jit = parseFloat(p.jitter).toFixed(2) + ' ms'; }
        }
        return `<tr>
            <td>D${d.id}</td>
            <td>${rssiStr}</td>
            <td>${lat}</td>
            <td>${jit}</td>
            <td>${qualityBadge(d.rssi)}</td>
        </tr>`;
    });
    tbody.innerHTML = rows.join('');
}

function pushHistory(wifi, fiveg) {
    const h = state.history;
    const label = state.tick.toString();
    h.labels.push(label);
    if (h.labels.length > MAX_HISTORY) h.labels.shift();

    // WiFi RSSI & latency per pair
    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            const key = `${p.drone_a}↔${p.drone_b}`;
            if (!h.rssiWifi[key]) h.rssiWifi[key] = [];
            if (!h.latWifi[key])  h.latWifi[key]  = [];
            const rssi = (p.rssi === 'blocked' || p.rssi === 'error') ? null : parseFloat(p.rssi);
            const lat  = p.latency === '---' ? null : parseFloat(p.latency);
            h.rssiWifi[key].push(rssi);
            h.latWifi[key].push(lat);
            if (h.rssiWifi[key].length > MAX_HISTORY) h.rssiWifi[key].shift();
            if (h.latWifi[key].length > MAX_HISTORY) h.latWifi[key].shift();
        });
    }

    // 5G RSSI per drone
    if (fiveg?.drones) {
        fiveg.drones.forEach(d => {
            const key = `D${d.id}→gNB`;
            if (!h.rssi5g[key]) h.rssi5g[key] = [];
            const rssi = (d.rssi === null || d.rssi === 'BLOCKED') ? null : parseFloat(d.rssi);
            h.rssi5g[key].push(rssi);
            if (h.rssi5g[key].length > MAX_HISTORY) h.rssi5g[key].shift();
        });
    }

    // 5G latency per pair
    if (fiveg?.pairs) {
        fiveg.pairs.forEach(p => {
            const key = `${p.drone_a}↔${p.drone_b} 5G`;
            if (!h.lat5g[key]) h.lat5g[key] = [];
            h.lat5g[key].push(parseFloat(p.latency));
            if (h.lat5g[key].length > MAX_HISTORY) h.lat5g[key].shift();
        });
    }
}

function updateTimeCharts() {
    const h = state.history;

    const showWifi = (activeScenario === 'both' || activeScenario === 'wifi');
    const show5g   = (activeScenario === 'both' || activeScenario === '5g');

    // RSSI chart: wifi pairs + 5G drones (filtered)
    let datasets = [];
    let idx = 0;
    if (showWifi) {
        Object.entries(h.rssiWifi).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `WiFi ${key}`,
                data: [...data],
                borderColor: c.wifi,
                backgroundColor: c.wifi + '22',
                borderWidth: 2,
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    if (show5g) {
        Object.entries(h.rssi5g).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `5G ${key}`,
                data: [...data],
                borderColor: c.fiveg,
                backgroundColor: c.fiveg + '22',
                borderWidth: 2,
                borderDash: [5, 3],
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    chartRSSI.data.labels = [...h.labels];
    chartRSSI.data.datasets = datasets;
    chartRSSI.update('none');

    // Latency chart: wifi + 5G (filtered)
    datasets = [];
    idx = 0;
    if (showWifi) {
        Object.entries(h.latWifi).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `WiFi ${key}`,
                data: [...data],
                borderColor: c.wifi,
                backgroundColor: c.wifi + '22',
                borderWidth: 2,
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    if (show5g) {
        Object.entries(h.lat5g).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `5G ${key}`,
                data: [...data],
                borderColor: c.fiveg,
                backgroundColor: c.fiveg + '22',
                borderWidth: 2,
                borderDash: [5, 3],
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    chartLatency.data.labels = [...h.labels];
    chartLatency.data.datasets = datasets;
    chartLatency.update('none');
}

function updateComparisonCharts(wifi, fiveg) {
    // RSSI comparison bar chart
    const pairLabels = [];
    const wifiRssi = [];
    const fivegRssi = [];

    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            pairLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            const r = (p.rssi === 'blocked' || p.rssi === 'error') ? null : parseFloat(p.rssi);
            wifiRssi.push(r);
        });
    }

    // For 5G we average RSSI of both drones in each pair
    if (fiveg?.pairs && fiveg?.drones) {
        const droneRssi = {};
        fiveg.drones.forEach(d => { droneRssi[d.id] = d.rssi; });
        // Match the same pair order as WiFi
        if (wifi?.pairs) {
            wifi.pairs.forEach(p => {
                const ra = droneRssi[p.drone_a], rb = droneRssi[p.drone_b];
                if (ra != null && rb != null && ra !== 'BLOCKED' && rb !== 'BLOCKED') {
                    fivegRssi.push((parseFloat(ra) + parseFloat(rb)) / 2);
                } else {
                    fivegRssi.push(null);
                }
            });
        }
    }

    chartCompRSSI.data.labels = pairLabels;
    chartCompRSSI.data.datasets = [
        { label: 'WiFi', data: wifiRssi, backgroundColor: C.wifiBg, borderColor: C.wifi, borderWidth: 2, borderRadius: 6 },
        { label: '5G NR', data: fivegRssi, backgroundColor: C.fivegBg, borderColor: C.fiveg, borderWidth: 2, borderRadius: 6 },
    ];
    chartCompRSSI.update('none');

    // Latency comparison
    const latLabels = [];
    const wifiLat = [];
    const fivegLat = [];

    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            latLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            wifiLat.push(p.latency === '---' ? null : parseFloat(p.latency));
        });
    }
    if (fiveg?.pairs) {
        fiveg.pairs.forEach((p, i) => {
            if (i < latLabels.length) fivegLat.push(parseFloat(p.latency));
        });
    }

    chartCompLat.data.labels = latLabels;
    chartCompLat.data.datasets = [
        { label: 'WiFi', data: wifiLat, backgroundColor: C.wifiBg, borderColor: C.wifi, borderWidth: 2, borderRadius: 6 },
        { label: '5G NR', data: fivegLat, backgroundColor: C.fivegBg, borderColor: C.fiveg, borderWidth: 2, borderRadius: 6 },
    ];
    chartCompLat.update('none');

    // Distance chart
    const distLabels = [];
    const distVals = [];
    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            distLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            distVals.push(parseFloat(p.distance));
        });
    }
    chartDist.data.labels = distLabels;
    chartDist.data.datasets = [{
        label: 'Distance',
        data: distVals,
        backgroundColor: C.greenBg,
        borderColor: C.green,
        borderWidth: 2,
        borderRadius: 6,
    }];
    chartDist.update('none');
}

// ─────────────────────────────────────────────────────────────
// LOG
// ─────────────────────────────────────────────────────────────

const logBody = document.getElementById('logBody');
function addLog(msg, type = 'data') {
    const el = document.createElement('div');
    el.className = `log-entry ${type}`;
    const now = new Date().toLocaleTimeString('fr-FR');
    el.textContent = `[${now}] ${msg}`;
    logBody.appendChild(el);
    if (logBody.children.length > 200) logBody.removeChild(logBody.firstChild);
    logBody.scrollTop = logBody.scrollHeight;
}

document.getElementById('btnClearLog').addEventListener('click', () => {
    logBody.innerHTML = '';
    addLog('Journal effacé.', 'info');
});

// ─────────────────────────────────────────────────────────────
// STATUS
// ─────────────────────────────────────────────────────────────

const pill = document.getElementById('statusPill');
function setStatus(ok, text) {
    pill.className = 'status-pill ' + (ok ? 'connected' : 'error');
    pill.querySelector('.status-text').textContent = text;
}

// Clock
setInterval(() => {
    document.getElementById('clock').textContent = new Date().toLocaleTimeString('fr-FR');
}, 1000);

// ─────────────────────────────────────────────────────────────
// TABS — filter WiFi / 5G / both
// ─────────────────────────────────────────────────────────────

document.querySelectorAll('.tab').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        applyFilter(btn.dataset.scenario);
        addLog(`Filtre: ${btn.textContent}`, 'info');
    });
});

// ─────────────────────────────────────────────────────────────
// POLLING
// ─────────────────────────────────────────────────────────────

let prevTick = -1;

async function poll() {
    try {
        const res = await fetch(API + '/data');
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();

        if (!state.connected) {
            setStatus(true, 'Connecté');
            addLog('Connexion au backend établie.', 'info');
            state.connected = true;
        }

        const wifi  = data.wifi;
        const fiveg = data.fiveg;
        const livePositions = data.positions || {};
        state.tick  = data.tick || state.tick;
        state.currentWifi = wifi;
        state.current5g   = fiveg;
        state.livePositions = livePositions;

        // Only push history if tick changed
        if (state.tick !== prevTick) {
            state.tick = Math.max(state.tick, prevTick + 1);
            pushHistory(wifi, fiveg);
            prevTick = state.tick;

            if (wifi?.pairs?.length || fiveg?.drones?.length) {
                const nPairs = wifi?.pairs?.length || 0;
                const nDrones5g = fiveg?.drones?.length || 0;
                addLog(`Tick ${state.tick}: WiFi ${nPairs} paires | 5G ${nDrones5g} drones`, 'data');
            }
        }

        updateKPIs(wifi, fiveg);
        updateWifiTable(wifi);
        update5gTable(fiveg);
        updateTimeCharts();
        updateComparisonCharts(wifi, fiveg);
        drawMap(wifi, fiveg, livePositions);

    } catch (err) {
        if (state.connected) {
            setStatus(false, 'Déconnecté');
            addLog(`Erreur: ${err.message}`, 'error');
            state.connected = false;
        }
    }
}

// Start polling
setInterval(poll, POLL_MS);
poll();

// Initial draw
drawMap(null, null, {});
