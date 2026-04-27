/* ============================================================
   SODAMCS — Standalone Dashboard Script
   Smart Object Detection & Adaptive Motor Control System
   - Multi-page (Dashboard / Sampling / Settings)
   - Sequential 2-stage Kalman Filter (matches research firmware)
   - Custom canvas-based combined chart (no external libs)
   ============================================================ */

(() => {
  // ===================== CONFIG / DEFAULTS =====================
  const HISTORY_LIMIT = 60;
  const LOG_LIMIT     = 60;
  const SAMPLE_LIMIT  = 500;
  const TICK_MS       = 220;
  const SAMPLE_TICK   = 200;

  // ===================== STATE =================================
  const state = {
    // MQTT
    mqttBroker:   "broker.hivemq.com",
    mqttPort:     1883,
    mqttClientId: "esp12f-sodamcs",
    mqttTopic:    "iot/sodamcs/data",
    mqttConnected: false,
    espConnected:  false,
    rssi:          -72,

    // Sensors
    ultrasonicActive: false,
    infraredActive:   false,
    filterActive:     false,

    // Threshold
    brakeThreshold: 20,

    // Kalman tuning
    kalmanQ:      0.05,
    kalmanRUltra: 10.0,
    kalmanRIr:    5.0,

    // Live
    ultrasonicRaw: 0,
    infraredRaw:   0,
    filteredDistance: 0,
    chartHistory:  [],

    // Motor / relay
    motorPWM: 0,
    motorRPM: 0,
    relayActive: false,

    // Logs
    logs: [],
    logId: 0,

    // Sampling
    samplingActive: false,
    samplingTrigger: "time",
    samplingIntervalMs: 1000,
    samplingDistanceDelta: 5,
    samplingSpeedDelta: 10,
    samples: [],
    sampleId: 0,
    lastSample: null,

    // Internal Kalman state
    kx: 80,
    kp: 1,
    targetDistance: 80,
    targetVel: 0,
  };

  // ===================== DOM HELPERS ============================
  const $ = (id) => document.getElementById(id);
  const $$ = (sel) => Array.from(document.querySelectorAll(sel));

  function fmt(n, d = 1) {
    if (n === null || n === undefined || isNaN(n)) return "--";
    return Number(n).toFixed(d);
  }
  function tsHHMMSS(ms) {
    const d = new Date(ms);
    return d.toLocaleTimeString("id-ID", { hour12: false });
  }

  // ===================== LOG ===================================
  function pushLog(level, tag, msg) {
    state.logs.unshift({ id: ++state.logId, ts: Date.now(), level, tag, msg });
    if (state.logs.length > LOG_LIMIT) state.logs.length = LOG_LIMIT;
    renderLogs();
  }
  function renderLogs() {
    const root = $("log-list");
    root.innerHTML = "";
    state.logs.forEach((l) => {
      const div = document.createElement("div");
      div.className = `log-line ${l.level}`;
      div.innerHTML = `
        <span class="ts">${tsHHMMSS(l.ts)}</span>
        <span class="tag">[${l.tag}]</span>
        <span class="msg">${l.msg}</span>
      `;
      root.appendChild(div);
    });
  }

  // ===================== PAGE ROUTING ===========================
  function setPage(name) {
    $$(".page").forEach((p) => p.classList.toggle("active", p.id === `page-${name}`));
    $$(".nav-tab").forEach((t) => t.classList.toggle("active", t.dataset.page === name));
  }
  $$(".nav-tab").forEach((t) =>
    t.addEventListener("click", () => setPage(t.dataset.page))
  );

  // ===================== MQTT CONNECT ===========================
  $("btn-connect").addEventListener("click", () => {
    if (!state.mqttConnected) connectMqtt();
    else disconnectMqtt();
  });

  function connectMqtt() {
    state.mqttBroker   = $("in-broker").value.trim() || "broker.hivemq.com";
    state.mqttPort     = parseInt($("in-port").value, 10) || 1883;
    state.mqttClientId = $("in-client").value.trim() || "esp12f-sodamcs";
    state.mqttTopic    = $("in-topic").value.trim() || "iot/sodamcs/data";

    pushLog("info", "MQTT", `Menghubungkan ke ${state.mqttBroker}:${state.mqttPort}…`);
    $("btn-connect-text").textContent = "MENGHUBUNGKAN…";
    $("btn-connect").disabled = true;

    setTimeout(() => {
      state.mqttConnected = true;
      pushLog("ok", "MQTT", `Terhubung ke broker ${state.mqttBroker}`);
      setTimeout(() => {
        state.espConnected = true;
        state.rssi = -60 - Math.floor(Math.random() * 20);
        pushLog("ok", "ESP", `ESP-12F terdeteksi pada topic ${state.mqttTopic}`);
        $("btn-connect").disabled = false;
        renderConnState();
      }, 600);
      renderConnState();
    }, 800);
  }

  function disconnectMqtt() {
    state.mqttConnected = false;
    state.espConnected = false;
    state.ultrasonicActive = false;
    state.infraredActive = false;
    state.filterActive = false;
    state.samplingActive = false;
    state.ultrasonicRaw = 0;
    state.infraredRaw = 0;
    state.filteredDistance = 0;
    state.chartHistory = [];
    state.motorPWM = 0;
    state.motorRPM = 0;
    state.relayActive = false;
    pushLog("warn", "MQTT", "Koneksi diputus oleh pengguna");
    renderAll();
  }

  // ===================== SENSOR / FILTER TOGGLES =================
  $("btn-us").addEventListener("click", () => {
    if (!state.espConnected) return;
    state.ultrasonicActive = !state.ultrasonicActive;
    pushLog("info", "SENS", `HC-SR04 ${state.ultrasonicActive ? "diaktifkan" : "dinonaktifkan"}`);
    renderAll();
  });
  $("btn-ir").addEventListener("click", () => {
    if (!state.espConnected) return;
    state.infraredActive = !state.infraredActive;
    pushLog("info", "SENS", `Sharp GP2Y0A21 ${state.infraredActive ? "diaktifkan" : "dinonaktifkan"}`);
    renderAll();
  });
  $("btn-filter").addEventListener("click", () => {
    if (!state.espConnected) return;
    state.filterActive = !state.filterActive;
    pushLog("info", "FLT", `Kalman Filter ${state.filterActive ? "ON" : "OFF"}`);
    renderAll();
  });

  // ===================== THRESHOLD ===============================
  function syncThreshold(v) {
    v = Math.max(2, Math.min(200, parseInt(v, 10) || 0));
    state.brakeThreshold = v;
    $("in-threshold").value = v;
    $("thr-slider").value = v;
  }
  $("in-threshold").addEventListener("input", (e) => syncThreshold(e.target.value));
  $("thr-slider").addEventListener("input", (e) => syncThreshold(e.target.value));

  // ===================== KALMAN TUNING (Settings) ================
  function setKalman(part, raw) {
    const v = Number(raw);
    if (!Number.isFinite(v)) return;
    if (part === "q")  state.kalmanQ      = Math.max(0.001, Math.min(1, v));
    if (part === "ru") state.kalmanRUltra = Math.max(0.5,   Math.min(50, v));
    if (part === "ri") state.kalmanRIr    = Math.max(0.5,   Math.min(50, v));
    syncKalmanInputs();
  }
  function syncKalmanInputs() {
    $("num-q").value  = state.kalmanQ.toFixed(4);
    $("num-ru").value = state.kalmanRUltra.toFixed(2);
    $("num-ri").value = state.kalmanRIr.toFixed(2);
    $("sld-q").value  = state.kalmanQ;
    $("sld-ru").value = state.kalmanRUltra;
    $("sld-ri").value = state.kalmanRIr;
    $("set-active-q").textContent  = state.kalmanQ.toFixed(4);
    $("set-active-ru").textContent = state.kalmanRUltra.toFixed(2);
    $("set-active-ri").textContent = state.kalmanRIr.toFixed(2);
  }

  // Draft state: edits don't take effect until Apply
  const draft = { q: state.kalmanQ, ru: state.kalmanRUltra, ri: state.kalmanRIr };
  function bindParam(part, slider, num) {
    const sld = $(slider), nm = $(num);
    const sync = (v) => {
      const f = Number(v);
      if (!Number.isFinite(f)) return;
      draft[part] = f;
      const dec = part === "q" ? 4 : 2;
      sld.value = f;
      nm.value  = f.toFixed(dec);
      updateApplyState();
    };
    sld.addEventListener("input", (e) => sync(e.target.value));
    nm.addEventListener("input",  (e) => sync(e.target.value));
  }
  bindParam("q",  "sld-q",  "num-q");
  bindParam("ru", "sld-ru", "num-ru");
  bindParam("ri", "sld-ri", "num-ri");

  function updateApplyState() {
    const dirty =
      Number(draft.q.toFixed(4))  !== Number(state.kalmanQ.toFixed(4)) ||
      Number(draft.ru.toFixed(2)) !== Number(state.kalmanRUltra.toFixed(2)) ||
      Number(draft.ri.toFixed(2)) !== Number(state.kalmanRIr.toFixed(2));
    $("btn-apply-kalman").disabled = !dirty;
    $("apply-text").textContent = dirty ? "TERAPKAN PERUBAHAN" : "TIDAK ADA PERUBAHAN";
  }

  $("btn-apply-kalman").addEventListener("click", () => {
    setKalman("q",  draft.q);
    setKalman("ru", draft.ru);
    setKalman("ri", draft.ri);
    pushLog("ok", "FLT", `Parameter Kalman diperbarui: Q=${state.kalmanQ.toFixed(4)}, Ru=${state.kalmanRUltra.toFixed(2)}, Ri=${state.kalmanRIr.toFixed(2)}`);
    const f = $("saved-flash");
    f.classList.remove("hide");
    setTimeout(() => f.classList.add("hide"), 1800);
    updateApplyState();
  });

  $("btn-reset-kalman").addEventListener("click", () => {
    state.kalmanQ = 0.05;
    state.kalmanRUltra = 10.0;
    state.kalmanRIr = 5.0;
    draft.q = 0.05; draft.ru = 10.0; draft.ri = 5.0;
    syncKalmanInputs();
    pushLog("info", "FLT", "Parameter Kalman direset ke default (Q=0.05, Ru=10.0, Ri=5.0)");
    updateApplyState();
  });

  // ===================== SAMPLING UI =============================
  $$(".seg-btn").forEach((b) =>
    b.addEventListener("click", () => {
      $$(".seg-btn").forEach((x) => x.classList.toggle("active", x === b));
      state.samplingTrigger = b.dataset.mode;
      $("trig-time").classList.toggle("hide", state.samplingTrigger !== "time");
      $("trig-dist").classList.toggle("hide", state.samplingTrigger !== "distance");
      $("trig-spd").classList.toggle("hide",  state.samplingTrigger !== "speed");
      $("trig-param-label").textContent =
        state.samplingTrigger === "time"     ? "INTERVAL (ms)" :
        state.samplingTrigger === "distance" ? "DELTA JARAK (cm)" :
                                               "DELTA PWM (%)";
      $("smp-mode-disp").textContent = state.samplingTrigger.toUpperCase();
    })
  );
  $("trig-time").addEventListener("input", (e) => state.samplingIntervalMs    = Math.max(100, parseInt(e.target.value, 10) || 1000));
  $("trig-dist").addEventListener("input", (e) => state.samplingDistanceDelta = Math.max(1,   parseInt(e.target.value, 10) || 5));
  $("trig-spd").addEventListener("input",  (e) => state.samplingSpeedDelta    = Math.max(1,   parseInt(e.target.value, 10) || 10));

  $("btn-start").addEventListener("click", () => {
    if (!state.espConnected) return;
    state.samplingActive = true;
    state.lastSample = null;
    pushLog("ok", "SMP", `Sampling dimulai (mode: ${state.samplingTrigger})`);
    renderSamplingUI();
  });
  $("btn-stop").addEventListener("click", () => {
    state.samplingActive = false;
    pushLog("info", "SMP", "Sampling dihentikan");
    renderSamplingUI();
  });
  $("btn-clear").addEventListener("click", () => {
    state.samples = [];
    state.sampleId = 0;
    pushLog("info", "SMP", "Tabel sampling dikosongkan");
    renderSamplingTable();
    renderSamplingUI();
  });
  $("btn-export").addEventListener("click", exportCSV);
  $("btn-clearlog").addEventListener("click", () => { state.logs = []; renderLogs(); });

  function exportCSV() {
    if (state.samples.length === 0) return;
    const header = "id,timestamp,iso_time,ultrasonic_cm,infrared_cm,filtered_cm,pwm_pct,rpm,relay\n";
    const rows = state.samples
      .slice()
      .reverse()
      .map((r) => [
        r.id,
        r.ts,
        new Date(r.ts).toISOString(),
        r.ultrasonic.toFixed(2),
        r.infrared.toFixed(2),
        r.filtered.toFixed(2),
        r.pwm.toFixed(1),
        r.rpm,
        r.relay ? "TRIPPED" : "ARMED",
      ].join(","))
      .join("\n");
    const blob = new Blob([header + rows], { type: "text/csv;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `sodamcs_samples_${Date.now()}.csv`;
    a.click();
    URL.revokeObjectURL(url);
    pushLog("ok", "SMP", `Export CSV — ${state.samples.length} record`);
  }

  // ===================== SIMULATION TICK =========================
  setInterval(() => {
    if (!state.espConnected) return;

    // Smooth random walk target distance
    let v = state.targetVel + (Math.random() - 0.5) * 4;
    v = Math.max(-12, Math.min(12, v));
    state.targetVel = v;
    let target = state.targetDistance + v;
    if (target < 5)   { target = 5;   state.targetVel = Math.abs(state.targetVel); }
    if (target > 250) { target = 250; state.targetVel = -Math.abs(state.targetVel); }
    state.targetDistance = target;

    // Generate raw readings
    const usNoise = (Math.random() - 0.5) * 4;
    const irNoise = (Math.random() - 0.5) * 14;
    const usReading = state.ultrasonicActive ? Math.max(2, Math.min(400, target + usNoise)) : 0;
    const irReading = state.infraredActive   ? Math.max(8, Math.min(150, target + irNoise)) : 0;

    state.ultrasonicRaw = usReading;
    state.infraredRaw   = irReading;

    const haveUS = state.ultrasonicActive;
    const haveIR = state.infraredActive;
    const fused = (haveUS && haveIR) ? (usReading + irReading) / 2
                : haveUS ? usReading
                : haveIR ? irReading : 0;

    let filtered = fused;
    const filterOn = state.filterActive && (haveUS || haveIR);
    if (filterOn) {
      // ===== Sequential 2-stage Kalman =====
      const Q  = state.kalmanQ;
      const Ru = state.kalmanRUltra;
      const Ri = state.kalmanRIr;

      // PREDICT
      const P_pred = state.kp + Q;

      // STAGE 1 — Ultrasonic
      let x_upd1 = state.kx;
      let P_upd1 = P_pred;
      if (haveUS) {
        const K1 = P_pred / (P_pred + Ru);
        x_upd1   = state.kx + K1 * (usReading - state.kx);
        P_upd1   = (1 - K1) * P_pred;
      }
      // STAGE 2 — Infrared (uses stage 1 output as prior)
      let xNew = x_upd1, pNew = P_upd1;
      if (haveIR) {
        const K2 = P_upd1 / (P_upd1 + Ri);
        xNew = x_upd1 + K2 * (irReading - x_upd1);
        pNew = (1 - K2) * P_upd1;
      }
      state.kx = xNew; state.kp = pNew;
      filtered = xNew;
    } else {
      state.kx = fused || 0; state.kp = 1;
    }
    state.filteredDistance = filtered;

    // Chart history (single timeline)
    if (haveUS || haveIR) {
      state.chartHistory.push({
        t: Date.now(),
        us:   haveUS ? Number(usReading.toFixed(2)) : null,
        ir:   haveIR ? Number(irReading.toFixed(2)) : null,
        filt: filterOn ? Number(filtered.toFixed(2)) : null,
      });
      if (state.chartHistory.length > HISTORY_LIMIT) state.chartHistory.shift();
    }

    // Auto motor + relay
    let pwm = 0, relay = false;
    if (haveUS || haveIR) {
      if (filtered < state.brakeThreshold) {
        pwm = 0; relay = true;
      } else {
        const span = Math.max(20, 250 - state.brakeThreshold);
        pwm = Math.max(0, Math.min(100, ((filtered - state.brakeThreshold) / span) * 100));
        relay = false;
      }
    }
    state.motorPWM = pwm;
    state.motorRPM = Math.round((pwm / 100) * 3600);

    if (state.relayActive !== relay) {
      if (relay) pushLog("warn", "RLY", `Pengereman darurat aktif — jarak ${filtered.toFixed(1)} cm < threshold ${state.brakeThreshold} cm`);
      else       pushLog("info", "RLY", "Relay normal — beban tersambung");
    }
    state.relayActive = relay;

    if (Math.random() < 0.15) state.rssi = -55 - Math.floor(Math.random() * 25);

    renderLive();
  }, TICK_MS);

  // ===================== SAMPLING TICK ===========================
  setInterval(() => {
    if (!state.samplingActive) return;
    const now = Date.now();
    const last = state.lastSample;
    let should = false;
    if (!last) should = true;
    else if (state.samplingTrigger === "time"     && now - last.ts >= state.samplingIntervalMs) should = true;
    else if (state.samplingTrigger === "distance" && Math.abs(state.filteredDistance - last.distance) >= state.samplingDistanceDelta) should = true;
    else if (state.samplingTrigger === "speed"    && Math.abs(state.motorPWM - last.pwm) >= state.samplingSpeedDelta) should = true;

    if (should) {
      const rec = {
        id: ++state.sampleId,
        ts: now,
        ultrasonic: state.ultrasonicRaw,
        infrared:   state.infraredRaw,
        filtered:   state.filteredDistance,
        pwm:        state.motorPWM,
        rpm:        state.motorRPM,
        relay:      state.relayActive,
      };
      state.samples.unshift(rec);
      if (state.samples.length > SAMPLE_LIMIT) state.samples.length = SAMPLE_LIMIT;
      state.lastSample = { ts: now, distance: state.filteredDistance, pwm: state.motorPWM };
      renderSamplingTable();
      renderSamplingUI();
    }
  }, SAMPLE_TICK);

  // ===================== CANVAS CHART ============================
  const chart = $("combined-chart");
  const ctx = chart.getContext("2d");
  function drawChart() {
    const dpr = window.devicePixelRatio || 1;
    const cssW = chart.clientWidth;
    const cssH = chart.clientHeight;
    if (chart.width !== cssW * dpr || chart.height !== cssH * dpr) {
      chart.width  = cssW * dpr;
      chart.height = cssH * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);

    const data = state.chartHistory;
    if (data.length < 2) { $("chart-empty").classList.remove("hide"); return; }
    $("chart-empty").classList.add("hide");

    // Find min/max across visible series
    let min = Infinity, max = -Infinity;
    data.forEach((p) => {
      [p.us, p.ir, p.filt].forEach((v) => {
        if (v == null) return;
        if (v < min) min = v;
        if (v > max) max = v;
      });
    });
    if (!isFinite(min)) { min = 0; max = 100; }
    const pad = Math.max(2, (max - min) * 0.1);
    min -= pad; max += pad;
    if (max - min < 5) { max = min + 5; }

    // Layout
    const padL = 40, padR = 12, padT = 8, padB = 22;
    const w = cssW - padL - padR;
    const h = cssH - padT - padB;

    // Grid
    ctx.strokeStyle = "rgba(110,140,160,0.18)";
    ctx.lineWidth = 1;
    ctx.font = "10px ui-monospace, Menlo, monospace";
    ctx.fillStyle = "rgba(150,180,200,0.55)";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    const gridLines = 4;
    for (let i = 0; i <= gridLines; i++) {
      const y = padT + (h / gridLines) * i;
      const value = max - ((max - min) / gridLines) * i;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + w, y);
      ctx.stroke();
      ctx.fillText(value.toFixed(0), padL - 6, y);
    }
    ctx.setLineDash([]);

    // Y axis label
    ctx.save();
    ctx.translate(12, padT + h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.fillText("cm", 0, 0);
    ctx.restore();

    // X label
    ctx.textAlign = "center";
    ctx.fillText("waktu →", padL + w / 2, padT + h + 14);

    // Plot lines
    const xAt = (i) => padL + (w * i) / Math.max(1, data.length - 1);
    const yAt = (v) => padT + h - ((v - min) / (max - min)) * h;

    function plot(key, color, width, opacity = 1) {
      ctx.strokeStyle = color;
      ctx.globalAlpha = opacity;
      ctx.lineWidth = width;
      ctx.lineJoin = "round";
      ctx.beginPath();
      let started = false;
      data.forEach((p, i) => {
        const v = p[key];
        if (v == null) { started = false; return; }
        const x = xAt(i), y = yAt(v);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    if (state.ultrasonicActive) plot("us",   "hsl(190 90% 55%)", 1.4, 0.85);
    if (state.infraredActive)   plot("ir",   "hsl(280 80% 65%)", 1.4, 0.85);
    if (state.filterActive)     plot("filt", "hsl(45 95% 60%)",  2.4);
  }

  // ===================== RENDER FUNCTIONS ========================
  function renderConnState() {
    // pills
    const pm = $("pill-mqtt");
    pm.classList.toggle("pill-on",  state.mqttConnected);
    pm.classList.toggle("pill-off", !state.mqttConnected);
    $("pill-mqtt-text").textContent = state.mqttConnected ? "ON" : "OFF";

    const pe = $("pill-esp");
    pe.classList.toggle("pill-pri", state.espConnected);
    pe.classList.toggle("pill-err", !state.espConnected);
    $("pill-esp-text").textContent = state.espConnected ? `${state.rssi} dBm` : "OFF";

    // MQTT badge + button
    const mb = $("mqtt-badge");
    mb.className = "badge " + (state.mqttConnected ? "badge-emerald" : "badge-muted");
    mb.textContent = state.mqttConnected ? "TERSAMBUNG" : "TERPUTUS";
    $("btn-connect-text").textContent = state.mqttConnected ? "PUTUSKAN KONEKSI" : "SAMBUNGKAN KE BROKER";

    // Sensor / filter / start buttons enabled state
    $("btn-us").disabled     = !state.espConnected;
    $("btn-ir").disabled     = !state.espConnected;
    $("btn-filter").disabled = !(state.espConnected && (state.ultrasonicActive || state.infraredActive));
    $("btn-start").disabled  = !state.espConnected || state.samplingActive;
  }

  function renderLive() {
    // Ultrasonic
    const usOn = state.ultrasonicActive && state.espConnected;
    $("us-val").textContent = usOn ? fmt(state.ultrasonicRaw, 1) : "--";
    $("us-badge").className = "badge " + (usOn ? "badge-cyan" : "badge-muted");
    $("us-badge").textContent = usOn ? "AKTIF" : "OFF";
    $("us-ico").className = "ico-box " + (usOn ? "ico-cyan ico-pulse" : "ico-muted");
    $("us-stream").classList.toggle("hide", !usOn);
    document.querySelector("#us-val").parentElement.classList.toggle("us-on", usOn);
    $("btn-us-text").textContent = usOn ? "MATIKAN HC-SR04" : "AKTIFKAN HC-SR04";
    $("btn-us").classList.toggle("btn-destructive", usOn);
    $("btn-us").classList.toggle("btn-secondary", !usOn);

    // Infrared
    const irOn = state.infraredActive && state.espConnected;
    $("ir-val").textContent = irOn ? fmt(state.infraredRaw, 1) : "--";
    $("ir-badge").className = "badge " + (irOn ? "badge-purple" : "badge-muted");
    $("ir-badge").textContent = irOn ? "AKTIF" : "OFF";
    $("ir-ico").className = "ico-box " + (irOn ? "ico-purple ico-pulse" : "ico-muted");
    $("ir-stream").classList.toggle("hide", !irOn);
    document.querySelector("#ir-val").parentElement.classList.toggle("ir-on", irOn);
    const v = irOn ? Math.max(0.4, Math.min(3.1, 3.1 - ((state.infraredRaw - 10) / 70) * 2.7)) : 0;
    $("ir-volt").textContent = `${v.toFixed(2)}V`;
    $("btn-ir-text").textContent = irOn ? "MATIKAN GP2Y0A21" : "AKTIFKAN GP2Y0A21";
    $("btn-ir").classList.toggle("btn-destructive", irOn);
    $("btn-ir").classList.toggle("btn-secondary", !irOn);

    // Filter
    const haveSensor = usOn || irOn;
    const fltOn = state.filterActive && state.espConnected && haveSensor;
    $("flt-badge").className = "badge " + (fltOn ? "badge-primary" : "badge-muted");
    $("flt-badge").textContent = fltOn ? "FILTER ON" : "FILTER OFF";
    $("flt-ico").className = "ico-box " + (fltOn ? "ico-primary ico-pulse" : "ico-muted");
    $("btn-filter-text").textContent = fltOn ? "DISABLE KALMAN FILTER" : "ENABLE KALMAN FILTER";
    $("btn-filter").classList.toggle("btn-destructive", fltOn);
    $("btn-filter").disabled = !(state.espConnected && haveSensor);

    // Fusion pill
    const fusionText = (usOn && irOn) ? "FUSI 2 SENSOR"
                     : usOn ? "HC-SR04 ONLY"
                     : irOn ? "GP2Y0A21 ONLY" : "TIDAK ADA INPUT";
    $("fusion-pill").textContent = fusionText;

    // Reading tiles
    const usTile  = document.querySelector('.reading-tile[data-color="cyan"] .rt-val');
    const irTile  = document.querySelector('.reading-tile[data-color="purple"] .rt-val');
    const fltTile = document.querySelector('.reading-tile.highlight .rt-val');
    $("rt-us").textContent  = usOn ? fmt(state.ultrasonicRaw, 1) : "--";
    $("rt-ir").textContent  = irOn ? fmt(state.infraredRaw, 1)   : "--";
    $("rt-flt").textContent = fltOn ? fmt(state.filteredDistance, 2) : "--";
    usTile.classList.toggle("active",  usOn);
    irTile.classList.toggle("active",  irOn);
    fltTile.classList.toggle("active", fltOn);

    // Threshold "current"
    const ref = haveSensor ? (fltOn ? state.filteredDistance : (usOn ? state.ultrasonicRaw : state.infraredRaw)) : null;
    $("thr-current").textContent = ref == null ? "—" : `${ref.toFixed(1)} cm`;
    const thrSafe = ref == null || ref >= state.brakeThreshold;
    $("thr-status").className = thrSafe ? "ok" : "warn";
    $("thr-status").textContent =
      ref == null ? "menunggu data…" :
      thrSafe ? "AMAN" : "PENGEREMAN!";
    $("thr-badge").className = "badge " + (thrSafe ? "badge-emerald" : "badge-destructive");
    $("thr-badge").textContent = thrSafe ? "AMAN" : "BAHAYA";

    // Motor
    const motorOn = state.espConnected && haveSensor && state.motorPWM > 0;
    $("motor-badge").className = "badge " + (motorOn ? "badge-primary" : "badge-muted");
    $("motor-badge").textContent = motorOn ? "RUNNING" : "STOPPED";
    $("motor-ico").className = "ico-box " + (motorOn ? "ico-primary" : "ico-muted");
    $("motor-pwm").innerHTML = `${state.motorPWM.toFixed(0)}<span>%</span>`;
    $("motor-pwm").className = "big-num " + (motorOn ? "primary" : "muted");
    $("motor-rpm").innerHTML = `${state.motorRPM}<span>RPM</span>`;
    $("motor-rpm").className = "big-num " + (motorOn ? "primary" : "muted");
    $("motor-bar").style.width = `${state.motorPWM}%`;
    $("motor-wheel").classList.toggle("on", motorOn);
    const mInfo = $("motor-info");
    mInfo.className = "info-strip " + (motorOn ? "on" : "");
    $("motor-info-text").textContent =
      !state.espConnected ? "ESP-12F belum terhubung." :
      !haveSensor ? "Aktifkan sensor untuk memulai kontrol motor adaptif." :
      motorOn ? `IRLZ44N gate aktif — PWM ${state.motorPWM.toFixed(0)}% mengontrol motor secara otomatis berdasarkan jarak.`
              : "Motor berhenti — pengereman darurat aktif atau jarak < threshold.";

    // Saklar Pemutus
    const armed = state.espConnected && haveSensor;
    const sakBadge = $("saklar-badge");
    sakBadge.className = "badge-sm " + (state.relayActive ? "badge-destructive" : armed ? "badge-emerald" : "badge-muted");
    sakBadge.textContent = !armed ? "STANDBY" : state.relayActive ? "OPEN" : "CLOSED";
    $("saklar-ico").className = "ico-box-sm " + (state.relayActive ? "ico-destructive" : "ico-muted");
    const saklarTile = $("saklar-tile");
    saklarTile.classList.toggle("danger", state.relayActive);
    const sakState = $("saklar-state");
    sakState.textContent = state.relayActive ? "TERPUTUS" : "TERHUBUNG";
    sakState.className = "st-val " + (state.relayActive ? "danger" : armed ? "emerald" : "muted");
    $("saklar-desc").textContent = state.relayActive
      ? "Aliran daya ke motor diputus oleh relay (mode pengereman darurat)."
      : "Aliran daya ke motor tersambung normal.";

    // LED Indikator
    const ledBadge = $("led-badge");
    ledBadge.className = "badge-sm " + (state.relayActive ? "badge-amber" : "badge-muted");
    ledBadge.textContent = state.relayActive ? "ON" : "OFF";
    $("led-ico").className = "ico-box-sm " + (state.relayActive ? "ico-amber" : "ico-muted");
    const ledTile = $("led-tile");
    ledTile.classList.toggle("amber", state.relayActive);
    const ledState = $("led-state");
    ledState.textContent = state.relayActive ? "MENYALA" : "MATI";
    ledState.className = "st-val " + (state.relayActive ? "amber" : "muted");
    $("led-desc").textContent = state.relayActive
      ? "LED peringatan menyala — sistem dalam kondisi pengereman darurat."
      : "LED dalam keadaan padam — kondisi sistem normal.";

    const relayInfo = $("relay-info");
    relayInfo.className = "info-strip " + (state.relayActive ? "danger" : "");
    $("relay-info-text").textContent =
      !armed ? "Sistem standby — menunggu data sensor untuk mulai memantau jarak aman."
      : state.relayActive
        ? `Pengereman darurat — jarak ${state.filteredDistance.toFixed(1)} cm < threshold ${state.brakeThreshold} cm. Saklar pemutus terbuka, LED indikator menyala.`
        : `Sistem normal — jarak ${state.filteredDistance.toFixed(1)} cm > threshold ${state.brakeThreshold} cm. Motor diizinkan berputar.`;

    // Sampling top stat
    $("smp-total").textContent = state.samples.length;
    $("smp-status").textContent = state.samplingActive ? "RECORDING" : "IDLE";
    $("smp-status").className = "big-num " + (state.samplingActive ? "primary" : "muted");

    // Settings status
    const setDot = $("set-dot");
    setDot.classList.toggle("active", fltOn);
    $("set-status").textContent =
      !state.espConnected ? "ESP-12F belum terhubung" :
      !state.filterActive ? "Filter belum diaktifkan di Dashboard" :
      "Filter ON — perubahan parameter berlaku langsung";

    // Header connection
    $("pill-esp-text").textContent = state.espConnected ? `${state.rssi} dBm` : "OFF";

    drawChart();
  }

  function renderSamplingUI() {
    const active = state.samplingActive;
    $("btn-start").classList.toggle("hide", active);
    $("btn-stop").classList.toggle("hide", !active);
    $("btn-export").disabled = state.samples.length === 0;
    $("btn-clear").disabled  = state.samples.length === 0 || active;
    $("btn-start").disabled  = !state.espConnected;
  }

  function renderSamplingTable() {
    const tbody = $("smp-tbody");
    if (state.samples.length === 0) {
      tbody.innerHTML = `<tr class="empty"><td colspan="8">Belum ada data — mulai sampling untuk mengisi tabel.</td></tr>`;
      return;
    }
    tbody.innerHTML = state.samples
      .slice(0, 200)
      .map((r) => `
        <tr>
          <td>${r.id}</td>
          <td>${tsHHMMSS(r.ts)}.${String(r.ts % 1000).padStart(3, "0")}</td>
          <td>${r.ultrasonic.toFixed(2)}</td>
          <td>${r.infrared.toFixed(2)}</td>
          <td>${r.filtered.toFixed(2)}</td>
          <td>${r.pwm.toFixed(1)}</td>
          <td>${r.rpm}</td>
          <td class="${r.relay ? "relay-on" : "relay-off"}">${r.relay ? "TRIPPED" : "ARMED"}</td>
        </tr>
      `)
      .join("");
  }

  function renderAll() {
    renderConnState();
    renderLive();
    renderSamplingUI();
    renderSamplingTable();
  }

  // ===================== INIT ==================================
  syncKalmanInputs();
  updateApplyState();
  pushLog("ok", "INIT", "SODAMCS Dashboard siap — masukkan kredensial MQTT untuk mulai");
  renderAll();

  // Re-draw chart on resize
  window.addEventListener("resize", drawChart);
})();
