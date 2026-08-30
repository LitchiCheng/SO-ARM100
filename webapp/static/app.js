/* SO-ARM100 Web 控制台前端：WS 遥测 + REST 指令 + canvas 可视化（无依赖） */

const JOINTS = [
  { id: 1, name: "shoulder_pan" },
  { id: 2, name: "shoulder_lift" },
  { id: 3, name: "elbow_flex" },
  { id: 4, name: "wrist_flex" },
  { id: 5, name: "wrist_roll" },
  { id: 6, name: "gripper" },
];
const HISTORY_LEN = 300;          // 60s @ 5Hz
const PALETTE = ["#4c9aff", "#46d16a", "#f5a524", "#e879a0", "#9a7bff", "#5fd3d3"];

// 与 webapp/arm.py WRITABLE_CONFIG 保持一致（name, min, max）
const WRITABLE = [
  ["Min_Angle_Limit", 0, 4095], ["Max_Angle_Limit", 0, 4095],
  ["Max_Temperature_Limit", 30, 99], ["Max_Voltage_Limit", 8, 16],
  ["Min_Voltage_Limit", 6, 15], ["Max_Torque_Limit", 0, 100],
  ["Minimum_Startup_Force", 0, 1000], ["CW_Dead_Zone", 0, 30],
  ["CCW_Dead_Zone", 0, 30], ["Protection_Current", 0, 1000],
  ["Acceleration", 0, 255], ["Maximum_Acceleration", 0, 1000],
];

let telemetry = null;
let armed = false;
let limits = [];
const history = {};      // 当前位置历史（每关节环形缓冲）
const goalHistory = {};  // 下发/目标位置历史（可与当前对比收敛情况）

/* ---------------- API / toast ---------------- */

async function api(path, body) {
  const opt = { method: body === undefined ? "GET" : "POST" };
  if (body !== undefined) {
    opt.headers = { "Content-Type": "application/json" };
    opt.body = JSON.stringify(body);
  }
  const r = await fetch(path, opt);
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    const msg = data.detail || `${r.status} ${r.statusText}`;
    if (r.status === 423) toast(msg, "warn");
    throw new Error(msg);
  }
  return data;
}

function toast(msg, kind = "") {
  const el = document.createElement("div");
  el.className = "toast " + kind;
  el.textContent = msg;
  document.getElementById("toast-wrap").appendChild(el);
  setTimeout(() => el.remove(), 4000);
}

/* ---------------- WebSocket 遥测 ---------------- */

let wsRetry = 1000;

function setConn(on) {
  const b = document.getElementById("conn-badge");
  b.textContent = on ? "● 已连接" : "● 已断开";
  b.className = "badge " + (on ? "on" : "off");
}

function wsConnect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`);
  ws.onopen = () => { setConn(true); wsRetry = 1000; };
  ws.onmessage = (e) => { telemetry = JSON.parse(e.data); scheduleRender(); };
  ws.onclose = () => { setConn(false); setTimeout(wsConnect, wsRetry = Math.min(wsRetry * 2, 5000)); };
  ws.onerror = () => ws.close();
}

/* ---------------- 渲染（rAF 节流） ---------------- */

let renderQueued = false;
function scheduleRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(() => { renderQueued = false; render(); });
}

function render() {
  const t = telemetry;
  if (!t || !Array.isArray(t.joints)) return;
  document.getElementById("err-badge").classList.toggle("hidden", !(t.errors > 0));
  if (!t.joints.length) return;
  pushHistory(t);
  if (window.Arm3D && window.Arm3D.setTelemetry) window.Arm3D.setTelemetry(t);
  drawSpark();
  renderCards(t);
  renderSliderVals(t);
  const allFree = t.joints.every(j => j.torque === 0);
  document.getElementById("free-badge").classList.toggle("hidden", !allFree);
}

/* ---------------- 滑杆控制 ---------------- */

function buildSliders() {
  const wrap = document.getElementById("joint-sliders");
  wrap.innerHTML = "";
  JOINTS.forEach((jt, k) => {
    const lim = limits[k];
    const lo = lim ? lim.lower_deg : -180, hi = lim ? lim.upper_deg : 180;
    // 固件 Min/Max_Angle_Limit 窗口比滑杆范围更窄时，目标会被电机钳位（显示琥珀色提示）
    const hwNarrow = lim && lim.hw_lower_deg != null &&
      (lim.hw_lower_deg > lo + 1 || lim.hw_upper_deg < hi - 1);
    const row = document.createElement("div");
    row.className = "joint-row";
    row.innerHTML = `
      <span class="jname"><b>J${jt.id}</b> ${jt.name}</span>
      <span class="slider-wrap">
        <input type="range" id="sl-${jt.id}" min="${lo}" max="${hi}" step="0.5" value="0">
        <span class="slider-lim"><span>${lo.toFixed(1)}°</span><span>${hi.toFixed(1)}°</span></span>
      </span>
      <span class="jvals"><span class="target" id="tgt-${jt.id}">目标 —</span>
      <span class="cur" id="cur-${jt.id}">当前 —</span>
      <span class="hwl" id="hwl-${jt.id}" ${hwNarrow ? "" : "hidden"}>${hwNarrow ? `固件限位 ${lim.hw_lower_deg.toFixed(1)}°~${lim.hw_upper_deg.toFixed(1)}°` : ""}</span>
      <span class="zero-row"><button id="zero-${jt.id}" class="mini-btn" title="标零：把当前位置设为该关节的新 0°（不产生运动，请确认关节已静止）">⊙ 标零</button>
      <span class="zero-flag" id="zflag-${jt.id}" title="已标零：当前 0° = 标零时的位置" ${lim && lim.zeroed_at ? "" : "hidden"}>已标零</span></span></span>`;
    wrap.appendChild(row);
    document.getElementById(`zero-${jt.id}`).onclick = () => doZero(jt.id);
    const sl = row.querySelector("input");
    setSliderFill(sl, lo, hi);
    sl.addEventListener("input", () => {
      document.getElementById(`tgt-${jt.id}`).textContent = `目标 ${(+sl.value).toFixed(1)}°`;
      setSliderFill(sl, lo, hi);
    });
    sl.addEventListener("change", async () => {
      const deg = +sl.value;
      if (!armed) { toast("未允许运动：请先打开「允许运动」", "warn"); return; }
      try {
        const r = await api(`/api/joints/${jt.id}/position`, { deg });
        toast(`J${jt.id} → 目标 ${r.clamped_deg}°（raw ${r.raw}）`);
      } catch (e) { toast(e.message, "err"); }
    });
  });
}

function renderSliderVals(t) {
  for (const j of t.joints) {
    const el = document.getElementById(`cur-${j.id}`);
    if (el) el.textContent = `当前 ${j.pos_deg != null ? j.pos_deg.toFixed(1) : "—"}°`;
  }
}

// 滑杆已走部分着色（accent → border 渐变，跟随当前值）
function setSliderFill(sl, lo, hi) {
  const v = +sl.value;
  const pct = hi > lo ? Math.max(0, Math.min(100, ((v - lo) / (hi - lo)) * 100)) : 0;
  sl.style.background = `linear-gradient(90deg, var(--accent) 0%, var(--accent) ${pct}%, var(--border) ${pct}%, var(--border) 100%)`;
}

// 重新拉取限位（含标零后的平移窗口/固件窗口/零位标记）并重建滑杆
async function refreshLimits() {
  try {
    limits = await (await fetch("/api/limits")).json();
    buildSliders();
  } catch (e) { toast("刷新限位失败: " + e.message, "err"); }
}

/* ---------------- 标零（单关节：当前位置 = 0°） ---------------- */

async function doZero(mid) {
  const j = telemetry ? telemetry.joints[mid - 1] : null;
  const cur = j && j.pos_deg != null ? j.pos_deg.toFixed(1) : "—";
  if (j && j.moving) { toast(`J${mid} 正在运动中，请等它静止后再标零`, "warn"); return; }
  if (!confirm(
    `J${mid} 标零：把当前位置（显示 ${cur}°）设为该关节的新 0°。\n\n` +
    `· 只改该关节 Offset 寄存器，不产生运动（已确认关节静止）\n` +
    `· 该关节的显示/滑杆/限位窗口此后相对新零位\n` +
    `· 该关节固件限位寄存器将重置为全范围（0~4095），需要时请在配置页重设\n` +
    `· 3D 模型 / MuJoCo 仿真对齐不受影响\n` +
    `· examples 脚本按出厂零位换算，标零后运行请留意\n\n` +
    `确认标零？`
  )) return;
  try {
    const r = await api(`/api/joints/${mid}/zero`, { confirm: true });
    toast(`J${mid} 已标零：${r.display_deg_before}° → 0°（滑杆窗口 ${r.window_deg[0].toFixed(1)}°~${r.window_deg[1].toFixed(1)}°）`);
    await refreshLimits();
  } catch (e) { toast(e.message, "err"); }
}

/* ---------------- 遥测卡片 ---------------- */

function buildCards() {
  const wrap = document.getElementById("joint-cards");
  JOINTS.forEach(jt => {
    const card = document.createElement("div");
    card.className = "card";
    card.innerHTML = `
      <div class="card-head">J${jt.id} ${jt.name}
        <span class="chips">
          <span class="chip" id="chp-${jt.id}">torque —</span>
          <span class="chip" id="chm-${jt.id}">静止</span>
        </span>
      </div>
      <div class="big" id="big-${jt.id}">—</div>
      <div class="raw" id="raw-${jt.id}">raw — / goal —</div>
      <div class="meter-row"><span class="lbl">温度</span><div class="meter"><div id="mt-${jt.id}"></div></div><span class="val" id="mv-${jt.id}">—</span></div>
      <div class="meter-row"><span class="lbl">电压</span><div class="meter"><div id="mv2-${jt.id}"></div></div><span class="val" id="vv-${jt.id}">—</span></div>
      <div class="meter-row"><span class="lbl">负载</span><div class="meter center"><div id="ml-${jt.id}"></div></div><span class="val" id="lv-${jt.id}">—</span></div>
      <div class="meter-row"><span class="lbl">电流</span><div class="meter"><div id="mc-${jt.id}"></div></div><span class="val" id="cv-${jt.id}">—</span></div>`;
    wrap.appendChild(card);
  });
}

function setMeter(id, frac) {
  document.getElementById(id).style.width = Math.max(0, Math.min(100, frac * 100)) + "%";
}

function renderCards(t) {
  for (const j of t.joints) {
    const id = j.id;
    document.getElementById(`big-${id}`).textContent =
      j.pos_deg != null ? `${j.pos_deg.toFixed(1)}°` : "—";
    document.getElementById(`raw-${id}`).textContent =
      `raw ${j.pos_raw ?? "—"} / goal ${j.goal_deg != null ? j.goal_deg.toFixed(1) + "°" : "—"}`;

    const chp = document.getElementById(`chp-${id}`);
    chp.textContent = "torque " + (j.torque === 1 ? "使能" : j.torque === 2 ? "阻尼" : j.torque === 0 ? "关闭" : "—");
    chp.className = "chip " + (j.torque === 1 ? "on" : j.torque === 2 ? "moving" : j.torque === 0 ? "off" : "");
    const chm = document.getElementById(`chm-${id}`);
    chm.textContent = j.moving ? "运动中" : "静止";
    chm.className = "chip" + (j.moving ? " moving" : "");

    setMeter(`mt-${id}`, (j.temp ?? 0) / 70);
    const mv = document.getElementById(`mv-${id}`);
    mv.textContent = j.temp != null ? j.temp + "°C" : "—";
    mv.className = "val " + (j.temp == null ? "" : j.temp > 50 ? "temp-hot" : j.temp >= 40 ? "temp-warn" : "temp-ok");

    setMeter(`mv2-${id}`, (j.voltage ?? 0) / 16);
    document.getElementById(`vv-${id}`).textContent = j.voltage != null ? j.voltage.toFixed(1) + " V" : "—";

    const load = j.load ?? 0;
    const lw = Math.min(50, Math.abs(load) / 2);
    const lel = document.getElementById(`ml-${id}`);
    lel.style.width = lw + "%";
    lel.style.left = load < 0 ? (50 - lw) + "%" : "50%";
    lel.style.background = "var(--amber)";
    document.getElementById(`lv-${id}`).textContent = j.load != null ? load : "—";

    setMeter(`mc-${id}`, (j.current ?? 0) / 1000);
    document.getElementById(`cv-${id}`).textContent = j.current != null ? j.current : "—";
  }
}

/* ---------------- 手臂 3D 显示（arm3d.js / three.js，MJCF 同源运动学） ---------------- */

// 2D 示意已替换为 webapp/static/arm3d.js（ES module，index.html 中 importmap 引入）：
// 真实 STL 网格 + trs MJCF 关节链，遥测经 window.Arm3D.setTelemetry(t) 注入（见 render()）。

/* ---------------- 位置历史 sparkline ---------------- */

const sparkCanvas = document.getElementById("spark-canvas");
const sctx = sparkCanvas.getContext("2d");

function pushHistory(t) {
  for (const j of t.joints) {
    if (j.pos_deg == null) continue;
    const h = history[j.id] || (history[j.id] = []);
    h.push(j.pos_deg);
    if (h.length > HISTORY_LEN) h.shift();
    const g = goalHistory[j.id] || (goalHistory[j.id] = []);
    g.push(j.goal_deg);  // 可为 null（读失败），绘制时断开
    if (g.length > HISTORY_LEN) g.shift();
  }
}

// 画一条曲线：vals 中 null 段断开；dashed 时虚线
function strokeSeries(ctx, vals, X, Y, color, dashed) {
  ctx.strokeStyle = color;
  ctx.lineWidth = dashed ? 1.2 : 1.6;
  if (dashed) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  let pen = false;
  vals.forEach((v, i) => {
    if (v == null) { pen = false; return; }
    if (!pen) { ctx.moveTo(X(i), Y(v)); pen = true; }
    else ctx.lineTo(X(i), Y(v));
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawSpark() {
  // 响应式：按容器 CSS 宽度 × dpr 重设画布（限制 dpr ≤ 2）
  const cssW = sparkCanvas.clientWidth || sparkCanvas.width;
  const cssH = sparkCanvas.clientHeight || 260;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const W = Math.round(cssW * dpr), H = Math.round(cssH * dpr);
  if (sparkCanvas.width !== W || sparkCanvas.height !== H) {
    sparkCanvas.width = W;
    sparkCanvas.height = H;
  }
  sctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  sctx.clearRect(0, 0, cssW, cssH);
  const laneH = cssH / 6;
  JOINTS.forEach((jt, k) => {
    const y0 = k * laneH, y1 = y0 + laneH;
    sctx.strokeStyle = "#1a1f29";
    sctx.lineWidth = 1;
    sctx.beginPath(); sctx.moveTo(0, y1 - 0.5); sctx.lineTo(cssW, y1 - 0.5); sctx.stroke();
    sctx.fillStyle = "#8b8d98";
    sctx.font = "11px monospace";
    sctx.fillText(`J${jt.id}`, 6, y0 + 16);

    const h = history[jt.id];
    const g = goalHistory[jt.id];
    const lim = limits[k];
    if (!lim || !h || h.length < 2) return;
    const lo = lim.lower_deg - 10, hi = lim.upper_deg + 10;
    const X = i => 54 + ((HISTORY_LEN - h.length + i) / (HISTORY_LEN - 1)) * (cssW - 74);
    const Y = v => y1 - 6 - ((Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo)) * (laneH - 12);

    if (g && g.length >= 2) strokeSeries(sctx, g, X, Y, PALETTE[k] + "73", true); // 下发（虚线）
    strokeSeries(sctx, h, X, Y, PALETTE[k], false);                                // 当前（实线）
  });
}

/* ---------------- 配置 tabs ---------------- */

function bindTabs() {
  document.querySelectorAll(".tab").forEach(btn => {
    btn.onclick = () => {
      document.querySelectorAll(".tab").forEach(b => b.classList.toggle("active", b === btn));
      document.querySelectorAll(".tab-body").forEach(b =>
        b.classList.toggle("hidden", b.id !== "tab-" + btn.dataset.tab));
    };
  });
}

function buildPidTab() {
  const body = document.getElementById("tab-pid");
  body.innerHTML = `
    <div class="cfg-toolbar">
      <button id="pid-read-all">读取全部</button>
      <span class="hint">P/D/I ∈ [0,255]；写入需确认并回读</span>
    </div>
    <table class="cfg-table">
      <thead><tr><th>关节</th><th>P</th><th>D</th><th>I</th><th></th><th>调参</th><th>回读</th></tr></thead>
      <tbody id="pid-rows"></tbody>
    </table>`;
  const tb = document.getElementById("pid-rows");
  JOINTS.forEach(jt => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>J${jt.id} ${jt.name}</td>
      <td><input type="number" id="pid${jt.id}-p" min="0" max="255"></td>
      <td><input type="number" id="pid${jt.id}-d" min="0" max="255"></td>
      <td><input type="number" id="pid${jt.id}-i" min="0" max="255"></td>
      <td><button id="pid${jt.id}-write">写入</button></td>
      <td><button id="pid${jt.id}-tune" title="步进响应法自动调 P/D（关节将往复小幅运动约 1~2 分钟）">调参</button></td>
      <td class="rb" id="pid${jt.id}-rb">—</td>`;
    tb.appendChild(tr);
  });
  JOINTS.forEach(jt => {
    document.getElementById(`pid${jt.id}-tune`).onclick = () => startTune(jt.id);
  });
  document.getElementById("pid-read-all").onclick = async () => {
    for (const jt of JOINTS) {
      try {
        const r = await api(`/api/joints/${jt.id}/pid`);
        document.getElementById(`pid${jt.id}-p`).value = r.p;
        document.getElementById(`pid${jt.id}-d`).value = r.d;
        document.getElementById(`pid${jt.id}-i`).value = r.i;
        document.getElementById(`pid${jt.id}-rb`).textContent = `P=${r.p} D=${r.d} I=${r.i}`;
      } catch (e) { toast(`J${jt.id} 读取 PID 失败: ${e.message}`, "err"); }
    }
  };
  JOINTS.forEach(jt => {
    document.getElementById(`pid${jt.id}-write`).onclick = async () => {
      const p = +document.getElementById(`pid${jt.id}-p`).value;
      const d = +document.getElementById(`pid${jt.id}-d`).value;
      const i = +document.getElementById(`pid${jt.id}-i`).value;
      if (!confirm(`写入 J${jt.id} PID: P=${p} D=${d} I=${i}？`)) return;
      try {
        const r = await api(`/api/joints/${jt.id}/pid`, { p, i, d, confirm: true });
        document.getElementById(`pid${jt.id}-rb`).textContent =
          `P=${r.readback.p} D=${r.readback.d} I=${r.readback.i}`;
        toast(`J${jt.id} PID 写入成功`);
      } catch (e) { toast(e.message, "err"); }
    };
  });
}

function buildOffsetTab() {
  const body = document.getElementById("tab-offset");
  body.innerHTML = `
    <div class="cfg-toolbar">
      <button id="off-read-all">读取全部</button>
      <span class="hint">自动回中 = setOffsetCurrent：约 2 秒，会把角度限位重置为 0/4095，
      请先把该关节摆到「当前位置 = 零点」姿态再执行</span>
    </div>
    <table class="cfg-table">
      <thead><tr><th>关节</th><th>当前</th><th>设为</th><th>操作</th><th>回读</th></tr></thead>
      <tbody id="off-rows"></tbody>
    </table>`;
  const tb = document.getElementById("off-rows");
  JOINTS.forEach(jt => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>J${jt.id} ${jt.name}</td>
      <td id="off${jt.id}-cur">—</td>
      <td><input type="number" id="off${jt.id}-set" min="0" max="4095"></td>
      <td><button id="off${jt.id}-write">写入</button>
          <button id="off${jt.id}-home">自动回中</button></td>
      <td class="rb" id="off${jt.id}-rb">—</td>`;
    tb.appendChild(tr);
  });
  document.getElementById("off-read-all").onclick = async () => {
    for (const jt of JOINTS) {
      try {
        const r = await api(`/api/joints/${jt.id}/offset`);
        document.getElementById(`off${jt.id}-cur`).textContent = r.offset;
      } catch (e) { toast(`J${jt.id} 读取 offset 失败: ${e.message}`, "err"); }
    }
  };
  JOINTS.forEach(jt => {
    document.getElementById(`off${jt.id}-write`).onclick = async () => {
      const v = +document.getElementById(`off${jt.id}-set`).value;
      if (!confirm(`写入 J${jt.id} Offset = ${v}？`)) return;
      try {
        const r = await api(`/api/joints/${jt.id}/offset`, { offset: v, confirm: true });
        document.getElementById(`off${jt.id}-rb`).textContent = String(r.readback);
        document.getElementById(`off${jt.id}-cur`).textContent = r.readback;
        toast(`J${jt.id} Offset 写入成功`);
      } catch (e) { toast(e.message, "err"); }
    };
    document.getElementById(`off${jt.id}-home`).onclick = async () => {
      if (!confirm(`J${jt.id} 自动回中：约 2 秒，且会把角度限位重置为 0/4095。确认关节已处于零点姿态？`)) return;
      try {
        const r = await api(`/api/joints/${jt.id}/offset/home`, { confirm: true });
        document.getElementById(`off${jt.id}-rb`).textContent =
          `homing=${r.homing_offset} (limits 0/4095)`;
        toast(`J${jt.id} 自动回中完成`);
      } catch (e) { toast(e.message, "err"); }
    };
  });
}

function buildTableTab() {
  const body = document.getElementById("tab-table");
  body.innerHTML = `
    <div class="cfg-toolbar">
      <select id="tbl-motor"></select>
      <button id="tbl-read">读取全部</button>
      <span class="hint">控制表只读 dump（部分条目固件不支持会省略）</span>
    </div>
    <div class="readonly-table"><table class="cfg-table" id="tbl-out">
      <thead><tr><th>条目</th><th>值</th></tr></thead><tbody></tbody>
    </table></div>
    <hr style="border:none;border-top:1px solid var(--border);margin:14px 0">
    <div class="cfg-toolbar">
      <label>白名单写入：</label>
      <select id="tbl-wname"></select>
      <input type="number" id="tbl-wval">
      <button id="tbl-wwrite">写入</button>
      <span class="rb hint" id="tbl-wrb"></span>
    </div>`;
  const sel = document.getElementById("tbl-motor");
  JOINTS.forEach(jt => {
    const o = document.createElement("option");
    o.value = jt.id; o.textContent = `J${jt.id} ${jt.name}`;
    sel.appendChild(o);
  });
  const wsel = document.getElementById("tbl-wname");
  WRITABLE.forEach(([n, lo, hi]) => {
    const o = document.createElement("option");
    o.value = n; o.textContent = `${n} [${lo}..${hi}]`;
    wsel.appendChild(o);
  });
  document.getElementById("tbl-read").onclick = async () => {
    const mid = +sel.value;
    try {
      const r = await api(`/api/joints/${mid}/config`);
      const tb = document.querySelector("#tbl-out tbody");
      tb.innerHTML = "";
      for (const [k, v] of Object.entries(r)) {
        const tr = document.createElement("tr");
        tr.innerHTML = `<td>${k}</td><td>${v}</td>`;
        tb.appendChild(tr);
      }
      toast(`J${mid} 控制表已读取（${Object.keys(r).length} 项）`);
    } catch (e) { toast(e.message, "err"); }
  };
  document.getElementById("tbl-wwrite").onclick = async () => {
    const mid = +sel.value;
    const name = wsel.value;
    const val = +document.getElementById("tbl-wval").value;
    if (!confirm(`写入 J${mid} ${name} = ${val}？`)) return;
    try {
      const r = await api(`/api/joints/${mid}/config`, { name, value: val, confirm: true });
      document.getElementById("tbl-wrb").textContent = `回读: ${r.readback}`;
      toast(`J${mid} ${name} 写入成功`);
    } catch (e) { toast(e.message, "err"); }
  };
}

function buildBusTab() {
  const body = document.getElementById("tab-bus");
  body.innerHTML = `
    <div class="cfg-toolbar">
      <button id="bus-read">读取</button>
      <span class="hint">切换波特率 = 广播写电机寄存器 → 切 host 侧 → 读回校验；
      若失败总线将不可达，需用原波特率恢复</span>
    </div>
    <div id="bus-out"></div>
    <div class="cfg-toolbar">
      <label>切换到</label><select id="bus-index"></select>
      <button id="bus-apply">应用</button>
    </div>`;
  document.getElementById("bus-read").onclick = async () => {
    try {
      const r = await api("/api/baudrate");
      const sel = document.getElementById("bus-index");
      sel.innerHTML = "";
      for (const [idx, bps] of Object.entries(r.table)) {
        const o = document.createElement("option");
        o.value = idx;
        o.textContent = `索引 ${idx} = ${bps.toLocaleString()} baud${r.motor_baud_index == idx ? "（当前电机值）" : ""}`;
        sel.appendChild(o);
      }
      const mismatch = r.motor_baud_bps != null && r.motor_baud_bps !== r.port_baudrate;
      document.getElementById("bus-out").innerHTML = `
        <table class="cfg-table">
          <tr><td>Host 侧波特率</td><td>${r.port_baudrate?.toLocaleString() ?? "—"}</td></tr>
          <tr><td>电机寄存器（J1）</td><td>索引 ${r.motor_baud_index ?? "—"} = ${r.motor_baud_bps?.toLocaleString() ?? "—"} baud</td></tr>
          <tr><td>一致性</td><td class="${mismatch ? "mismatch" : "rb"}">
            ${mismatch ? "⚠ 不一致！通信可能异常" : "一致 ✓"}</td></tr>
        </table>`;
    } catch (e) { toast(e.message, "err"); }
  };
  document.getElementById("bus-apply").onclick = async () => {
    const idx = +document.getElementById("bus-index").value;
    if (!confirm(`切换波特率到索引 ${idx}？通信将短暂中断；若失败总线不可达，需用原波特率恢复。确认？`)) return;
    try {
      const r = await api("/api/baudrate", { index: idx, confirm: true });
      toast(`波特率已切换: ${r.port_baudrate.toLocaleString()}`);
      document.getElementById("bus-read").click();
    } catch (e) { toast("波特率切换失败: " + e.message, "err"); }
  };
}

/* ---------------- 全局控制 ---------------- */

async function doEstop() {
  try {
    await api("/api/estop", {});  // 必须传 body：api() 无 body 时发 GET，服务端只有 POST
    toast("E-STOP：全部关节扭矩已释放（自由状态）", "warn");
  } catch (e) { toast("E-STOP 失败: " + e.message, "err"); }
}

// E-STOP 后恢复：使能全部关节扭矩（mode 1；手臂会朝各关节当前 Goal_Position 运动）
async function doTorqueOn() {
  try {
    await api("/api/torque/all", { mode: 1 });
    toast("全部关节扭矩已使能");
  } catch (e) { toast("扭矩使能失败: " + e.message, "err"); }
}

// USB 插拔后串口句柄失效：重新打开串口并刷新限位/状态（不动作）
async function doReconnect(port) {
  const btn = document.getElementById("reconnect-btn");
  btn.disabled = true;
  try {
    const s = await api("/api/reconnect", port ? { port } : {});
    if (!s.connected) throw new Error("端口未打开（" + s.port + " 是否存在？）");
    toast(`重连成功（${s.port} @ ${s.port_baudrate?.toLocaleString()}）`);
    // 刷新 bus 徽标 / 解锁状态 / 滑杆（含实时固件限位）
    document.getElementById("bus-badge").textContent =
      "bus " + (s.port_baudrate != null ? s.port_baudrate.toLocaleString() : "—");
    const t = document.getElementById("arm-toggle");
    t.checked = !!s.armed;
    armed = !!s.armed;
    await refreshLimits();
  } catch (e) {
    toast("重连失败: " + e.message, "err");
    // 设备节点常被重新分配（ttyACM0 → ttyACM1）：询问新节点后重试一次
    const p = prompt("重连失败。USB 重新插入后设备节点可能已变化，\n输入新的设备节点（如 /dev/ttyACM1），留空取消：");
    if (p && p.trim()) doReconnect(p.trim());
  } finally {
    btn.disabled = false;
  }
}

/* ---------------- 点位（保存 / 应用 / 删除） ---------------- */

async function loadPoses() {
  const list = document.getElementById("pose-list");
  list.innerHTML = "";
  let poses = {};
  try { poses = await api("/api/poses"); } catch (e) { /* 服务未就绪 */ }
  for (const [name, p] of Object.entries(poses)) {
    const vals = p.deg || p.raw || [];
    const item = document.createElement("div");
    item.className = "pose-item";
    const valsTxt = vals.map((v, i) => `J${i + 1}=${(+v).toFixed(1)}°`).join("  ");
    item.innerHTML = `
      <span class="pname" title="${valsTxt}">${name}</span>
      <span class="pvals" title="${valsTxt}">${valsTxt}</span>
      <button class="p-apply">应用</button>
      <button class="p-del">删除</button>`;
    item.querySelector(".p-apply").onclick = async () => {
      if (!armed) { toast("未允许运动：请先打开「允许运动」", "warn"); return; }
      if (!confirm(`应用点位「${name}」：\n${valsTxt}\n\n全部 6 个关节将运动（限速 500 步/s）。请确认机械臂处于安全位置？`)) return;
      try {
        await api("/api/poses/apply", { name, confirm: true });
        toast(`点位「${name}」已下发`);
      } catch (e) { toast(e.message, "err"); }
    };
    item.querySelector(".p-del").onclick = async () => {
      if (!confirm(`删除点位「${name}」？`)) return;
      try {
        await api("/api/poses/delete", { name });
        loadPoses();
      } catch (e) { toast(e.message, "err"); }
    };
    list.appendChild(item);
  }
}

function bindPose() {
  document.getElementById("pose-save-btn").onclick = async () => {
    const name = prompt("给当前点位起个名字（1~20 字符）：");
    if (!name || !name.trim()) return;
    try {
      const r = await api("/api/poses", { name: name.trim() });
      toast(`点位「${name.trim()}」已保存`);
      loadPoses();
    } catch (e) { toast(e.message, "err"); }
  };
}

/* ---------------- PID 自动调参 ---------------- */

let tuneTimer = null;

async function startTune(mid) {
  if (!armed) { toast("未允许运动：请先打开「允许运动」", "warn"); return; }
  const step = 10;
  if (!confirm(
    `对 J${mid} 做 PID 自动调参：\n\n` +
    `· 关节会做 ±${step}° 的小幅往复运动，最多 4 轮，约 1~2 分钟\n` +
    `· 过程中其他运动指令会被拒绝（E-STOP 随时可用）\n` +
    `· 结束后写入测得的最优 P/D，I 保持原值，目标回写到起点\n\n` +
    `确认开始？`
  )) return;
  try {
    await api(`/api/joints/${mid}/pid/tune`, { confirm: true, step_deg: step });
    pollTune(mid);
  } catch (e) { toast(e.message, "err"); }
}

async function pollTune(mid) {
  const btn = document.getElementById(`pid${mid}-tune`);
  const tick = async () => {
    let st;
    try { st = await api(`/api/joints/${mid}/pid/tune`); } catch (e) { return; }
    if (btn) {
      btn.disabled = !["starting", "waiting", "tuning"].includes(st.state);
      btn.textContent = (["starting", "waiting", "tuning"].includes(st.state))
        ? `调参中 ${st.trial || ""}` : "调参";
    }
    if (st.state === "tuning" || st.state === "waiting" || st.state === "starting") {
      tuneTimer = setTimeout(tick, 1000);
    } else {
      clearTimeout(tuneTimer);
      tuneTimer = null;
      if (st.state === "done" && st.result) {
        const r = st.result;
        document.getElementById(`pid${mid}-p`).value = r.applied.p;
        document.getElementById(`pid${mid}-d`).value = r.applied.d;
        document.getElementById(`pid${mid}-i`).value = r.applied.i;
        document.getElementById(`pid${mid}-rb`).textContent =
          `P=${r.readback.p} D=${r.readback.d} I=${r.readback.i}（自动调参，原 P=${r.original.p} D=${r.original.d}）`;
        toast(`J${mid} 调参完成：P=${r.applied.p} D=${r.applied.d} I=${r.applied.i}（原 P=${r.original.p} D=${r.original.d}）`);
      } else if (st.state === "failed") {
        toast(`J${mid} 调参中止：${st.msg}`, "err");
      }
    }
  };
  tick();
}

function bindControls() {
  document.getElementById("estop-btn").onclick = doEstop;
  document.getElementById("torque-on-btn").onclick = doTorqueOn;
  document.getElementById("reconnect-btn").onclick = doReconnect;
  bindPose();
  document.addEventListener("keydown", e => {
    if (e.key === "Escape" && !/input|select|textarea/i.test(e.target.tagName)) doEstop();
  });

  document.getElementById("arm-toggle").onchange = async (e) => {
    try {
      const r = await api("/api/arm", { armed: e.target.checked });
      armed = r.armed;
      toast(armed ? "已允许运动（armed）" : "已禁止运动（disarmed）", armed ? "warn" : "");
    } catch (err) {
      e.target.checked = !armed;
      toast(err.message, "err");
    }
  };

  document.getElementById("speed-apply").onclick = async () => {
    const v = +document.getElementById("speed-input").value;
    try {
      await api("/api/speed", { speed: v });
      toast(`限速已应用: ${v} 步/s`);
    } catch (e) { toast(e.message, "err"); }
  };

  document.querySelectorAll(".preset-btn").forEach(btn => {
    btn.onclick = async () => {
      if (!armed) { toast("未允许运动：请先打开「允许运动」", "warn"); return; }
      const name = btn.dataset.preset;
      if (!confirm(`发送运动指令：预设 ${name}（限速 500 步/s）。请确认机械臂处于安全位置？`)) return;
      try {
        const r = await api("/api/presets", { name, confirm: true });
        toast(`预设 ${name} 已下发: ` + r.results.map(x => `J${x.id}=${x.clamped_deg}°`).join(" "));
      } catch (e) { toast(e.message, "err"); }
    };
  });
}

/* ---------------- 启动 ---------------- */

(async function init() {
  try {
    limits = await (await fetch("/api/limits")).json();
  } catch (e) { toast("无法获取限位: " + e.message, "err"); }
  buildSliders();
  buildCards();
  buildPidTab();
  buildOffsetTab();
  buildTableTab();
  buildBusTab();
  bindTabs();
  bindControls();
  loadPoses();
  try {
    const s = await api("/api/status");
    document.getElementById("bus-badge").textContent =
      "bus " + (s.port_baudrate != null ? s.port_baudrate.toLocaleString() : "—");
    const t = document.getElementById("arm-toggle");
    t.checked = !!s.armed;
    armed = !!s.armed;
  } catch (e) { /* 服务未就绪，WS 连接后仍会更新遥测 */ }
  wsConnect();
})();
