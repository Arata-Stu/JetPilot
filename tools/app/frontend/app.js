const state = {
  tab: "dashboard",
  config: null,
  tasks: [],
  rosbags: [],
  maps: [],
  selectedTaskId: null,
  terminalCollapsed: false,
  logText: "",
  stream: null,
  jetsonInspect: null,
};

const tabs = [
  ["dashboard", "Dashboard"],
  ["rosbags", "Rosbags"],
  ["map-builder", "Map Builder"],
  ["maps", "Maps"],
  ["jetson", "Jetson"],
  ["terminal", "Terminal"],
];

const $ = (id) => document.getElementById(id);
const esc = (value) =>
  String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
const js = (value) => JSON.stringify(String(value ?? ""));

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const contentType = response.headers.get("content-type") || "";
  const payload = contentType.includes("application/json")
    ? await response.json()
    : await response.text();
  if (!response.ok) {
    const message = typeof payload === "string" ? payload : payload.error || "request failed";
    throw new Error(message);
  }
  return payload;
}

function copyText(text) {
  navigator.clipboard.writeText(String(text ?? ""));
}

function fmtBytes(bytes) {
  const value = Number(bytes || 0);
  if (value < 1024) return `${value} B`;
  const units = ["KB", "MB", "GB", "TB"];
  let current = value / 1024;
  for (const unit of units) {
    if (current < 1024) return `${current.toFixed(current < 10 ? 1 : 0)} ${unit}`;
    current /= 1024;
  }
  return `${current.toFixed(0)} PB`;
}

function fmtTime(seconds) {
  if (!seconds) return "-";
  return new Date(seconds * 1000).toLocaleString();
}

function commandText(task) {
  return (task?.command || []).map((part) => JSON.stringify(part)).join(" ");
}

function trimTrailingSlash(value) {
  return String(value || "").replace(/\/+$/, "");
}

async function refreshAll() {
  const [config, tasks, rosbags, maps] = await Promise.all([
    api("/api/config"),
    api("/api/tasks"),
    api("/api/rosbags/local"),
    api("/api/maps/local"),
  ]);
  state.config = config;
  state.tasks = tasks.tasks || [];
  state.rosbags = rosbags.rosbags || [];
  state.maps = maps.maps || [];
  if (!state.selectedTaskId && state.tasks[0]) state.selectedTaskId = state.tasks[0].task_id;
  render();
}

function setTab(tab) {
  state.tab = tab;
  render();
}

function render() {
  const app = $("app");
  app.innerHTML = `
    <div class="app">
      <header class="topbar">
        <div class="brand"><strong>JetPilot Console</strong><span>local workflow manager</span></div>
        <nav class="nav">
          ${tabs
            .map(
              ([key, label]) =>
                `<button class="${state.tab === key ? "active" : ""}" onclick="setTab('${key}')">${label}</button>`,
            )
            .join("")}
        </nav>
        <div class="top-actions">
          <button onclick="refreshAll()">Refresh</button>
          <button class="ghost" onclick="toggleTerminal()">${state.terminalCollapsed ? "Show Log" : "Hide Log"}</button>
        </div>
      </header>
      <main class="content">${renderPage()}</main>
      ${renderTerminal()}
    </div>
  `;
  scrollLogToEnd();
}

function isEditingField() {
  return ["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName || "");
}

function updateLogOnly() {
  const log = $("task-log");
  if (log) {
    log.textContent = state.logText || "Select a task to view logs.";
    scrollLogToEnd();
  }
}

function renderPage() {
  if (state.tab === "rosbags") return renderRosbags();
  if (state.tab === "map-builder") return renderMapBuilder();
  if (state.tab === "maps") return renderMaps();
  if (state.tab === "jetson") return renderJetson();
  if (state.tab === "terminal") return renderTerminalPage();
  return renderDashboard();
}

function renderDashboard() {
  const running = state.tasks.filter((task) => ["queued", "running", "stopping"].includes(task.status));
  const completeMaps = state.maps.filter((item) => item.complete_runtime_bundle);
  const recentMaps = state.maps.slice(0, 3);
  const recentBags = state.rosbags.slice(0, 3);
  return `
    <div class="page">
      <div class="grid-3">
        ${metric("Running tasks", running.length, `${state.tasks.length} total task records`)}
        ${metric("Rosbags", state.rosbags.length, state.config ? state.config.record_root : "")}
        ${metric("Runtime maps", completeMaps.length, `${state.maps.length} map folders scanned`)}
      </div>
      <section class="panel">
        <div class="panel-header">
          <h2>Running Tasks</h2>
          <span class="spacer"></span>
          <button onclick="setTab('jetson')">Jetson</button>
          <button onclick="setTab('map-builder')">Map Builder</button>
          <button onclick="refreshAll()">Refresh</button>
        </div>
        <div class="panel-body">${renderTaskTable(running.length ? running : state.tasks.slice(0, 4))}</div>
      </section>
      <div class="grid-2">
        <section class="panel">
          <div class="panel-header"><h2>Recent Rosbags</h2><span class="spacer"></span><button onclick="setTab('rosbags')">Open</button></div>
          <div class="panel-body">${renderCompactList(recentBags, "rosbag")}</div>
        </section>
        <section class="panel">
          <div class="panel-header"><h2>Recent Maps</h2><span class="spacer"></span><button onclick="setTab('maps')">Open</button></div>
          <div class="panel-body">${renderCompactList(recentMaps, "map")}</div>
        </section>
      </div>
    </div>
  `;
}

function metric(label, value, sub) {
  return `
    <section class="panel metric">
      <div class="panel-body">
        <div class="metric-label">${esc(label)}</div>
        <div class="metric-value">${esc(value)}</div>
        <div class="metric-sub">${esc(sub)}</div>
      </div>
    </section>
  `;
}

function renderCompactList(items, kind) {
  if (!items.length) return `<div class="empty">No ${kind === "map" ? "maps" : "rosbags"} found.</div>`;
  return `
    <div class="mini-list">
      ${items
        .map((item) => {
          const ready = kind === "map" ? item.complete_runtime_bundle : true;
          return `
            <div class="mini-row">
              <div>
                <strong>${esc(item.name)}</strong>
                <div class="path" title="${esc(item.path)}">${esc(item.path)}</div>
              </div>
              <div class="actions">
                ${kind === "map" ? `<span class="status ${ready ? "success" : "failed"}">${ready ? "ready" : "incomplete"}</span>` : ""}
                <button onclick="copyText(${js(item.path)})">Copy</button>
                ${kind === "rosbag" ? `<button onclick="useRosbag(${js(item.path)})">Build</button>` : `<button onclick="fillTransferLocal(${js(item.path)})">Transfer</button>`}
              </div>
            </div>`;
        })
        .join("")}
    </div>
  `;
}

function renderMapBuilder() {
  return `
    <div class="page">
      <section class="panel">
        <div class="panel-header">
          <h2>VGL / VSLAM Build</h2>
          <span class="spacer"></span>
          <button onclick="fillMapDir()">Suggest Name</button>
        </div>
        <div class="panel-body">${renderMapBuildForm()}</div>
      </section>
      <div class="grid-3">
        ${stageCard("1", "Build", "cuVGL, cuVSLAM, snapshot", "Start VGL/VSLAM Build")}
        ${stageCard("2", "Edit", "HD map raster and browser editor", "Prepare Raster")}
        ${stageCard("3", "Review", "raceline and preview image", "Generate Preview")}
      </div>
    </div>
  `;
}

function stageCard(step, title, detail, action) {
  return `
    <section class="panel stage-card">
      <div class="panel-body">
        <div class="stage-step">${esc(step)}</div>
        <h3>${esc(title)}</h3>
        <p>${esc(detail)}</p>
        <span class="chip">${esc(action)}</span>
      </div>
    </section>
  `;
}

function renderMapBuildForm() {
  return `
    <div class="form-grid">
      <div class="field full">
        <label>Rosbag</label>
        <select id="build-rosbag">
          <option value="">Select rosbag</option>
          ${state.rosbags.map((bag) => `<option value="${esc(bag.path)}">${esc(bag.name)} - ${esc(bag.path)}</option>`).join("")}
        </select>
      </div>
      <div class="field full">
        <label>Output map directory</label>
        <input id="build-map-dir" placeholder="${esc((state.config?.map_root || "/workspaces/map") + "/course_a")}" />
      </div>
      <div class="field">
        <label>Mapping steps</label>
        <input id="build-steps" value="edex compute_poses cuvgl" />
      </div>
      <div class="field full">
        <label>Camera topic config</label>
        <input id="build-topic-config" placeholder="Default JetPilot vgl_camera_topics.yaml" />
      </div>
      <div class="actions full">
        <button class="primary" onclick="startMapBuild()">Start VGL/VSLAM Build</button>
        <button onclick="copyMapBuildCommand()">Copy Equivalent Command</button>
      </div>
    </div>
  `;
}

function renderRosbags() {
  return `
    <div class="page">
      <section class="panel">
        <div class="panel-header"><h2>Local Rosbags</h2><span class="spacer"></span><button onclick="refreshAll()">Scan</button></div>
        <div class="table-wrap">${state.rosbags.length ? rosbagTable() : `<div class="empty">No metadata.yaml files under ${esc(state.config?.record_root || "")}</div>`}</div>
      </section>
    </div>
  `;
}

function rosbagTable() {
  return `
    <table>
      <thead><tr><th>Name</th><th>Path</th><th>Size</th><th>Modified</th><th></th></tr></thead>
      <tbody>
        ${state.rosbags
          .map(
            (bag) => `
              <tr>
                <td>${esc(bag.name)}</td>
                <td><div class="path" title="${esc(bag.path)}">${esc(bag.path)}</div></td>
                <td>${fmtBytes(bag.size_bytes)}</td>
                <td>${fmtTime(bag.modified_at)}</td>
                <td class="actions">
                  <button onclick="copyText(${js(bag.path)})">Copy</button>
                  <button onclick="useRosbag(${js(bag.path)})">Use</button>
                </td>
              </tr>`,
          )
          .join("")}
      </tbody>
    </table>
  `;
}

function renderMaps() {
  return `
    <div class="page">
      <section class="panel">
        <div class="panel-header"><h2>Local Maps</h2><span class="spacer"></span><button onclick="refreshAll()">Scan</button></div>
        <div class="table-wrap">${state.maps.length ? mapTable() : `<div class="empty">No map folders under ${esc(state.config?.map_root || "")}</div>`}</div>
      </section>
    </div>
  `;
}

function mapTable() {
  return `
    <table>
      <thead><tr><th>Name</th><th>Path</th><th>Artifacts</th><th>Size</th><th></th></tr></thead>
      <tbody>
        ${state.maps
          .map((map) => {
            const artifactKeys = ["cuvgl_map", "cuvslam_map", "hd_map", "centerline_csv", "raceline_csv", "line_preview"];
            return `
              <tr>
                <td>${esc(map.name)}<br>${map.complete_runtime_bundle ? `<span class="status success">runtime ready</span>` : `<span class="status failed">incomplete</span>`}</td>
                <td><div class="path" title="${esc(map.path)}">${esc(map.path)}</div></td>
                <td><div class="chips">${artifactKeys
                  .map((key) => `<span class="chip ${map.artifacts[key]?.exists ? "ok" : "missing"}">${key}</span>`)
                  .join("")}</div></td>
                <td>${fmtBytes(map.size_bytes)}</td>
                <td class="actions">
                  <button onclick="copyText(${js(map.path)})">Copy</button>
                  <button onclick="runMapStage('prepare-hd-raster', ${js(map.path)})">Raster</button>
                  <button onclick="runMapStage('generate-raceline', ${js(map.path)})">Raceline</button>
                  <button onclick="runMapStage('generate-preview', ${js(map.path)})">Preview</button>
                  <button onclick="fillTransferLocal(${js(map.path)})">Transfer</button>
                </td>
              </tr>`;
          })
          .join("")}
      </tbody>
    </table>
  `;
}

function renderJetson() {
  return `
    <div class="page">
      <div class="grid-2">
        <section class="panel">
          <div class="panel-header"><h2>Connection</h2></div>
          <div class="panel-body">${renderJetsonTarget()}</div>
        </section>
        <section class="panel">
          <div class="panel-header"><h2>Remote State</h2><span class="spacer"></span><button onclick="copyJetsonInspect()">Copy SSH</button></div>
          <div class="panel-body">${renderJetsonSummary()}</div>
        </section>
      </div>
      <section class="panel">
        <div class="panel-header"><h2>Transfers</h2></div>
        <div class="panel-body">${renderJetsonTransfers()}</div>
      </section>
    </div>
  `;
}

function renderJetsonTarget() {
  const config = state.config || {};
  const host = state.jetsonInspect?.host || (config.jetson_ips || [])[0] || "";
  const user = state.jetsonInspect?.user || config.jetson_user || "tamiya";
  return `
    <div class="form-grid jetson-target">
      <div class="field">
        <label>Jetson host</label>
        <input id="jetson-host" value="${esc(host)}" />
      </div>
      <div class="field">
        <label>SSH user</label>
        <input id="jetson-user" value="${esc(user)}" />
      </div>
      <div class="field full">
        <label>Remote map root</label>
        <input id="jetson-map-root" value="${esc(config.jetson_map_root || "")}" />
      </div>
      <div class="field full">
        <label>Remote rosbag root</label>
        <input id="jetson-record-root" value="${esc(config.jetson_record_root || "")}" />
      </div>
      <div class="actions full">
        <button class="primary" onclick="inspectJetson()">Inspect Jetson</button>
        <button onclick="copyJetsonInspect()">Copy SSH</button>
      </div>
      <div class="quick-hosts full">
        ${(config.jetson_ips || [])
          .map((ip) => `<button class="ghost" onclick="setJetsonHost(${js(ip)})">${esc(ip)}</button>`)
          .join("")}
      </div>
    </div>
  `;
}

function renderJetsonSummary() {
  const result = state.jetsonInspect;
  if (!result) {
    return `
      <div class="remote-state">
        <div class="state-tile"><span>SSH</span><strong>not checked</strong></div>
        <div class="state-tile"><span>latest</span><strong>-</strong></div>
        <div class="state-tile"><span>maps</span><strong>-</strong></div>
        <div class="state-tile"><span>rosbags</span><strong>-</strong></div>
      </div>
    `;
  }
  const sections = parseJetsonOutput(result.output || "");
  const latest = firstContentLine(sections.latest) || "-";
  const maps = contentLines(sections.maps).length;
  const rosbags = contentLines(sections.rosbags).length;
  return `
    <div class="remote-state">
      <div class="state-tile ${result.ok ? "ok" : "bad"}"><span>SSH</span><strong>${result.ok ? "online" : "failed"}</strong></div>
      <div class="state-tile"><span>latest</span><strong title="${esc(latest)}">${esc(shortName(latest))}</strong></div>
      <div class="state-tile"><span>maps</span><strong>${esc(maps)}</strong></div>
      <div class="state-tile"><span>rosbags</span><strong>${esc(rosbags)}</strong></div>
    </div>
    ${sectionOutput("Disk", sections.disk)}
    ${sectionOutput("Maps", sections.maps)}
    ${sectionOutput("Rosbags", sections.rosbags)}
    ${result.error ? `<div class="notice">${esc(result.error)}</div>` : ""}
  `;
}

function renderJetsonTransfers() {
  const config = state.config || {};
  return `
    <div class="transfer-grid">
      <section class="transfer-card">
        <h3>Pull rosbags</h3>
        <div class="form-grid">
          <div class="field full"><label>From Jetson</label><input id="pull-remote" value="${esc(config.jetson_record_root || "")}" /></div>
          <div class="field full"><label>To notebook</label><input id="pull-local" value="${esc(config.record_root || "")}" /></div>
          <div class="actions full">
            <button class="primary" onclick="startJetsonPull()">Start Pull</button>
            <button onclick="copyPullCommand()">Copy Command</button>
          </div>
        </div>
      </section>
      <section class="transfer-card">
        <h3>Push map bundle</h3>
        <div class="form-grid">
          <div class="field full"><label>From notebook</label><input id="push-local" value="" placeholder="${esc(config.map_root || "")}/course_a" /></div>
          <div class="field full"><label>To Jetson</label><input id="push-remote" value="${esc(config.jetson_map_root || "")}" /></div>
          <div class="actions full">
            <button class="primary" onclick="startJetsonPush()">Start Push</button>
            <button onclick="copyPushCommand()">Copy Command</button>
          </div>
        </div>
      </section>
    </div>
  `;
}

function renderTerminalPage() {
  return `
    <div class="page">
      <section class="panel">
        <div class="panel-header"><h2>Run Command</h2></div>
        <div class="panel-body">
          <div class="form-grid">
            <div class="field"><label>Title</label><input id="custom-title" value="Custom command" /></div>
            <div class="field"><label>Working directory</label><input id="custom-cwd" value="${esc(state.config?.repo_root || "")}" /></div>
            <div class="field full"><label>Command</label><textarea id="custom-command" placeholder="echo hello"></textarea></div>
            <div class="actions full"><button class="primary" onclick="runCustomCommand()">Run</button><button onclick="copyText($('custom-command').value)">Copy</button></div>
          </div>
        </div>
      </section>
      <section class="panel">
        <div class="panel-header"><h2>Task History</h2><span class="spacer"></span><button onclick="refreshAll()">Refresh</button></div>
        <div class="panel-body">${renderTaskTable(state.tasks)}</div>
      </section>
    </div>
  `;
}

function renderTaskTable(tasks) {
  if (!tasks.length) return `<div class="empty">No tasks yet.</div>`;
  return `
    <table>
      <thead><tr><th>Status</th><th>Title</th><th>PID / PGID</th><th>Started</th><th></th></tr></thead>
      <tbody>
        ${tasks
          .map(
            (task) => `
              <tr>
                <td><span class="status ${esc(task.status)}">${esc(task.status)}</span></td>
                <td>${esc(task.title)}<div class="path" title="${esc(commandText(task))}">${esc(commandText(task))}</div></td>
                <td class="mono">${esc(task.pid || "-")} / ${esc(task.pgid || "-")}</td>
                <td>${esc(task.started_at || "-")}</td>
                <td class="actions">
                  <button onclick="selectTask(${js(task.task_id)})">Log</button>
                  <button onclick="copyText(${js(commandText(task))})">Copy</button>
                  <button class="danger" onclick="stopTask(${js(task.task_id)})" ${["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"}>Stop</button>
                </td>
              </tr>`,
          )
          .join("")}
      </tbody>
    </table>
  `;
}

function renderTerminal() {
  const task = state.tasks.find((item) => item.task_id === state.selectedTaskId);
  return `
    <section class="terminal ${state.terminalCollapsed ? "collapsed" : ""}">
      <div class="terminal-bar">
        <button class="ghost" onclick="toggleTerminal()">${state.terminalCollapsed ? "Open" : "Close"}</button>
        <div class="terminal-title">${esc(task?.title || "No task selected")}</div>
        <div class="terminal-meta">${task ? `pid ${task.pid || "-"} / pgid ${task.pgid || "-"} / ${task.status}` : ""}</div>
        <span class="spacer"></span>
        <button onclick="copyText(${js(commandText(task || {}))})" ${task ? "" : "disabled"}>Copy Command</button>
        <button onclick="copyText(state.logText)" ${task ? "" : "disabled"}>Copy Log</button>
        <button class="danger" onclick="stopTask(${js(task?.task_id || "")})" ${task && ["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"}>Stop</button>
      </div>
      <div class="terminal-body">
        <div class="task-tabs">
          ${state.tasks
            .slice(0, 40)
            .map(
              (item) => `
                <button class="task-tab ${item.task_id === state.selectedTaskId ? "active" : ""}" onclick="selectTask(${js(item.task_id)})">
                  <strong>${esc(item.title)}</strong>
                  <span class="status ${esc(item.status)}">${esc(item.status)}</span>
                </button>`,
            )
            .join("")}
        </div>
        <div class="log-pane"><pre id="task-log" class="log">${esc(state.logText || "Select a task to view logs.")}</pre></div>
      </div>
    </section>
  `;
}

function toggleTerminal() {
  state.terminalCollapsed = !state.terminalCollapsed;
  render();
}

function scrollLogToEnd() {
  const pane = document.querySelector(".log-pane");
  if (pane) pane.scrollTop = pane.scrollHeight;
}

async function selectTask(taskId) {
  state.selectedTaskId = taskId;
  state.logText = "";
  if (state.stream) state.stream.close();
  const task = state.tasks.find((item) => item.task_id === taskId);
  if (!task) {
    render();
    return;
  }
  state.terminalCollapsed = false;
  state.stream = new EventSource(`/api/tasks/${encodeURIComponent(taskId)}/stream`);
  state.stream.onmessage = (event) => {
    const payload = JSON.parse(event.data);
    if (payload.chunk) state.logText += payload.chunk;
    if (payload.task) {
      const index = state.tasks.findIndex((item) => item.task_id === payload.task.task_id);
      if (index >= 0) state.tasks[index] = payload.task;
    }
    if (isEditingField()) updateLogOnly();
    else render();
  };
  state.stream.onerror = () => {
    if (state.stream) state.stream.close();
  };
  render();
}

async function stopTask(taskId) {
  if (!taskId) return;
  await api(`/api/tasks/${encodeURIComponent(taskId)}/stop`, { method: "POST", body: "{}" });
  await refreshAll();
}

async function runCustomCommand() {
  const payload = {
    title: $("custom-title").value,
    kind: "custom",
    cwd: $("custom-cwd").value,
    command: $("custom-command").value,
  };
  const result = await api("/api/tasks/run", { method: "POST", body: JSON.stringify(payload) });
  await refreshAll();
  selectTask(result.task.task_id);
}

function fillMapDir() {
  const select = $("build-rosbag");
  const bagPath = select?.value || state.rosbags[0]?.path || "";
  const bagName = bagPath.split("/").filter(Boolean).pop() || "map";
  const stamp = new Date().toISOString().replace(/[-:]/g, "").slice(0, 15);
  $("build-map-dir").value = `${state.config?.map_root || "/workspaces/map"}/${bagName}_map_${stamp}`;
}

function useRosbag(path) {
  state.tab = "map-builder";
  render();
  $("build-rosbag").value = path;
  fillMapDir();
}

async function startMapBuild() {
  const payload = {
    rosbag: $("build-rosbag").value,
    map_dir: $("build-map-dir").value,
    topic_config: $("build-topic-config").value,
    steps: $("build-steps").value,
    enable_rviz: false,
  };
  const result = await api("/api/maps/build-vgl-vslam", { method: "POST", body: JSON.stringify(payload) });
  await refreshAll();
  selectTask(result.task.task_id);
}

function copyMapBuildCommand() {
  const rosbag = $("build-rosbag")?.value || "<rosbag>";
  const mapDir = $("build-map-dir")?.value || "<map_dir>";
  copyText(`jetpilot_map build-vgl-vslam --rosbag ${rosbag} --map-dir ${mapDir}`);
}

async function runMapStage(stage, mapDir) {
  const endpoint = `/api/maps/${stage}`;
  const result = await api(endpoint, { method: "POST", body: JSON.stringify({ map_dir: mapDir }) });
  await refreshAll();
  selectTask(result.task.task_id);
}

function fillTransferLocal(path) {
  state.tab = "jetson";
  render();
  $("push-local").value = path;
  $("push-remote").value = state.config?.jetson_map_root || "";
}

async function inspectJetson() {
  const params = new URLSearchParams({
    host: $("jetson-host").value,
    user: $("jetson-user").value,
    map_root: $("jetson-map-root").value,
    record_root: $("jetson-record-root").value,
  });
  const result = await api(`/api/jetson/inspect?${params.toString()}`);
  state.jetsonInspect = result;
  render();
}

function copyJetsonInspect() {
  copyText(`ssh ${$("jetson-user").value}@${$("jetson-host").value}`);
}

function setJetsonHost(host) {
  const input = $("jetson-host");
  if (input) input.value = host;
}

function jetsonTarget() {
  return {
    host: $("jetson-host")?.value || state.jetsonInspect?.host || state.config?.jetson_ips?.[0] || "",
    user: $("jetson-user")?.value || state.jetsonInspect?.user || state.config?.jetson_user || "tamiya",
  };
}

async function startTransfer(direction, paths = null) {
  const target = jetsonTarget();
  const payload = {
    host: target.host,
    user: target.user,
    remote_path: paths?.remote || "",
    local_path: paths?.local || "",
  };
  const endpoint = direction === "jetson-to-local" ? "/api/transfers/jetson-to-local" : "/api/transfers/local-to-jetson";
  const result = await api(endpoint, { method: "POST", body: JSON.stringify(payload) });
  await refreshAll();
  selectTask(result.task.task_id);
}

function startJetsonPull() {
  return startTransfer("jetson-to-local", {
    remote: $("pull-remote").value,
    local: $("pull-local").value,
  });
}

function startJetsonPush() {
  return startTransfer("local-to-jetson", {
    remote: $("push-remote").value,
    local: $("push-local").value,
  });
}

function copyPullCommand() {
  const target = jetsonTarget();
  copyText(`rsync -avhP --info=progress2 ${target.user}@${target.host}:${$("pull-remote").value} ${trimTrailingSlash($("pull-local").value)}/`);
}

function copyPushCommand() {
  const target = jetsonTarget();
  copyText(`rsync -avhP --info=progress2 ${trimTrailingSlash($("push-local").value)}/ ${target.user}@${target.host}:${trimTrailingSlash($("push-remote").value)}/`);
}

function parseJetsonOutput(output) {
  const sections = {};
  let current = "output";
  for (const line of String(output || "").split("\n")) {
    const match = line.match(/^\[([^\]]+)\]$/);
    if (match) {
      current = match[1];
      sections[current] = [];
    } else {
      if (!sections[current]) sections[current] = [];
      sections[current].push(line);
    }
  }
  return sections;
}

function contentLines(lines = []) {
  return lines.map((line) => line.trim()).filter(Boolean);
}

function firstContentLine(lines = []) {
  return contentLines(lines)[0] || "";
}

function shortName(path) {
  if (!path || path === "-") return "-";
  return path.split("/").filter(Boolean).pop() || path;
}

function sectionOutput(title, lines = []) {
  const text = contentLines(lines).join("\n");
  if (!text) return "";
  return `
    <div class="section-output">
      <div class="section-output-title">${esc(title)}</div>
      <pre class="log">${esc(text)}</pre>
    </div>
  `;
}

window.setTab = setTab;
window.refreshAll = refreshAll;
window.toggleTerminal = toggleTerminal;
window.copyText = copyText;
window.selectTask = selectTask;
window.stopTask = stopTask;
window.runCustomCommand = runCustomCommand;
window.fillMapDir = fillMapDir;
window.useRosbag = useRosbag;
window.startMapBuild = startMapBuild;
window.copyMapBuildCommand = copyMapBuildCommand;
window.runMapStage = runMapStage;
window.fillTransferLocal = fillTransferLocal;
window.inspectJetson = inspectJetson;
window.copyJetsonInspect = copyJetsonInspect;
window.setJetsonHost = setJetsonHost;
window.startTransfer = startTransfer;
window.startJetsonPull = startJetsonPull;
window.startJetsonPush = startJetsonPush;
window.copyPullCommand = copyPullCommand;
window.copyPushCommand = copyPushCommand;

refreshAll().catch((error) => {
  $("app").innerHTML = `<div class="content"><div class="notice">Failed to load JetPilot Console: ${esc(error.message)}</div></div>`;
});

setInterval(() => {
  api("/api/tasks")
    .then((data) => {
      state.tasks = data.tasks || [];
      if (!isEditingField()) render();
    })
    .catch(() => {});
}, 5000);
