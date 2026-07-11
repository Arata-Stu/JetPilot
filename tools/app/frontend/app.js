const state = {
  tab: "dashboard",
  config: null,
  tasks: [],
  rosbags: [],
  maps: [],
  cameraTopicConfigs: [],
  selectedTaskId: null,
  terminalCollapsed: false,
  logDialogOpen: false,
  logStickToEnd: true,
  logText: "",
  stream: null,
  jetsonInspect: null,
};

const tabs = [
  ["dashboard", "Dashboard"],
  ["rosbags", "Rosbags"],
  ["map-builder", "Map Builder"],
  ["joy-profile", "Joy Profile"],
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

async function apiText(path) {
  const response = await fetch(path);
  const text = await response.text();
  if (!response.ok) throw new Error(text || `${response.status} ${response.statusText}`);
  return text;
}

function toast(message, type = "success") {
  const region = $("toast-region");
  if (!region) return;
  const item = document.createElement("div");
  item.className = `toast ${type === "error" ? "error" : ""}`;
  item.textContent = message;
  region.append(item);
  setTimeout(() => item.remove(), 3200);
}

async function copyText(text, message = "Copied") {
  try {
    const value = String(text ?? "");
    if (navigator.clipboard?.writeText) {
      await navigator.clipboard.writeText(value);
    } else {
      const textarea = document.createElement("textarea");
      textarea.value = value;
      textarea.setAttribute("readonly", "");
      textarea.style.position = "fixed";
      textarea.style.opacity = "0";
      document.body.append(textarea);
      textarea.select();
      const copied = document.execCommand("copy");
      textarea.remove();
      if (!copied) throw new Error("Clipboard is not available");
    }
    toast(message);
  } catch (error) {
    toast(`Copy failed: ${error.message}`, "error");
  }
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

function sh(value) {
  const text = String(value ?? "");
  if (!text) return "''";
  if (/^[A-Za-z0-9_/:=.,@%+-]+$/.test(text)) return text;
  return `'${text.replaceAll("'", "'\\''")}'`;
}

async function refreshAll() {
  const [config, tasks, rosbags, maps, cameraTopicConfigs] = await Promise.all([
    api("/api/config"),
    api("/api/tasks"),
    api("/api/rosbags/local"),
    api("/api/maps/local"),
    api("/api/map-builder/camera-topic-configs"),
  ]);
  state.config = config;
  state.tasks = tasks.tasks || [];
  state.rosbags = rosbags.rosbags || [];
  state.maps = maps.maps || [];
  state.cameraTopicConfigs = cameraTopicConfigs.configs || [];
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
      ${renderLogDialog()}
      <div class="toast-region" id="toast-region" aria-live="polite"></div>
    </div>
  `;
  scrollLogToEnd();
}

function isEditingField() {
  return ["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName || "");
}

function updateLogOnly(chunk = "", append = false) {
  updateLogElement($("task-log"), chunk, append);
  updateLogElement($("console-output"), chunk, append);
  updateTaskChrome();
  scrollLogToEnd();
}

function renderPage() {
  if (state.tab === "rosbags") return renderRosbags();
  if (state.tab === "map-builder") return renderMapBuilder();
  if (state.tab === "joy-profile") return renderJoyProfile();
  if (state.tab === "maps") return renderMaps();
  if (state.tab === "jetson") return renderJetson();
  if (state.tab === "terminal") return renderTerminalPage();
  return renderDashboard();
}

function renderJoyProfile() {
  return `
    <div class="page joy-profile-page">
      <div class="joy-profile-toolbar">
        <div>
          <strong>Joy Profile Editor</strong>
          <span>load, edit, test, and export controller YAML</span>
        </div>
        <a class="button-link" href="/joy-profile-editor" target="_blank" rel="noreferrer">Open in new tab</a>
      </div>
      <iframe class="joy-profile-frame" src="/joy-profile-editor" title="Joy Profile Editor"></iframe>
    </div>
  `;
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
          <button onclick="fillMapDir()">Suggest Map Name</button>
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
  const topicConfigs = state.cameraTopicConfigs || [];
  const recommended = topicConfigs.find((config) => config.recommended) || topicConfigs[0];
  const recommendedPath = recommended?.path || "";
  const defaultMapBase = state.config?.map_root || "/workspaces/map";
  const topicOptions = topicConfigs
    .map((config) => {
      const confidence = config.recommended ? "recommended" : `score ${config.score}`;
      return `
        <option value="${esc(config.path)}" ${config.path === recommendedPath ? "selected" : ""}>
          ${esc(config.name)} - ${esc(confidence)}
        </option>`;
    })
    .join("");
  return `
    <div class="form-grid">
      <div class="field full">
        <label>Rosbag</label>
        <select id="build-rosbag">
          <option value="">Select rosbag</option>
          ${state.rosbags.map((bag) => `<option value="${esc(bag.path)}">${esc(bag.name)} - ${esc(bag.path)}</option>`).join("")}
        </select>
      </div>
      <div class="field">
        <label>Output base directory</label>
        <input id="build-map-base" value="${esc(defaultMapBase)}" oninput="updateMapDirPreview()" />
      </div>
      <div class="field">
        <label>Map name</label>
        <input id="build-map-name" placeholder="course_a" oninput="updateMapDirPreview()" />
      </div>
      <div class="field full">
        <label>Output map directory preview</label>
        <div id="build-map-dir-preview" class="path path-preview">${esc(defaultMapBase)}/&lt;map_name&gt;</div>
      </div>
      <div class="field">
        <label>Mapping steps</label>
        <input id="build-steps" value="edex compute_poses cuvgl" />
      </div>
      <div class="field">
        <label>Predicted camera topic config</label>
        <select id="build-topic-config-select" onchange="applyCameraTopicConfig()">
          <option value="">Use backend default</option>
          ${topicOptions || `<option value="" disabled>No localization YAML found</option>`}
        </select>
        <div id="build-topic-config-preview" class="field-hint">${esc(recommendedPath || "No predicted config found")}</div>
      </div>
      <div class="field">
        <label>Manual camera topic config</label>
        <input id="build-topic-config" placeholder="${esc(recommendedPath || "Default JetPilot vgl_camera_topics.yaml")}" oninput="updateCameraTopicPreview()" />
      </div>
      <div class="actions full">
        <button onclick="applyCameraTopicConfig()" ${recommendedPath ? "" : "disabled"}>Use Predicted Path</button>
        <button onclick="copySelectedCameraTopicConfig()">Copy Topic Config</button>
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
  const sequences = jetsonRosbagSequences();
  const sequenceOptions = sequences
    .map((sequence) => `<option value="${esc(sequence.path)}">${esc(sequence.name)} - ${esc(sequence.modified || sequence.path)}</option>`)
    .join("");
  const sequenceList = sequences
    .slice(0, 12)
    .map(
      (sequence) => `
        <div class="mini-row">
          <div>
            <strong>${esc(sequence.name)}</strong>
            <div class="path" title="${esc(sequence.path)}">${esc(sequence.path)}</div>
          </div>
          <div class="actions">
            <button onclick="useJetsonRosbag(${js(sequence.path)})">Use</button>
            <button class="primary" onclick="pullJetsonRosbag(${js(sequence.path)})">Pull</button>
          </div>
        </div>`,
    )
    .join("");
  return `
    <div class="transfer-grid">
      <section class="transfer-card">
        <h3>Pull one rosbag sequence</h3>
        <div class="form-grid">
          <div class="field full">
            <label>Discovered by metadata.yaml</label>
            <select id="pull-remote-select" onchange="useJetsonRosbag(this.value)">
              <option value="">Select inspected sequence</option>
              ${sequenceOptions}
            </select>
          </div>
          <div class="field full"><label>From Jetson</label><input id="pull-remote" value="" placeholder="${esc(config.jetson_record_root || "")}/<sequence>" /></div>
          <div class="field full"><label>To notebook</label><input id="pull-local" value="${esc(config.record_root || "")}" /></div>
          <div class="actions full">
            <button class="primary" onclick="startJetsonPull()">Pull Selected Sequence</button>
            <button onclick="copyPullCommand()">Copy rsync Command</button>
          </div>
          ${
            sequences.length
              ? `<div class="mini-list full">${sequenceList}</div>`
              : `<div class="notice full">Inspect Jetson first. Rosbag sequences are discovered by searching for metadata.yaml under the remote rosbag root.</div>`
          }
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
                  <button onclick="openTaskLog(${js(task.task_id)})">Open Log</button>
                  <button onclick="copyTaskCommand(${js(task.task_id)})">Copy Command</button>
                  <button onclick="copyTaskLog(${js(task.task_id)})">Copy Log</button>
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
        <div id="terminal-title" class="terminal-title">${esc(task?.title || "No task selected")}</div>
        <div id="terminal-meta" class="terminal-meta">${task ? `pid ${task.pid || "-"} / pgid ${task.pgid || "-"} / ${task.status}` : ""}</div>
        <span class="spacer"></span>
        <button onclick="openLogDialog()" ${task ? "" : "disabled"}>Open Console</button>
        <button onclick="copySelectedTaskCommand()" ${task ? "" : "disabled"}>Copy Command</button>
        <button onclick="copySelectedTaskLog()" ${task ? "" : "disabled"}>Copy Log</button>
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
        <div class="log-pane" onscroll="handleLogScroll(event)"><pre id="task-log" class="log">${esc(state.logText || "Select a task to view logs.")}</pre></div>
      </div>
    </section>
  `;
}

function renderLogDialog() {
  const task = state.tasks.find((item) => item.task_id === state.selectedTaskId);
  if (!state.logDialogOpen) return "";
  return `
    <div class="dialog-backdrop" onclick="closeLogDialog()">
      <section class="console-dialog" role="dialog" aria-modal="true" aria-labelledby="console-title" onclick="event.stopPropagation()">
        <div class="dialog-header">
          <div>
            <p class="eyebrow">Task console</p>
            <h2 id="console-title">${esc(task?.title || "Console")}</h2>
            <span id="console-meta">${task ? `pid ${esc(task.pid || "-")} / pgid ${esc(task.pgid || "-")} / ${esc(task.status)}` : "No task selected"}</span>
          </div>
          <div class="dialog-actions">
            <button onclick="copySelectedTaskCommand()" ${task ? "" : "disabled"}>Copy Command</button>
            <button onclick="copySelectedTaskLog()" ${task ? "" : "disabled"}>Copy Log</button>
            <button class="danger" onclick="stopTask(${js(task?.task_id || "")})" ${task && ["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"}>Stop</button>
            <button class="icon-button" onclick="closeLogDialog()" aria-label="Close console">x</button>
          </div>
        </div>
        <pre id="console-output" class="dialog-log" onscroll="handleLogScroll(event)">${esc(state.logText || "Select a task to view logs.")}</pre>
      </section>
    </div>
  `;
}

function toggleTerminal() {
  state.terminalCollapsed = !state.terminalCollapsed;
  render();
}

function selectedTask() {
  return state.tasks.find((item) => item.task_id === state.selectedTaskId);
}

function updateLogElement(element, chunk = "", append = false) {
  if (!element) return;
  if (chunk && append) {
    element.textContent += chunk;
    return;
  }
  element.textContent = state.logText || "Select a task to view logs.";
}

function updateTaskChrome() {
  const task = selectedTask();
  if ($("terminal-title")) $("terminal-title").textContent = task?.title || "No task selected";
  if ($("terminal-meta")) $("terminal-meta").textContent = task ? `pid ${task.pid || "-"} / pgid ${task.pgid || "-"} / ${task.status}` : "";
  if ($("console-title")) $("console-title").textContent = task?.title || "Console";
  if ($("console-meta")) $("console-meta").textContent = task ? `pid ${task.pid || "-"} / pgid ${task.pgid || "-"} / ${task.status}` : "No task selected";
}

function isNearBottom(element) {
  if (!element) return true;
  return element.scrollHeight - element.scrollTop - element.clientHeight < 28;
}

function handleLogScroll(event) {
  state.logStickToEnd = isNearBottom(event.currentTarget);
}

function scrollLogToEnd(force = false) {
  if (!force && !state.logStickToEnd) return;
  const pane = document.querySelector(".log-pane");
  if (pane) pane.scrollTop = pane.scrollHeight;
  scrollDialogLogToEnd(force);
}

function scrollDialogLogToEnd(force = false) {
  if (!force && !state.logStickToEnd) return;
  const dialogLog = $("console-output");
  if (dialogLog) dialogLog.scrollTop = dialogLog.scrollHeight;
}

function openLogDialog() {
  state.logDialogOpen = true;
  render();
}

function closeLogDialog() {
  state.logDialogOpen = false;
  render();
}

async function openTaskLog(taskId) {
  await selectTask(taskId, { openDialog: true });
}

async function selectTask(taskId, options = {}) {
  state.selectedTaskId = taskId;
  state.logText = "";
  state.logStickToEnd = true;
  if (state.stream) state.stream.close();
  const task = state.tasks.find((item) => item.task_id === taskId);
  if (!task) {
    render();
    return;
  }
  state.terminalCollapsed = false;
  if (options.openDialog) state.logDialogOpen = true;
  state.stream = new EventSource(`/api/tasks/${encodeURIComponent(taskId)}/stream?tail=1000`);
  state.stream.onmessage = (event) => {
    const payload = JSON.parse(event.data);
    const chunk = payload.chunk || "";
    const append = Boolean(state.logText && chunk);
    if (chunk) state.logText += chunk;
    if (payload.task) {
      const index = state.tasks.findIndex((item) => item.task_id === payload.task.task_id);
      if (index >= 0) state.tasks[index] = payload.task;
    }
    updateLogOnly(chunk, append);
  };
  state.stream.onerror = () => {
    if (state.stream) state.stream.close();
  };
  render();
}

function logHistoryIsBeingRead() {
  return Boolean(state.selectedTaskId && !state.logStickToEnd && (!state.terminalCollapsed || state.logDialogOpen));
}

function taskById(taskId) {
  return state.tasks.find((item) => item.task_id === taskId);
}

function copyTaskCommand(taskId) {
  copyText(commandText(taskById(taskId) || {}), "Command copied");
}

async function copyTaskLog(taskId) {
  try {
    const text = taskId === state.selectedTaskId && state.logText
      ? state.logText
      : await apiText(`/api/tasks/${encodeURIComponent(taskId)}/log`);
    await copyText(text, "Log copied");
  } catch (error) {
    toast(`Copy failed: ${error.message}`, "error");
  }
}

function copySelectedTaskCommand() {
  if (!state.selectedTaskId) return;
  copyTaskCommand(state.selectedTaskId);
}

function copySelectedTaskLog() {
  if (!state.selectedTaskId) return;
  return copyTaskLog(state.selectedTaskId);
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
  if ($("build-map-base") && !$("build-map-base").value.trim()) {
    $("build-map-base").value = state.config?.map_root || "/workspaces/map";
  }
  if ($("build-map-name")) $("build-map-name").value = `${bagName}_map_${stamp}`;
  updateMapDirPreview();
}

function useRosbag(path) {
  state.tab = "map-builder";
  render();
  $("build-rosbag").value = path;
  fillMapDir();
}

function selectedCameraTopicConfig() {
  const manual = $("build-topic-config")?.value.trim();
  if (manual) return manual;
  return $("build-topic-config-select")?.value || "";
}

function applyCameraTopicConfig() {
  const selected = $("build-topic-config-select")?.value || "";
  if ($("build-topic-config")) $("build-topic-config").value = selected;
  updateCameraTopicPreview();
}

function copySelectedCameraTopicConfig() {
  copyText(selectedCameraTopicConfig());
}

function updateCameraTopicPreview() {
  const preview = $("build-topic-config-preview");
  if (preview) preview.textContent = selectedCameraTopicConfig() || "Backend default will be used.";
}

function outputMapBase() {
  return trimTrailingSlash($("build-map-base")?.value || state.config?.map_root || "/workspaces/map");
}

function outputMapName() {
  return String($("build-map-name")?.value || "").replace(/^\/+|\/+$/g, "");
}

function outputMapDir(options = {}) {
  const base = outputMapBase();
  const name = outputMapName();
  if (name) return `${base}/${name}`;
  return options.placeholder ? `${base}/<map_name>` : "";
}

function updateMapDirPreview() {
  const preview = $("build-map-dir-preview");
  if (preview) preview.textContent = outputMapDir({ placeholder: true });
}

async function startMapBuild() {
  const mapDir = outputMapDir();
  if (!mapDir) {
    window.alert("Map name is required.");
    return;
  }
  const payload = {
    rosbag: $("build-rosbag").value,
    map_dir: mapDir,
    topic_config: selectedCameraTopicConfig(),
    steps: $("build-steps").value,
    enable_rviz: false,
  };
  const result = await api("/api/maps/build-vgl-vslam", { method: "POST", body: JSON.stringify(payload) });
  await refreshAll();
  selectTask(result.task.task_id);
}

function copyMapBuildCommand() {
  const rosbag = $("build-rosbag")?.value || "<rosbag>";
  const mapDir = outputMapDir({ placeholder: true });
  const topicConfig = selectedCameraTopicConfig();
  const steps = $("build-steps")?.value || "edex compute_poses cuvgl";
  const topicArg = topicConfig ? ` --topic-config ${sh(topicConfig)}` : "";
  copyText(`jetpilot_map build-vgl-vslam --rosbag ${sh(rosbag)} --map-dir ${sh(mapDir)} --steps ${sh(steps)}${topicArg}`);
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

function pullRemotePath() {
  return $("pull-remote")?.value.trim() || $("pull-remote-select")?.value || "";
}

function pullLocalPath() {
  return $("pull-local")?.value.trim() || state.config?.record_root || "";
}

function useJetsonRosbag(path) {
  if ($("pull-remote")) $("pull-remote").value = path || "";
  if ($("pull-remote-select") && path) $("pull-remote-select").value = path;
}

function pullJetsonRosbag(path) {
  useJetsonRosbag(path);
  return startJetsonPull();
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
  if (!pullRemotePath()) {
    window.alert("Select or enter one Jetson rosbag sequence first.");
    return null;
  }
  return startTransfer("jetson-to-local", {
    remote: pullRemotePath(),
    local: pullLocalPath(),
  });
}

function startJetsonPush() {
  return startTransfer("local-to-jetson", {
    remote: $("push-remote").value,
    local: $("push-local").value,
  });
}

function copyPullCommand() {
  if (!pullRemotePath()) {
    window.alert("Select or enter one Jetson rosbag sequence first.");
    return;
  }
  const target = jetsonTarget();
  copyText(`rsync -avhP --info=progress2 ${sh(`${target.user}@${target.host}:${pullRemotePath()}`)} ${sh(trimTrailingSlash(pullLocalPath()) + "/")}`);
}

function copyPushCommand() {
  const target = jetsonTarget();
  copyText(`rsync -avhP --info=progress2 ${sh(trimTrailingSlash($("push-local").value) + "/")} ${sh(`${target.user}@${target.host}:${trimTrailingSlash($("push-remote").value)}/`)}`);
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

function jetsonRosbagSequences() {
  const sections = parseJetsonOutput(state.jetsonInspect?.output || "");
  const seen = new Set();
  return contentLines(sections.rosbags)
    .map((line) => {
      const match = line.match(/^(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2})\s+(.+)$/);
      const modified = match ? match[1] : "";
      const path = match ? match[2] : line;
      return {
        modified,
        path,
        name: shortName(path),
      };
    })
    .filter((sequence) => {
      if (!sequence.path || seen.has(sequence.path)) return false;
      seen.add(sequence.path);
      return true;
    })
    .reverse();
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
window.openTaskLog = openTaskLog;
window.openLogDialog = openLogDialog;
window.closeLogDialog = closeLogDialog;
window.handleLogScroll = handleLogScroll;
window.copyTaskCommand = copyTaskCommand;
window.copyTaskLog = copyTaskLog;
window.copySelectedTaskCommand = copySelectedTaskCommand;
window.copySelectedTaskLog = copySelectedTaskLog;
window.stopTask = stopTask;
window.runCustomCommand = runCustomCommand;
window.fillMapDir = fillMapDir;
window.useRosbag = useRosbag;
window.selectedCameraTopicConfig = selectedCameraTopicConfig;
window.applyCameraTopicConfig = applyCameraTopicConfig;
window.copySelectedCameraTopicConfig = copySelectedCameraTopicConfig;
window.updateCameraTopicPreview = updateCameraTopicPreview;
window.updateMapDirPreview = updateMapDirPreview;
window.startMapBuild = startMapBuild;
window.copyMapBuildCommand = copyMapBuildCommand;
window.runMapStage = runMapStage;
window.fillTransferLocal = fillTransferLocal;
window.inspectJetson = inspectJetson;
window.copyJetsonInspect = copyJetsonInspect;
window.setJetsonHost = setJetsonHost;
window.startTransfer = startTransfer;
window.useJetsonRosbag = useJetsonRosbag;
window.pullJetsonRosbag = pullJetsonRosbag;
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
      if (!isEditingField() && !logHistoryIsBeingRead()) render();
      else updateTaskChrome();
    })
    .catch(() => {});
}, 5000);
