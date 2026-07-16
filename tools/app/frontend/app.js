const state = {
  tab: "dashboard",
  config: null,
  tasks: [],
  rosbags: [],
  maps: [],
  cameraTopicConfigs: [],
  localIps: [],
  selectedMapPath: null,
  selectedMapDetail: null,
  mapLayers: {
    landmark: true,
    left_bound: true,
    right_bound: true,
    centerline: true,
    raceline: true,
    section_gates: true,
    section_labels: true,
  },
  selectedTaskId: null,
  fpv: {
    host: "",
    codec: "h264",
    width: 424,
    height: 240,
    fps: 60,
    port: 5004,
    payload: 96,
    displaySink: "glimagesink",
    noDisplay: false,
  },
  terminalCollapsed: false,
  logDialogOpen: false,
  logStickToEnd: true,
  logText: "",
  stream: null,
  jetsonTarget: null,
  jetsonInspect: null,
  jetsonInspectBusy: false,
};

const tabs = [
  ["dashboard", "Dashboard"],
  ["rosbags", "Rosbags"],
  ["map-builder", "Map Builder"],
  ["joy-profile", "Joy Profile"],
  ["fpv", "FPV"],
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
const js = (value) => esc(JSON.stringify(String(value ?? "")));

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

function apiPath(path, params = {}) {
  const query = new URLSearchParams(params);
  return `${path}?${query.toString()}`;
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
  const [config, tasks, rosbags, maps, cameraTopicConfigs, localIps] = await Promise.all([
    api("/api/config"),
    api("/api/tasks"),
    api("/api/rosbags/local"),
    api("/api/maps/local"),
    api("/api/map-builder/camera-topic-configs"),
    api("/api/network/local-ips").catch(() => ({ ips: [] })),
  ]);
  state.config = config;
  state.tasks = tasks.tasks || [];
  state.rosbags = rosbags.rosbags || [];
  state.maps = maps.maps || [];
  state.cameraTopicConfigs = cameraTopicConfigs.configs || [];
  state.localIps = localIps.ips || [];
  if (!state.fpv.host && state.localIps[0]) state.fpv.host = state.localIps[0];
  if (state.selectedMapPath && !state.maps.some((item) => item.path === state.selectedMapPath)) {
    state.selectedMapPath = null;
    state.selectedMapDetail = null;
  }
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
  requestAnimationFrame(drawMapPreview);
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
  if (state.tab === "fpv") return renderFpv();
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

function renderFpv() {
  const running = state.tasks.filter((task) => task.kind === "fpv-viewer" && ["queued", "running", "stopping"].includes(task.status));
  const fpv = state.fpv;
  return `
    <div class="page fpv-page">
      <section class="panel">
        <div class="panel-header">
          <h2>FPV RTP Viewer</h2>
          <span class="spacer"></span>
          <button onclick="copyFpvReceiverCommand()">Copy Command</button>
          <button class="primary" onclick="startFpvViewer()">Start Viewer</button>
        </div>
        <div class="panel-body">
          <div class="form-grid">
            <div class="field">
              <label>Mac / notebook IP</label>
              <input id="fpv-host" value="${esc(fpv.host)}" placeholder="10.42.0.161" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>Codec</label>
              <select id="fpv-codec" onchange="updateFpvCommandPreview()">
                ${["h264", "h265", "mjpeg", "raw"].map((codec) => `<option value="${codec}" ${fpv.codec === codec ? "selected" : ""}>${codec}</option>`).join("")}
              </select>
            </div>
            <div class="field">
              <label>Display sink</label>
              <select id="fpv-display-sink" onchange="updateFpvCommandPreview()">
                ${["glimagesink", "autovideosink", "xvimagesink", "ximagesink"].map((sink) => `<option value="${sink}" ${fpv.displaySink === sink ? "selected" : ""}>${sink}</option>`).join("")}
              </select>
            </div>
            <div class="field">
              <label>Width</label>
              <input id="fpv-width" type="number" min="1" value="${esc(fpv.width)}" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>Height</label>
              <input id="fpv-height" type="number" min="1" value="${esc(fpv.height)}" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>FPS</label>
              <input id="fpv-fps" type="number" min="1" value="${esc(fpv.fps)}" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>Port</label>
              <input id="fpv-port" type="number" min="1" value="${esc(fpv.port)}" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>Payload</label>
              <input id="fpv-payload" type="number" min="0" value="${esc(fpv.payload)}" oninput="updateFpvCommandPreview()" />
            </div>
            <label class="check-row">
              <input id="fpv-no-display" type="checkbox" ${fpv.noDisplay ? "checked" : ""} onchange="updateFpvCommandPreview()" />
              <span>Receive without display</span>
            </label>
            <div class="field full">
              <label>Receiver command</label>
              <textarea id="fpv-command" readonly>${esc(buildFpvReceiverCommand(readFpvForm(false)))}</textarea>
            </div>
            <div class="field full">
              <label>Jetson bringup command</label>
              <textarea id="fpv-jetson-command" readonly>${esc(buildFpvJetsonCommand(readFpvForm(false)))}</textarea>
            </div>
            <div class="actions full">
              ${state.localIps.map((ip) => `<button class="ghost" onclick="setFpvHost(${js(ip)})">${esc(ip)}</button>`).join("")}
            </div>
            <div class="actions full">
              <button class="primary" onclick="startFpvViewer()">Start Viewer</button>
              <button onclick="copyFpvReceiverCommand()">Copy Command</button>
              <button onclick="copyFpvJetsonCommand()">Copy Jetson Command</button>
              ${running.length ? `<button class="danger" onclick="stopTask(${js(running[0].task_id)})">Stop Running Viewer</button>` : ""}
            </div>
          </div>
        </div>
      </section>
      <section class="panel">
        <div class="panel-header"><h2>FPV Tasks</h2><span class="spacer"></span><button onclick="refreshAll()">Refresh</button></div>
        <div class="panel-body">${renderTaskTable(state.tasks.filter((task) => task.kind === "fpv-viewer").slice(0, 8))}</div>
      </section>
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
      ${renderAutonomyPipeline()}
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

function renderAutonomyPipeline() {
  const map = pipelineMap();
  const steps = pipelineSteps(map);
  const nextStep = steps.find((step) => step.status !== "done");
  const mapName = map ? map.name : "No map selected yet";
  return `
    <section class="panel pipeline-panel">
      <div class="panel-header">
        <h2>Autonomy Pipeline</h2>
        <span class="pipeline-target">${esc(mapName)}</span>
        <span class="spacer"></span>
        <button onclick="setTab('rosbags')">Rosbags</button>
        <button onclick="setTab('map-builder')">Map Builder</button>
        <button onclick="setTab('maps')">Maps</button>
      </div>
      <div class="panel-body">
        <div class="pipeline-next">
          <div>
            <span>Next step</span>
            <strong>${esc(nextStep ? nextStep.title : "Ready for Jetson")}</strong>
          </div>
          <p>${esc(nextStep ? nextStep.next : "The selected map has the runtime bundle needed for autonomous driving prep.")}</p>
        </div>
        <div class="pipeline-rail">
          ${steps.map((step, index) => pipelineStep(step, index + 1)).join("")}
        </div>
      </div>
    </section>
  `;
}

function pipelineMap() {
  if (state.selectedMapPath) {
    const selected = state.maps.find((item) => item.path === state.selectedMapPath);
    if (selected) {
      const selectedScore = mapProgressScore(selected);
      const nested = [...state.maps]
        .filter((item) => item.path.startsWith(`${selected.path}/`))
        .sort((a, b) => mapProgressScore(b) - mapProgressScore(a))[0];
      if (nested && mapProgressScore(nested) > selectedScore) return nested;
      return selected;
    }
  }
  if (!state.maps.length) return null;
  return [...state.maps].sort((a, b) => mapProgressScore(b) - mapProgressScore(a))[0];
}

function mapProgressScore(map) {
  const weights = {
    cuvgl_map: 10,
    cuvslam_map: 10,
    snapshot: 5,
    landmark_yaml: 2,
    landmark_image: 2,
    hd_map: 3,
    centerline_csv: 3,
    raceline_csv: 3,
    line_preview: 1,
  };
  return Object.entries(weights).reduce(
    (score, [key, weight]) => score + (map.artifacts?.[key]?.exists ? weight : 0),
    0,
  );
}

function artifactExists(map, key) {
  return Boolean(map?.artifacts?.[key]?.exists);
}

function hasRunningTask(kind) {
  return state.tasks.some((task) => task.kind === kind && ["queued", "running", "stopping"].includes(task.status));
}

function isActiveTask(task) {
  return ["queued", "running", "stopping"].includes(task.status);
}

function isMapTask(task) {
  return ["map-build", "prepare-hd-raster", "generate-raceline", "generate-preview"].includes(task.kind);
}

function mapTaskSignature(tasks) {
  return tasks
    .filter(isMapTask)
    .map((task) => `${task.task_id}:${task.status}:${task.ended_at || ""}`)
    .join("|");
}

function shouldRefreshMapsAfterTaskPoll(previousTasks, nextTasks) {
  const previousActive = previousTasks.some((task) => isMapTask(task) && isActiveTask(task));
  const nextActive = nextTasks.some((task) => isMapTask(task) && isActiveTask(task));
  return previousActive || nextActive || mapTaskSignature(previousTasks) !== mapTaskSignature(nextTasks);
}

function pipelineSteps(map) {
  const hasBag = state.rosbags.length > 0;
  const hasVisualMap = artifactExists(map, "cuvgl_map") && artifactExists(map, "cuvslam_map");
  const hasRaster = artifactExists(map, "landmark_yaml") && artifactExists(map, "landmark_image");
  const hasHdMap = artifactExists(map, "hd_map") && artifactExists(map, "centerline_csv");
  const hasRaceline = artifactExists(map, "raceline_csv");
  const hasPreview = artifactExists(map, "line_preview");
  const mapPath = map?.path || "";
  return [
    {
      title: "Choose rosbag",
      detail: "Driving log used as the source for map creation.",
      status: hasBag ? "done" : "blocked",
      next: hasBag ? "Use one rosbag to build the visual map." : "Record or pull a rosbag first.",
      action: "Open Rosbags",
      onclick: "setTab('rosbags')",
    },
    {
      title: "Build visual map",
      detail: "Creates cuVGL, cuVSLAM, and the VSLAM snapshot.",
      status: hasRunningTask("map-build") ? "running" : hasVisualMap ? "done" : hasBag ? "ready" : "blocked",
      next: hasVisualMap ? "Prepare the landmark raster next." : "Start VGL/VSLAM build from a selected rosbag.",
      action: "Open Builder",
      onclick: "setTab('map-builder')",
    },
    {
      title: "Prepare landmark raster",
      detail: "Exports the image used for HD map line editing.",
      status: hasRunningTask("prepare-hd-raster") ? "running" : hasRaster ? "done" : hasVisualMap ? "ready" : "blocked",
      next: hasRaster ? "Draw or inspect HD map lines." : "Generate the landmark image for this map.",
      action: "Run Raster",
      onclick: mapPath ? `runMapStage('prepare-hd-raster', ${js(mapPath)})` : "setTab('maps')",
    },
    {
      title: "Create HD map lines",
      detail: "Left/right bounds and primary centerline.",
      status: hasHdMap ? "done" : hasRaster ? "ready" : "blocked",
      next: hasHdMap ? "Generate raceline from the centerline." : "Open the map workspace and check the line state.",
      action: "Open Workspace",
      onclick: mapPath ? `openMapWorkspace(${js(mapPath)})` : "setTab('maps')",
    },
    {
      title: "Generate raceline",
      detail: "Builds the running line used by control.",
      status: hasRunningTask("generate-raceline") ? "running" : hasRaceline ? "done" : hasHdMap ? "ready" : "blocked",
      next: hasRaceline ? "Generate a preview to visually confirm it." : "Generate the raceline after HD map lines exist.",
      action: "Run Raceline",
      onclick: mapPath ? `runMapStage('generate-raceline', ${js(mapPath)})` : "setTab('maps')",
    },
    {
      title: "Review preview",
      detail: "Checks map shape, bounds, centerline, and raceline together.",
      status: hasRunningTask("generate-preview") ? "running" : hasPreview ? "done" : hasRaceline ? "ready" : "blocked",
      next: hasPreview ? "Push the map bundle to Jetson." : "Generate preview after raceline exists.",
      action: "Run Preview",
      onclick: mapPath ? `runMapStage('generate-preview', ${js(mapPath)})` : "setTab('maps')",
    },
    {
      title: "Send to Jetson",
      detail: "Transfers the runtime-ready map bundle.",
      status: map?.complete_runtime_bundle ? "done" : hasRaceline ? "ready" : "blocked",
      next: map?.complete_runtime_bundle ? "Transfer this bundle and set it for driving." : "Complete the map bundle before transfer.",
      action: "Transfer",
      onclick: mapPath ? `fillTransferLocal(${js(mapPath)})` : "setTab('jetson')",
    },
  ];
}

function pipelineStep(step, number) {
  return `
    <div class="pipeline-step ${esc(step.status)}">
      <div class="pipeline-number">${esc(number)}</div>
      <div class="pipeline-step-body">
        <div class="pipeline-step-top">
          <strong>${esc(step.title)}</strong>
          <span class="status ${pipelineStatusClass(step.status)}">${pipelineStatusLabel(step.status)}</span>
        </div>
        <p>${esc(step.detail)}</p>
        <button onclick="${step.onclick}" ${step.status === "blocked" ? "disabled" : ""}>${esc(step.action)}</button>
      </div>
    </div>
  `;
}

function pipelineStatusLabel(status) {
  return {
    done: "done",
    running: "running",
    ready: "ready",
    blocked: "waiting",
  }[status] || status;
}

function pipelineStatusClass(status) {
  if (status === "done") return "success";
  if (status === "running") return "running";
  if (status === "ready") return "queued";
  return "stopped";
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
      <div class="map-workspace-layout">
        <section class="panel map-list-panel">
          <div class="panel-header"><h2>Local Maps</h2><span class="spacer"></span><button onclick="refreshAll()">Scan</button></div>
          <div class="panel-body">${state.maps.length ? mapList() : `<div class="empty">No map folders under ${esc(state.config?.map_root || "")}</div>`}</div>
        </section>
        <section class="panel map-workspace-panel">
          <div class="panel-header">
            <h2>Map Workspace</h2>
            <span class="spacer"></span>
            ${state.selectedMapPath ? `<button onclick="refreshSelectedMap()">Reload Map</button>` : ""}
          </div>
          <div class="panel-body">${renderMapWorkspace()}</div>
        </section>
      </div>
    </div>
  `;
}

function mapList() {
  return `
    <div class="map-list">
      ${state.maps
        .map((map) => {
          const selected = state.selectedMapPath === map.path;
          const artifactKeys = ["cuvgl_map", "cuvslam_map", "hd_map", "centerline_csv", "raceline_csv", "line_preview"];
          return `
            <article class="map-list-item ${selected ? "selected" : ""}">
              <div class="map-list-main">
                <strong>${esc(map.name)}</strong>
                <div class="path" title="${esc(map.path)}">${esc(map.path)}</div>
                <div class="chips">${artifactKeys
                  .map((key) => `<span class="chip ${map.artifacts[key]?.exists ? "ok" : "missing"}">${artifactLabel(key)}</span>`)
                  .join("")}</div>
              </div>
              <div class="map-list-actions">
                ${map.complete_runtime_bundle ? `<span class="status success">runtime ready</span>` : `<span class="status failed">incomplete</span>`}
                <button class="primary" onclick="openMapWorkspace(${js(map.path)})">${selected ? "Viewing" : "Open"}</button>
                <button onclick="copyText(${js(map.path)})">Copy</button>
                <button onclick="runMapStage('prepare-hd-raster', ${js(map.path)})">Raster</button>
                <button onclick="runMapStage('generate-raceline', ${js(map.path)})">Raceline</button>
                <button onclick="runMapStage('generate-preview', ${js(map.path)})">Preview</button>
                <button onclick="fillTransferLocal(${js(map.path)})">Transfer</button>
              </div>
            </article>`;
        })
        .join("")}
    </div>
  `;
}

function artifactLabel(key) {
  return {
    cuvgl_map: "cuVGL",
    cuvslam_map: "cuVSLAM",
    snapshot: "snapshot",
    landmark_yaml: "landmark YAML",
    landmark_image: "landmark image",
    hd_map: "HD map",
    centerline_csv: "centerline",
    raceline_csv: "raceline",
    line_preview: "preview",
  }[key] || key;
}

function renderMapWorkspace() {
  const detail = state.selectedMapDetail;
  if (!state.selectedMapPath) {
    return `<div class="empty">Select a map to inspect its shape, lines, sections, and runtime readiness.</div>`;
  }
  if (!detail || detail.map?.path !== state.selectedMapPath) {
    return `<div class="empty">Loading map workspace...</div>`;
  }
  return `
    <div class="map-workspace">
      <div class="map-workspace-top">
        <div>
          <h3>${esc(detail.map.name)}</h3>
          <div class="path" title="${esc(detail.map.path)}">${esc(detail.map.path)}</div>
        </div>
        <div class="actions">
          <button onclick="runMapStage('prepare-hd-raster', ${js(detail.map.path)})">Raster</button>
          <button onclick="runMapStage('generate-raceline', ${js(detail.map.path)})">Raceline</button>
          <button onclick="runMapStage('generate-preview', ${js(detail.map.path)})">Preview</button>
          <button onclick="fillTransferLocal(${js(detail.map.path)})">Transfer</button>
        </div>
      </div>
      <div class="map-preview-grid">
        <div class="map-preview-shell">
          <canvas id="map-preview-canvas" width="900" height="620"></canvas>
        </div>
        <aside class="map-side-panel">
          ${renderLayerToggles()}
          ${renderMapInspector(detail)}
        </aside>
      </div>
    </div>
  `;
}

function renderLayerToggles() {
  const layers = [
    ["landmark", "Landmark"],
    ["left_bound", "Left bound"],
    ["right_bound", "Right bound"],
    ["centerline", "Centerline"],
    ["raceline", "Raceline"],
    ["section_gates", "Section gates"],
    ["section_labels", "Labels"],
  ];
  return `
    <div class="inspector-block">
      <h4>Layers</h4>
      <div class="layer-grid">
        ${layers
          .map(
            ([key, label]) => `
              <label class="layer-toggle">
                <input type="checkbox" ${state.mapLayers[key] ? "checked" : ""} onchange="toggleMapLayer(${js(key)}, this.checked)" />
                <span>${esc(label)}</span>
              </label>`,
          )
          .join("")}
      </div>
    </div>
  `;
}

function renderMapInspector(detail) {
  const stats = detail.stats || {};
  const artifacts = detail.map.artifacts || {};
  const lanes = detail.hd_map?.lanes || [];
  const sections = detail.hd_map?.sections || [];
  return `
    <div class="inspector-block">
      <h4>Readiness</h4>
      <div class="artifact-grid">
        ${["cuvgl_map", "cuvslam_map", "landmark_image", "hd_map", "centerline_csv", "raceline_csv", "line_preview"]
          .map((key) => `<div class="artifact-row"><span>${esc(artifactLabel(key))}</span><strong class="${artifacts[key]?.exists ? "ok" : "missing"}">${artifacts[key]?.exists ? "ok" : "missing"}</strong></div>`)
          .join("")}
      </div>
    </div>
    <div class="inspector-block">
      <h4>Map Shape</h4>
      <div class="stat-grid">
        ${statTile("lanes", stats.lane_count || 0)}
        ${statTile("primary", stats.primary_lane_id || "-")}
        ${statTile("center pts", stats.primary_centerline_points || 0)}
        ${statTile("sections", stats.section_count || 0)}
        ${statTile("gates", stats.section_gate_count || 0)}
        ${statTile("raceline pts", stats.raceline_points || 0)}
      </div>
    </div>
    <div class="inspector-block">
      <h4>Lanes</h4>
      ${lanes.length ? lanes.map(renderLaneSummary).join("") : `<div class="notice">No HD map lanes found yet.</div>`}
    </div>
    <div class="inspector-block">
      <h4>Sections</h4>
      ${sections.length ? sections.slice(0, 8).map(renderSectionSummary).join("") : `<div class="notice">No section gates found yet.</div>`}
      ${sections.length > 8 ? `<div class="notice">${sections.length - 8} more sections hidden in this summary.</div>` : ""}
    </div>
  `;
}

function statTile(label, value) {
  return `<div class="stat-tile"><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`;
}

function renderLaneSummary(lane) {
  return `
    <div class="summary-row">
      <strong>${esc(lane.id)}${lane.primary ? " / primary" : ""}</strong>
      <span>${lane.closed_loop ? "closed" : "open"} / center ${esc((lane.centerline || []).length)} pts</span>
    </div>
  `;
}

function renderSectionSummary(section) {
  const speed = section.speed_override_mps == null ? "default" : `${section.speed_override_mps} m/s`;
  return `
    <div class="summary-row">
      <strong>${esc(section.id)}</strong>
      <span>${esc(section.start_gate_id)} -> ${esc(section.end_gate_id)} / ${esc(speed)}</span>
    </div>
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
  const target = state.jetsonTarget || {};
  const host = target.host || state.jetsonInspect?.host || (config.jetson_ips || [])[0] || "";
  const user = target.user || state.jetsonInspect?.user || config.jetson_user || "tamiya";
  const mapRoot = target.map_root || config.jetson_map_root || "";
  const recordRoot = target.record_root || config.jetson_record_root || "";
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
        <input id="jetson-map-root" value="${esc(mapRoot)}" />
      </div>
      <div class="field full">
        <label>Remote rosbag root</label>
        <input id="jetson-record-root" value="${esc(recordRoot)}" />
      </div>
      <div class="actions full">
        <button class="primary" onclick="inspectJetson()" ${state.jetsonInspectBusy ? "disabled" : ""}>
          ${state.jetsonInspectBusy ? "Inspecting..." : "Inspect Jetson"}
        </button>
        <button onclick="copyJetsonInspect()">Copy SSH</button>
        ${state.jetsonInspectBusy ? `<span class="inline-status">Checking SSH connection...</span>` : ""}
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
  if (state.jetsonInspectBusy && !result) {
    return `
      <div class="remote-state">
        <div class="state-tile checking"><span>SSH</span><strong>checking</strong></div>
        <div class="state-tile"><span>latest</span><strong>-</strong></div>
        <div class="state-tile"><span>maps</span><strong>-</strong></div>
        <div class="state-tile"><span>rosbags</span><strong>-</strong></div>
      </div>
      <div class="notice">Inspect Jetson is running. This can take up to about 12 seconds when the Jetson is slow or unreachable.</div>
    `;
  }
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
  const checkedAt = result.inspected_at ? new Date(result.inspected_at).toLocaleString() : "";
  return `
    <div class="remote-state">
      <div class="state-tile ${state.jetsonInspectBusy ? "checking" : result.ok ? "ok" : "bad"}"><span>SSH</span><strong>${state.jetsonInspectBusy ? "checking" : result.ok ? "online" : "failed"}</strong></div>
      <div class="state-tile"><span>latest</span><strong title="${esc(latest)}">${esc(shortName(latest))}</strong></div>
      <div class="state-tile"><span>maps</span><strong>${esc(maps)}</strong></div>
      <div class="state-tile"><span>rosbags</span><strong>${esc(rosbags)}</strong></div>
    </div>
    ${checkedAt ? `<div class="inline-summary">Last inspected: ${esc(checkedAt)}</div>` : ""}
    ${state.jetsonInspectBusy ? `<div class="notice">Inspect Jetson is running. Previous results are still shown until the new check finishes.</div>` : ""}
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

function readNumberInput(id, fallback) {
  const value = Number($(id)?.value);
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function readFpvForm(updateState = true) {
  const next = {
    host: $("fpv-host")?.value.trim() || state.fpv.host,
    codec: $("fpv-codec")?.value || state.fpv.codec,
    width: readNumberInput("fpv-width", state.fpv.width),
    height: readNumberInput("fpv-height", state.fpv.height),
    fps: readNumberInput("fpv-fps", state.fpv.fps),
    port: readNumberInput("fpv-port", state.fpv.port),
    payload: readNumberInput("fpv-payload", state.fpv.payload),
    displaySink: $("fpv-display-sink")?.value || state.fpv.displaySink,
    noDisplay: Boolean($("fpv-no-display")?.checked ?? state.fpv.noDisplay),
  };
  if (updateState) state.fpv = next;
  return next;
}

function buildFpvReceiverCommand(fpv = state.fpv) {
  const env = [
    `CODEC=${sh(fpv.codec)}`,
    `WIDTH=${sh(fpv.width)}`,
    `HEIGHT=${sh(fpv.height)}`,
    `FPS=${sh(fpv.fps)}`,
    `PORT=${sh(fpv.port)}`,
    `PAYLOAD=${sh(fpv.payload)}`,
    `DISPLAY_SINK=${sh(fpv.displaySink)}`,
  ];
  if (fpv.noDisplay) env.push("NO_DISPLAY=true");
  return `${env.join(" ")} ./tools/rtp_video_experiment/rtp_receiver.sh`;
}

function buildFpvJetsonCommand(fpv = state.fpv) {
  return [
    "ros2 launch jetpilot_system_launch bringup.launch.py",
    "enable_sensor_kit:=true",
    "sensor_kit_enable_rtp_stream:=true",
    `sensor_kit_rtp_host:=${sh(fpv.host || "<mac-ip>")}`,
    `sensor_kit_rtp_port:=${sh(fpv.port)}`,
    `sensor_kit_rtp_codec:=${sh(fpv.codec)}`,
    `sensor_kit_rtp_fps:=${sh(fpv.fps)}`,
    `sensor_kit_rtp_bitrate:=${sh(4000000)}`,
    `sensor_kit_rtp_gop:=${sh(fpv.fps)}`,
    `sensor_kit_rtp_mtu:=${sh(1200)}`,
    `sensor_kit_rtp_payload:=${sh(fpv.payload)}`,
  ].join(" \\\n  ");
}

async function startFpvViewer() {
  const fpv = readFpvForm();
  const command = buildFpvReceiverCommand(fpv);
  const result = await api("/api/tasks/run", {
    method: "POST",
    body: JSON.stringify({
      title: `FPV RTP Viewer ${fpv.codec} ${fpv.width}x${fpv.height}@${fpv.fps}`,
      kind: "fpv-viewer",
      command,
      cwd: state.config?.repo_root || "",
    }),
  });
  await refreshAll();
  selectTask(result.task.task_id);
  toast("FPV viewer started");
}

function copyFpvReceiverCommand() {
  const fpv = readFpvForm();
  const command = buildFpvReceiverCommand(fpv);
  const preview = $("fpv-command");
  if (preview) preview.value = command;
  copyText(command, "FPV receiver command copied");
}

function copyFpvJetsonCommand() {
  const fpv = readFpvForm();
  const command = buildFpvJetsonCommand(fpv);
  const preview = $("fpv-jetson-command");
  if (preview) preview.value = command;
  copyText(command, "Jetson bringup command copied");
}

function setFpvHost(host) {
  state.fpv.host = host;
  const input = $("fpv-host");
  if (input) input.value = host;
  updateFpvCommandPreview();
}

function updateFpvCommandPreview() {
  const fpv = readFpvForm();
  const receiverPreview = $("fpv-command");
  if (receiverPreview) receiverPreview.value = buildFpvReceiverCommand(fpv);
  const jetsonPreview = $("fpv-jetson-command");
  if (jetsonPreview) jetsonPreview.value = buildFpvJetsonCommand(fpv);
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

async function openMapWorkspace(path) {
  state.tab = "maps";
  state.selectedMapPath = path;
  state.selectedMapDetail = null;
  render();
  try {
    const detail = await api(apiPath("/api/maps/detail", { path }));
    state.selectedMapDetail = detail;
    render();
  } catch (error) {
    toast(`Map load failed: ${error.message}`, "error");
    render();
  }
}

function refreshSelectedMap() {
  if (!state.selectedMapPath) return;
  return openMapWorkspace(state.selectedMapPath);
}

function toggleMapLayer(layer, checked) {
  state.mapLayers[layer] = Boolean(checked);
  drawMapPreview();
}

function drawMapPreview() {
  const canvas = $("map-preview-canvas");
  if (!canvas || !state.selectedMapDetail) return;
  const detail = state.selectedMapDetail;
  const imageUrl = detail.raster?.image_url || detail.preview_image_url || "";
  const draw = (image = null) => {
    const raster = detail.raster || {};
    const naturalWidth = image?.naturalWidth || raster.width || 900;
    const naturalHeight = image?.naturalHeight || raster.height || 620;
    canvas.width = Math.max(320, naturalWidth);
    canvas.height = Math.max(240, naturalHeight);
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#0b0d10";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (image && state.mapLayers.landmark) {
      ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "rgba(8, 10, 12, 0.08)";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    } else {
      drawGrid(ctx, canvas.width, canvas.height);
    }
    drawMapLayers(ctx, detail, canvas.width, canvas.height);
  };
  if (imageUrl) {
    const image = new Image();
    image.onload = () => draw(image);
    image.onerror = () => draw(null);
    image.src = imageUrl;
  } else {
    draw(null);
  }
}

function drawGrid(ctx, width, height) {
  ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y < height; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawMapLayers(ctx, detail, width, height) {
  const toPixel = mapPointProjector(detail, width, height);
  const lanes = detail.hd_map?.lanes || [];
  for (const lane of lanes) {
    if (state.mapLayers.left_bound) drawPolyline(ctx, (lane.left_bound || []).map(toPixel), "#45c478", 3, lane.closed_loop);
    if (state.mapLayers.right_bound) drawPolyline(ctx, (lane.right_bound || []).map(toPixel), "#d878d8", 3, lane.closed_loop);
    if (state.mapLayers.centerline) drawPolyline(ctx, (lane.centerline || []).map(toPixel), "#e7c84b", lane.primary ? 4 : 2, lane.closed_loop);
  }
  if (state.mapLayers.centerline) {
    drawPolyline(ctx, (detail.centerline_csv?.points || []).map(toPixel), "#5aa8ff", 2, false);
  }
  if (state.mapLayers.raceline) {
    drawPolyline(ctx, (detail.raceline_csv?.points || []).map(toPixel), "#ff6d6d", 3, false);
  }
  if (state.mapLayers.section_gates) {
    for (const gate of detail.hd_map?.section_gates || []) {
      const line = (gate.line || []).map(toPixel);
      drawPolyline(ctx, line, "#ffffff", 2, false);
      if (state.mapLayers.section_labels && line.length >= 2) {
        const x = (line[0][0] + line[1][0]) * 0.5;
        const y = (line[0][1] + line[1][1]) * 0.5;
        drawLabel(ctx, gate.id, x + 6, y - 6);
      }
    }
  }
}

function drawPolyline(ctx, points, color, width, closed) {
  const clean = points.filter((point) => Number.isFinite(point[0]) && Number.isFinite(point[1]));
  if (clean.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(clean[0][0], clean[0][1]);
  for (const point of clean.slice(1)) ctx.lineTo(point[0], point[1]);
  if (closed && clean.length >= 3) ctx.closePath();
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(clean[0][0], clean[0][1], Math.max(3, width + 1), 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawLabel(ctx, text, x, y) {
  ctx.save();
  ctx.font = "12px ui-sans-serif, system-ui";
  const metrics = ctx.measureText(text);
  ctx.fillStyle = "rgba(8, 10, 12, 0.78)";
  ctx.fillRect(x - 4, y - 15, metrics.width + 8, 19);
  ctx.fillStyle = "#f2f5f8";
  ctx.fillText(text, x, y);
  ctx.restore();
}

function mapPointProjector(detail, width, height) {
  const raster = detail.raster || {};
  if (raster.resolution_m_per_px && raster.width && raster.height) {
    const origin = raster.origin_xy_yaw || [0, 0, 0];
    const scaleX = width / raster.width;
    const scaleY = height / raster.height;
    return (point) => {
      const dx = Number(point[0]) - Number(origin[0] || 0);
      const dy = Number(point[1]) - Number(origin[1] || 0);
      const yaw = Number(origin[2] || 0);
      const cos = Math.cos(yaw);
      const sin = Math.sin(yaw);
      const gridX = (cos * dx + sin * dy) / raster.resolution_m_per_px;
      const gridY = (-sin * dx + cos * dy) / raster.resolution_m_per_px;
      return [gridX * scaleX, ((raster.height - 1) - gridY) * scaleY];
    };
  }

  const points = collectMapPoints(detail);
  if (!points.length) return (point) => [Number(point[0] || 0), Number(point[1] || 0)];
  const xs = points.map((point) => Number(point[0]));
  const ys = points.map((point) => Number(point[1]));
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const pad = 32;
  const spanX = Math.max(1e-6, maxX - minX);
  const spanY = Math.max(1e-6, maxY - minY);
  const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
  return (point) => [pad + (Number(point[0]) - minX) * scale, height - pad - (Number(point[1]) - minY) * scale];
}

function collectMapPoints(detail) {
  const points = [];
  for (const lane of detail.hd_map?.lanes || []) {
    points.push(...(lane.left_bound || []), ...(lane.right_bound || []), ...(lane.centerline || []));
  }
  for (const gate of detail.hd_map?.section_gates || []) points.push(...(gate.line || []));
  points.push(...(detail.centerline_csv?.points || []), ...(detail.raceline_csv?.points || []));
  return points.filter((point) => Array.isArray(point) && point.length >= 2);
}

function fillTransferLocal(path) {
  state.tab = "jetson";
  render();
  $("push-local").value = path;
  $("push-remote").value = state.config?.jetson_map_root || "";
}

async function inspectJetson() {
  const target = {
    host: $("jetson-host").value,
    user: $("jetson-user").value,
    map_root: $("jetson-map-root").value,
    record_root: $("jetson-record-root").value,
  };
  state.jetsonTarget = target;
  state.jetsonInspectBusy = true;
  render();
  const params = new URLSearchParams(target);
  try {
    const result = await api(`/api/jetson/inspect?${params.toString()}`);
    state.jetsonInspect = { ...result, inspected_at: new Date().toISOString() };
  } catch (error) {
    state.jetsonInspect = {
      ok: false,
      host: target.host,
      user: target.user,
      error: error.message || String(error),
      output: "",
      inspected_at: new Date().toISOString(),
    };
  } finally {
    state.jetsonInspectBusy = false;
    render();
  }
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
  copyText(`rsync -avhP ${sh(`${target.user}@${target.host}:${pullRemotePath()}`)} ${sh(trimTrailingSlash(pullLocalPath()) + "/")}`);
}

function copyPushCommand() {
  const target = jetsonTarget();
  copyText(`rsync -avhP ${sh(trimTrailingSlash($("push-local").value) + "/")} ${sh(`${target.user}@${target.host}:${trimTrailingSlash($("push-remote").value)}/`)}`);
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
window.startFpvViewer = startFpvViewer;
window.copyFpvReceiverCommand = copyFpvReceiverCommand;
window.copyFpvJetsonCommand = copyFpvJetsonCommand;
window.setFpvHost = setFpvHost;
window.updateFpvCommandPreview = updateFpvCommandPreview;
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
window.openMapWorkspace = openMapWorkspace;
window.refreshSelectedMap = refreshSelectedMap;
window.toggleMapLayer = toggleMapLayer;
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
    .then(async (data) => {
      const nextTasks = data.tasks || [];
      if (shouldRefreshMapsAfterTaskPoll(state.tasks, nextTasks)) {
        state.tasks = nextTasks;
        await refreshAll();
        return;
      }
      state.tasks = nextTasks;
      if (!isEditingField() && !logHistoryIsBeingRead()) render();
      else updateTaskChrome();
    })
    .catch(() => {});
}, 5000);
