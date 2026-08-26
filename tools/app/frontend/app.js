const DEFAULT_RACELINE_VEHICLE_WIDTH_M = 0.25;
const DEFAULT_RACELINE_SAFETY_MARGIN_M = 0.05;
const DEFAULT_RACELINE_MAX_SPEED_MPS = 3.0;
const DEFAULT_RACELINE_MIN_SPEED_MPS = 0.8;
const DEFAULT_RACELINE_LATERAL_ACCEL_LIMIT_MPS2 = 2.5;
const DEFAULT_RACELINE_ACCEL_LIMIT_MPS2 = 1.5;
const DEFAULT_RACELINE_DECEL_LIMIT_MPS2 = 2.5;
const DEFAULT_HD_RASTER_AUTO_CROP_PERCENTILE = 99.0;
const DEFAULT_HD_RASTER_AUTO_CROP_MIN_RETAINED_RATIO = 0.75;
const DEFAULT_HD_RASTER_PATH_CROP_DISTANCE_M = 0.0;
const DEFAULT_HD_RASTER_PATH_CROP_MIN_RETAINED_RATIO = 0.2;
const PREFLIGHT_CACHE_MS = 15_000;
const LIVE_LOG_INITIAL_TAIL_LINES = 400;
const MAX_LIVE_LOG_CHARS = 400_000;
const LIVE_LOG_TRUNCATION_NOTICE = "[Showing the latest live log only. Use Copy Log for the full log.]\n...\n";
const DEFAULT_MAP_EDITOR_ASSIST_RANGE_POINTS = 4;
const DEFAULT_MAP_EDITOR_ASSIST_SPACING_M = 0.08;
const MAX_MAP_EDITOR_ASSIST_RANGE_POINTS = 30;
const MAX_MAP_EDITOR_ASSIST_RESAMPLE_POINTS = 1200;
const MAP_EDITOR_FIELDS = ["left_bound", "right_bound", "centerline"];

const state = {
  tab: "dashboard",
  config: null,
  tasks: [],
  rosbags: [],
  rosbagTrash: [],
  maps: [],
  e2eModels: [],
  e2ePipeline: {
    datasets: [],
    runs: [],
    experiments: [],
    deployProfiles: [],
    deployPresets: [],
    datasetRoot: "",
    runRoot: "",
    datasetName: "e2e_dataset",
    datasetDir: "",
    runName: "pilotnet_run",
    runDir: "",
    experiment: "pilotnet_scratch",
    datasetTask: "control",
    imageTopic: "/realsense/color/image_raw",
    controlTopic: "/teleop/control_cmd",
    odometryTopic: "/visual_slam/tracking/odometry",
    imuTopic: "/sensors/imu",
    inputWidth: 212,
    inputHeight: 120,
    maxControlDtSec: 0.1,
    maxOdometryDtSec: 0.15,
    trajectoryPoints: 10,
    trajectoryHorizonSec: 1.5,
    trajectoryScaleM: 5.0,
    imuWindowSec: 0.5,
    imuSamples: 10,
    jpegQuality: 92,
    batchSize: 64,
    numWorkers: 4,
    epochs: 30,
    learningRate: 0.001,
    finetuneEpochs: 20,
    finetuneLearningRate: 0.0001,
    valFraction: 0.2,
    fraction: 1.0,
    weightDecay: 0.0001,
    seed: 42,
    device: "",
    deployProfile: "",
    deployPreset: "",
    deployHost: "",
    deployUser: "",
    remoteRoot: "",
    buildEngine: true,
  },
  objectDetectionPipeline: {
    datasets: [],
    runs: [],
    models: [],
    baseModels: ["yolov8n.pt"],
    deployProfiles: [],
    trainingRoot: "",
    datasetRoot: "",
    runRoot: "",
    modelRoot: "",
    datasetYaml: "",
    runDir: "",
    sourceRunDir: "",
    runName: "yolov8n_224",
    trainingMode: "train",
    baseModel: "yolov8n.pt",
    epochs: 100,
    batch: 32,
    device: "0",
    workers: 8,
    patience: 30,
    seed: 42,
    freeze: 0,
    opset: 17,
    simplify: true,
    deployProfile: "",
    deployHost: "",
    deployUser: "",
    remoteRoot: "",
    deployName: "yolov8n_224",
    buildEngine: true,
  },
  listControls: {
    rosbags: { query: "", sort: "newest", groupByDate: true, collapsedDays: {} },
    rosbagTrash: { query: "", sort: "newest", groupByDate: true, collapsedDays: {} },
    maps: { query: "", sort: "newest", groupByDate: true, collapsedDays: {} },
    analyses: { query: "", sort: "newest", groupByDate: true, collapsedDays: {} },
  },
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
    odometry: false,
    section_gates: true,
    section_labels: true,
  },
  mapEditor: {
    enabled: false,
    mapPath: "",
    activeField: "left_bound",
    dirty: false,
    selected: null,
    dragging: null,
    zoom: 1,
    showCenterline: true,
    primaryLaneId: "lane_001",
    lanes: [],
    undoStack: [],
    redoStack: [],
    dragSnapshot: null,
    assistRangePoints: DEFAULT_MAP_EDITOR_ASSIST_RANGE_POINTS,
    assistSpacingM: DEFAULT_MAP_EDITOR_ASSIST_SPACING_M,
  },
  sectionEditor: {
    enabled: false,
    mapPath: "",
    dirty: false,
    selectedGateId: "",
    gates: [],
  },
  racelineGeneration: {
    vehicleWidthM: DEFAULT_RACELINE_VEHICLE_WIDTH_M,
    safetyMarginM: DEFAULT_RACELINE_SAFETY_MARGIN_M,
    maxSpeedMps: DEFAULT_RACELINE_MAX_SPEED_MPS,
    minSpeedMps: DEFAULT_RACELINE_MIN_SPEED_MPS,
    lateralAccelLimitMps2: DEFAULT_RACELINE_LATERAL_ACCEL_LIMIT_MPS2,
    accelLimitMps2: DEFAULT_RACELINE_ACCEL_LIMIT_MPS2,
    decelLimitMps2: DEFAULT_RACELINE_DECEL_LIMIT_MPS2,
  },
  hdRasterGeneration: {
    cropMode: "path",
    autoCropPercentile: DEFAULT_HD_RASTER_AUTO_CROP_PERCENTILE,
    autoCropMinRetainedRatio: DEFAULT_HD_RASTER_AUTO_CROP_MIN_RETAINED_RATIO,
    pathCropDistanceM: 1.5,
    pathCropMinRetainedRatio: DEFAULT_HD_RASTER_PATH_CROP_MIN_RETAINED_RATIO,
  },
  simulation: {
    source: "raceline",
    playing: false,
    mapPath: "",
    rafId: 0,
    lastTickMs: 0,
    x: 0,
    y: 0,
    yaw: 0,
    speed: 0,
    time: 0,
    distance: 0,
    lapCount: 0,
    lastProgress: 0,
    nearestIndex: 0,
    targetIndex: 0,
    targetPoint: null,
    steeringRad: 0,
    throttle: 0,
    brake: 0,
    crossTrackErrorM: 0,
    maxCrossTrackErrorM: 0,
    trajectory: [],
    settings: {
      targetSpeedMps: 2.0,
      wheelbaseM: 0.26,
      minLookaheadM: 0.45,
      maxLookaheadM: 1.8,
      lookaheadGainS: 0.35,
      maxSteeringRad: 0.45,
      maxAccelMps2: 1.4,
      maxDecelMps2: 2.3,
      dragPerS: 0.04,
      dtS: 0.02,
    },
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
    transport: "webrtc",
    displaySink: "glimagesink",
    noDisplay: false,
    starting: false,
    webrtcPlaying: false,
    webrtcClientState: "closed",
    webrtcClientError: "",
    webrtcStats: null,
    webrtcLastProgressAtMs: 0,
    webrtcLastFramesDecoded: 0,
    webrtcLastVideoTime: 0,
    webrtcRtpObservedAtMs: 0,
    browserStatus: {
      available: false,
      running: false,
      session_id: "",
      frame_count: 0,
      jpeg_bytes: 0,
      rtp_packet_count: 0,
      rtp_bytes: 0,
      settings: null,
      last_frame_age_s: null,
      last_error: "",
    },
  },
  ui: {
    dashboardWorkflow: "overview",
    pendingActions: {},
  },
  terminalCollapsed: false,
  logDialogOpen: false,
  logStickToEnd: true,
  logText: "",
  stream: null,
  jetsonTarget: null,
  jetsonInspect: null,
  jetsonInspectBusy: false,
  jetsonTransfer: {
    selectedPullPaths: [],
    activePullPaths: [],
    currentPath: "",
    running: false,
    currentIndex: 0,
    total: 0,
  },
  preflight: {
    entries: {},
    pendingExecutions: {},
    revisions: {},
  },
  analysis: {
    selectedBagPath: "",
    selectedMapPath: "",
    topicConfigPath: "",
    bagDetail: null,
    bagDetailLoading: false,
    imageTopic: "",
    controlTopic: "",
    modeTopic: "",
    poseTopic: "",
    speedTopic: "",
    trajectoryMode: "auto",
    offlineLocalizationMode: "auto",
    maxFps: 15,
    offlineObjectDetection: false,
    objectDetectionModelRoot: "/workspaces/ros2_ws/models/yolov8/latest",
    objectDetectionImageTopic: "",
    objectDetectionCameraInfoTopic: "",
    objectDetectionSourceWidth: 424,
    objectDetectionSourceHeight: 240,
    objectDetectionNetworkWidth: 224,
    objectDetectionNetworkHeight: 224,
    objectDetectionMaxFps: 15,
    objectDetectionConfidence: 0.35,
    objectDetectionNms: 0.45,
    objectDetectionReplayRate: 0.5,
    analyses: [],
    selectedId: "",
    detail: null,
    timeline: null,
    mapDetail: null,
    loadingResult: false,
    playing: false,
    currentTime: 0,
    playbackRate: 1,
    rafId: 0,
    lastTickMs: 0,
    renderedFrameIndex: -1,
    renderedFrameKey: "",
    lastVisualUpdateMs: 0,
    imageViewMode: "single",
    renderedChannel: "",
    activeBufferImg: "",
    pendingFrameKey: "",
    frameRequestSerial: 0,
    channelRender: {},
    visualFailures: {},
    jetsonSelectedSeries: [],
    e2eMode: "supervised",
    e2eModelPath: "",
    e2eProvider: "auto",
    e2eTeacherTopic: "/teleop/control_cmd",
    e2ePredictionTopic: "/auto/control_cmd",
    e2eAppliedTopic: "/vehicle/control_cmd",
    e2eDiagnosticTopic: "/e2e/diagnostics",
    e2eImuTopic: "/sensors/imu",
    e2eSectionTopic: "/localization/current_section",
    e2eManualOnly: true,
    e2eDeadlineMs: 33.3,
  },
};

const tabs = [
  ["dashboard", "Dashboard"],
  ["rosbags", "Rosbags"],
  ["bag-analysis", "Bag Analysis"],
  ["object-detection", "Object Detection"],
  ["e2e-analysis", "E2E Analysis"],
  ["map-builder", "Map Builder"],
  ["joy-profile", "Joy Profile"],
  ["fpv", "FPV"],
  ["maps", "Maps"],
  ["jetson", "Jetson"],
  ["terminal", "Terminal"],
];

const mapPreviewImages = new Map();
const preflightRequests = new Map();
let selectedMapRefreshInFlight = false;
let mapBuildPreflightTimer = null;
let racelinePreflightTimer = null;
let analysisPreflightTimer = null;
let forceVisiblePreflightOnce = false;
let fpvHeartbeatBusy = false;
let fpvPeerConnection = null;
let fpvPeerSessionId = "";
let fpvRemoteStream = null;
let fpvLifecycleGeneration = 0;

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
    const error = new Error(message);
    error.status = response.status;
    error.payload = payload;
    throw error;
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

function stablePreflightValue(value) {
  if (Array.isArray(value)) return value.map(stablePreflightValue);
  if (!value || typeof value !== "object") return value;
  return Object.keys(value)
    .sort()
    .reduce((result, key) => {
      if (value[key] !== undefined) result[key] = stablePreflightValue(value[key]);
      return result;
    }, {});
}

function preflightKey(action, payload = {}) {
  return JSON.stringify({ action, payload: stablePreflightValue(payload) });
}

function preflightToken(action, payload = {}) {
  const key = preflightKey(action, payload);
  let hash = 2166136261;
  for (let index = 0; index < key.length; index += 1) {
    hash ^= key.charCodeAt(index);
    hash = Math.imul(hash, 16777619);
  }
  return `pf-${(hash >>> 0).toString(36)}-${key.length.toString(36)}`;
}

function preflightText(value) {
  if (value == null) return "";
  if (typeof value === "string" || typeof value === "number") return String(value);
  if (Array.isArray(value)) return value.map(preflightText).filter(Boolean).join("; ");
  return preflightText(value.message || value.detail || value.title || value.reason || value.action);
}

function normalizePreflightStatus(value, fallback = "checking") {
  const status = String(value || "").trim().toLowerCase().replaceAll("_", "-");
  if (["pass", "passed", "ok", "ready", "success", "done"].includes(status)) return "pass";
  if (["warning", "warn", "optional", "degraded"].includes(status)) return "warning";
  if (["blocked", "block", "missing", "failed", "fail", "error", "invalid"].includes(status)) return "blocked";
  if (["checking", "loading", "pending", "queued", "running"].includes(status)) return "checking";
  if (["unavailable", "unknown"].includes(status)) return "unavailable";
  return fallback;
}

function normalizePreflightCheck(check = {}, index = 0) {
  let status = normalizePreflightStatus(check.status || check.state || check.severity, "");
  if (!status) {
    if (check.passed === true || check.ok === true) status = "pass";
    else if (check.passed === false || check.ok === false) status = check.required === false ? "warning" : "blocked";
    else status = "checking";
  }
  return {
    id: String(check.id || `check-${index + 1}`),
    status,
    title: preflightText(check.title || check.label || check.name || check.id) || `Requirement ${index + 1}`,
    detail: preflightText(check.detail || check.message || check.reason),
    remediation: preflightText(check.remediation || check.fix || check.next || check.action),
  };
}

function normalizePreflightResult(action, payload, result = {}) {
  const checks = Array.isArray(result.checks) ? result.checks.map(normalizePreflightCheck) : [];
  let status = normalizePreflightStatus(result.status, "");
  if (!status) {
    if (checks.some((check) => check.status === "blocked")) status = "blocked";
    else if (checks.some((check) => check.status === "warning")) status = "warning";
    else if (result.ready === true) status = "pass";
    else if (result.ready === false) status = "blocked";
    else status = "checking";
  }
  const ready = result.ready == null ? ["pass", "warning"].includes(status) : Boolean(result.ready);
  const summary = preflightText(result.summary) || {
    pass: "All required inputs are available.",
    warning: "Ready to run with warnings.",
    blocked: "Required inputs are missing.",
    checking: "Checking the selected inputs.",
  }[status] || "Preflight status is unavailable.";
  return {
    action,
    payload,
    status,
    ready,
    summary,
    checks,
    error: "",
    updatedAt: Date.now(),
  };
}

function preflightEntry(action, payload = {}) {
  return state.preflight.entries[preflightKey(action, payload)] || null;
}

function preflightStatusLabel(status) {
  return {
    pass: "Ready",
    warning: "Ready with warnings",
    blocked: "Blocked",
    checking: "Checking",
    unavailable: "Check unavailable",
  }[status] || "Checking";
}

function preflightBlockingReason(entry) {
  if (!entry || entry.status === "checking") return "Checking requirements before this task can run.";
  if (entry.status === "unavailable") return "Preflight is unavailable. Retry the check before starting this task.";
  const blocker = entry.checks?.find((check) => check.status === "blocked");
  if (blocker) return blocker.remediation || blocker.detail || blocker.title;
  return entry.ready ? "" : entry.summary || "Required inputs are missing.";
}

function preflightExecutionMapDir(payload = {}) {
  return trimTrailingSlash(payload.map_dir || "");
}

function preflightExecutionResource(action, payload = {}) {
  if (action === "analyze-rosbag") return trimTrailingSlash(payload.rosbag || "");
  return preflightExecutionMapDir(payload);
}

function preflightExecutionToken(action, payload = {}) {
  return preflightToken("execution", {
    action,
    resource: preflightExecutionResource(action, payload),
  });
}

function preflightMapResourceToken(payload = {}) {
  return preflightToken("map-resource", { map_dir: preflightExecutionMapDir(payload) });
}

function commandContainsMapDir(command, mapDir) {
  if (!mapDir) return false;
  const escaped = mapDir.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  return new RegExp(`(^|[^A-Za-z0-9_.-])${escaped}(?=$|[^A-Za-z0-9_.-])`).test(
    (command || []).join("\n"),
  );
}

function runningPreflightTask(action, payload = {}) {
  if (action === "analyze-rosbag") {
    const rosbag = trimTrailingSlash(payload.rosbag || "");
    if (!rosbag) return null;
    return state.tasks.find((task) => {
      if (!isActiveTask(task) || task.kind !== action) return false;
      if (trimTrailingSlash(task._preflightRosbag || "") === rosbag) return true;
      return commandContainsMapDir(task.command, rosbag);
    }) || null;
  }
  const mapDir = preflightExecutionMapDir(payload);
  if (!mapDir) return null;
  return state.tasks.find((task) => {
    if (!isActiveTask(task)) return false;
    if (task.resource_key === `map-dir:${mapDir}`) return true;
    if (task.kind !== action) return false;
    if (task._preflightAction === action && trimTrailingSlash(task._preflightMapDir || "") === mapDir) return true;
    return commandContainsMapDir(task.command, mapDir);
  }) || null;
}

function preflightExecutionState(action, payload = {}) {
  const token = preflightExecutionToken(action, payload);
  const resource = preflightExecutionResource(action, payload);
  const mapDir = preflightExecutionMapDir(payload);
  const pending = state.preflight.pendingExecutions[token]
    || Object.values(state.preflight.pendingExecutions).find((item) => item.resource && item.resource === resource);
  if (pending) {
    return {
      locked: true,
      reason: "This task is being checked and started. Please wait.",
    };
  }
  const task = runningPreflightTask(action, payload);
  if (task) {
    return {
      locked: true,
      reason: `${task.title || action} is already ${task.status} for this ${action === "analyze-rosbag" ? "rosbag" : "map"}.`,
    };
  }
  return { locked: false, reason: "" };
}

function preflightAllowsRun(entry) {
  return Boolean(entry && entry.ready && ["pass", "warning"].includes(entry.status));
}

function preflightButtonState(action, payload = {}) {
  const entry = preflightEntry(action, payload);
  const execution = preflightExecutionState(action, payload);
  const disabled = execution.locked || !preflightAllowsRun(entry);
  return {
    entry,
    disabled,
    reason: execution.reason || (disabled ? preflightBlockingReason(entry) : ""),
  };
}

function preflightButtonAttrs(action, payload = {}) {
  const token = preflightToken(action, payload);
  const buttonState = preflightButtonState(action, payload);
  return [
    `data-preflight-token="${esc(token)}"`,
    `data-preflight-execution-token="${esc(preflightExecutionToken(action, payload))}"`,
    `data-preflight-map-resource-token="${esc(preflightMapResourceToken(payload))}"`,
    'data-preflight-role="button"',
    buttonState.disabled ? "disabled" : "",
    buttonState.reason ? `title="${esc(buttonState.reason)}"` : "",
    buttonState.disabled ? 'aria-disabled="true"' : 'aria-disabled="false"',
  ].filter(Boolean).join(" ");
}

function preflightCheckIcon(status) {
  return { pass: "✓", warning: "!", blocked: "×", checking: "…", unavailable: "?" }[status] || "…";
}

function preflightPanelContent(entry, options = {}) {
  const current = entry || {
    status: "checking",
    ready: false,
    summary: "Checking the selected inputs and available data.",
    checks: [],
  };
  let checks = current.checks || [];
  if (options.micro) {
    checks = checks.filter((check) => check.status !== "pass").slice(0, 1);
  } else if (options.compact) {
    const actionable = checks.filter((check) => check.status !== "pass");
    checks = actionable.length ? actionable.slice(0, 3) : checks.slice(0, 2);
  }
  const checksHtml = checks.length
    ? `<div class="preflight-checks">${checks.map((check) => `
        <div class="preflight-check ${esc(check.status)}">
          <span class="preflight-check-icon" aria-hidden="true">${preflightCheckIcon(check.status)}</span>
          <div>
            <strong>${esc(check.title)}</strong>
            ${check.detail ? `<p>${esc(check.detail)}</p>` : ""}
            ${check.remediation ? `<div class="preflight-remediation"><span>Next:</span> ${esc(check.remediation)}</div>` : ""}
          </div>
        </div>`).join("")}</div>`
    : current.status === "checking" && !options.micro
      ? `<div class="preflight-check checking"><span class="preflight-check-icon" aria-hidden="true">…</span><div><strong>Inspecting requirements</strong><p>This updates automatically when an input changes.</p></div></div>`
      : "";
  const execution = current.action ? preflightExecutionState(current.action, current.payload || {}) : { locked: false, reason: "" };
  const executionHtml = execution.locked
    ? `<div class="preflight-execution-note"><span aria-hidden="true">…</span>${esc(execution.reason)}</div>`
    : "";
  const retryHtml = ["blocked", "unavailable"].includes(current.status) && current.action
    ? `<div class="preflight-retry"><span>${current.status === "unavailable" ? "Execution stays locked until readiness can be verified." : "Inputs may have changed since the last check."}</span><button onclick="retryPreflightToken('${esc(preflightToken(current.action, current.payload || {}))}')">${current.status === "unavailable" ? "Retry check" : "Recheck"}</button></div>`
    : "";
  return `
    <div class="preflight-heading">
      <div>
        <span class="preflight-eyebrow">Preflight</span>
        <strong>${esc(options.title || "Task readiness")}</strong>
      </div>
      <span class="preflight-status ${esc(current.status)}">${esc(preflightStatusLabel(current.status))}</span>
    </div>
    <p class="preflight-summary">${esc(current.summary)}</p>
    ${executionHtml}
    ${checksHtml}
    ${retryHtml}
  `;
}

function renderReadinessPanel(action, payload = {}, options = {}) {
  const token = preflightToken(action, payload);
  return `
    <div
      class="preflight-panel ${options.compact ? "compact" : ""} ${options.micro ? "micro" : ""}"
      data-preflight-token="${esc(token)}"
      data-preflight-execution-token="${esc(preflightExecutionToken(action, payload))}"
      data-preflight-map-resource-token="${esc(preflightMapResourceToken(payload))}"
      data-preflight-role="panel"
      data-preflight-title="${esc(options.title || "Task readiness")}"
      data-preflight-compact="${options.compact ? "true" : "false"}"
      data-preflight-micro="${options.micro ? "true" : "false"}"
    >${preflightPanelContent(preflightEntry(action, payload), options)}</div>
  `;
}

function renderPreflightButtonReason(action, payload = {}) {
  const token = preflightToken(action, payload);
  const buttonState = preflightButtonState(action, payload);
  return `<div class="preflight-button-reason ${buttonState.reason ? "visible" : ""}" data-preflight-token="${esc(token)}" data-preflight-execution-token="${esc(preflightExecutionToken(action, payload))}" data-preflight-map-resource-token="${esc(preflightMapResourceToken(payload))}" data-preflight-role="reason">${esc(buttonState.reason)}</div>`;
}

function preflightEntryForToken(token) {
  return Object.values(state.preflight.entries).find(
    (entry) => preflightToken(entry.action, entry.payload || {}) === token,
  ) || null;
}

function applyPreflightDomElement(element, entry, buttonState) {
  const role = element.dataset.preflightRole;
  if (role === "button") {
    element.disabled = buttonState.disabled;
    element.setAttribute("aria-disabled", String(buttonState.disabled));
    element.classList.toggle("preflight-unverified", entry?.status === "unavailable");
    if (buttonState.reason) element.title = buttonState.reason;
    else element.removeAttribute("title");
  } else if (role === "reason") {
    element.textContent = buttonState.reason;
    element.classList.toggle("visible", Boolean(buttonState.reason));
  } else if (role === "panel") {
    element.classList.toggle("compact", element.dataset.preflightCompact === "true");
    element.classList.toggle("micro", element.dataset.preflightMicro === "true");
    element.innerHTML = preflightPanelContent(entry, {
      title: element.dataset.preflightTitle || "Task readiness",
      compact: element.dataset.preflightCompact === "true",
      micro: element.dataset.preflightMicro === "true",
    });
  }
}

function updatePreflightDom(action, payload = {}) {
  const token = preflightToken(action, payload);
  const executionToken = preflightExecutionToken(action, payload);
  const mapResourceToken = preflightMapResourceToken(payload);
  const entry = preflightEntry(action, payload);
  const visited = new Set();
  document.querySelectorAll(`[data-preflight-token="${token}"]`).forEach((element) => {
    visited.add(element);
    applyPreflightDomElement(element, entry, preflightButtonState(action, payload));
  });
  [
    `[data-preflight-execution-token="${executionToken}"]`,
    `[data-preflight-map-resource-token="${mapResourceToken}"]`,
  ].forEach((selector) => {
    document.querySelectorAll(selector).forEach((element) => {
      if (visited.has(element)) return;
      visited.add(element);
      const related = preflightEntryForToken(element.dataset.preflightToken || "");
      if (!related) return;
      applyPreflightDomElement(
        element,
        related,
        preflightButtonState(related.action, related.payload || {}),
      );
    });
  });
}

function refreshVisiblePreflightDom() {
  Object.values(state.preflight.entries).forEach((entry) => {
    updatePreflightDom(entry.action, entry.payload || {});
  });
}

async function preflightApi(action, payload = {}) {
  const response = await fetch("/api/preflight", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ ...payload, action }),
  });
  const contentType = response.headers.get("content-type") || "";
  const result = contentType.includes("application/json") ? await response.json() : await response.text();
  if (!response.ok && !(result && typeof result === "object" && ("ready" in result || "checks" in result))) {
    const message = typeof result === "string" ? result : result?.error || "preflight request failed";
    throw new Error(message);
  }
  return result;
}

function requestPreflight(action, payload = {}, options = {}) {
  const key = preflightKey(action, payload);
  const revision = state.preflight.revisions[key] || 0;
  const existing = state.preflight.entries[key];
  if (!options.force && existing && Date.now() - existing.updatedAt < PREFLIGHT_CACHE_MS) {
    updatePreflightDom(action, payload);
    return Promise.resolve(existing);
  }
  if (preflightRequests.has(key)) return preflightRequests.get(key);

  if (!existing) {
    state.preflight.entries[key] = {
      action,
      payload,
      status: "checking",
      ready: false,
      summary: "Checking the selected inputs and available data.",
      checks: [],
      error: "",
      updatedAt: Date.now(),
    };
    updatePreflightDom(action, payload);
  }

  const request = preflightApi(action, payload)
    .then((result) => {
      const entry = normalizePreflightResult(action, payload, result);
      if ((state.preflight.revisions[key] || 0) === revision) {
        state.preflight.entries[key] = entry;
        updatePreflightDom(action, payload);
      }
      return entry;
    })
    .catch((error) => {
      const entry = {
        action,
        payload,
        status: "unavailable",
        ready: false,
        summary: "The readiness service could not be reached. Retry the check before starting this task.",
        checks: [],
        error: error.message || String(error),
        updatedAt: Date.now(),
      };
      if ((state.preflight.revisions[key] || 0) === revision) {
        state.preflight.entries[key] = entry;
        updatePreflightDom(action, payload);
      }
      return entry;
    })
    .finally(() => {
      if (preflightRequests.get(key) === request) preflightRequests.delete(key);
    });
  preflightRequests.set(key, request);
  return request;
}

async function confirmPreflight(action, payload = {}) {
  const entry = await requestPreflight(action, payload, { force: true });
  if (preflightAllowsRun(entry)) return true;
  toast(preflightBlockingReason(entry), "error");
  return false;
}

function retryPreflightToken(token) {
  const entry = Object.values(state.preflight.entries).find(
    (item) => preflightToken(item.action, item.payload || {}) === token,
  );
  if (!entry) return;
  requestPreflight(entry.action, entry.payload || {}, { force: true });
}

function acquirePreflightExecution(action, payload = {}) {
  const token = preflightExecutionToken(action, payload);
  if (preflightExecutionState(action, payload).locked) return false;
  state.preflight.pendingExecutions[token] = {
    action,
    mapDir: preflightExecutionMapDir(payload),
    resource: preflightExecutionResource(action, payload),
    startedAt: Date.now(),
  };
  updatePreflightDom(action, payload);
  return true;
}

function releasePreflightExecution(action, payload = {}) {
  delete state.preflight.pendingExecutions[preflightExecutionToken(action, payload)];
  updatePreflightDom(action, payload);
}

function rememberStartedTask(task, action, payload = {}) {
  if (!task?.task_id) return;
  const remembered = {
    ...task,
    _preflightAction: action,
    _preflightMapDir: preflightExecutionMapDir(payload),
    _preflightRosbag: action === "analyze-rosbag" ? trimTrailingSlash(payload.rosbag || "") : "",
  };
  const index = state.tasks.findIndex((item) => item.task_id === task.task_id);
  if (index >= 0) state.tasks[index] = remembered;
  else state.tasks.unshift(remembered);
  if (!state.selectedTaskId) state.selectedTaskId = remembered.task_id;
}

function captureMapTaskConflict(error) {
  const active = error?.payload?.active_task;
  if (!active?.task_id) return false;
  const resourceKey = String(active.resource_key || "");
  const mapDir = resourceKey.startsWith("map-dir:") ? resourceKey.slice("map-dir:".length) : "";
  rememberStartedTask(active, active.kind || "map-task", { map_dir: mapDir });
  refreshVisiblePreflightDom();
  toast(error?.payload?.error || "Another map task is already using this map folder.", "error");
  return true;
}

function invalidatePreflightEntry(action, payload = {}) {
  const key = preflightKey(action, payload);
  state.preflight.revisions[key] = (state.preflight.revisions[key] || 0) + 1;
  delete state.preflight.entries[key];
  preflightRequests.delete(key);
}

function invalidateMapPreflights(mapDir) {
  const normalized = trimTrailingSlash(mapDir || "");
  if (!normalized) return;
  Object.values(state.preflight.entries).forEach((entry) => {
    if (preflightExecutionMapDir(entry.payload || {}) === normalized) {
      invalidatePreflightEntry(entry.action, entry.payload || {});
    }
  });
}

function invalidatePreflightsForTask(task) {
  if (!isMapTask(task)) return;
  const resourceMapDir = String(task.resource_key || "").startsWith("map-dir:")
    ? trimTrailingSlash(String(task.resource_key).slice("map-dir:".length))
    : "";
  Object.values(state.preflight.entries).forEach((entry) => {
    const mapDir = preflightExecutionMapDir(entry.payload || {});
    if (!mapDir) return;
    const rememberedMatch = task._preflightAction === task.kind
      && trimTrailingSlash(task._preflightMapDir || "") === mapDir;
    if (resourceMapDir === mapDir || rememberedMatch || commandContainsMapDir(task.command, mapDir)) {
      invalidatePreflightEntry(entry.action, entry.payload || {});
    }
  });
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

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function actionBusy(key) {
  return Boolean(key && state.ui.pendingActions[key]);
}

function actionButtonAttrs(key, busyTitle = "Working...") {
  if (!actionBusy(key)) return `data-action-key="${esc(key)}"`;
  return `disabled data-action-key="${esc(key)}" data-action-busy="true" title="${esc(busyTitle)}"`;
}

function actionButtonLabel(key, label, busyLabel = "Working...") {
  return actionBusy(key) ? busyLabel : label;
}

function beginAction(key, message = "") {
  if (!key || actionBusy(key)) return false;
  state.ui.pendingActions[key] = { message, startedAt: Date.now() };
  render();
  return true;
}

function endAction(key, options = {}) {
  if (!key) return;
  delete state.ui.pendingActions[key];
  if (options.renderAfter !== false) render();
}

function confirmAction({ title, target = "", detail = "", destructive = false }) {
  const lines = [
    title || "Run this action?",
    target ? `Target: ${target}` : "",
    detail,
    destructive ? "This can change or remove local/remote files." : "",
  ].filter(Boolean);
  return window.confirm(lines.join("\n\n"));
}

function runWorkflowAction(workflow) {
  if (workflow === "map-build") setTab("map-builder");
  else if (workflow === "bag-analysis") setTab("bag-analysis");
  else if (workflow === "e2e") setTab("e2e-analysis");
  else if (workflow === "fpv") setTab("fpv");
  else if (workflow === "jetson") setTab("jetson");
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

function timestampMs(value) {
  if (value == null || value === "") return 0;
  if (typeof value === "number") return value > 100000000000 ? value : value * 1000;
  const text = String(value);
  if (/^\d+(\.\d+)?$/.test(text)) {
    const number = Number(text);
    return number > 100000000000 ? number : number * 1000;
  }
  const parsed = Date.parse(text);
  return Number.isFinite(parsed) ? parsed : 0;
}

function itemModifiedMs(item, kind) {
  if (kind === "analyses") {
    const manifest = item?.manifest || {};
    const status = item?.status || {};
    return timestampMs(status.updated_at || manifest.created_at || item.updated_at || item.created_at || item.analysis_id);
  }
  return timestampMs(item?.modified_at);
}

function listControl(kind) {
  if (!state.listControls[kind]) {
    state.listControls[kind] = { query: "", sort: "newest", groupByDate: true, collapsedDays: {} };
  }
  return state.listControls[kind];
}

function itemSearchText(item, kind) {
  if (kind === "analyses") {
    const id = analysisRecordId(item);
    const status = analysisRecordStatus(item);
    const manifest = item?.manifest || {};
    const request = manifest.request || {};
    const resolved = manifest.resolved || {};
    return [
      id,
      item.name,
      item.label,
      item.title,
      manifest.label,
      status,
      request.rosbag,
      resolved.rosbag,
      request.map_dir,
      resolved.map_dir,
      item.rosbag,
      item.bag_path,
      item.map_dir,
      item.map_path,
    ].filter(Boolean).join(" ").toLowerCase();
  }
  if (kind === "maps") {
    return [item.name, item.display_name, item.path, item.complete_runtime_bundle ? "runtime ready" : "incomplete"]
      .filter(Boolean).join(" ").toLowerCase();
  }
  return [item.name, item.display_name, item.path, item.metadata_path, item.original_path, item.favorite ? "favorite protected star" : "", item.trashed ? "trash deleted" : "", item.topic_count_hint]
    .filter(Boolean).join(" ").toLowerCase();
}

function filteredSortedItems(items, kind) {
  const control = listControl(kind);
  const query = String(control.query || "").trim().toLowerCase();
  return [...(items || [])]
    .filter((item) => !query || itemSearchText(item, kind).includes(query))
    .sort((a, b) => {
      const delta = itemModifiedMs(b, kind) - itemModifiedMs(a, kind);
      return control.sort === "oldest" ? -delta : delta;
    });
}

function dateGroupKey(ms) {
  if (!ms) return "unknown";
  return new Date(ms).toISOString().slice(0, 10);
}

function dateGroupLabel(key) {
  if (key === "unknown") return "Unknown date";
  const date = new Date(`${key}T00:00:00`);
  return date.toLocaleDateString(undefined, { year: "numeric", month: "short", day: "numeric" });
}

function groupItemsByDate(items, kind) {
  const groups = [];
  const byKey = new Map();
  for (const item of items) {
    const key = dateGroupKey(itemModifiedMs(item, kind));
    if (!byKey.has(key)) {
      const group = { key, items: [] };
      byKey.set(key, group);
      groups.push(group);
    }
    byKey.get(key).items.push(item);
  }
  return groups;
}

function renderListControls(kind, shown, total, placeholder = "Search") {
  const control = listControl(kind);
  return `
    <div class="list-controls">
      <input
        id="list-filter-${esc(kind)}"
        value="${esc(control.query || "")}"
        placeholder="${esc(placeholder)}"
        oninput="updateListControl(${js(kind)}, 'query', this.value)"
      />
      <select onchange="updateListControl(${js(kind)}, 'sort', this.value)">
        <option value="newest" ${control.sort !== "oldest" ? "selected" : ""}>Newest first</option>
        <option value="oldest" ${control.sort === "oldest" ? "selected" : ""}>Oldest first</option>
      </select>
      <label class="check-row">
        <input type="checkbox" ${control.groupByDate ? "checked" : ""} onchange="updateListControl(${js(kind)}, 'groupByDate', this.checked)" />
        <span>Group by date</span>
      </label>
      <span class="inline-status">${shown}/${total}</span>
    </div>
  `;
}

function renderDateGroupedList(kind, items, renderItems) {
  const control = listControl(kind);
  if (!control.groupByDate) return renderItems(items);
  return groupItemsByDate(items, kind).map((group) => {
    const collapsed = Boolean(control.collapsedDays?.[group.key]);
    return `
      <details class="date-group" ${collapsed ? "" : "open"} ontoggle="toggleDateGroup(${js(kind)}, ${js(group.key)}, this.open)">
        <summary><span>${esc(dateGroupLabel(group.key))}</span><strong>${group.items.length}</strong></summary>
        ${renderItems(group.items)}
      </details>
    `;
  }).join("");
}

function updateListControl(kind, field, value) {
  const control = listControl(kind);
  const activeId = document.activeElement?.id || "";
  const selectionStart = document.activeElement?.selectionStart;
  const selectionEnd = document.activeElement?.selectionEnd;
  if (field === "query") control.query = String(value || "");
  else if (field === "sort") control.sort = value === "oldest" ? "oldest" : "newest";
  else if (field === "groupByDate") control.groupByDate = Boolean(value);
  render();
  if (field === "query" && activeId === `list-filter-${kind}`) {
    requestAnimationFrame(() => {
      const input = $(activeId);
      if (!input) return;
      input.focus();
      if (Number.isFinite(selectionStart) && Number.isFinite(selectionEnd) && input.setSelectionRange) {
        input.setSelectionRange(selectionStart, selectionEnd);
      }
    });
  }
}

function toggleDateGroup(kind, key, open) {
  const control = listControl(kind);
  control.collapsedDays = control.collapsedDays || {};
  control.collapsedDays[key] = !open;
}

function commandText(task) {
  return (task?.command || []).map((part) => JSON.stringify(part)).join(" ");
}

function trimTrailingSlash(value) {
  return String(value || "").replace(/\/+$/, "");
}

function isAnalysisTab(tab = state.tab) {
  return tab === "bag-analysis" || tab === "e2e-analysis";
}

function sh(value) {
  const text = String(value ?? "");
  if (!text) return "''";
  if (/^[A-Za-z0-9_/:=.,@%+-]+$/.test(text)) return text;
  return `'${text.replaceAll("'", "'\\''")}'`;
}

async function refreshAll() {
  const previousFpvSession = state.fpv.browserStatus?.session_id || "";
  const [config, tasks, rosbags, rosbagTrash, maps, cameraTopicConfigs, localIps, analyses, e2eModels, e2ePipeline, objectDetectionPipeline, fpvStatus] = await Promise.all([
    api("/api/config"),
    api("/api/tasks"),
    api("/api/rosbags/local"),
    api("/api/rosbags/trash").catch(() => ({ rosbags: [] })),
    api("/api/maps/local"),
    api("/api/map-builder/camera-topic-configs"),
    api("/api/network/local-ips").catch(() => ({ ips: [] })),
    api("/api/analyses").catch(() => ({ analyses: [] })),
    api("/api/e2e/models").catch(() => ({ models: [] })),
    api("/api/e2e/pipeline").catch(() => ({ datasets: [], runs: [], experiments: [], deploy_profiles: [], deploy_presets: [] })),
    api("/api/object-detection/pipeline").catch(() => ({ datasets: [], runs: [], models: [], base_models: [], deploy_profiles: [], defaults: {} })),
    api("/api/fpv/status").catch(() => ({ fpv: { available: false, running: false } })),
  ]);
  state.config = config;
  state.tasks = tasks.tasks || [];
  state.rosbags = rosbags.rosbags || [];
  state.rosbagTrash = rosbagTrash.rosbags || [];
  state.maps = maps.maps || [];
  state.cameraTopicConfigs = cameraTopicConfigs.configs || [];
  state.localIps = localIps.ips || [];
  state.analysis.analyses = normalizeAnalysisList(analyses);
  state.e2eModels = e2eModels.models || [];
  state.e2ePipeline.datasets = e2ePipeline.datasets || [];
  state.e2ePipeline.runs = e2ePipeline.runs || [];
  state.e2ePipeline.experiments = e2ePipeline.experiments || [];
  state.e2ePipeline.deployProfiles = e2ePipeline.deploy_profiles || [];
  state.e2ePipeline.deployPresets = e2ePipeline.deploy_presets || [];
  state.e2ePipeline.datasetRoot = e2ePipeline.dataset_root || "";
  state.e2ePipeline.runRoot = e2ePipeline.run_root || "";
  if (!state.e2ePipeline.datasetDir && state.e2ePipeline.datasets[0]) {
    state.e2ePipeline.datasetDir = state.e2ePipeline.datasets[0].path || "";
  }
  const activeDataset = state.e2ePipeline.datasets.find(
    (item) => item.path === state.e2ePipeline.datasetDir,
  );
  const activeExperiment = state.e2ePipeline.experiments.find(
    (item) => item.id === state.e2ePipeline.experiment,
  );
  if (activeDataset && activeExperiment?.task !== activeDataset.task) {
    state.e2ePipeline.experiment = state.e2ePipeline.experiments.find(
      (item) => item.task === activeDataset.task,
    )?.id || "";
  }
  if (!state.e2ePipeline.runDir && state.e2ePipeline.runs[0]) {
    state.e2ePipeline.runDir = state.e2ePipeline.runs[0].path || "";
  }
  if (!state.e2ePipeline.deployProfile) {
    state.e2ePipeline.deployProfile = e2ePipeline.default_deploy_profile || state.e2ePipeline.deployProfiles[0]?.id || "";
  }
  if (!state.e2ePipeline.deployPreset) {
    state.e2ePipeline.deployPreset = e2ePipeline.default_deploy_preset || state.e2ePipeline.deployPresets[0]?.id || "";
  }
  if (!state.analysis.e2eModelPath && state.e2eModels[0]) {
    state.analysis.e2eModelPath = state.e2eModels[0].path || "";
  }
  const detection = state.objectDetectionPipeline;
  detection.datasets = objectDetectionPipeline.datasets || [];
  detection.runs = objectDetectionPipeline.runs || [];
  detection.models = objectDetectionPipeline.models || [];
  detection.baseModels = objectDetectionPipeline.base_models?.length
    ? objectDetectionPipeline.base_models
    : detection.baseModels;
  detection.deployProfiles = objectDetectionPipeline.deploy_profiles || [];
  detection.trainingRoot = objectDetectionPipeline.training_root || "";
  detection.datasetRoot = objectDetectionPipeline.dataset_root || "";
  detection.runRoot = objectDetectionPipeline.run_root || "";
  detection.modelRoot = objectDetectionPipeline.model_root || "";
  const detectionDefaults = objectDetectionPipeline.defaults || {};
  if (!detection.datasetYaml || !detection.datasets.some((item) => objectDetectionDatasetPath(item) === detection.datasetYaml)) {
    detection.datasetYaml = objectDetectionDatasetPath(detection.datasets[0]);
  }
  if (!detection.runDir || !detection.runs.some((item) => item.path === detection.runDir)) {
    detection.runDir = detection.runs[0]?.path || "";
  }
  if (!detection.sourceRunDir || !detection.runs.some((item) => item.path === detection.sourceRunDir)) {
    detection.sourceRunDir = detection.runs[0]?.path || "";
  }
  if (!detection.deployProfile) {
    detection.deployProfile = objectDetectionPipeline.default_deploy_profile
      || detection.deployProfiles[0]?.id
      || "";
  }
  if (!detection.remoteRoot) detection.remoteRoot = detectionDefaults.remote_root || "";
  if (!detection.deployUser) detection.deployUser = detectionDefaults.deploy_user || "";
  if (!detection.deployHost) detection.deployHost = detectionDefaults.deploy_host || "";
  const selectedDetectionRun = detection.runs.find((item) => item.path === detection.runDir);
  if (selectedDetectionRun?.name && detection.deployName === "yolov8n_224") {
    detection.deployName = selectedDetectionRun.name;
  }
  const availableDetectionModels = [
    ...detection.models,
    ...detection.runs.filter((item) => item.model_root).map((item) => ({
      name: item.name,
      path: item.model_root,
      source: "training_run",
    })),
  ];
  if (
    availableDetectionModels.length
    && !availableDetectionModels.some((item) => item.path === state.analysis.objectDetectionModelRoot)
  ) {
    state.analysis.objectDetectionModelRoot = availableDetectionModels[0].path || state.analysis.objectDetectionModelRoot;
  }
  state.fpv.browserStatus = fpvStatus.fpv || state.fpv.browserStatus;
  noteFpvWebRtcRtpObserved(state.fpv.browserStatus);
  if (
    fpvPeerConnection
    && (!state.fpv.browserStatus?.running || state.fpv.browserStatus.session_id !== previousFpvSession)
  ) {
    closeFpvPeerConnection();
    resetFpvWebRtcPlaybackState();
  }
  let selectedMapReloadPath = null;
  if (state.selectedMapPath && !state.maps.some((item) => item.path === state.selectedMapPath)) {
    const replacement = state.maps.find((item) => item.path.startsWith(`${state.selectedMapPath}/`))
      || state.maps.find((item) => state.selectedMapPath.startsWith(`${item.path}/`));
    state.selectedMapPath = replacement?.path || null;
    state.selectedMapDetail = null;
    selectedMapReloadPath = replacement?.path || null;
  }
  if (!state.selectedTaskId && state.tasks[0]) state.selectedTaskId = state.tasks[0].task_id;
  if (selectedMapReloadPath) {
    try {
      state.selectedMapDetail = await api(apiPath("/api/maps/detail", { path: selectedMapReloadPath }));
    } catch {
      state.selectedMapDetail = null;
    }
  }
  forceVisiblePreflightOnce = true;
  if (state.tab === "fpv" && state.fpv.browserStatus?.running && fpvMediaElement()) {
    updateFpvBrowserStatusDom();
  } else {
    render();
  }
}

function setTab(tab) {
  if (isAnalysisTab(state.tab) && !isAnalysisTab(tab)) pauseAnalysisPlayback();
  if (state.tab === "maps" && tab !== "maps") stopSimulationLoop();
  if (
    state.tab === "fpv"
    && tab !== "fpv"
    && (
      state.fpv.starting
      || fpvReceiverCanAutoStop(state.fpv.browserStatus)
    )
  ) {
    stopBrowserFpv({ silent: true, renderAfter: false });
  }
  state.tab = tab;
  render();
}

function render() {
  stopAnalysisAnimationFrame();
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
          <button class="ghost" onclick="clearWebCache()">Clear Cache</button>
          <button class="ghost" onclick="toggleTerminal()">${state.terminalCollapsed ? "Show Log" : "Hide Log"}</button>
        </div>
      </header>
      <main class="content">${renderPage()}</main>
      ${renderTerminal()}
      ${renderLogDialog()}
      <div class="toast-region" id="toast-region" aria-live="polite"></div>
    </div>
  `;
  attachFpvRemoteStream();
  scrollLogToEnd();
  const forcePreflight = forceVisiblePreflightOnce;
  forceVisiblePreflightOnce = false;
  requestAnimationFrame(() => {
    drawMapPreview();
    drawSimulationPreview();
    if (isAnalysisTab()) mountAnalysisViewer();
    scheduleVisiblePreflights({ force: forcePreflight });
  });
}

function isEditingField() {
  return ["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName || "");
}

async function clearWebCache() {
  try {
    if (window.caches?.keys) {
      const names = await window.caches.keys();
      await Promise.all(names.map((name) => window.caches.delete(name)));
    }
    if (navigator.serviceWorker?.getRegistrations) {
      const registrations = await navigator.serviceWorker.getRegistrations();
      await Promise.all(registrations.map((registration) => registration.unregister()));
    }
  } catch (error) {
    console.warn("Web cache clear failed", error);
  }
  const nextUrl = new URL(window.location.href);
  nextUrl.searchParams.set("ui_cache", String(Date.now()));
  window.location.replace(nextUrl.toString());
}

function appendLiveLog(chunk = "") {
  if (!chunk) return;
  state.logText += chunk;
  if (state.logText.length > MAX_LIVE_LOG_CHARS) {
    state.logText = LIVE_LOG_TRUNCATION_NOTICE + state.logText.slice(-MAX_LIVE_LOG_CHARS);
  }
}

function updateLogOnly(chunk = "", append = false) {
  updateLogElement($("task-log"), chunk, append);
  updateLogElement($("console-output"), chunk, append);
  updateTaskChrome();
  scrollLogToEnd();
}

function renderPage() {
  if (state.tab === "rosbags") return renderRosbags();
  if (state.tab === "bag-analysis") return renderBagAnalysis();
  if (state.tab === "object-detection") return renderObjectDetectionPipeline();
  if (state.tab === "e2e-analysis") return renderE2EAnalysis();
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
  const browserStatus = fpv.browserStatus || {};
  const browserRunning = Boolean(browserStatus.running);
  const browserStarting = Boolean(fpv.starting);
  const browserBusy = browserRunning || browserStarting;
  const selectedTransport = fpv.transport || "webrtc";
  const activeTransport = fpvStatusTransport(browserStatus);
  const browserUsesWebRtc = browserRunning && activeTransport === "webrtc";
  const browserOwnsWebRtc = fpvWebRtcSessionIsOwned(browserStatus);
  const browserHasFrames = browserUsesWebRtc
    ? Boolean(fpv.webrtcPlaying || Number(fpv.webrtcStats?.framesDecoded || 0) > 0)
    : Number(browserStatus.frame_count || 0) > 0;
  const browserStalled = fpvBrowserIsStalled(browserStatus);
  const destinationIssue = fpvDestinationIssue(fpv);
  const executionDisabled = !state.config?.custom_commands_enabled;
  const executionDisabledAttrs = executionDisabled || browserBusy
    ? `disabled title="${browserBusy ? "Stop the browser view before opening an external viewer" : "Enable trusted local custom commands to start the viewer"}"`
    : "";
  const browserSettingsDisabled = browserBusy
    ? `disabled title="Stop the browser receiver before changing its RTP settings"`
    : "";
  const webRtcUnavailable = selectedTransport === "webrtc" && !browserStatus.webrtc?.available;
  const webRtcCodecUnsupported = selectedTransport === "webrtc" && fpv.codec !== "h264";
  const browserWebRtcUnsupported = selectedTransport === "webrtc" && !window.RTCPeerConnection;
  const browserTransportUnavailable = selectedTransport === "mjpeg"
    ? !browserStatus.available
    : webRtcUnavailable || webRtcCodecUnsupported || browserWebRtcUnsupported;
  const browserStartReason = running.length
    ? "Stop the external viewer first"
    : browserStarting
      ? "Browser receiver is starting"
      : browserRunning
        ? "Browser view is already running"
        : webRtcCodecUnsupported
          ? "WebRTC passthrough currently requires H.264"
          : browserWebRtcUnsupported
            ? "This browser does not support WebRTC"
            : webRtcUnavailable
              ? (browserStatus.webrtc?.error || "WebRTC dependencies are unavailable")
              : "GStreamer is not available on the Console host";
  const browserStartDisabled = browserTransportUnavailable || browserBusy || running.length
    ? `disabled title="${esc(browserStartReason)}"`
    : "";
  const streamUrl = browserRunning && activeTransport === "mjpeg" && browserStatus.session_id
    ? apiPath("/api/fpv/stream", { session: browserStatus.session_id })
    : "";
  const startLabel = browserStarting
    ? "Starting..."
    : selectedTransport === "webrtc"
      ? "Start WebRTC"
      : "Start MJPEG";
  return `
    <div class="page fpv-page">
      <section class="panel fpv-live-panel">
        <div class="panel-header">
          <h2>Browser Live View</h2>
          <span id="fpv-browser-badge" class="status ${fpv.webrtcClientError || browserStatus.last_error ? "failed" : browserUsesWebRtc && !browserOwnsWebRtc ? "running" : browserStalled ? "stopping" : browserRunning ? "running" : "idle"}">${fpv.webrtcClientError || browserStatus.last_error ? "ERROR" : browserRunning ? (browserUsesWebRtc && !browserOwnsWebRtc ? "OTHER TAB" : browserStalled ? "STALLED" : browserHasFrames ? "LIVE" : browserUsesWebRtc && fpv.webrtcClientState === "connecting" ? "CONNECTING" : "WAITING") : "STOPPED"}</span>
          <span class="spacer"></span>
          <button class="primary" onclick="startBrowserFpv()" ${browserStartDisabled}>${startLabel}</button>
          <button onclick="stopBrowserFpv()" ${browserBusy ? "" : "disabled"}>Stop</button>
        </div>
        <div class="panel-body fpv-live-body">
          <div class="fpv-video-stage">
            ${browserUsesWebRtc ? `<video id="fpv-browser-video" autoplay playsinline muted onplaying="handleFpvBrowserVideoPlaying()" onwaiting="handleFpvBrowserVideoWaiting()"></video>` : ""}
            ${streamUrl ? `<img id="fpv-browser-image" src="${esc(streamUrl)}" alt="Live RTP camera" onload="handleFpvBrowserImageLoad()" onerror="handleFpvBrowserImageError()" />` : ""}
            <div id="fpv-video-placeholder" class="fpv-video-placeholder ${browserRunning && browserHasFrames && !browserStalled ? "" : "visible"}">
              <strong>${browserUsesWebRtc && !browserOwnsWebRtc ? "WebRTC session is open in another page" : browserStalled ? esc(fpvBrowserStallMessage(browserStatus)) : browserStarting ? "Starting browser receiver..." : browserRunning ? `Waiting for RTP on UDP ${esc(browserStatus.settings?.port || fpv.port)}...` : "Browser receiver is stopped"}</strong>
              <span>${activeTransport === "webrtc" ? "The Console passes H.264 into a low-latency WebRTC video track." : "The Console receives RTP and relays browser-safe MJPEG here."}</span>
            </div>
          </div>
          <div class="fpv-live-footer">
            <span id="fpv-browser-stats">${esc(fpvBrowserStatusText(browserStatus))}</span>
            <span>${activeTransport === "webrtc" ? "WebRTC H.264 passthrough · no JPEG conversion" : "MJPEG compatibility relay"} · video always scales to fit.</span>
          </div>
          <div id="fpv-browser-error" class="notice full" ${browserStatus.last_error || fpv.webrtcClientError ? "" : "hidden"}>${esc(fpv.webrtcClientError || browserStatus.last_error || "")}</div>
          ${running.length ? `<div class="notice full">An external receiver is using this RTP port. Stop it before starting the browser view.</div>` : ""}
        </div>
      </section>
      <section class="panel">
        <div class="panel-header">
          <h2>RTP Receiver Settings</h2>
          <span class="spacer"></span>
          <button onclick="copyFpvReceiverCommand()">Copy Command</button>
          <button class="${actionBusy("fpv:external") ? "is-busy" : ""}" onclick="startFpvViewer()" ${executionDisabledAttrs} ${actionButtonAttrs("fpv:external", "External viewer is starting...")}>${esc(actionButtonLabel("fpv:external", "Open External Viewer", "Starting..."))}</button>
        </div>
        <div class="panel-body">
          <div class="form-grid">
            <div class="field">
              <label>Mac / notebook IP</label>
              <input id="fpv-host" value="${esc(fpv.host)}" placeholder="10.42.0.161" oninput="updateFpvCommandPreview()" />
            </div>
            <div class="field">
              <label>Codec</label>
              <select id="fpv-codec" onchange="handleFpvCodecChange()" ${browserSettingsDisabled}>
                ${["h264", "h265", "mjpeg", "raw"].map((codec) => `<option value="${codec}" ${fpv.codec === codec ? "selected" : ""}>${codec}</option>`).join("")}
              </select>
            </div>
            <div class="field">
              <label>Browser transport</label>
              <select id="fpv-transport" onchange="handleFpvTransportChange()" ${browserSettingsDisabled}>
                <option value="webrtc" ${selectedTransport === "webrtc" ? "selected" : ""}>WebRTC (low latency, H.264)</option>
                <option value="mjpeg" ${selectedTransport === "mjpeg" ? "selected" : ""}>MJPEG (compatibility)</option>
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
              <input id="fpv-width" type="number" min="1" value="${esc(fpv.width)}" oninput="updateFpvCommandPreview()" ${browserSettingsDisabled} />
            </div>
            <div class="field">
              <label>Height</label>
              <input id="fpv-height" type="number" min="1" value="${esc(fpv.height)}" oninput="updateFpvCommandPreview()" ${browserSettingsDisabled} />
            </div>
            <div class="field">
              <label>FPS</label>
              <input id="fpv-fps" type="number" min="1" value="${esc(fpv.fps)}" oninput="updateFpvCommandPreview()" ${browserSettingsDisabled} />
            </div>
            <div class="field">
              <label>Port</label>
              <input id="fpv-port" type="number" min="1" value="${esc(fpv.port)}" oninput="updateFpvCommandPreview()" ${browserSettingsDisabled} />
            </div>
            <div class="field">
              <label>Payload</label>
              <input id="fpv-payload" type="number" min="0" max="127" value="${esc(fpv.codec === "mjpeg" ? 26 : fpv.payload)}" oninput="updateFpvCommandPreview()" ${browserSettingsDisabled || (fpv.codec === "mjpeg" ? "disabled title=\"JPEG RTP uses payload type 26\"" : "")} />
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
            <div id="fpv-destination-notice" class="notice full" ${destinationIssue ? "" : "hidden"}>${esc(destinationIssue)}</div>
            <div class="actions full">
              <button class="primary" onclick="startBrowserFpv()" ${browserStartDisabled}>${startLabel}</button>
              <button class="${actionBusy("fpv:external") ? "is-busy" : ""}" onclick="startFpvViewer()" ${executionDisabledAttrs} ${actionButtonAttrs("fpv:external", "External viewer is starting...")}>${esc(actionButtonLabel("fpv:external", "Open External Viewer", "Starting..."))}</button>
              <button onclick="copyFpvReceiverCommand()">Copy Command</button>
              <button onclick="copyFpvJetsonCommand()">Copy Jetson Command</button>
              ${running.length ? `<button class="danger ${actionBusy(`task:stop:${running[0].task_id}`) ? "is-busy" : ""}" onclick="stopTask(${js(running[0].task_id)})" ${actionButtonAttrs(`task:stop:${running[0].task_id}`, "Stop request is being sent...")}>${esc(actionButtonLabel(`task:stop:${running[0].task_id}`, "Stop Running Viewer", "Stopping Viewer..."))}</button>` : ""}
            </div>
            ${executionDisabled ? `<div class="notice full">Opening a separate desktop viewer is disabled by default. The embedded browser view remains available without enabling custom commands.</div>` : ""}
            ${webRtcUnavailable ? `<div class="notice full">WebRTC is unavailable: ${esc(browserStatus.webrtc?.error || "required GStreamer WebRTC components are missing")}. Select MJPEG compatibility mode or install the listed WebRTC packages.</div>` : ""}
            <div class="notice full">Use either the embedded browser view or the external GStreamer viewer. Two receivers should not bind the same UDP port at once.</div>
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
  const workflow = currentDashboardWorkflow();
  return `
    <div class="page">
      ${renderWorkflowHome(workflow)}
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

function currentDashboardWorkflow() {
  const valid = ["overview", "map-build", "bag-analysis", "e2e", "fpv", "jetson"];
  return valid.includes(state.ui.dashboardWorkflow) ? state.ui.dashboardWorkflow : "overview";
}

function dashboardWorkflowItems() {
  const runningKinds = new Set(
    state.tasks
      .filter((task) => ["queued", "running", "stopping"].includes(task.status))
      .map((task) => task.kind),
  );
  const analysisReady = state.analysis.analyses.filter((item) => analysisRecordStatus(item) === "success").length;
  const map = pipelineMap();
  const mapSteps = map ? pipelineSteps(map) : [];
  const mapDone = mapSteps.filter((step) => step.status === "done").length;
  const mapTotal = mapSteps.length || 7;
  return [
    {
      key: "overview",
      title: "Overview",
      detail: "Active tasks, recent assets, and the best next action across JetPilot.",
      status: state.tasks.some(isActiveTask) ? "running" : "ready",
      progress: state.tasks.length ? Math.min(100, Math.round((state.tasks.filter(isFinishedTask).length / state.tasks.length) * 100)) : 0,
      action: "Review Tasks",
      next: state.tasks.some(isActiveTask) ? "A task is running. Open the console if you need the live log." : "Choose a workflow below, then start from its next action.",
    },
    {
      key: "map-build",
      title: "Map Build",
      detail: "Build VGL/VSLAM maps, edit HD lines, generate racelines, and prepare Jetson bundles.",
      status: runningKinds.has("map-build") || runningKinds.has("prepare-hd-raster") || runningKinds.has("generate-raceline") || runningKinds.has("generate-preview") ? "running" : map?.complete_runtime_bundle ? "ready" : "waiting",
      progress: Math.round((mapDone / mapTotal) * 100),
      action: "Open Builder",
      next: map ? (mapSteps.find((step) => step.status !== "done")?.next || "Selected map looks runtime-ready.") : "Select or pull a rosbag, then build a visual map.",
    },
    {
      key: "bag-analysis",
      title: "Bag Analysis",
      detail: "Preprocess recorded drives into synchronized video, control, mode, speed, and trajectory views.",
      status: runningKinds.has("analyze-rosbag") ? "running" : state.rosbags.length ? "ready" : "waiting",
      progress: state.rosbags.length ? (analysisReady ? 100 : 35) : 0,
      action: "New Analysis",
      next: state.rosbags.length ? "Select a bag, inspect topics, and run the analysis preprocessor." : "Record or pull a rosbag before analysis.",
    },
    {
      key: "e2e",
      title: "E2E Analysis",
      detail: "Run offline inference, compare teacher and prediction, or evaluate driven bags by map section.",
      status: runningKinds.has("analyze-e2e") ? "running" : state.rosbags.length ? "ready" : "waiting",
      progress: state.rosbags.length ? (state.e2eModels.length ? 70 : 40) : 0,
      action: "New E2E Analysis",
      next: state.rosbags.length ? "Select supervised, offline-localization, or recorded-localization mode." : "Record or pull a rosbag before E2E analysis.",
    },
    {
      key: "fpv",
      title: "FPV Check",
      detail: "Start the browser RTP receiver, verify packets, and compare WebRTC/MJPEG behavior.",
      status: state.fpv.browserStatus?.running ? "running" : "ready",
      progress: state.fpv.browserStatus?.running ? 70 : 20,
      action: "Open FPV",
      next: state.fpv.browserStatus?.running ? fpvBrowserStatusText(state.fpv.browserStatus) : "Pick codec, host, and transport, then start the browser receiver.",
    },
    {
      key: "jetson",
      title: "Jetson Sync",
      detail: "Inspect the Jetson, pull rosbags, and push runtime-ready map bundles.",
      status: state.jetsonTransfer.running || state.jetsonInspectBusy ? "running" : state.jetsonInspect?.ok ? "ready" : "waiting",
      progress: state.jetsonInspect?.ok ? 60 : 0,
      action: "Open Jetson",
      next: state.jetsonInspect?.ok ? "Choose a transfer direction and verify the target path before starting." : "Inspect Jetson before transfers.",
    },
  ];
}

function renderWorkflowHome(workflow) {
  const items = dashboardWorkflowItems();
  const selected = items.find((item) => item.key === workflow) || items[0];
  return `
    <section class="panel workflow-panel">
      <div class="panel-header">
        <h2>Workflow</h2>
        <span class="workflow-target">${esc(selected.title)}</span>
        <span class="spacer"></span>
        <button class="primary" onclick="runWorkflowAction(${js(selected.key)})">${esc(selected.action)}</button>
      </div>
      <div class="panel-body">
        <div class="workflow-switcher">
          ${items.map((item) => renderWorkflowCard(item, item.key === selected.key)).join("")}
        </div>
        <div class="workflow-focus">
          <div>
            <span>Next action</span>
            <strong>${esc(selected.title)}</strong>
          </div>
          <p>${esc(selected.next)}</p>
          <button onclick="runWorkflowAction(${js(selected.key)})">${esc(selected.action)}</button>
        </div>
        ${workflow === "map-build" ? renderAutonomyPipeline({ embedded: true }) : renderWorkflowProgress(selected)}
      </div>
    </section>
  `;
}

function renderWorkflowCard(item, selected) {
  return `
    <button class="workflow-card ${selected ? "active" : ""}" onclick="selectDashboardWorkflow(${js(item.key)})">
      <span class="status ${pipelineStatusClass(item.status)}">${esc(pipelineStatusLabel(item.status))}</span>
      <strong>${esc(item.title)}</strong>
      <p>${esc(item.detail)}</p>
      <div class="workflow-progress"><span style="width:${Math.max(0, Math.min(100, item.progress)).toFixed(0)}%"></span></div>
    </button>
  `;
}

function renderWorkflowProgress(item) {
  const steps = {
    overview: [
      ["Watch running tasks", state.tasks.some(isActiveTask)],
      ["Use recent assets", Boolean(state.rosbags.length || state.maps.length || state.analysis.analyses.length)],
      ["Open the focused workflow", false],
    ],
    "bag-analysis": [
      ["Select rosbag", Boolean(state.analysis.selectedBagPath || state.rosbags.length)],
      ["Inspect topics", Boolean(state.analysis.bagDetail)],
      ["Configure signals", Boolean(state.analysis.imageTopic || state.analysis.selectedImageTopics?.length)],
      ["Run preprocessing", state.analysis.analyses.length > 0],
      ["Review drive viewer", state.analysis.analyses.some((entry) => analysisRecordStatus(entry) === "success")],
    ],
    e2e: [
      ["Select dataset", false],
      ["Preprocess frames", false],
      ["Train model", false],
      ["Compare runs", false],
      ["Export ONNX", false],
      ["Deploy model", false],
    ],
    fpv: [
      ["Set receiver IP", Boolean(state.fpv.host)],
      ["Choose codec/transport", Boolean(state.fpv.codec && state.fpv.transport)],
      ["Start receiver", Boolean(state.fpv.browserStatus?.running)],
      ["Verify packets", Number(state.fpv.browserStatus?.rtp_packet_count || state.fpv.browserStatus?.frame_count || 0) > 0],
    ],
    jetson: [
      ["Set SSH target", Boolean(jetsonTarget().host)],
      ["Inspect remote state", Boolean(state.jetsonInspect)],
      ["Choose transfer", Boolean(selectedJetsonPullPaths().length || $("push-local")?.value)],
      ["Run sync task", state.tasks.some((task) => task.kind?.startsWith("transfer-") && isActiveTask(task))],
    ],
  }[item.key] || [];
  return `
    <div class="workflow-progress-list">
      ${steps.map(([label, done], index) => `
        <div class="workflow-step ${done ? "done" : ""}">
          <span>${esc(index + 1)}</span>
          <strong>${esc(label)}</strong>
        </div>
      `).join("")}
    </div>
  `;
}

function selectDashboardWorkflow(workflow) {
  state.ui.dashboardWorkflow = workflow;
  render();
}

function renderAutonomyPipeline(options = {}) {
  const map = pipelineMap();
  const steps = pipelineSteps(map);
  const nextStep = steps.find((step) => step.status !== "done");
  const mapName = map ? mapDisplayName(map) : "No map selected yet";
  const tag = options.embedded ? "div" : "section";
  const header = options.embedded
    ? `<div class="pipeline-subheader"><h3>Map Build Progress</h3><span class="pipeline-target">${esc(mapName)}</span></div>`
    : `<div class="panel-header">
        <h2>Autonomy Pipeline</h2>
        <span class="pipeline-target">${esc(mapName)}</span>
        <span class="spacer"></span>
        <button onclick="setTab('rosbags')">Rosbags</button>
        <button onclick="setTab('map-builder')">Map Builder</button>
        <button onclick="setTab('maps')">Maps</button>
      </div>`;
  return `
    <${tag} class="${options.embedded ? "pipeline-embedded" : "panel"} pipeline-panel">
      ${header}
      <div class="panel-body">
        ${renderPipelineMapChooser(map)}
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
    </${tag}>
  `;
}

function renderPipelineMapChooser(map) {
  if (!state.maps.length) {
    return `<div class="pipeline-map-chooser"><span>Target map</span><strong>No map available yet</strong><button onclick="setTab('map-builder')">Map Builder</button></div>`;
  }
  const selectedPath = map?.path || "";
  return `
    <div class="pipeline-map-chooser">
      <label for="pipeline-map-select">Target map</label>
      <select id="pipeline-map-select" onchange="selectPipelineMap(this.value)">
        ${state.maps
          .map((item) => {
            const label = mapOptionLabel(item);
            return `<option value="${esc(item.path)}" ${item.path === selectedPath ? "selected" : ""}>${esc(label)}</option>`;
          })
          .join("")}
      </select>
      <button onclick="${selectedPath ? `openMapWorkspace(${js(selectedPath)})` : "setTab('maps')"}" ${selectedPath ? "" : "disabled"}>Open Target</button>
    </div>
  `;
}

function mapOptionLabel(map) {
  const display = mapDisplayName(map);
  const suffix = map.name && map.name !== display ? ` / ${map.name}` : "";
  const stateLabel = map.complete_runtime_bundle ? "ready" : "incomplete";
  return `${display}${suffix} - ${stateLabel}`;
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

function selectPipelineMap(path) {
  const selected = state.maps.find((item) => item.path === path);
  if (!selected) return;
  state.selectedMapPath = selected.path;
  if (state.selectedMapDetail?.map?.path !== selected.path) state.selectedMapDetail = null;
  render();
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

function mapBuildPreflightPayload() {
  return {
    rosbag: $("build-rosbag")?.value || "",
    map_dir: outputMapDir(),
    topic_config: selectedCameraTopicConfig(),
    steps: $("build-steps")?.value || "edex compute_poses cuvgl",
    enable_rviz: false,
  };
}

function initialMapBuildPreflightPayload(defaultMapBase, recommendedTopicConfig) {
  return {
    rosbag: "",
    map_dir: "",
    topic_config: recommendedTopicConfig || "",
    steps: "edex compute_poses cuvgl",
    enable_rviz: false,
    output_base: defaultMapBase,
  };
}

function mapStagePreflightPayload(stage, mapDir) {
  const payload = { map_dir: mapDir };
  if (stage === "prepare-hd-raster") {
    payload.auto_crop_percentile = state.hdRasterGeneration.autoCropPercentile;
    payload.auto_crop_min_retained_ratio = state.hdRasterGeneration.autoCropMinRetainedRatio;
    payload.path_crop_distance_m = state.hdRasterGeneration.pathCropDistanceM;
    payload.path_crop_min_retained_ratio = state.hdRasterGeneration.pathCropMinRetainedRatio;
  }
  if (stage === "generate-raceline") {
    payload.vehicle_width_m = state.racelineGeneration.vehicleWidthM;
    payload.safety_margin_m = state.racelineGeneration.safetyMarginM;
    payload.max_speed_mps = state.racelineGeneration.maxSpeedMps;
    payload.min_speed_mps = state.racelineGeneration.minSpeedMps;
    payload.lateral_accel_limit_mps2 = state.racelineGeneration.lateralAccelLimitMps2;
    payload.accel_limit_mps2 = state.racelineGeneration.accelLimitMps2;
    payload.decel_limit_mps2 = state.racelineGeneration.decelLimitMps2;
  }
  return payload;
}

function mapStageDomToken(stage, mapDir) {
  return preflightToken("map-dom", { stage, map_dir: mapDir });
}

function renderMapStageButton(stage, mapDir, label, options = {}) {
  const payload = mapStagePreflightPayload(stage, mapDir);
  const actionKey = `map-stage:${stage}:${mapDir}`;
  const classList = [options.className || "", actionBusy(actionKey) ? "is-busy" : ""].filter(Boolean).join(" ");
  const classes = classList ? ` class="${esc(classList)}"` : "";
  return `<button${classes} onclick="runMapStage('${esc(stage)}', ${js(mapDir)})" ${preflightButtonAttrs(stage, payload)} ${actionButtonAttrs(actionKey, "Map stage is starting...")} data-map-preflight-stage="${esc(stage)}" data-map-preflight-token="${esc(mapStageDomToken(stage, mapDir))}">${esc(actionButtonLabel(actionKey, label, "Starting..."))}</button>`;
}

function renderMapStageReadiness(stage, mapDir, title, options = {}) {
  const payload = mapStagePreflightPayload(stage, mapDir);
  const panel = renderReadinessPanel(stage, payload, {
    title,
    compact: options.compact !== false,
    micro: Boolean(options.micro),
  });
  return panel.replace(
    "data-preflight-role=\"panel\"",
    `data-preflight-role="panel" data-map-preflight-stage="${esc(stage)}" data-map-preflight-token="${esc(mapStageDomToken(stage, mapDir))}"`,
  );
}

function bindMapBuildPreflight(payload) {
  const token = preflightToken("map-build", payload);
  const executionToken = preflightExecutionToken("map-build", payload);
  const mapResourceToken = preflightMapResourceToken(payload);
  ["map-build-preflight", "map-build-start", "map-build-preflight-reason"].forEach((id) => {
    const element = $(id);
    if (!element) return;
    if (element.dataset.preflightRole) {
      element.dataset.preflightToken = token;
      element.dataset.preflightExecutionToken = executionToken;
      element.dataset.preflightMapResourceToken = mapResourceToken;
    }
    element.querySelectorAll?.("[data-preflight-role]").forEach((child) => {
      child.dataset.preflightToken = token;
      child.dataset.preflightExecutionToken = executionToken;
      child.dataset.preflightMapResourceToken = mapResourceToken;
    });
  });
  updatePreflightDom("map-build", payload);
}

function bindMapStagePreflight(stage, mapDir, payload) {
  const domToken = mapStageDomToken(stage, mapDir);
  const token = preflightToken(stage, payload);
  const executionToken = preflightExecutionToken(stage, payload);
  const mapResourceToken = preflightMapResourceToken(payload);
  document.querySelectorAll(`[data-map-preflight-token="${domToken}"]`).forEach((element) => {
    element.dataset.preflightToken = token;
    element.dataset.preflightExecutionToken = executionToken;
    element.dataset.preflightMapResourceToken = mapResourceToken;
  });
  updatePreflightDom(stage, payload);
}

function scheduleMapBuildPreflight(options = {}) {
  if (!$("build-rosbag")) return;
  clearTimeout(mapBuildPreflightTimer);
  const check = () => {
    const payload = mapBuildPreflightPayload();
    bindMapBuildPreflight(payload);
    requestPreflight("map-build", payload, { force: Boolean(options.force) });
  };
  if (options.immediate) check();
  else mapBuildPreflightTimer = setTimeout(check, 250);
}

function scheduleRacelinePreflight(mapDir, options = {}) {
  clearTimeout(racelinePreflightTimer);
  const check = () => {
    const payload = mapStagePreflightPayload("generate-raceline", mapDir);
    bindMapStagePreflight("generate-raceline", mapDir, payload);
    requestPreflight("generate-raceline", payload, { force: Boolean(options.force) });
  };
  if (options.immediate) check();
  else racelinePreflightTimer = setTimeout(check, 250);
}

function scheduleVisiblePreflights(options = {}) {
  if ($("build-rosbag")) scheduleMapBuildPreflight({ immediate: true, force: Boolean(options.force) });
  if (state.tab === "bag-analysis") {
    scheduleAnalysisPreflight({ immediate: true, force: Boolean(options.force) });
    return;
  }
  if (state.tab === "e2e-analysis") {
    scheduleE2EPreflight({ immediate: true, force: Boolean(options.force) });
    return;
  }
  if (state.tab === "dashboard") {
    const map = pipelineMap();
    if (!map?.path) return;
    ["prepare-hd-raster", "generate-raceline", "generate-preview"].forEach((stage) => {
      requestPreflight(stage, mapStagePreflightPayload(stage, map.path), { force: Boolean(options.force) });
    });
    return;
  }
  if (state.tab !== "maps") return;
  const seen = new Set();
  state.maps.forEach((map) => {
    ["prepare-hd-raster", "generate-raceline", "generate-preview"].forEach((stage) => {
      const payload = mapStagePreflightPayload(stage, map.path);
      const key = preflightKey(stage, payload);
      if (seen.has(key)) return;
      seen.add(key);
      requestPreflight(stage, payload, { force: Boolean(options.force) });
    });
  });
}

function capturePreflightError(action, payload, error) {
  const result = error?.payload?.preflight || error?.payload;
  if (!result || typeof result !== "object" || !("ready" in result || "checks" in result || "status" in result)) {
    return false;
  }
  cachePreflightResult(action, payload, result);
  return true;
}

function cachePreflightResult(action, payload, result) {
  const entry = normalizePreflightResult(action, payload, result);
  state.preflight.entries[preflightKey(action, payload)] = entry;
  updatePreflightDom(action, payload);
  return entry;
}

function hasRunningTask(kind) {
  return state.tasks.some((task) => task.kind === kind && ["queued", "running", "stopping"].includes(task.status));
}

function isActiveTask(task) {
  return ["queued", "running", "stopping"].includes(task.status);
}

function isFinishedTask(task) {
  return task && ["success", "failed", "stopped", "lost"].includes(task.status);
}

function isMapTask(task) {
  return ["map-build", "prepare-hd-raster", "generate-raceline", "generate-preview"].includes(task.kind);
}

function mapTaskFinishedSince(previous, next) {
  if (!isMapTask(next) || !isFinishedTask(next)) return false;
  if (!previous) return true;
  return previous.status !== next.status || previous.ended_at !== next.ended_at;
}

function mapTaskSignature(tasks) {
  return tasks
    .filter(isMapTask)
    .map((task) => `${task.task_id}:${task.status}:${task.ended_at || ""}`)
    .join("|");
}

function shouldRefreshMapsAfterTaskPoll(previousTasks, nextTasks) {
  if (state.tab === "maps") return false;
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
      preflightAction: mapPath ? "prepare-hd-raster" : "",
      preflightPayload: mapPath ? mapStagePreflightPayload("prepare-hd-raster", mapPath) : null,
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
      next: hasRaceline ? "Open the workspace to see the red raceline overlay, or generate a preview image next." : "Generate the raceline after HD map lines exist.",
      action: hasRaceline ? "View Raceline" : "Run Raceline",
      onclick: mapPath ? (hasRaceline ? `openMapWorkspace(${js(mapPath)})` : `runMapStage('generate-raceline', ${js(mapPath)})`) : "setTab('maps')",
      preflightAction: mapPath && !hasRaceline ? "generate-raceline" : "",
      preflightPayload: mapPath && !hasRaceline ? mapStagePreflightPayload("generate-raceline", mapPath) : null,
    },
    {
      title: "Review preview",
      detail: "Checks map shape, bounds, centerline, and raceline together.",
      status: hasRunningTask("generate-preview") ? "running" : hasPreview ? "done" : hasRaceline ? "ready" : "blocked",
      next: hasPreview ? "Push the map bundle to Jetson." : "Generate preview after raceline exists.",
      action: "Run Preview",
      onclick: mapPath ? `runMapStage('generate-preview', ${js(mapPath)})` : "setTab('maps')",
      preflightAction: mapPath ? "generate-preview" : "",
      preflightPayload: mapPath ? mapStagePreflightPayload("generate-preview", mapPath) : null,
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
  const preflightAttrs = step.preflightAction
    ? preflightButtonAttrs(step.preflightAction, step.preflightPayload || {})
    : step.status === "blocked" ? "disabled" : "";
  return `
    <div class="pipeline-step ${esc(step.status)}">
      <div class="pipeline-number">${esc(number)}</div>
      <div class="pipeline-step-body">
        <div class="pipeline-step-top">
          <strong>${esc(step.title)}</strong>
          <span class="status ${pipelineStatusClass(step.status)}">${pipelineStatusLabel(step.status)}</span>
        </div>
        <p>${esc(step.detail)}</p>
        <button onclick="${step.onclick}" ${preflightAttrs}>${esc(step.action)}</button>
        ${step.preflightAction ? renderReadinessPanel(step.preflightAction, step.preflightPayload || {}, { title: step.action, compact: true, micro: true }) : ""}
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
                <strong>${esc(item.display_name || item.name)}</strong>
                <div class="path" title="${esc(item.path)}">${esc(item.path)}</div>
              </div>
              <div class="actions">
                ${kind === "map" ? `<span class="status ${ready ? "success" : "failed"}">${ready ? "ready" : "incomplete"}</span>` : ""}
                <button onclick="copyText(${js(item.path)})">Copy</button>
                ${kind === "rosbag" ? `<button onclick="openBagAnalysis(${js(item.path)})">Analyze</button><button onclick="useRosbag(${js(item.path)})">Build</button>` : `<button onclick="fillTransferLocal(${js(item.path)})">Transfer</button>`}
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
  const initialPreflightPayload = initialMapBuildPreflightPayload(defaultMapBase, recommendedPath);
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
        <select id="build-rosbag" onchange="selectMapBuildRosbag(this.value)">
          <option value="">Select rosbag</option>
          ${state.rosbags.map((bag) => `<option value="${esc(bag.path)}">${esc(bag.display_name || bag.name)} - ${esc(bag.path)}</option>`).join("")}
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
        <input id="build-steps" value="edex compute_poses cuvgl" oninput="scheduleMapBuildPreflight()" />
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
      <div id="map-build-preflight" class="full" data-map-build-preflight>
        ${renderReadinessPanel("map-build", initialPreflightPayload, { title: "VGL / VSLAM build readiness" })}
      </div>
      <div class="actions full">
        <button id="map-build-start" class="primary ${actionBusy("map-build:start") ? "is-busy" : ""}" onclick="startMapBuild()" ${preflightButtonAttrs("map-build", initialPreflightPayload)} ${actionButtonAttrs("map-build:start", "Map build is starting...")}>${esc(actionButtonLabel("map-build:start", "Start VGL/VSLAM Build", "Starting Build..."))}</button>
        <button onclick="copyMapBuildCommand()">Copy Equivalent Command</button>
      </div>
      <div id="map-build-preflight-reason" class="full">
        ${renderPreflightButtonReason("map-build", initialPreflightPayload)}
      </div>
    </div>
  `;
}

function renderRosbags() {
  const items = filteredSortedItems(state.rosbags, "rosbags");
  const trashItems = filteredSortedItems(state.rosbagTrash, "rosbagTrash");
  return `
    <div class="page">
      <section class="panel">
        <div class="panel-header"><h2>Local Rosbags</h2><span class="spacer"></span><button onclick="refreshAll()">Scan</button></div>
        <div class="panel-body">
          ${renderListControls("rosbags", items.length, state.rosbags.length, "Search rosbag name or path")}
          ${state.rosbags.length
            ? (items.length ? renderDateGroupedList("rosbags", items, rosbagTable) : `<div class="empty">No rosbags match the current filter.</div>`)
            : `<div class="empty">No metadata.yaml files under ${esc(state.config?.record_root || "")}</div>`}
        </div>
      </section>
      <section class="panel">
        <div class="panel-header">
          <h2>Rosbag Trash</h2>
          <span class="muted">${esc(state.config?.record_root ? `${state.config.record_root}/.jetpilot_trash` : "")}</span>
          <span class="spacer"></span>
          <button onclick="refreshAll()">Scan</button>
        </div>
        <div class="panel-body">
          ${renderListControls("rosbagTrash", trashItems.length, state.rosbagTrash.length, "Search trashed rosbag name or path")}
          ${state.rosbagTrash.length
            ? (trashItems.length ? renderDateGroupedList("rosbagTrash", trashItems, (groupItems) => rosbagTable(groupItems, { trash: true })) : `<div class="empty">No trashed rosbags match the current filter.</div>`)
            : `<div class="empty">Trash is empty. Moving a rosbag to Trash keeps the data here until you delete it forever.</div>`}
        </div>
      </section>
    </div>
  `;
}

function rosbagTable(items = state.rosbags, options = {}) {
  const isTrash = Boolean(options.trash);
  return `
    <div class="table-wrap">
      <table>
        <thead><tr><th>Name</th><th>Path</th>${isTrash ? "<th>Original</th>" : ""}<th>Size</th><th>Modified</th><th></th></tr></thead>
        <tbody>
          ${items
            .map(
              (bag) => {
                const label = bag.display_name || bag.name;
                const favoriteTitle = bag.favorite ? "Protected from Trash" : "Not protected";
                return `
                <tr>
                  <td>
                    <div class="name-cell">
                      ${!isTrash ? `<button class="icon-button ${bag.favorite ? "is-active" : ""}" title="${esc(favoriteTitle)}" onclick="toggleRosbagFavorite(${js(bag.path)}, ${bag.favorite ? "false" : "true"})">${bag.favorite ? "★" : "☆"}</button>` : ""}
                      <div>
                        <strong>${esc(label)}</strong>
                        ${label !== bag.name ? `<div class="muted">${esc(bag.name)}</div>` : ""}
                      </div>
                    </div>
                  </td>
                  <td><div class="path" title="${esc(bag.path)}">${esc(bag.path)}</div></td>
                  ${isTrash ? `<td><div class="path" title="${esc(bag.original_path || "")}">${esc(bag.original_path || "-")}</div></td>` : ""}
                  <td>${fmtBytes(bag.size_bytes)}</td>
                  <td>${fmtTime(bag.modified_at)}</td>
                  <td class="actions">
                    <button onclick="copyText(${js(bag.path)})">Copy</button>
                    ${isTrash
                      ? `<button class="primary" onclick="restoreRosbag(${js(bag.path)})">Restore</button><button class="danger" onclick="deleteTrashedRosbag(${js(bag.path)})">Delete Forever</button>`
                      : `<button class="primary" onclick="openBagAnalysis(${js(bag.path)})">Analyze</button><button onclick="useRosbag(${js(bag.path)})">Use</button><button onclick="renameRosbag(${js(bag.path)}, ${js(bag.name)})">Rename</button><button class="danger" onclick="deleteRosbag(${js(bag.path)})" ${bag.favorite ? "disabled title=\"Remove the star before moving to Trash\"" : ""}>Move to Trash</button>`}
                  </td>
                </tr>`;
              },
            )
            .join("")}
        </tbody>
      </table>
    </div>
  `;
}

function normalizeAnalysisList(payload) {
  const raw = Array.isArray(payload) ? payload : payload?.analyses || payload?.jobs || payload?.items || [];
  return raw
    .filter((item) => item && typeof item === "object")
    .map(normalizeAnalysisRecord)
    .sort((a, b) => String(b.updated_at || b.created_at || b.started_at || "").localeCompare(String(a.updated_at || a.created_at || a.started_at || "")));
}

function normalizeAnalysisRecord(item) {
  const manifest = item?.manifest && typeof item.manifest === "object" ? item.manifest : {};
  const statusRecord = item?.status && typeof item.status === "object" ? item.status : {};
  const request = manifest.request && typeof manifest.request === "object" ? manifest.request : {};
  const resolved = manifest.resolved && typeof manifest.resolved === "object" ? manifest.resolved : {};
  const manifestBag = manifest.rosbag && typeof manifest.rosbag === "object" ? manifest.rosbag.path : manifest.rosbag;
  const manifestMap = manifest.map && typeof manifest.map === "object" ? manifest.map : {};
  const resolvedMap = resolved.map_dir || resolved.map_path || "";
  const mapPath = manifestMap.path || manifest.map_path || manifest.map_dir || resolvedMap || request.map_dir || item.map_path || item.map_dir || "";
  const rosbag = manifestBag || manifest.bag_path || resolved.rosbag || request.rosbag || item.rosbag || item.bag_path || "";
  return {
    ...manifest,
    ...item,
    analysis_id: item.analysis_id || item.id || manifest.analysis_id || manifest.id || "",
    status: statusRecord.status || statusRecord.state || (typeof item.status === "string" ? item.status : "") || manifest.status || "pending",
    progress: statusRecord.progress ?? item.progress ?? manifest.progress ?? 0,
    stage: statusRecord.stage || statusRecord.phase || item.stage || item.phase || manifest.stage || manifest.phase || "",
    phase: statusRecord.phase || item.phase || manifest.phase || "",
    message: statusRecord.message || item.message || manifest.message || "",
    updated_at: statusRecord.updated_at || item.updated_at || manifest.updated_at || manifest.created_at || "",
    task_id: statusRecord.task_id || item.task_id || item.task?.task_id || manifest.task_id || "",
    rosbag,
    bag_path: rosbag,
    map: { ...manifestMap, path: mapPath || manifestMap.path || "" },
    map_path: mapPath,
    map_dir: mapPath,
    missing: [
      ...(Array.isArray(manifest.missing) ? manifest.missing : []),
      ...(Array.isArray(manifest.missing_data) ? manifest.missing_data : []),
      ...(Array.isArray(manifest.warnings) ? manifest.warnings : []),
      ...(Array.isArray(statusRecord.warnings) ? statusRecord.warnings : []),
    ],
  };
}

function analysisRecordId(item) {
  return String(item?.id || item?.analysis_id || item?.job_id || "");
}

function analysisRecordStatus(item) {
  const value = String(item?.status || item?.state || item?.task?.status || "pending").toLowerCase();
  if (["complete", "completed", "success", "ready"].includes(value)) return "success";
  if (["failed", "error", "lost"].includes(value)) return "failed";
  if (["stopped", "cancelled", "canceled"].includes(value)) return "stopped";
  if (["queued", "pending"].includes(value)) return "queued";
  return "running";
}

function analysisRecordProgress(item) {
  const raw = item?.progress;
  if (typeof raw === "number") return Math.max(0, Math.min(100, raw <= 1 ? raw * 100 : raw));
  if (raw && typeof raw === "object") {
    const value = Number(raw.percent ?? raw.value ?? 0);
    if (Number.isFinite(value)) return Math.max(0, Math.min(100, value <= 1 && value > 0 ? value * 100 : value));
    const completed = Number(raw.completed || 0);
    const total = Number(raw.total || 0);
    if (total > 0) return Math.max(0, Math.min(100, completed * 100 / total));
  }
  return analysisRecordStatus(item) === "success" ? 100 : 0;
}

function analysisRecordStage(item) {
  const progress = item?.progress;
  return String(
    item?.stage
      || item?.phase
      || (progress && typeof progress === "object" ? progress.stage || progress.message : "")
      || item?.message
      || analysisRecordStatus(item),
  );
}

function analysisRecordMissing(item) {
  const raw = item?.missing || item?.missing_data || item?.warnings || item?.issues || [];
  return (Array.isArray(raw) ? raw : [raw])
    .map((entry) => preflightText(entry))
    .filter(Boolean);
}

function analysisTopicRecords() {
  const raw = state.analysis.bagDetail?.topics || [];
  return raw
    .map((topic) => ({
      name: String(topic?.name || topic?.topic || topic?.topic_metadata?.name || ""),
      type: String(topic?.type || topic?.topic_type || topic?.topic_metadata?.type || ""),
      count: Number(topic?.count ?? topic?.message_count ?? topic?.message_count_hint ?? 0),
    }))
    .filter((topic) => topic.name);
}

function analysisTopics(kind) {
  const topics = analysisTopicRecords();
  const matches = {
    image: (topic) => /sensor_msgs\/(msg\/)?(Image|CompressedImage)$/.test(topic.type) || /(^|\/)(image|image_raw|event_image)(\/|$)/i.test(topic.name),
    control: (topic) => /ControlCommand$/.test(topic.type) || /control_cmd$/i.test(topic.name),
    mode: (topic) => /OperationModeState$/.test(topic.type) || /operation_mode\/state$/i.test(topic.name),
    pose: (topic) => /nav_msgs\/(msg\/)?Odometry$/.test(topic.type) || /(^|\/)(odometry|odom)$/i.test(topic.name),
    speed: (topic) => /(Float32|Float64|Odometry)$/.test(topic.type) && /(speed|velocity|odometry)/i.test(topic.name),
  }[kind];
  return matches ? topics.filter(matches) : topics;
}

function preferredAnalysisTopic(kind, topics) {
  const preferences = {
    image: [
      "/realsense/color/image_raw/compressed",
      "/realsense/color/image_raw",
      "/oakd_lite/rgb/image_raw/compressed",
      "/oakd_lite/rgb/image_raw",
      "/event_camera/event_image",
      "/realsense/infra1/image_rect_raw",
    ],
    control: ["/vehicle/control_cmd", "/auto/control_cmd", "/teleop/control_cmd", "/propo/control_cmd"],
    mode: ["/operation_mode/state"],
    pose: ["/visual_slam/tracking/odometry", "/localization/odometry", "/odom"],
    speed: ["/visual_slam/tracking/odometry", "/commands/motor/speed"],
  }[kind] || [];
  return preferences.map((name) => topics.find((topic) => topic.name === name)).find(Boolean)?.name || topics[0]?.name || "";
}

function cameraTopicConfigForTopics(topics) {
  const topicNames = new Set(
    (topics || [])
      .map((topic) => String(topic?.name || topic?.topic || topic || ""))
      .filter(Boolean),
  );
  if (!topicNames.size) return null;

  const matches = (state.cameraTopicConfigs || []).filter((config) => {
    const required = Array.isArray(config.required_topics) ? config.required_topics.filter(Boolean) : [];
    return required.length > 0 && required.every((topic) => topicNames.has(String(topic)));
  });
  return matches.sort((left, right) => {
    const topicDifference = (right.required_topics?.length || 0) - (left.required_topics?.length || 0);
    if (topicDifference) return topicDifference;
    return Number(right.score || 0) - Number(left.score || 0);
  })[0] || null;
}

function defaultCameraTopicConfig() {
  const configs = state.cameraTopicConfigs || [];
  return configs.find((config) => config.recommended) || configs[0] || null;
}

function analysisTopicOptions(kind, selected, optional = true) {
  const topics = analysisTopics(kind);
  const emptyLabel = optional ? "Auto / unavailable" : `Select ${kind} topic`;
  return `<option value="">${esc(emptyLabel)}</option>${topics.map((topic) => {
    const detail = [topic.type, topic.count ? `${topic.count} msgs` : ""].filter(Boolean).join(" / ");
    return `<option value="${esc(topic.name)}" ${topic.name === selected ? "selected" : ""}>${esc(topic.name)}${detail ? ` - ${esc(detail)}` : ""}</option>`;
  }).join("")}`;
}

function analysisPreflightPayload() {
  const preset = state.analysis.analysisPreset || "telemetry";

  let selectedTopics = Array.isArray(state.analysis.selectedImageTopics)
    ? [...state.analysis.selectedImageTopics].filter(Boolean)
    : [];

  // If no checkbox state exists yet, default to single imageTopic
  if (!selectedTopics.length && state.analysis.imageTopic) {
    selectedTopics.push(state.analysis.imageTopic);
  }
  selectedTopics = [...new Set(selectedTopics)];

  // Primary topic MUST be one of the selected topics
  let primary = state.analysis.primaryImageTopic;
  if (!primary || !selectedTopics.includes(primary)) {
    primary = selectedTopics[0] || state.analysis.imageTopic || "";
  }

  const isTelemetryMode = preset === "telemetry";
  const mapDir = isTelemetryMode ? "" : state.analysis.selectedMapPath;
  const trajectoryMode = isTelemetryMode
    ? (state.analysis.poseTopic ? "recorded" : "auto")
    : state.analysis.trajectoryMode;

  return {
    rosbag: state.analysis.selectedBagPath,
    map_dir: mapDir,
    topic_config: isTelemetryMode ? "" : state.analysis.topicConfigPath,
    image_topic: primary,
    image_topics: selectedTopics,
    primary_image_topic: primary,
    control_topic: state.analysis.controlTopic,
    mode_topic: state.analysis.modeTopic,
    pose_topic: state.analysis.poseTopic,
    speed_topic: state.analysis.speedTopic,
    trajectory_mode: trajectoryMode,
    offline_localization_mode: isTelemetryMode ? "" : state.analysis.offlineLocalizationMode,
    max_fps: state.analysis.maxFps,
    offline_object_detection: Boolean(state.analysis.offlineObjectDetection),
    object_detection_model_root: state.analysis.objectDetectionModelRoot,
    object_detection_image_topic: state.analysis.objectDetectionImageTopic,
    object_detection_camera_info_topic: state.analysis.objectDetectionCameraInfoTopic,
    object_detection_source_width: Number(state.analysis.objectDetectionSourceWidth) || 424,
    object_detection_source_height: Number(state.analysis.objectDetectionSourceHeight) || 240,
    object_detection_network_width: Number(state.analysis.objectDetectionNetworkWidth) || 224,
    object_detection_network_height: Number(state.analysis.objectDetectionNetworkHeight) || 224,
    object_detection_max_fps: Number(state.analysis.objectDetectionMaxFps) || 15,
    object_detection_confidence: Number(state.analysis.objectDetectionConfidence),
    object_detection_nms: Number(state.analysis.objectDetectionNms),
    object_detection_replay_rate: Number(state.analysis.objectDetectionReplayRate) || 0.5,
    preset: preset,
  };
}

function toggleAnalysisImageTopic(topicName) {
  if (!Array.isArray(state.analysis.selectedImageTopics)) {
    state.analysis.selectedImageTopics = [];
  }
  const idx = state.analysis.selectedImageTopics.indexOf(topicName);
  if (idx >= 0) {
    state.analysis.selectedImageTopics.splice(idx, 1);
    if (state.analysis.primaryImageTopic === topicName) {
      state.analysis.primaryImageTopic = state.analysis.selectedImageTopics[0] || "";
    }
  } else {
    state.analysis.selectedImageTopics.push(topicName);
    if (!state.analysis.primaryImageTopic) {
      state.analysis.primaryImageTopic = topicName;
    }
  }
  state.analysis.imageTopic = state.analysis.primaryImageTopic;
  const payload = analysisPreflightPayload();
  bindAnalysisPreflight(payload);
  if (state.tab === "e2e-analysis") scheduleE2EPreflight({ immediate: true, force: true });
  else scheduleAnalysisPreflight({ immediate: true, force: true });
  if (isAnalysisTab()) render();
}

function setAnalysisPrimaryTopic(topicName) {
  state.analysis.primaryImageTopic = topicName;
  state.analysis.imageTopic = topicName;
  if (!Array.isArray(state.analysis.selectedImageTopics)) {
    state.analysis.selectedImageTopics = [topicName];
  } else if (!state.analysis.selectedImageTopics.includes(topicName)) {
    state.analysis.selectedImageTopics.push(topicName);
  }
  const payload = analysisPreflightPayload();
  bindAnalysisPreflight(payload);
  if (state.tab === "e2e-analysis") scheduleE2EPreflight({ immediate: true, force: true });
  else scheduleAnalysisPreflight({ immediate: true, force: true });
  if (isAnalysisTab()) render();
}

function setAnalysisPreset(preset) {
  state.analysis.analysisPreset = preset;
  if (preset === "telemetry") {
    state.analysis.selectedMapPath = "";
  }
  const payload = analysisPreflightPayload();
  bindAnalysisPreflight(payload);
  scheduleAnalysisPreflight({ immediate: true, force: true });
  if (isAnalysisTab()) render();
}

function renderAnalysisImageTopicsSelector() {
  const imageTopics = analysisTopics("image");
  if (!imageTopics.length) {
    return `<div class="field-hint" style="color:#e06c75;">No image topics found in this rosbag.</div>`;
  }

  let selected = state.analysis.selectedImageTopics;
  if (!Array.isArray(selected) || !selected.length) {
    const defaultTopic = preferredAnalysisTopic("image", imageTopics);
    selected = defaultTopic ? [defaultTopic] : [imageTopics[0].name];
    state.analysis.selectedImageTopics = selected;
  }
  let primary = state.analysis.primaryImageTopic || selected[0] || imageTopics[0].name;
  state.analysis.primaryImageTopic = primary;

  return `
    <div style="display:flex; flex-direction:column; gap:0.4rem; background:#111519; border:1px solid #28333e; border-radius:6px; padding:0.5rem 0.6rem;">
      ${imageTopics.map((item) => {
        const name = item.name;
        const isChecked = selected.includes(name);
        const isPrimary = primary === name;
        return `
          <div style="display:flex; align-items:center; justify-content:space-between; gap:0.5rem; padding:0.35rem 0.6rem; background:${isChecked ? '#1a242f' : '#14181c'}; border:1px solid ${isChecked ? '#386fa4' : '#222930'}; border-radius:5px;">
            <label style="display:flex; align-items:center; gap:0.5rem; cursor:pointer; font-size:0.85rem; color:${isChecked ? '#fff' : '#8a99a8'}; flex:1; margin:0;">
              <input type="checkbox" ${isChecked ? "checked" : ""} onchange="toggleAnalysisImageTopic(${js(name)})" />
              <strong style="font-weight:600;">${esc(name)}</strong>
              <span style="font-size:0.75rem; color:#687582;">${esc(item.type || "")}${item.count ? ` (${item.count} msgs)` : ""}</span>
            </label>
            ${isChecked ? `
              <label style="display:flex; align-items:center; gap:0.3rem; font-size:0.78rem; color:${isPrimary ? '#5aa8ff' : '#687582'}; cursor:pointer; margin:0; font-weight:${isPrimary ? '700' : '400'};">
                <input type="radio" name="primary_image_topic_group" ${isPrimary ? "checked" : ""} onchange="setAnalysisPrimaryTopic(${js(name)})" />
                ${isPrimary ? "Primary Clock" : "Set Primary"}
              </label>
            ` : ""}
          </div>
        `;
      }).join("")}
    </div>
  `;
}

function renderBagAnalysis() {
  return `
    <div class="page analysis-page">
      <div class="analysis-create-layout">
        <section class="panel analysis-create-panel">
          <div class="panel-header">
            <h2>New Bag Analysis</h2>
            <span class="spacer"></span>
            <button onclick="refreshAnalysisData()">Refresh</button>
          </div>
          <div class="panel-body">${renderAnalysisForm()}</div>
        </section>
        <section class="panel analysis-list-panel">
          <div class="panel-header"><h2>Analysis Jobs</h2><span class="spacer"></span></div>
          <div class="panel-body" id="analysis-job-list">${renderAnalysisList()}</div>
        </section>
      </div>
      <section class="panel analysis-viewer-panel">
        <div class="panel-header">
          <h2>Drive Viewer</h2>
          <span class="spacer"></span>
          ${state.analysis.selectedId ? `<button onclick="reloadAnalysisResult()">Reload Result</button>` : ""}
        </div>
        <div class="panel-body" id="analysis-viewer-body">${renderAnalysisViewer()}</div>
      </section>
    </div>
  `;
}

function e2eAnalysisPayload() {
  const analysis = state.analysis;
  const mode = analysis.e2eMode || "supervised";
  const selectedImages = Array.isArray(analysis.selectedImageTopics)
    ? analysis.selectedImageTopics.filter(Boolean)
    : [];
  const primaryImage = analysis.primaryImageTopic || analysis.imageTopic || selectedImages[0] || "";
  const model = state.e2eModels.find((item) => item.path === analysis.e2eModelPath);
  const modelTask = String(model?.task || model?.output?.task || "control");
  const trajectoryMode = mode === "offline_localization"
    ? "offline"
    : mode === "recorded_localization" || (mode === "supervised" && modelTask === "trajectory")
      ? "recorded"
      : "none";
  return {
    analysis_kind: "e2e",
    e2e_mode: mode,
    rosbag: analysis.selectedBagPath,
    label: `${shortName(analysis.selectedBagPath) || "E2E"} / ${mode}`,
    image_topic: primaryImage,
    primary_image_topic: primaryImage,
    image_topics: [...new Set([primaryImage, ...selectedImages].filter(Boolean))],
    control_topic: mode === "supervised" ? analysis.e2eTeacherTopic : analysis.e2ePredictionTopic,
    teacher_control_topic: analysis.e2eTeacherTopic,
    prediction_control_topic: analysis.e2ePredictionTopic,
    applied_control_topic: analysis.e2eAppliedTopic,
    comparison_control_topic: analysis.e2eAppliedTopic,
    mode_topic: analysis.modeTopic,
    pose_topic: analysis.poseTopic,
    speed_topic: analysis.speedTopic,
    e2e_diagnostic_topic: analysis.e2eDiagnosticTopic,
    e2e_imu_topic: analysis.e2eImuTopic,
    section_topic: analysis.e2eSectionTopic,
    trajectory_mode: trajectoryMode,
    offline_localization_mode: analysis.offlineLocalizationMode,
    map_dir: mode === "supervised" ? "" : analysis.selectedMapPath,
    topic_config: analysis.topicConfigPath,
    model_path: mode === "supervised" ? analysis.e2eModelPath : "",
    inference_provider: analysis.e2eProvider,
    manual_only: Boolean(analysis.e2eManualOnly),
    deadline_ms: Number(analysis.e2eDeadlineMs) || 33.3,
    max_fps: Number(analysis.maxFps) || 15,
  };
}

function updateE2EOption(key, value) {
  if (!(key in state.analysis)) return;
  if (key === "e2eManualOnly") state.analysis[key] = Boolean(value);
  else if (["e2eDeadlineMs", "maxFps"].includes(key)) state.analysis[key] = Number(value) || (key === "maxFps" ? 15 : 33.3);
  else state.analysis[key] = String(value ?? "");
  render();
  scheduleE2EPreflight();
}

function scheduleE2EPreflight(options = {}) {
  if (state.tab !== "e2e-analysis") return;
  clearTimeout(analysisPreflightTimer);
  const check = () => requestPreflight("analyze-e2e", e2eAnalysisPayload(), { force: Boolean(options.force) });
  if (options.immediate) check();
  else analysisPreflightTimer = setTimeout(check, 250);
}

const E2E_PIPELINE_TASK_KINDS = [
  "e2e-preprocess",
  "e2e-train",
  "e2e-export-onnx",
  "e2e-deploy",
];

function isE2EPipelineTask(task) {
  return E2E_PIPELINE_TASK_KINDS.includes(task?.kind);
}

function updateE2EPipelineOption(key, value) {
  if (!(key in state.e2ePipeline)) return;
  const booleanKeys = ["buildEngine"];
  const numberKeys = [
    "inputWidth", "inputHeight", "maxControlDtSec", "jpegQuality", "batchSize",
    "maxOdometryDtSec", "trajectoryPoints", "trajectoryHorizonSec", "trajectoryScaleM",
    "imuWindowSec", "imuSamples",
    "numWorkers", "epochs", "learningRate", "finetuneEpochs",
    "finetuneLearningRate", "valFraction", "fraction", "weightDecay", "seed",
  ];
  if (booleanKeys.includes(key)) state.e2ePipeline[key] = Boolean(value);
  else if (numberKeys.includes(key)) state.e2ePipeline[key] = Number(value);
  else state.e2ePipeline[key] = String(value ?? "");
  if (key === "deployProfile") {
    const profile = state.e2ePipeline.deployProfiles.find((item) => item.id === state.e2ePipeline.deployProfile);
    if (profile) state.e2ePipeline.deployHost = profile.host === "__manual__" ? "" : (profile.host || "");
    if (profile) {
      state.e2ePipeline.deployUser = profile.user || "";
      state.e2ePipeline.remoteRoot = profile.remote_root || "";
    }
  }
  if (key === "datasetDir") {
    const dataset = selectedE2EDataset();
    const current = state.e2ePipeline.experiments.find((item) => item.id === state.e2ePipeline.experiment);
    if (dataset && current?.task !== dataset.task) {
      state.e2ePipeline.experiment = state.e2ePipeline.experiments.find((item) => item.task === dataset.task)?.id || "";
    }
  }
  if (key === "runDir") {
    const run = selectedE2ERun();
    if (run?.task === "trajectory") state.e2ePipeline.deployPreset = "camera_trajectory";
    else if (run?.task === "control") state.e2ePipeline.deployPreset = "camera_control";
  }
  render();
}

function selectedE2EDataset() {
  return state.e2ePipeline.datasets.find((item) => item.path === state.e2ePipeline.datasetDir) || null;
}

function selectedE2ERun() {
  return state.e2ePipeline.runs.find((item) => item.path === state.e2ePipeline.runDir) || null;
}

function selectedE2EDeployProfile() {
  return state.e2ePipeline.deployProfiles.find((item) => item.id === state.e2ePipeline.deployProfile) || null;
}

async function startE2EPipelineTask(action, endpoint, title, target, payload) {
  if (!confirmAction({ title, target, detail: "The command will run as a visible, cancellable Console task." })) return;
  if (!beginAction(action, title)) return;
  try {
    const result = await api(endpoint, { method: "POST", body: JSON.stringify(payload) });
    if (result.task) {
      rememberStartedTask(result.task, action, payload);
      state.selectedTaskId = result.task.task_id;
      state.tasks = [result.task, ...state.tasks.filter((item) => item.task_id !== result.task.task_id)];
    }
    toast(`${title} started`);
  } catch (error) {
    toast(`${title} failed to start: ${error.message}`, "error");
  } finally {
    endAction(action, { renderAfter: state.tab === "e2e-analysis" });
  }
}

function createE2EDataset() {
  const pipeline = state.e2ePipeline;
  return startE2EPipelineTask(
    "e2e-pipeline:dataset",
    "/api/e2e/datasets/create",
    "Create E2E dataset",
    pipeline.datasetName,
    {
      rosbag: state.analysis.selectedBagPath,
      dataset_name: pipeline.datasetName,
      image_topic: pipeline.imageTopic || state.analysis.imageTopic,
      control_topic: pipeline.controlTopic,
      odometry_topic: pipeline.odometryTopic,
      imu_topic: pipeline.imuTopic,
      task: pipeline.datasetTask,
      input_width: pipeline.inputWidth,
      input_height: pipeline.inputHeight,
      max_control_dt_sec: pipeline.maxControlDtSec,
      max_odometry_dt_sec: pipeline.maxOdometryDtSec,
      trajectory_points: pipeline.trajectoryPoints,
      trajectory_horizon_sec: pipeline.trajectoryHorizonSec,
      trajectory_scale_m: pipeline.trajectoryScaleM,
      imu_window_sec: pipeline.imuWindowSec,
      imu_samples: pipeline.imuSamples,
      jpeg_quality: pipeline.jpegQuality,
    },
  );
}

function trainE2EModel() {
  const pipeline = state.e2ePipeline;
  return startE2EPipelineTask(
    "e2e-pipeline:train",
    "/api/e2e/training/start",
    "Train E2E model",
    pipeline.runName,
    {
      dataset_dir: pipeline.datasetDir,
      run_name: pipeline.runName,
      experiment: pipeline.experiment,
      batch_size: pipeline.batchSize,
      num_workers: pipeline.numWorkers,
      epochs: pipeline.epochs,
      learning_rate: pipeline.learningRate,
      finetune_epochs: pipeline.finetuneEpochs,
      finetune_learning_rate: pipeline.finetuneLearningRate,
      val_fraction: pipeline.valFraction,
      fraction: pipeline.fraction,
      weight_decay: pipeline.weightDecay,
      seed: pipeline.seed,
      device: pipeline.device,
    },
  );
}

function exportE2EOnnx() {
  const run = selectedE2ERun();
  return startE2EPipelineTask(
    "e2e-pipeline:export",
    "/api/e2e/export-onnx",
    "Export E2E ONNX",
    run?.name || state.e2ePipeline.runDir,
    { run_dir: state.e2ePipeline.runDir },
  );
}

function deployE2EModel() {
  const pipeline = state.e2ePipeline;
  const run = selectedE2ERun();
  const profile = selectedE2EDeployProfile();
  return startE2EPipelineTask(
    "e2e-pipeline:deploy",
    "/api/e2e/deploy",
    "Deploy E2E model",
    `${pipeline.deployUser || profile?.user || ""}@${pipeline.deployHost || profile?.host || ""}`,
    {
      model_path: run?.onnx_path || "",
      profile: pipeline.deployProfile,
      preset: pipeline.deployPreset,
      user: pipeline.deployUser,
      host: pipeline.deployHost,
      remote_root: pipeline.remoteRoot,
      build_engine: pipeline.buildEngine,
    },
  );
}

function useE2ERunForOfflineEval() {
  const run = selectedE2ERun();
  if (!run?.onnx_path) {
    toast("Export model.onnx before starting offline evaluation.", "error");
    return;
  }
  state.analysis.e2eMode = "supervised";
  state.analysis.e2eModelPath = run.onnx_path;
  render();
  requestAnimationFrame(() => $("e2e-offline-eval")?.scrollIntoView({ behavior: "smooth", block: "start" }));
  toast(`Offline evaluation model selected: ${run.name}`);
}

function e2ePipelineStageState() {
  const pipeline = state.e2ePipeline;
  const run = selectedE2ERun();
  return [
    ["Dataset", pipeline.datasets.length ? `${pipeline.datasets.length} ready` : "not created", Boolean(pipeline.datasets.length)],
    ["Training", pipeline.runs.some((item) => item.best_checkpoint) ? `${pipeline.runs.length} run(s)` : "not trained", pipeline.runs.some((item) => item.best_checkpoint)],
    ["ONNX", pipeline.runs.some((item) => item.onnx_path) ? "exported" : "not exported", pipeline.runs.some((item) => item.onnx_path)],
    ["Offline eval", state.analysis.analyses.some((item) => analysisRecordKind(item) === "e2e") ? "result available" : "not evaluated", state.analysis.analyses.some((item) => analysisRecordKind(item) === "e2e")],
    ["Jetson", state.tasks.some((item) => item.kind === "e2e-deploy" && item.status === "success") ? "deployed" : (run?.onnx_path ? "ready to deploy" : "waiting for ONNX"), state.tasks.some((item) => item.kind === "e2e-deploy" && item.status === "success")],
  ];
}

function renderE2EPipeline() {
  const pipeline = state.e2ePipeline;
  const dataset = selectedE2EDataset();
  const run = selectedE2ERun();
  const profile = selectedE2EDeployProfile();
  const hasTwoStages = pipeline.experiment === "mobilenet_head_then_finetune";
  const pipelineTasks = state.tasks.filter(isE2EPipelineTask).slice(0, 8);
  const imageTopic = pipeline.imageTopic || state.analysis.imageTopic;
  const experiments = dataset
    ? pipeline.experiments.filter((item) => item.task === dataset.task)
    : pipeline.experiments;
  return `
    <section class="panel e2e-pipeline-panel">
      <div class="panel-header"><h2>E2E Training & Deployment</h2><span class="spacer"></span><button onclick="refreshAll()">Refresh artifacts</button></div>
      <div class="panel-body">
        <div class="e2e-pipeline-progress">
          ${e2ePipelineStageState().map(([label, detail, done], index) => `<div class="${done ? "done" : ""}"><span>${index + 1}</span><strong>${esc(label)}</strong><small>${esc(detail)}</small></div>`).join("")}
        </div>
        <div class="e2e-pipeline-grid">
          <article class="e2e-pipeline-stage">
            <header><span>01</span><div><strong>Create dataset</strong><small>Images + control / future trajectory + causal IMU</small></div></header>
            <div class="field"><label>Rosbag</label><select onchange="selectAnalysisBag(this.value)"><option value="">Select rosbag</option>${state.rosbags.map((item) => `<option value="${esc(item.path)}" ${item.path === state.analysis.selectedBagPath ? "selected" : ""}>${esc(item.display_name || item.name)}</option>`).join("")}</select></div>
            <div class="field"><label>Dataset name</label><input value="${esc(pipeline.datasetName)}" onchange="updateE2EPipelineOption('datasetName', this.value)" /><div class="field-hint">${esc(pipeline.datasetRoot)}</div></div>
            <div class="field"><label>Learning task</label><select onchange="updateE2EPipelineOption('datasetTask', this.value)">${[["control","Control (steering + throttle)"],["trajectory","Trajectory (future odometry)"]].map(([value,label]) => `<option value="${value}" ${pipeline.datasetTask === value ? "selected" : ""}>${label}</option>`).join("")}</select></div>
            <div class="field"><label>Image topic</label><select onchange="updateE2EPipelineOption('imageTopic', this.value)">${analysisTopicOptions("image", imageTopic)}</select></div>
            <div class="field"><label>Teacher control</label><select onchange="updateE2EPipelineOption('controlTopic', this.value)">${analysisTopicOptions("control", pipeline.controlTopic)}</select></div>
            <div class="field"><label>Odometry for trajectory GT</label><input value="${esc(pipeline.odometryTopic)}" onchange="updateE2EPipelineOption('odometryTopic', this.value)" /></div>
            <div class="field"><label>IMU topic</label><input value="${esc(pipeline.imuTopic)}" onchange="updateE2EPipelineOption('imuTopic', this.value)" /></div>
            <div class="e2e-compact-fields"><label>Width<input type="number" min="32" value="${esc(pipeline.inputWidth)}" onchange="updateE2EPipelineOption('inputWidth', this.value)" /></label><label>Height<input type="number" min="32" value="${esc(pipeline.inputHeight)}" onchange="updateE2EPipelineOption('inputHeight', this.value)" /></label><label>Max Δt (s)<input type="number" min="0.001" step="0.01" value="${esc(pipeline.maxControlDtSec)}" onchange="updateE2EPipelineOption('maxControlDtSec', this.value)" /></label></div>
            ${pipeline.datasetTask === "trajectory" ? `<div class="e2e-compact-fields"><label>Points<input type="number" min="2" value="${esc(pipeline.trajectoryPoints)}" onchange="updateE2EPipelineOption('trajectoryPoints', this.value)" /></label><label>Horizon (s)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.trajectoryHorizonSec)}" onchange="updateE2EPipelineOption('trajectoryHorizonSec', this.value)" /></label><label>Scale (m)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.trajectoryScaleM)}" onchange="updateE2EPipelineOption('trajectoryScaleM', this.value)" /></label></div>` : ""}
            <details><summary>Alignment & IMU</summary><div class="e2e-compact-fields"><label>Odom Δt<input type="number" min="0.001" step="0.01" value="${esc(pipeline.maxOdometryDtSec)}" onchange="updateE2EPipelineOption('maxOdometryDtSec', this.value)" /></label><label>IMU window (s)<input type="number" min="0.01" step="0.1" value="${esc(pipeline.imuWindowSec)}" onchange="updateE2EPipelineOption('imuWindowSec', this.value)" /></label><label>IMU samples<input type="number" min="1" value="${esc(pipeline.imuSamples)}" onchange="updateE2EPipelineOption('imuSamples', this.value)" /></label></div></details>
            <button class="primary ${actionBusy("e2e-pipeline:dataset") ? "is-busy" : ""}" onclick="createE2EDataset()" ${state.analysis.selectedBagPath && imageTopic && (pipeline.datasetTask === "trajectory" ? pipeline.odometryTopic : pipeline.controlTopic) ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:dataset", "Dataset creation is starting...")}>${esc(actionButtonLabel("e2e-pipeline:dataset", "Create dataset", "Starting..."))}</button>
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>02</span><div><strong>Train model</strong><small>Adjust repeatable training parameters</small></div></header>
            <div class="field"><label>Dataset</label><select onchange="updateE2EPipelineOption('datasetDir', this.value)"><option value="">Select dataset</option>${pipeline.datasets.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.datasetDir ? "selected" : ""}>${esc(item.task)} · ${esc(item.name)} — ${esc(item.sample_count)} samples</option>`).join("")}</select></div>
            <div class="field"><label>Run name</label><input value="${esc(pipeline.runName)}" onchange="updateE2EPipelineOption('runName', this.value)" /><div class="field-hint">${esc(pipeline.runRoot)}</div></div>
            <div class="field"><label>Experiment</label><select onchange="updateE2EPipelineOption('experiment', this.value)">${experiments.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.experiment ? "selected" : ""}>${esc(item.label)}</option>`).join("")}</select></div>
            <div class="e2e-compact-fields"><label>Epochs<input type="number" min="1" value="${esc(pipeline.epochs)}" onchange="updateE2EPipelineOption('epochs', this.value)" /></label><label>Learning rate<input type="number" min="0.00000001" step="0.0001" value="${esc(pipeline.learningRate)}" onchange="updateE2EPipelineOption('learningRate', this.value)" /></label><label>Batch<input type="number" min="1" value="${esc(pipeline.batchSize)}" onchange="updateE2EPipelineOption('batchSize', this.value)" /></label></div>
            ${hasTwoStages ? `<div class="e2e-compact-fields"><label>Fine-tune epochs<input type="number" min="1" value="${esc(pipeline.finetuneEpochs)}" onchange="updateE2EPipelineOption('finetuneEpochs', this.value)" /></label><label>Fine-tune LR<input type="number" min="0.00000001" step="0.0001" value="${esc(pipeline.finetuneLearningRate)}" onchange="updateE2EPipelineOption('finetuneLearningRate', this.value)" /></label></div>` : ""}
            <details><summary>Advanced parameters</summary><div class="e2e-compact-fields"><label>Data fraction<input type="number" min="0.001" max="1" step="0.05" value="${esc(pipeline.fraction)}" onchange="updateE2EPipelineOption('fraction', this.value)" /></label><label>Validation<input type="number" min="0.01" max="0.9" step="0.05" value="${esc(pipeline.valFraction)}" onchange="updateE2EPipelineOption('valFraction', this.value)" /></label><label>Workers<input type="number" min="0" value="${esc(pipeline.numWorkers)}" onchange="updateE2EPipelineOption('numWorkers', this.value)" /></label><label>Weight decay<input type="number" min="0" max="1" step="0.0001" value="${esc(pipeline.weightDecay)}" onchange="updateE2EPipelineOption('weightDecay', this.value)" /></label><label>Seed<input type="number" min="0" value="${esc(pipeline.seed)}" onchange="updateE2EPipelineOption('seed', this.value)" /></label><label>Device<select onchange="updateE2EPipelineOption('device', this.value)">${[["","Auto"],["cuda","CUDA"],["mps","Apple MPS"],["cpu","CPU"]].map(([value,label]) => `<option value="${value}" ${pipeline.device === value ? "selected" : ""}>${label}</option>`).join("")}</select></label></div></details>
            <button class="primary ${actionBusy("e2e-pipeline:train") ? "is-busy" : ""}" onclick="trainE2EModel()" ${pipeline.datasetDir ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:train", "Training is starting...")}>${esc(actionButtonLabel("e2e-pipeline:train", "Start training", "Starting..."))}</button>
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>03</span><div><strong>Evaluate & export</strong><small>Checkpoint → ONNX → offline eval</small></div></header>
            <div class="field"><label>Training run</label><select onchange="updateE2EPipelineOption('runDir', this.value)"><option value="">Select run</option>${pipeline.runs.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.runDir ? "selected" : ""}>${esc(item.task)} · ${esc(item.name)} — ${esc(item.model || item.status)}</option>`).join("")}</select></div>
            ${run ? `<div class="e2e-run-summary"><span>${run.best_checkpoint ? "Checkpoint ready" : "No best checkpoint"}</span><span>${run.onnx_path ? "ONNX ready" : "ONNX not exported"}</span><small>${esc(run.path)}</small></div>` : `<div class="empty compact">Select a completed training run.</div>`}
            <div class="button-stack"><button class="primary ${actionBusy("e2e-pipeline:export") ? "is-busy" : ""}" onclick="exportE2EOnnx()" ${run?.best_checkpoint ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:export", "ONNX export is starting...")}>${esc(actionButtonLabel("e2e-pipeline:export", "Export ONNX", "Starting..."))}</button><button onclick="useE2ERunForOfflineEval()" ${run?.onnx_path ? "" : "disabled"}>Use in offline eval</button></div>
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>04</span><div><strong>Deploy to Jetson</strong><small>SCP + remote trtexec + model.plan</small></div></header>
            <div class="field"><label>Connection profile</label><select onchange="updateE2EPipelineOption('deployProfile', this.value)">${pipeline.deployProfiles.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployProfile ? "selected" : ""}>${esc(item.label)} — ${esc(item.host === "__manual__" ? "manual" : item.host)}</option>`).join("")}</select></div>
            <div class="e2e-compact-fields"><label>SSH user<input value="${esc(pipeline.deployUser || profile?.user || "")}" onchange="updateE2EPipelineOption('deployUser', this.value)" /></label><label>Host<input value="${esc(pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "")}" onchange="updateE2EPipelineOption('deployHost', this.value)" /></label></div>
            <div class="field"><label>Model preset</label><select onchange="updateE2EPipelineOption('deployPreset', this.value)">${pipeline.deployPresets.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployPreset ? "selected" : ""}>${esc(item.label)}</option>`).join("")}</select></div>
            <label class="check-row"><input type="checkbox" ${pipeline.buildEngine ? "checked" : ""} onchange="updateE2EPipelineOption('buildEngine', this.checked)" /><span>Build TensorRT engine on Jetson with trtexec (FP16)</span></label>
            <div class="field-hint">${esc(pipeline.remoteRoot || profile?.remote_root || "")}</div>
            <button class="primary ${actionBusy("e2e-pipeline:deploy") ? "is-busy" : ""}" onclick="deployE2EModel()" ${run?.onnx_path ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:deploy", "Deployment is starting...")}>${esc(actionButtonLabel("e2e-pipeline:deploy", "Transfer & deploy", "Starting..."))}</button>
          </article>
        </div>
        <details class="e2e-pipeline-tasks" ${pipelineTasks.some(isActiveTask) ? "open" : ""}><summary>Pipeline tasks (${pipelineTasks.length})</summary>${renderTaskTable(pipelineTasks)}</details>
      </div>
    </section>`;
}

const OBJECT_DETECTION_PIPELINE_TASK_KINDS = [
  "object-detection-validate-dataset",
  "object-detection-validate",
  "object-detection-train",
  "object-detection-export",
  "object-detection-export-onnx",
  "object-detection-deploy",
];

function isObjectDetectionPipelineTask(task) {
  return OBJECT_DETECTION_PIPELINE_TASK_KINDS.includes(task?.kind);
}

function objectDetectionDatasetPath(dataset) {
  return dataset?.data_yaml || dataset?.path || "";
}

function selectedObjectDetectionDataset() {
  return state.objectDetectionPipeline.datasets.find(
    (item) => objectDetectionDatasetPath(item) === state.objectDetectionPipeline.datasetYaml,
  ) || null;
}

function selectedObjectDetectionRun() {
  return state.objectDetectionPipeline.runs.find(
    (item) => item.path === state.objectDetectionPipeline.runDir,
  ) || null;
}

function selectedObjectDetectionSourceRun() {
  return state.objectDetectionPipeline.runs.find(
    (item) => item.path === state.objectDetectionPipeline.sourceRunDir,
  ) || null;
}

function selectedObjectDetectionDeployProfile() {
  return state.objectDetectionPipeline.deployProfiles.find(
    (item) => item.id === state.objectDetectionPipeline.deployProfile,
  ) || null;
}

function objectDetectionModelCatalog() {
  const records = [
    ...state.objectDetectionPipeline.models,
    ...state.objectDetectionPipeline.runs
      .filter((item) => item.model_root || item.onnx_path)
      .map((item) => ({
        name: item.name,
        path: item.model_root || String(item.onnx_path || "").replace(/\/model\.onnx$/, ""),
        source: "training_run",
      })),
  ];
  const seen = new Set();
  return records.filter((item) => {
    const path = String(item.path || "");
    if (!path || seen.has(path)) return false;
    seen.add(path);
    return true;
  });
}

function updateObjectDetectionPipelineOption(key, value) {
  const pipeline = state.objectDetectionPipeline;
  if (!(key in pipeline)) return;
  const booleanKeys = ["buildEngine", "simplify"];
  const numberKeys = ["epochs", "batch", "workers", "patience", "seed", "freeze", "opset"];
  if (booleanKeys.includes(key)) pipeline[key] = Boolean(value);
  else if (numberKeys.includes(key)) pipeline[key] = Number(value);
  else pipeline[key] = String(value ?? "");

  if (key === "deployProfile") {
    const profile = selectedObjectDetectionDeployProfile();
    if (profile) {
      pipeline.deployUser = profile.user || "";
      pipeline.deployHost = profile.host === "__manual__" ? "" : (profile.host || "");
      pipeline.remoteRoot = profile.remote_root || pipeline.remoteRoot;
    }
  }
  if (key === "runDir") {
    const run = selectedObjectDetectionRun();
    if (run?.name) pipeline.deployName = run.name;
  }
  if (key === "sourceRunDir" && pipeline.trainingMode === "resume") {
    const run = selectedObjectDetectionSourceRun();
    if (run?.name) pipeline.runName = run.name;
  }
  if (key === "trainingMode" && pipeline.trainingMode === "resume") {
    const run = selectedObjectDetectionSourceRun();
    if (run?.name) pipeline.runName = run.name;
  }
  render();
}

async function startObjectDetectionPipelineTask(action, endpoint, title, target, payload) {
  if (!confirmAction({
    title,
    target,
    detail: "The command will run as a visible, cancellable Console task.",
  })) return;
  if (!beginAction(action, title)) return;
  try {
    const result = await api(endpoint, { method: "POST", body: JSON.stringify(payload) });
    if (result.task) {
      rememberStartedTask(result.task, action, payload);
      state.selectedTaskId = result.task.task_id;
      state.tasks = [result.task, ...state.tasks.filter((item) => item.task_id !== result.task.task_id)];
    }
    toast(`${title} started`);
  } catch (error) {
    toast(`${title} failed to start: ${error.message}`, "error");
  } finally {
    endAction(action, { renderAfter: state.tab === "object-detection" });
  }
}

function validateObjectDetectionDataset() {
  const pipeline = state.objectDetectionPipeline;
  const dataset = selectedObjectDetectionDataset();
  return startObjectDetectionPipelineTask(
    "object-detection:validate",
    "/api/object-detection/datasets/validate",
    "Validate object-detection dataset",
    dataset?.name || pipeline.datasetYaml,
    { dataset_yaml: pipeline.datasetYaml },
  );
}

function trainObjectDetectionModel() {
  const pipeline = state.objectDetectionPipeline;
  const sourceRun = selectedObjectDetectionSourceRun();
  const checkpoint = pipeline.trainingMode === "resume"
    ? sourceRun?.last_checkpoint
    : pipeline.trainingMode === "fine_tune"
      ? sourceRun?.best_checkpoint
      : "";
  const datasetYaml = pipeline.trainingMode === "resume"
    ? (sourceRun?.dataset_yaml || "")
    : pipeline.datasetYaml;
  return startObjectDetectionPipelineTask(
    "object-detection:train",
    "/api/object-detection/training/start",
    pipeline.trainingMode === "resume" ? "Resume YOLOv8 training" : "Train YOLOv8 detector",
    pipeline.trainingMode === "resume" ? (sourceRun?.name || pipeline.sourceRunDir) : pipeline.runName,
    {
      mode: pipeline.trainingMode,
      dataset_yaml: datasetYaml,
      run_name: pipeline.runName,
      base_model: pipeline.baseModel,
      checkpoint,
      epochs: pipeline.epochs,
      batch: pipeline.batch,
      device: pipeline.device,
      workers: pipeline.workers,
      patience: pipeline.patience,
      seed: pipeline.seed,
      freeze: pipeline.freeze,
      opset: pipeline.opset,
      export_after_training: true,
    },
  );
}

function exportObjectDetectionOnnx() {
  const pipeline = state.objectDetectionPipeline;
  const run = selectedObjectDetectionRun();
  return startObjectDetectionPipelineTask(
    "object-detection:export",
    "/api/object-detection/export-onnx",
    "Export YOLOv8 ONNX",
    run?.name || pipeline.runDir,
    {
      run_dir: pipeline.runDir,
      opset: pipeline.opset,
      simplify: pipeline.simplify,
    },
  );
}

function deployObjectDetectionModel() {
  const pipeline = state.objectDetectionPipeline;
  const run = selectedObjectDetectionRun();
  const profile = selectedObjectDetectionDeployProfile();
  const user = pipeline.deployUser || profile?.user || "";
  const host = pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "";
  return startObjectDetectionPipelineTask(
    "object-detection:deploy",
    "/api/object-detection/deploy",
    "Deploy YOLOv8 to Jetson",
    `${user}@${host}`,
    {
      run_dir: pipeline.runDir,
      model_path: run?.onnx_path || "",
      profile: pipeline.deployProfile,
      user,
      host,
      remote_root: pipeline.remoteRoot,
      model_name: pipeline.deployName,
      build_engine: pipeline.buildEngine,
    },
  );
}

function useObjectDetectionRunForBagAnalysis() {
  const run = selectedObjectDetectionRun();
  const modelRoot = run?.model_root
    || (run?.onnx_path ? String(run.onnx_path).replace(/\/model\.onnx$/, "") : "");
  if (!modelRoot) {
    toast("Export model.onnx before selecting it for Bag Analysis.", "error");
    return;
  }
  state.analysis.offlineObjectDetection = true;
  state.analysis.objectDetectionModelRoot = modelRoot;
  state.tab = "bag-analysis";
  render();
  toast(`Bag Analysis model selected: ${run.name}`);
}

function objectDetectionSplitTotals(dataset) {
  const splits = dataset?.splits && typeof dataset.splits === "object" ? Object.values(dataset.splits) : [];
  return splits.reduce((totals, split) => ({
    images: totals.images + Number(split?.images || 0),
    labels: totals.labels + Number(split?.label_files || split?.labels || 0),
    annotations: totals.annotations + Number(split?.annotations || 0),
  }), { images: 0, labels: 0, annotations: 0 });
}

function objectDetectionPipelineStageState() {
  const pipeline = state.objectDetectionPipeline;
  const selectedRun = selectedObjectDetectionRun();
  return [
    ["Dataset", pipeline.datasets.length ? `${pipeline.datasets.length} found` : "not found", Boolean(pipeline.datasets.length)],
    ["Training", pipeline.runs.some((item) => item.best_checkpoint) ? `${pipeline.runs.length} run(s)` : "not trained", pipeline.runs.some((item) => item.best_checkpoint)],
    ["ONNX", pipeline.runs.some((item) => item.onnx_path) ? "exported" : "not exported", pipeline.runs.some((item) => item.onnx_path)],
    ["Bag eval", state.analysis.analyses.some((item) => Boolean(item?.manifest?.resolved?.offline_object_detection)) ? "result available" : "ready after export", state.analysis.analyses.some((item) => Boolean(item?.manifest?.resolved?.offline_object_detection))],
    ["Jetson", state.tasks.some((item) => item.kind === "object-detection-deploy" && item.status === "success") ? "deployed" : (selectedRun?.onnx_path ? "ready" : "waiting for ONNX"), state.tasks.some((item) => item.kind === "object-detection-deploy" && item.status === "success")],
  ];
}

function renderObjectDetectionPipeline() {
  const pipeline = state.objectDetectionPipeline;
  const dataset = selectedObjectDetectionDataset();
  const run = selectedObjectDetectionRun();
  const sourceRun = selectedObjectDetectionSourceRun();
  const profile = selectedObjectDetectionDeployProfile();
  const totals = objectDetectionSplitTotals(dataset);
  const metrics = run?.metrics || run?.last_metrics || {};
  const pipelineTasks = state.tasks.filter(isObjectDetectionPipelineTask).slice(0, 10);
  const sourceRequired = ["fine_tune", "resume"].includes(pipeline.trainingMode);
  const sourceReady = pipeline.trainingMode === "fine_tune"
    ? Boolean(sourceRun?.best_checkpoint)
    : pipeline.trainingMode === "resume"
      ? Boolean(sourceRun?.last_checkpoint)
      : true;
  const canTrain = sourceReady && (
    pipeline.trainingMode === "resume"
      ? Boolean(sourceRun?.dataset_yaml)
      : Boolean(pipeline.datasetYaml && pipeline.runName)
  );
  const host = pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "";
  const user = pipeline.deployUser || profile?.user || "";
  return `
    <div class="page object-detection-page">
      <section class="panel e2e-pipeline-panel">
        <div class="panel-header"><h2>YOLOv8 Object Detection</h2><span class="spacer"></span><button onclick="refreshAll()">Refresh artifacts</button></div>
        <div class="panel-body">
          <div class="notice compact">Training and ONNX export run on this Notebook. TensorRT <code>model.plan</code> is generated on the selected Jetson.</div>
          <div class="e2e-pipeline-progress">
            ${objectDetectionPipelineStageState().map(([label, detail, done], index) => `<div class="${done ? "done" : ""}"><span>${index + 1}</span><strong>${esc(label)}</strong><small>${esc(detail)}</small></div>`).join("")}
          </div>
          <div class="e2e-pipeline-grid">
            <article class="e2e-pipeline-stage">
              <header><span>01</span><div><strong>Select & validate dataset</strong><small>Roboflow YOLOv8 data.yaml / vehicle + barrier</small></div></header>
              <div class="field"><label>Dataset</label><select onchange="updateObjectDetectionPipelineOption('datasetYaml', this.value)"><option value="">No dataset found</option>${pipeline.datasets.map((item) => { const path = objectDetectionDatasetPath(item); return `<option value="${esc(path)}" ${path === pipeline.datasetYaml ? "selected" : ""}>${esc(item.name || shortName(path))}${item.valid === false ? " — invalid" : ""}</option>`; }).join("")}</select><div class="field-hint">Copy or unzip a Roboflow YOLOv8 export under ${esc(pipeline.datasetRoot)}.</div></div>
              ${dataset ? `<div class="e2e-run-summary"><span>${esc(totals.images)} images</span><span>${esc(totals.annotations)} boxes</span><small>Classes: ${esc((dataset.classes || []).join(" → ") || "not inspected")}</small><small>${esc(objectDetectionDatasetPath(dataset))}</small></div>${dataset.error ? `<div class="notice error compact">${esc(dataset.error)}</div>` : ""}` : `<div class="empty compact">No data.yaml has been selected.</div>`}
              <button class="primary ${actionBusy("object-detection:validate") ? "is-busy" : ""}" onclick="validateObjectDetectionDataset()" ${pipeline.datasetYaml ? "" : "disabled"} ${actionButtonAttrs("object-detection:validate", "Dataset validation is starting...")}>${esc(actionButtonLabel("object-detection:validate", "Validate all labels", "Starting..."))}</button>
            </article>

            <article class="e2e-pipeline-stage">
              <header><span>02</span><div><strong>Train / fine-tune</strong><small>Fixed 224 × 224 input for the Isaac ROS decoder</small></div></header>
              <div class="field"><label>Training mode</label><select onchange="updateObjectDetectionPipelineOption('trainingMode', this.value)">${[["train","New training"],["fine_tune","Fine-tune best.pt"],["resume","Resume interrupted run"]].map(([value,label]) => `<option value="${value}" ${pipeline.trainingMode === value ? "selected" : ""}>${label}</option>`).join("")}</select></div>
              ${sourceRequired ? `<div class="field"><label>Source run</label><select onchange="updateObjectDetectionPipelineOption('sourceRunDir', this.value)"><option value="">Select source run</option>${pipeline.runs.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.sourceRunDir ? "selected" : ""}>${esc(item.name)} — ${pipeline.trainingMode === "resume" ? (item.last_checkpoint ? "last.pt ready" : "no last.pt") : (item.best_checkpoint ? "best.pt ready" : "no best.pt")}</option>`).join("")}</select></div>` : `<div class="field"><label>Initial weights</label><select onchange="updateObjectDetectionPipelineOption('baseModel', this.value)">${pipeline.baseModels.map((item) => { const value = typeof item === "string" ? item : item.id; const label = typeof item === "string" ? item : (item.label || item.id); return `<option value="${esc(value)}" ${value === pipeline.baseModel ? "selected" : ""}>${esc(label)}</option>`; }).join("")}</select></div>`}
              ${pipeline.trainingMode !== "resume" ? `<div class="field"><label>Run name</label><input value="${esc(pipeline.runName)}" onchange="updateObjectDetectionPipelineOption('runName', this.value)" /><div class="field-hint">${esc(pipeline.runRoot)}</div></div>` : `<div class="field"><label>Resume target</label><input value="${esc(sourceRun?.name || "")}" disabled /><div class="field-hint">The dataset and run directory are inherited from the interrupted run.</div></div>`}
              <div class="e2e-compact-fields"><label>Epochs<input type="number" min="1" max="10000" value="${esc(pipeline.epochs)}" onchange="updateObjectDetectionPipelineOption('epochs', this.value)" /></label><label>Batch<input type="number" min="1" max="1024" value="${esc(pipeline.batch)}" onchange="updateObjectDetectionPipelineOption('batch', this.value)" /></label><label>Device<select onchange="updateObjectDetectionPipelineOption('device', this.value)">${[["0","CUDA 0"],["1","CUDA 1"],["mps","Apple MPS"],["cpu","CPU"]].map(([value,label]) => `<option value="${value}" ${pipeline.device === value ? "selected" : ""}>${label}</option>`).join("")}</select></label></div>
              <details><summary>Advanced training parameters</summary><div class="e2e-compact-fields"><label>Workers<input type="number" min="0" max="64" value="${esc(pipeline.workers)}" onchange="updateObjectDetectionPipelineOption('workers', this.value)" /></label><label>Patience<input type="number" min="0" max="10000" value="${esc(pipeline.patience)}" onchange="updateObjectDetectionPipelineOption('patience', this.value)" /></label><label>Freeze layers<input type="number" min="0" max="1000" value="${esc(pipeline.freeze)}" onchange="updateObjectDetectionPipelineOption('freeze', this.value)" /></label><label>Seed<input type="number" min="0" value="${esc(pipeline.seed)}" onchange="updateObjectDetectionPipelineOption('seed', this.value)" /></label></div></details>
              <button class="primary ${actionBusy("object-detection:train") ? "is-busy" : ""}" onclick="trainObjectDetectionModel()" ${canTrain ? "" : "disabled"} ${actionButtonAttrs("object-detection:train", "Training is starting...")}>${esc(actionButtonLabel("object-detection:train", pipeline.trainingMode === "resume" ? "Resume training" : "Start training", "Starting..."))}</button>
            </article>

            <article class="e2e-pipeline-stage">
              <header><span>03</span><div><strong>Review & export</strong><small>best.pt → fixed-shape model.onnx + metadata</small></div></header>
              <div class="field"><label>Training run</label><select onchange="updateObjectDetectionPipelineOption('runDir', this.value)"><option value="">Select run</option>${pipeline.runs.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.runDir ? "selected" : ""}>${esc(item.name)} — ${item.best_checkpoint ? "best.pt" : "incomplete"}${item.onnx_path ? " + ONNX" : ""}</option>`).join("")}</select></div>
              ${run ? `<div class="e2e-run-summary"><span>${run.best_checkpoint ? "Checkpoint ready" : "No best checkpoint"}</span><span>${run.onnx_path ? "ONNX ready" : "ONNX not exported"}</span><small>${metrics.epoch !== undefined ? `Epoch ${esc(metrics.epoch)} · ` : ""}mAP50 ${esc(metrics.map50 ?? metrics["metrics/mAP50(B)"] ?? "-")} · mAP50-95 ${esc(metrics.map50_95 ?? metrics["metrics/mAP50-95(B)"] ?? "-")}</small><small>${esc(run.path)}</small></div>` : `<div class="empty compact">Select a training run.</div>`}
              <div class="e2e-compact-fields"><label>ONNX opset<input type="number" min="11" max="20" value="${esc(pipeline.opset)}" onchange="updateObjectDetectionPipelineOption('opset', this.value)" /></label><label class="check-row"><input type="checkbox" ${pipeline.simplify ? "checked" : ""} onchange="updateObjectDetectionPipelineOption('simplify', this.checked)" /><span>Simplify ONNX</span></label></div>
              <div class="button-stack"><button class="primary ${actionBusy("object-detection:export") ? "is-busy" : ""}" onclick="exportObjectDetectionOnnx()" ${run?.best_checkpoint ? "" : "disabled"} ${actionButtonAttrs("object-detection:export", "ONNX export is starting...")}>${esc(actionButtonLabel("object-detection:export", "Export ONNX", "Starting..."))}</button><button onclick="useObjectDetectionRunForBagAnalysis()" ${run?.onnx_path ? "" : "disabled"}>Use in Bag Analysis</button></div>
            </article>

            <article class="e2e-pipeline-stage">
              <header><span>04</span><div><strong>Deploy to Jetson</strong><small>SSH transfer + target-side TensorRT FP16 build</small></div></header>
              <div class="field"><label>Connection profile</label><select onchange="updateObjectDetectionPipelineOption('deployProfile', this.value)"><option value="">Select profile</option>${pipeline.deployProfiles.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployProfile ? "selected" : ""}>${esc(item.label || item.id)} — ${esc(item.host === "__manual__" ? "manual" : item.host)}</option>`).join("")}</select></div>
              <div class="e2e-compact-fields"><label>SSH user<input value="${esc(user)}" onchange="updateObjectDetectionPipelineOption('deployUser', this.value)" /></label><label>Host<input value="${esc(host)}" onchange="updateObjectDetectionPipelineOption('deployHost', this.value)" /></label></div>
              <div class="field"><label>Remote model root</label><input value="${esc(pipeline.remoteRoot || profile?.remote_root || "")}" onchange="updateObjectDetectionPipelineOption('remoteRoot', this.value)" /></div>
              <div class="field"><label>Model name</label><input value="${esc(pipeline.deployName)}" onchange="updateObjectDetectionPipelineOption('deployName', this.value)" /><div class="field-hint">Installed as &lt;remote root&gt;/${esc(pipeline.deployName || "model")}/model.onnx.</div></div>
              <label class="check-row"><input type="checkbox" ${pipeline.buildEngine ? "checked" : ""} onchange="updateObjectDetectionPipelineOption('buildEngine', this.checked)" /><span>Build TensorRT model.plan on the Jetson (FP16)</span></label>
              <button class="primary ${actionBusy("object-detection:deploy") ? "is-busy" : ""}" onclick="deployObjectDetectionModel()" ${run?.onnx_path && host && user ? "" : "disabled"} ${actionButtonAttrs("object-detection:deploy", "Deployment is starting...")}>${esc(actionButtonLabel("object-detection:deploy", "Transfer & build", "Starting..."))}</button>
            </article>
          </div>
          <details class="e2e-pipeline-tasks" ${pipelineTasks.some(isActiveTask) ? "open" : ""}><summary>Object-detection tasks (${pipelineTasks.length})</summary>${renderTaskTable(pipelineTasks)}</details>
        </div>
      </section>
    </div>`;
}

function renderE2EAnalysis() {
  return `
    <div class="page analysis-page e2e-analysis-page">
      ${renderE2EPipeline()}
      <div class="analysis-create-layout">
        <section class="panel analysis-create-panel" id="e2e-offline-eval">
          <div class="panel-header"><h2>New E2E Analysis</h2><span class="spacer"></span><button onclick="refreshAnalysisData()">Refresh</button></div>
          <div class="panel-body">${renderE2EAnalysisForm()}</div>
        </section>
        <section class="panel analysis-list-panel">
          <div class="panel-header"><h2>E2E Analysis Jobs</h2><span class="spacer"></span></div>
          <div class="panel-body" id="analysis-job-list">${renderAnalysisList("e2e")}</div>
        </section>
      </div>
      <section class="panel analysis-viewer-panel">
        <div class="panel-header"><h2>E2E Bag Viewer</h2><span class="spacer"></span>${state.analysis.selectedId ? `<button onclick="reloadAnalysisResult()">Reload Result</button>` : ""}</div>
        <div class="panel-body" id="analysis-viewer-body">${renderAnalysisViewer()}</div>
      </section>
    </div>`;
}

function renderE2EAnalysisForm() {
  const analysis = state.analysis;
  const mode = analysis.e2eMode || "supervised";
  const payload = e2eAnalysisPayload();
  const topicConfigs = state.cameraTopicConfigs || [];
  const modeCards = [
    ["supervised", "1. Offline teacher comparison", "Run the selected ONNX model on bag images and compare steering/throttle with manual commands."],
    ["offline_localization", "2. Driven bag + offline localization", "Re-localize images against an existing map (or scratch VSLAM), then aggregate results by section."],
    ["recorded_localization", "3. Recorded online E2E + VSLAM", "Use the pose, E2E commands, diagnostics, and compute metrics already recorded in the bag."],
  ];
  return `
    <div class="form-grid analysis-form e2e-form">
      <div class="field full"><label>1. Evaluation mode</label><div class="e2e-mode-grid">
        ${modeCards.map(([value, title, detail]) => `<button type="button" class="e2e-mode-card ${mode === value ? "selected" : ""}" onclick="updateE2EOption('e2eMode', ${js(value)})"><strong>${esc(title)}</strong><span>${esc(detail)}</span></button>`).join("")}
      </div></div>
      <div class="field full"><label for="e2e-bag">2. Rosbag</label><select id="e2e-bag" onchange="selectAnalysisBag(this.value)"><option value="">Select rosbag</option>${state.rosbags.map((item) => `<option value="${esc(item.path)}" ${item.path === analysis.selectedBagPath ? "selected" : ""}>${esc(item.display_name || item.name)} - ${esc(item.path)}</option>`).join("")}</select><div class="field-hint">${analysis.bagDetailLoading ? "Inspecting topics..." : analysis.selectedBagPath ? esc(analysis.selectedBagPath) : "Select the bag to evaluate."}</div></div>
      <div class="field full"><label>3. Image topics</label>${renderAnalysisImageTopicsSelector()}</div>
      ${mode === "supervised" ? `
        <div class="field full"><label for="e2e-model">ONNX model</label><select id="e2e-model" onchange="updateE2EOption('e2eModelPath', this.value)"><option value="">Select model</option>${state.e2eModels.map((model) => `<option value="${esc(model.path)}" ${model.path === analysis.e2eModelPath ? "selected" : ""}>${esc(model.task || model.output?.task || "control")} · ${esc(model.name)} — ${esc(model.relative_path || model.path)}</option>`).join("")}</select><div class="field-hint">${state.e2eModels.length ? `${state.e2eModels.length} exported model(s) found. Trajectory models use recorded odometry as GT.` : "No model.onnx was found under the configured outputs folders."}</div></div>
        <div class="field"><label>Teacher control</label><select onchange="updateE2EOption('e2eTeacherTopic', this.value)">${analysisTopicOptions("control", analysis.e2eTeacherTopic)}</select></div>
        <div class="field"><label>Inference provider</label><select onchange="updateE2EOption('e2eProvider', this.value)">${[["auto","Auto (CUDA, then CPU)"],["cuda","CUDA"],["cpu","CPU"]].map(([value,label]) => `<option value="${value}" ${analysis.e2eProvider === value ? "selected" : ""}>${label}</option>`).join("")}</select></div>
        <label class="field full check-row"><input type="checkbox" ${analysis.e2eManualOnly ? "checked" : ""} onchange="updateE2EOption('e2eManualOnly', this.checked)" /><span>Evaluate MANUAL sections only</span></label>
      ` : `
        <div class="field"><label>Recorded E2E output</label><select onchange="updateE2EOption('e2ePredictionTopic', this.value)">${analysisTopicOptions("control", analysis.e2ePredictionTopic)}</select></div>
        <div class="field"><label>Applied vehicle command (optional)</label><select onchange="updateE2EOption('e2eAppliedTopic', this.value)">${analysisTopicOptions("control", analysis.e2eAppliedTopic)}</select></div>
        <div class="field full"><label>Existing map</label><select onchange="updateE2EOption('selectedMapPath', this.value)"><option value="">No existing map${mode === "offline_localization" ? " / scratch VSLAM only" : ""}</option>${state.maps.map((map) => `<option value="${esc(map.path)}" ${map.path === analysis.selectedMapPath ? "selected" : ""}>${esc(mapOptionLabel(map))}</option>`).join("")}</select></div>
        ${mode === "offline_localization" ? `<div class="field"><label>Offline localization method</label><select onchange="updateE2EOption('offlineLocalizationMode', this.value)">${[["auto","Auto: VGL then VSLAM"],["vgl","VGL required"],["vslam","Saved-map VSLAM"],["vslam_from_scratch","VSLAM from scratch"]].map(([value,label]) => `<option value="${value}" ${analysis.offlineLocalizationMode === value ? "selected" : ""}>${label}</option>`).join("")}</select></div><div class="field"><label>Camera configuration</label><select onchange="updateE2EOption('topicConfigPath', this.value)"><option value="">Use backend default</option>${topicConfigs.map((config) => `<option value="${esc(config.path)}" ${config.path === analysis.topicConfigPath ? "selected" : ""}>${esc(config.name)}</option>`).join("")}</select></div>` : ""}
        <div class="field"><label>Recorded pose</label><select onchange="updateAnalysisOption('poseTopic', this.value)">${analysisTopicOptions("pose", analysis.poseTopic)}</select></div>
        <div class="field"><label>E2E diagnostics</label><input value="${esc(analysis.e2eDiagnosticTopic)}" onchange="updateE2EOption('e2eDiagnosticTopic', this.value)" /></div>
        <div class="field"><label>IMU for fusion model</label><input value="${esc(analysis.e2eImuTopic)}" onchange="updateE2EOption('e2eImuTopic', this.value)" /></div>
      `}
      <div class="field"><label>Operation mode</label><select onchange="updateAnalysisOption('modeTopic', this.value)">${analysisTopicOptions("mode", analysis.modeTopic)}</select></div>
      <div class="field"><label>Speed</label><select onchange="updateAnalysisOption('speedTopic', this.value)">${analysisTopicOptions("speed", analysis.speedTopic)}</select></div>
      <div class="field"><label>Max extraction FPS</label><input type="number" min="1" max="60" value="${esc(analysis.maxFps)}" onchange="updateE2EOption('maxFps', this.value)" /></div>
      <div class="field"><label>Deadline (ms)</label><input type="number" min="0.1" step="0.1" value="${esc(analysis.e2eDeadlineMs)}" onchange="updateE2EOption('e2eDeadlineMs', this.value)" /></div>
      <div class="field full e2e-action-row"><button class="primary ${actionBusy("e2e-analysis:start") ? "is-busy" : ""}" onclick="startE2EAnalysis()" ${preflightButtonAttrs("analyze-e2e", payload)} ${actionButtonAttrs("e2e-analysis:start", "E2E analysis is starting...")}>${esc(actionButtonLabel("e2e-analysis:start", "Start E2E Analysis", "Starting..."))}</button><button onclick="scheduleE2EPreflight({ immediate:true, force:true })">Recheck</button><span>${renderPreflightButtonReason("analyze-e2e", payload)}</span></div>
      <details class="field full"><summary>Preflight readiness</summary><div>${renderReadinessPanel("analyze-e2e", payload, { title: "E2E analysis readiness" })}</div></details>
    </div>`;
}

async function startE2EAnalysis() {
  const payload = e2eAnalysisPayload();
  if (!confirmAction({ title: "Start E2E analysis?", target: payload.rosbag || "No rosbag selected", detail: `Mode: ${payload.e2e_mode}` })) return;
  if (!beginAction("e2e-analysis:start", "Starting E2E analysis")) return;
  if (!acquirePreflightExecution("analyze-e2e", payload)) { endAction("e2e-analysis:start"); return; }
  try {
    if (!(await confirmPreflight("analyze-e2e", payload))) return;
    const result = await api("/api/e2e-analyses", { method: "POST", body: JSON.stringify(payload) });
    if (result.preflight) cachePreflightResult("analyze-e2e", payload, result.preflight);
    if (result.task) { rememberStartedTask(result.task, "analyze-e2e", payload); state.selectedTaskId = result.task.task_id; }
    const record = normalizeAnalysisRecord(result.analysis || result);
    const id = analysisRecordId(record);
    if (id) {
      pauseAnalysisPlayback();
      state.analysis.analyses = [record, ...state.analysis.analyses.filter((item) => analysisRecordId(item) !== id)];
      state.analysis.selectedId = id;
      state.analysis.detail = null;
      state.analysis.timeline = null;
      state.analysis.currentTime = 0;
    }
    toast("E2E analysis started");
  } catch (error) {
    if (!capturePreflightError("analyze-e2e", payload, error)) toast(`E2E analysis start failed: ${error.message}`, "error");
  } finally {
    releasePreflightExecution("analyze-e2e", payload);
    endAction("e2e-analysis:start", { renderAfter: state.tab === "e2e-analysis" });
  }
}

function renderAnalysisForm() {
  const analysis = state.analysis;
  const preset = analysis.analysisPreset || "telemetry";
  const payload = analysisPreflightPayload();
  const bag = state.rosbags.find((item) => item.path === analysis.selectedBagPath);
  const detail = analysis.bagDetail;
  const duration = Number(detail?.duration_s ?? detail?.duration_seconds ?? 0);
  const topicConfigs = state.cameraTopicConfigs || [];

  return `
    <div class="form-grid analysis-form">
      <!-- 1. Rosbag Selection -->
      <div class="field full">
        <label for="analysis-bag">1. Select Rosbag</label>
        <select id="analysis-bag" onchange="selectAnalysisBag(this.value)">
          <option value="">Select rosbag</option>
          ${state.rosbags.map((item) => `<option value="${esc(item.path)}" ${item.path === analysis.selectedBagPath ? "selected" : ""}>${esc(item.display_name || item.name)} - ${esc(item.path)}</option>`).join("")}
        </select>
        <div class="field-hint">${analysis.bagDetailLoading ? "Inspecting topics..." : bag ? `${esc(bag.path)}${duration > 0 ? ` / ${formatAnalysisClock(duration)}` : ""}` : "Choose a local rosbag to inspect its topics."}</div>
      </div>

      <!-- 2. Mode Selection Preset -->
      <div class="field full">
        <label>2. Select Analysis Mode</label>
        <div class="preset-selector-cards" style="display:flex; gap:0.75rem; margin-top:0.2rem;">
          <div class="preset-card" onclick="setAnalysisPreset('telemetry')" style="flex:1; padding:0.65rem 0.85rem; border:1.5px solid ${preset === 'telemetry' ? '#386fa4' : '#2d3744'}; background:${preset === 'telemetry' ? 'rgba(56,111,164,0.18)' : '#161b22'}; border-radius:6px; cursor:pointer;">
            <strong style="display:block; font-size:0.92rem; color:#fff; margin-bottom:0.15rem;">🎬 Video & Telemetry Only</strong>
            <span style="font-size:0.76rem; color:#8a99a8; line-height:1.25; display:block;">No VSLAM. Fast extraction for video playback & control signals.</span>
          </div>
          <div class="preset-card" onclick="setAnalysisPreset('map_vslam')" style="flex:1; padding:0.65rem 0.85rem; border:1.5px solid ${preset === 'map_vslam' ? '#386fa4' : '#2d3744'}; background:${preset === 'map_vslam' ? 'rgba(56,111,164,0.18)' : '#161b22'}; border-radius:6px; cursor:pointer;">
            <strong style="display:block; font-size:0.92rem; color:#fff; margin-bottom:0.15rem;">🗺️ Map Alignment & VSLAM</strong>
            <span style="font-size:0.76rem; color:#8a99a8; line-height:1.25; display:block;">Run VSLAM / VGL to align trajectory on an HD Map.</span>
          </div>
        </div>
      </div>

      <!-- Action Button Area (Placed at the top for zero scrolling) -->
      <div class="field full" style="background:rgba(255,255,255,0.02); border:1px solid #28333e; border-radius:8px; padding:0.75rem 0.85rem; display:flex; align-items:center; justify-content:space-between; gap:1rem; margin:0.25rem 0;">
        <div>
          <button id="analysis-start" class="primary ${actionBusy("analysis:start") ? "is-busy" : ""}" onclick="startBagAnalysis()" style="padding:0.55rem 1.4rem; font-size:0.95rem; font-weight:700;" ${preflightButtonAttrs("analyze-rosbag", payload)} ${actionButtonAttrs("analysis:start", "Analysis is starting...")}>${esc(actionButtonLabel("analysis:start", "Start Analysis", "Starting..."))}</button>
          <button onclick="scheduleAnalysisPreflight({ immediate: true, force: true })" style="padding:0.55rem 0.8rem;">Recheck</button>
        </div>
        <div id="analysis-preflight-reason" style="font-size:0.8rem; color:#8a99a8; text-align:right;">${renderPreflightButtonReason("analyze-rosbag", payload)}</div>
      </div>

      <!-- 3. Multiple Camera Selection -->
      <div class="field full">
        <label>3. Select Image Topics (Check to include multiple cameras)</label>
        ${renderAnalysisImageTopicsSelector()}
      </div>

      <!-- 4. Telemetry Signals -->
      <div class="field">
        <label for="analysis-control-topic">Control Topic</label>
        <select id="analysis-control-topic" onchange="updateAnalysisOption('controlTopic', this.value)">${analysisTopicOptions("control", analysis.controlTopic)}</select>
      </div>
      <div class="field">
        <label for="analysis-mode-topic">Operation Mode Topic</label>
        <select id="analysis-mode-topic" onchange="updateAnalysisOption('modeTopic', this.value)">${analysisTopicOptions("mode", analysis.modeTopic)}</select>
      </div>
      <div class="field">
        <label for="analysis-speed-topic">Speed Topic (optional)</label>
        <select id="analysis-speed-topic" onchange="updateAnalysisOption('speedTopic', this.value)">${analysisTopicOptions("speed", analysis.speedTopic)}</select>
      </div>
      <div class="field">
        <label for="analysis-pose-topic">Recorded Pose Topic (optional)</label>
        <select id="analysis-pose-topic" onchange="updateAnalysisOption('poseTopic', this.value)">${analysisTopicOptions("pose", analysis.poseTopic)}</select>
      </div>
      <div class="field">
        <label for="analysis-max-fps">Max FPS</label>
        <input id="analysis-max-fps" type="number" min="1" max="60" step="1" value="${esc(analysis.maxFps)}" onchange="updateAnalysisOption('maxFps', this.value)" />
      </div>

      <div class="field full" style="border-top:1px solid #2d3744; padding-top:0.75rem; margin-top:0.25rem;">
        <label class="check-row">
          <input type="checkbox" ${analysis.offlineObjectDetection ? "checked" : ""} onchange="updateAnalysisOption('offlineObjectDetection', this.checked)" />
          <span><strong>Run YOLOv8 after recording</strong><small style="display:block; color:#8a99a8;">Replay RGB + CameraInfo and create a model-specific detection sidecar.</small></span>
        </label>
      </div>
      ${analysis.offlineObjectDetection ? `
        <div class="field full">
          <label>YOLOv8 model directory</label>
          <select onchange="updateAnalysisOption('objectDetectionModelRoot', this.value)">
            <option value="">Select exported or deployed model</option>
            ${objectDetectionModelCatalog().map((item) => `<option value="${esc(item.path)}" ${item.path === analysis.objectDetectionModelRoot ? "selected" : ""}>${esc(item.name || shortName(item.path))} — ${item.source === "training_run" ? "training export" : "model catalog"}</option>`).join("")}
            ${analysis.objectDetectionModelRoot && !objectDetectionModelCatalog().some((item) => item.path === analysis.objectDetectionModelRoot) ? `<option value="${esc(analysis.objectDetectionModelRoot)}" selected>${esc(shortName(analysis.objectDetectionModelRoot))} — current path</option>` : ""}
          </select>
          <div class="field-hint">Select a local training export or a model under ros2_ws/models/yolov8. The directory must contain model.onnx.</div>
        </div>
        <div class="field">
          <label>Detector RGB Image</label>
          <select onchange="updateAnalysisOption('objectDetectionImageTopic', this.value)">
            <option value="">Select raw RGB image</option>
            ${analysisTopicRecords().filter((topic) => /sensor_msgs\/(msg\/)?Image$/.test(topic.type)).map((topic) => `<option value="${esc(topic.name)}" ${topic.name === analysis.objectDetectionImageTopic ? "selected" : ""}>${esc(topic.name)}</option>`).join("")}
          </select>
        </div>
        <div class="field">
          <label>Detector CameraInfo</label>
          <select onchange="updateAnalysisOption('objectDetectionCameraInfoTopic', this.value)">
            <option value="">Select CameraInfo</option>
            ${analysisTopicRecords().filter((topic) => /sensor_msgs\/(msg\/)?CameraInfo$/.test(topic.type)).map((topic) => `<option value="${esc(topic.name)}" ${topic.name === analysis.objectDetectionCameraInfoTopic ? "selected" : ""}>${esc(topic.name)}</option>`).join("")}
          </select>
        </div>
        <div class="field"><label>Inference FPS</label><input type="number" min="0.1" max="240" step="0.5" value="${esc(analysis.objectDetectionMaxFps)}" onchange="updateAnalysisOption('objectDetectionMaxFps', this.value)" /></div>
        <div class="field"><label>Replay rate</label><input type="number" min="0.05" max="4" step="0.05" value="${esc(analysis.objectDetectionReplayRate)}" onchange="updateAnalysisOption('objectDetectionReplayRate', this.value)" /><div class="field-hint">Use 0.5 when inference cannot keep up at real time.</div></div>
        <div class="field"><label>Confidence</label><input type="number" min="0" max="1" step="0.01" value="${esc(analysis.objectDetectionConfidence)}" onchange="updateAnalysisOption('objectDetectionConfidence', this.value)" /></div>
        <div class="field"><label>NMS IoU</label><input type="number" min="0" max="1" step="0.01" value="${esc(analysis.objectDetectionNms)}" onchange="updateAnalysisOption('objectDetectionNms', this.value)" /></div>
        <details class="field full"><summary>Image dimensions</summary><div class="e2e-compact-fields"><label>Source W<input type="number" min="1" value="${esc(analysis.objectDetectionSourceWidth)}" onchange="updateAnalysisOption('objectDetectionSourceWidth', this.value)" /></label><label>Source H<input type="number" min="1" value="${esc(analysis.objectDetectionSourceHeight)}" onchange="updateAnalysisOption('objectDetectionSourceHeight', this.value)" /></label><label>Network W<input type="number" min="32" value="${esc(analysis.objectDetectionNetworkWidth)}" onchange="updateAnalysisOption('objectDetectionNetworkWidth', this.value)" /></label><label>Network H<input type="number" min="32" value="${esc(analysis.objectDetectionNetworkHeight)}" onchange="updateAnalysisOption('objectDetectionNetworkHeight', this.value)" /></label></div></details>
      ` : ""}

      <!-- Map & VSLAM Configuration (Only when Map Mode is selected) -->
      ${preset === "map_vslam" ? `
        <div class="field full" style="border-top:1px solid #2d3744; padding-top:0.75rem; margin-top:0.25rem;">
          <strong style="color:#5aa8ff; font-size:0.88rem;">Map & VSLAM Configuration</strong>
        </div>
        <div class="field full">
          <label for="analysis-map">Map used for trajectory alignment</label>
          <select id="analysis-map" onchange="updateAnalysisOption('selectedMapPath', this.value)">
            <option value="">No map / telemetry only</option>
            ${state.maps.map((map) => `<option value="${esc(map.path)}" ${map.path === analysis.selectedMapPath ? "selected" : ""}>${esc(mapOptionLabel(map))}</option>`).join("")}
          </select>
        </div>
        <div class="field full">
          <label for="analysis-topic-config">Offline localization camera configuration</label>
          <select id="analysis-topic-config" onchange="updateAnalysisOption('topicConfigPath', this.value)">
            <option value="">Use backend default</option>
            ${topicConfigs.map((config) => `<option value="${esc(config.path)}" ${config.path === analysis.topicConfigPath ? "selected" : ""}>${esc(config.name)}${config.recommended ? " - recommended" : ` - score ${esc(config.score)}`}</option>`).join("")}
          </select>
        </div>
        <div class="field">
          <label for="analysis-trajectory-mode">Trajectory source</label>
          <select id="analysis-trajectory-mode" onchange="updateAnalysisOption('trajectoryMode', this.value)">
            ${[
              ["auto", "Auto: recorded pose, then offline localization"],
              ["recorded", "Recorded pose only"],
              ["offline", "Run offline localization"],
              ["none", "Do not create trajectory"],
            ].map(([value, label]) => `<option value="${value}" ${analysis.trajectoryMode === value ? "selected" : ""}>${label}</option>`).join("")}
          </select>
        </div>
        <div class="field full">
          <label for="analysis-offline-localization-mode">Offline localization method</label>
          <select id="analysis-offline-localization-mode" onchange="updateAnalysisOption('offlineLocalizationMode', this.value)">
            ${[
              ["auto", "Auto: try VGL, then VSLAM saved-map origin fallback"],
              ["vgl", "VGL required: stop if VGL cannot localize"],
              ["vslam", "VSLAM saved map only: use map origin as initial pose"],
              ["vslam_from_scratch", "VSLAM from scratch: track/map from origin without existing map"],
            ].map(([value, label]) => `<option value="${value}" ${analysis.offlineLocalizationMode === value ? "selected" : ""}>${label}</option>`).join("")}
          </select>
        </div>
      ` : ''}

      <!-- Detailed Readiness Report (Collapsible) -->
      <details class="full" style="margin-top:0.5rem; background:#111417; border:1px solid #252e37; border-radius:6px; padding:0.5rem 0.8rem;">
        <summary style="font-size:0.82rem; font-weight:600; color:#8a99a8; cursor:pointer;">Preflight Readiness Details</summary>
        <div id="analysis-preflight" style="margin-top:0.5rem;">${renderReadinessPanel("analyze-rosbag", payload, { title: "Rosbag analysis readiness" })}</div>
        ${detail ? renderAnalysisTopicCoverage() : ''}
      </details>
    </div>
  `;
}

function renderAnalysisTopicCoverage() {
  const topics = analysisTopicRecords();
  const groups = [
    ["Images", analysisTopics("image").length],
    ["Controls", analysisTopics("control").length],
    ["Modes", analysisTopics("mode").length],
    ["Poses", analysisTopics("pose").length],
    ["Speeds", analysisTopics("speed").length],
  ];
  return `
    <div class="analysis-topic-coverage full">
      <div><strong>${esc(topics.length)}</strong><span>topics</span></div>
      ${groups.map(([label, count]) => `<div class="${count ? "ok" : "missing"}"><strong>${esc(count)}</strong><span>${esc(label)}</span></div>`).join("")}
    </div>
  `;
}

function analysisRecordKind(item) {
  return String(item?.analysis_kind || item?.request?.analysis_kind || item?.manifest?.analysis_kind || "bag");
}

function renderAnalysisList(kind = state.tab === "e2e-analysis" ? "e2e" : "bag") {
  const source = state.analysis.analyses.filter((item) => kind === "e2e" ? analysisRecordKind(item) === "e2e" : analysisRecordKind(item) !== "e2e");
  if (!source.length) return `<div class="empty">No ${kind === "e2e" ? "E2E " : ""}analysis jobs yet.</div>`;
  const items = filteredSortedItems(source, "analyses");
  return `
    ${renderListControls("analyses", items.length, source.length, "Search analysis, rosbag, map, status")}
    ${items.length
      ? renderDateGroupedList("analyses", items, renderAnalysisCards)
      : `<div class="empty">No analyses match the current filter.</div>`}
  `;
}

function renderAnalysisCards(items) {
  return `<div class="analysis-list">${items.map((item) => {
    const id = analysisRecordId(item);
    const status = analysisRecordStatus(item);
    const progress = analysisRecordProgress(item);
    const missing = analysisRecordMissing(item);
    const selected = id && id === state.analysis.selectedId;
    const canOpen = Boolean(item.timeline_available || status === "success");
    const taskId = String(item.task_id || item.task?.task_id || item.manifest?.task_id || item.status?.task_id || "");
    const bagPath = String(item.rosbag || item.bag_path || item.source?.rosbag || "");
    const mapPath = String(item.map_dir || item.map_path || item.map?.path || "");
    return `
      <article class="analysis-job ${selected ? "selected" : ""}">
        <div class="analysis-job-heading">
          <div><strong>${esc(item.name || item.label || item.title || shortName(bagPath) || id || "Analysis")}</strong><span>${esc(id)}</span></div>
          <span class="status ${esc(status)}">${esc(status)}</span>
        </div>
        <div class="analysis-progress"><span style="width:${progress.toFixed(1)}%"></span></div>
        <div class="analysis-progress-label"><span>${esc(analysisRecordStage(item))}</span><strong>${progress.toFixed(0)}%</strong></div>
        ${bagPath ? `<div class="path" title="${esc(bagPath)}">Bag: ${esc(bagPath)}</div>` : ""}
        ${mapPath ? `<div class="path" title="${esc(mapPath)}">Map: ${esc(mapPath)}</div>` : ""}
        ${missing.length ? `<div class="analysis-missing"><strong>Missing / degraded data</strong>${missing.slice(0, 4).map((message) => `<span>${esc(message)}</span>`).join("")}</div>` : ""}
        <div class="actions">
          <button class="${status === "success" ? "primary" : ""}" onclick="focusAnalysisResult(${js(id)})" ${id && canOpen && !(selected && state.analysis.loadingResult) ? "" : "disabled"}>${selected && state.analysis.loadingResult ? "Loading..." : selected && state.analysis.timeline ? "Viewing" : canOpen ? "Open" : "Processing"}</button>
          ${taskId ? `<button onclick="openTaskLog(${js(taskId)})">Log</button><button class="danger ${actionBusy(`task:stop:${taskId}`) ? "is-busy" : ""}" onclick="stopTask(${js(taskId)})" ${["running", "queued"].includes(status) ? "" : "disabled"} ${actionButtonAttrs(`task:stop:${taskId}`, "Stop request is being sent...")}>${esc(actionButtonLabel(`task:stop:${taskId}`, "Stop", "Stopping..."))}</button>` : ""}
          <button class="danger" onclick="deleteAnalysis(${js(id)})" ${["running", "queued"].includes(status) ? "disabled" : ""} title="Delete this analysis and all its data">Delete</button>
        </div>
      </article>`;
  }).join("")}</div>`;
}

async function deleteAnalysis(analysisId) {
  if (!analysisId) return;
  if (!confirm(`Delete analysis "${analysisId}"?\nThis will permanently remove all frames, timeline, and localization data.`)) return;
  try {
    const res = await fetch(`/api/analyses/${encodeURIComponent(analysisId)}`, { method: "DELETE" });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      alert(`Failed to delete: ${data.error || res.status}`);
      return;
    }
    if (state.analysis.selectedId === analysisId) {
      state.analysis.selectedId = null;
      state.analysis.timeline = null;
      state.analysis.detail = null;
    }
    await refreshAnalysisData();
  } catch (err) {
    alert(`Delete error: ${err.message}`);
  }
}

async function deleteRosbag(path) {
  if (!path) return;
  if (!confirm(`Move rosbag "${shortName(path)}" to Trash?\n\nThe folder is kept under .jetpilot_trash and can be restored later:\n${path}`)) return;
  try {
    const res = await fetch(`/api/rosbags/delete?path=${encodeURIComponent(path)}`, { method: "DELETE" });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || `${res.status} ${res.statusText}`);
    if (state.analysis.selectedBagPath === path) {
      state.analysis.selectedBagPath = "";
      state.analysis.bagDetail = null;
    }
    const buildRosbag = $("build-rosbag");
    if (buildRosbag?.value === path) buildRosbag.value = "";
    toast("Rosbag moved to Trash");
    await refreshAll();
  } catch (error) {
    toast(`Move to Trash failed: ${error.message}`, "error");
  }
}

async function toggleRosbagFavorite(path, favorite) {
  if (!path) return;
  try {
    await api("/api/rosbags/favorite", {
      method: "POST",
      body: JSON.stringify({ path, favorite }),
    });
    toast(favorite ? "Rosbag protected" : "Rosbag protection removed");
    await refreshAll();
  } catch (error) {
    toast(`Could not update protection: ${error.message}`, "error");
  }
}

async function renameRosbag(path, currentName) {
  if (!path) return;
  const nextName = window.prompt("New rosbag folder name\nUse letters, numbers, _, ., and - only.", currentName || shortName(path));
  if (nextName === null) return;
  const cleanName = nextName.trim();
  if (!cleanName || cleanName === currentName) return;
  try {
    const result = await api("/api/rosbags/rename", {
      method: "POST",
      body: JSON.stringify({ path, name: cleanName }),
    });
    if (state.analysis.selectedBagPath === path) state.analysis.selectedBagPath = result.path || "";
    const buildRosbag = $("build-rosbag");
    if (buildRosbag?.value === path) buildRosbag.value = result.path || "";
    toast("Rosbag renamed");
    await refreshAll();
  } catch (error) {
    toast(`Rename failed: ${error.message}`, "error");
  }
}

async function restoreRosbag(path) {
  if (!path) return;
  try {
    await api("/api/rosbags/restore", {
      method: "POST",
      body: JSON.stringify({ path }),
    });
    toast("Rosbag restored");
    await refreshAll();
  } catch (error) {
    toast(`Restore failed: ${error.message}`, "error");
  }
}

async function deleteTrashedRosbag(path) {
  if (!path) return;
  const name = shortName(path);
  const first = window.confirm(`Delete trashed rosbag "${name}" forever?\n\nThis permanently removes the folder and cannot be restored from the UI:\n${path}`);
  if (!first) return;
  const typed = window.prompt(`Type DELETE to permanently delete "${name}".`);
  if (typed !== "DELETE") return;
  try {
    await api(`/api/rosbags/trash?path=${encodeURIComponent(path)}`, { method: "DELETE" });
    toast("Rosbag permanently deleted");
    await refreshAll();
  } catch (error) {
    toast(`Permanent delete failed: ${error.message}`, "error");
  }
}

async function deleteMapFolder(path) {
  if (!path) return;
  if (!confirm(`Delete map "${shortName(path)}"?\n\nThis permanently removes the map folder, including VSLAM maps, HD map versions, racelines, and previews:\n${path}`)) return;
  try {
    const res = await fetch(`/api/maps/delete?path=${encodeURIComponent(path)}`, { method: "DELETE" });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || `${res.status} ${res.statusText}`);
    if (state.selectedMapPath === path) {
      state.selectedMapPath = null;
      state.selectedMapDetail = null;
    }
    if (state.analysis.selectedMapPath === path) state.analysis.selectedMapPath = "";
    toast("Map deleted");
    await refreshAll();
  } catch (error) {
    toast(`Map delete failed: ${error.message}`, "error");
  }
}

function renderAnalysisViewer() {
  const analysis = state.analysis;
  if (analysis.loadingResult) return `<div class="empty">Loading normalized timeline, frames, and map...</div>`;
  if (!analysis.selectedId || !analysis.timeline) {
    const selected = analysis.analyses.find((item) => analysisRecordId(item) === analysis.selectedId);
    if (selected) {
      const status = analysisRecordStatus(selected);
      const taskId = String(selected.task_id || selected.task?.task_id || selected.manifest?.task_id || selected.status?.task_id || "");
      const message = String(selected.message || selected.status?.message || selected.manifest?.message || "");
      if (status === "failed" || status === "stopped") {
        return `
          <div class="analysis-viewer-empty failed-view">
            <strong>Analysis ${esc(status.toUpperCase())}: ${esc(analysisRecordStage(selected))}</strong>
            <span>${esc(message || "Check the task log for detailed ROS output and error backtraces.")}</span>
            ${taskId ? `<div class="actions" style="margin-top: 1rem;"><button class="primary" onclick="openTaskLog(${js(taskId)})">Open Task Log (${esc(taskId)})</button></div>` : ""}
          </div>
        `;
      }
      const waiting = status !== "success";
      return `
        <div class="analysis-viewer-empty">
          <strong>${waiting ? "Analysis is processing" : "Select a completed analysis"}</strong>
          <span>${waiting ? esc(analysisRecordStage(selected)) : "The synchronized image, commands, mode, speed, and trajectory will appear here."}</span>
          ${taskId && waiting ? `<div class="actions" style="margin-top: 1rem;"><button onclick="openTaskLog(${js(taskId)})">View Live Log</button></div>` : ""}
        </div>
      `;
    }
    return `<div class="analysis-viewer-empty"><strong>Select a completed analysis</strong><span>The synchronized image, commands, mode, speed, and trajectory will appear here.</span></div>`;
  }
  const timeline = analysis.timeline;
  const duration = analysisDuration(timeline);
  const frames = analysisFrames(timeline);
  const trajectory = analysisTrajectory(timeline);
  const consistency = analysisMapConsistency();
  const issues = analysisTimelineIssues();
  return `
    <div class="analysis-viewer">
      <div class="analysis-viewer-title">
        <div><strong>${esc(analysis.detail?.name || analysis.detail?.label || analysis.detail?.title || analysis.selectedId)}</strong><span>${esc(analysis.detail?.rosbag || analysis.detail?.bag_path || timeline.rosbag || "")}</span></div>
        <div class="chips">
          <span class="chip ${frames.length ? "ok" : "missing"}">${esc(frames.length)} frames</span>
          <span class="chip ${trajectory.samples.length ? "ok" : "missing"}">${esc(trajectory.samples.length)} poses</span>
          <span class="chip ${consistency.className}">${esc(consistency.label)}</span>
        </div>
      </div>
      ${consistency.message ? `<div class="analysis-consistency ${consistency.className}"><strong>Map consistency</strong><span>${esc(consistency.message)}</span></div>` : ""}
      ${renderAnalysisSources()}
      ${renderE2ESummaryPanel()}
      <div class="analysis-media-grid">
        <div class="analysis-image-panel">
          ${renderAnalysisCameraSelector()}
          <div class="analysis-image-stage">
            <img id="analysis-frame-image-a" alt="Selected rosbag image frame A" decoding="async" />
            <img id="analysis-frame-image-b" alt="Selected rosbag image frame B" decoding="async" />
            <div id="analysis-multi-image-grid" class="analysis-multi-image-grid"></div>
            <div id="analysis-frame-empty" class="analysis-frame-empty visible">${frames.length ? "Loading frame..." : "No image frames were extracted."}</div>
            <span id="analysis-frame-time" class="analysis-frame-time">${formatAnalysisClock(analysis.currentTime)}</span>
          </div>
          <div class="analysis-playback-controls">
            <button onclick="stepAnalysisFrame(-1)" title="Previous frame">|&lt;</button>
            <button id="analysis-play-button" class="primary" onclick="toggleAnalysisPlayback()">${analysis.playing ? "Pause" : "Play"}</button>
            <button onclick="stepAnalysisFrame(1)" title="Next frame">&gt;|</button>
            <button onclick="seekAnalysisRelative(-5)">-5s</button>
            <button onclick="seekAnalysisRelative(5)">+5s</button>
            <input id="analysis-seek" type="range" min="0" max="${esc(duration)}" step="0.01" value="${esc(Math.min(duration, analysis.currentTime))}" oninput="seekAnalysisTime(this.value)" aria-label="Analysis time" />
            <span id="analysis-clock-label" class="mono">${formatAnalysisClock(analysis.currentTime)} / ${formatAnalysisClock(duration)}</span>
            <select id="analysis-rate" onchange="setAnalysisPlaybackRate(this.value)" aria-label="Playback rate">
              ${[0.25, 0.5, 1, 1.5, 2, 4].map((rate) => `<option value="${rate}" ${analysis.playbackRate === rate ? "selected" : ""}>${rate}x</option>`).join("")}
            </select>
          </div>
        </div>
        <div class="analysis-map-panel">
          <canvas id="analysis-map-canvas" class="analysis-map-canvas"></canvas>
          <div id="analysis-map-empty" class="analysis-map-empty">${trajectory.samples.length ? (analysis.mapDetail ? "" : "Map background unavailable; showing trajectory extent.") : "No synchronized trajectory is available."}</div>
          <div class="analysis-speed-legend ${timeline.e2e?.metrics?.teacher_free ? "aggression" : ""}"><span>${timeline.e2e?.metrics?.teacher_free ? "calm" : "slow"}</span><i></i><span>${timeline.e2e?.metrics?.teacher_free ? "aggressive" : "fast"}</span></div>
        </div>
      </div>
      ${timeline.e2e?.task === "trajectory" ? `
        <div class="analysis-trajectory-panel">
          <div class="analysis-trajectory-heading"><div><strong>Local trajectory</strong><span>base_link / synchronized with video</span></div><div class="analysis-trajectory-legend"><span class="gt">GT</span><span class="pred">Prediction</span></div></div>
          <canvas id="analysis-e2e-trajectory-canvas" class="analysis-e2e-trajectory-canvas"></canvas>
        </div>` : ""}
      <div class="analysis-signal-cards">
        ${analysisSignalCard("Mode", "analysis-value-mode", "-")}
        ${analysisSignalCard("Steering", "analysis-value-steering", "-")}
        ${analysisSignalCard("Throttle", "analysis-value-throttle", "-")}
        ${analysisSignalCard("Brake", "analysis-value-brake", "-")}
        ${analysisSignalCard("Vehicle speed", "analysis-value-speed", "-", "analysis-label-speed")}
        ${timeline.e2e?.mode && timeline.e2e?.task !== "trajectory" ? analysisSignalCard("E2E steering", "analysis-value-e2e-steering", "-") : ""}
        ${timeline.e2e?.mode && timeline.e2e?.task !== "trajectory" ? analysisSignalCard("Steering error / applied Δ", "analysis-value-e2e-steering-error", "-") : ""}
        ${timeline.e2e?.mode ? analysisSignalCard("Pipeline latency", "analysis-value-e2e-latency", "-") : ""}
        ${timeline.e2e?.mode && timeline.e2e?.task !== "trajectory" ? analysisSignalCard("Aggressiveness", "analysis-value-e2e-aggression", "-") : ""}
        ${timeline.e2e?.mode && timeline.e2e?.task !== "trajectory" ? analysisSignalCard("Lateral acceleration", "analysis-value-e2e-latacc", "-") : ""}
        ${timeline.e2e?.mode ? analysisSignalCard("Section", "analysis-value-e2e-section", "-") : ""}
        ${timeline.e2e?.task === "trajectory" ? analysisSignalCard("Trajectory ADE", "analysis-value-e2e-trajectory-ade", "-") : ""}
        ${timeline.e2e?.task === "trajectory" ? analysisSignalCard("Trajectory FDE", "analysis-value-e2e-trajectory-fde", "-") : ""}
      </div>
      <div class="analysis-chart-shell">
        <canvas id="analysis-timeline-canvas" class="analysis-timeline-canvas" onclick="seekAnalysisFromTimeline(event)"></canvas>
      </div>
      ${renderJetsonStatsPanel()}
      ${issues.length ? `<div class="analysis-data-issues"><strong>Missing or degraded data</strong>${issues.map((issue) => `<span>${esc(issue)}</span>`).join("")}</div>` : ""}
    </div>
  `;
}

function e2eMetricText(value, suffix = "", digits = 3) {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "-";
}

function renderE2ESummaryPanel() {
  const e2e = state.analysis.timeline?.e2e;
  if (!e2e?.mode) return "";
  const metrics = e2e.metrics || {};
  const teacherFree = metrics.teacher_free || {};
  const cards = e2e.task === "trajectory"
    ? [
      ["Trajectory ADE", e2eMetricText(metrics.trajectory?.ade_m, " m")],
      ["Trajectory FDE", e2eMetricText(metrics.trajectory?.fde_m, " m")],
      ["Point error p95", e2eMetricText(metrics.trajectory?.point_error_m?.p95, " m")],
      ["Inference p50", e2eMetricText(metrics.inference_ms?.p50, " ms", 2)],
      ["Inference p95", e2eMetricText(metrics.inference_ms?.p95, " ms", 2)],
      ["GT missing", String(metrics.missing_teacher || 0)],
    ]
    : e2e.mode === "supervised"
    ? [
      ["Steering MAE", e2eMetricText(metrics.steering?.mae)],
      ["Steering RMSE", e2eMetricText(metrics.steering?.rmse)],
      ["Throttle MAE", e2eMetricText(metrics.throttle?.mae)],
      ["Inference p50", e2eMetricText(metrics.inference_ms?.p50, " ms", 2)],
      ["Inference p95", e2eMetricText(metrics.inference_ms?.p95, " ms", 2)],
      ["Deadline misses", `${Number(metrics.deadline_miss_count || 0)} (${e2eMetricText(Number(metrics.deadline_miss_rate || 0) * 100, "%", 1)})`],
    ]
    : [
      ["Control samples", String(metrics.sample_count || 0)],
      ["Pipeline p50", e2eMetricText(metrics.pipeline_latency_ms?.p50, " ms", 2)],
      ["Pipeline p95", e2eMetricText(metrics.pipeline_latency_ms?.p95, " ms", 2)],
      ["Decoder p95", e2eMetricText(metrics.decoder_callback_ms?.p95, " ms", 2)],
      ["Applied steer Δ MAE", e2eMetricText(metrics.steering_applied?.mae)],
      ["Deadline misses", String(metrics.deadline_miss_count || 0)],
    ];
  const sections = e2e.sections || [];
  const events = e2e.events || [];
  const worst = [...(e2e.predictions || [])]
    .filter((item) => Number.isFinite(Number(item.trajectory_ade_m ?? item.steering_error ?? item.steering_applied_error ?? item.cross_track_error_m)))
    .sort((a, b) => Number(b.trajectory_ade_m ?? Math.max(Math.abs(Number(b.steering_error ?? b.steering_applied_error) || 0), Number(b.cross_track_error_m) || 0)) - Number(a.trajectory_ade_m ?? Math.max(Math.abs(Number(a.steering_error ?? a.steering_applied_error) || 0), Number(a.cross_track_error_m) || 0)))
    .slice(0, 5);
  const sectionTable = e2e.task === "trajectory"
    ? `<table><thead><tr><th>Section</th><th>Samples</th><th>ADE</th><th>FDE</th><th>Point p95</th><th>CTE p95</th><th>Inference p95</th><th>Laps</th></tr></thead><tbody>${sections.map((section) => `<tr onclick="seekAnalysisTime(${Number(section.t_start) || 0})"><td>${esc(section.section)}</td><td>${esc(section.sample_count || 0)}</td><td>${e2eMetricText(section.trajectory?.ade_m, " m")}</td><td>${e2eMetricText(section.trajectory?.fde_m, " m")}</td><td>${e2eMetricText(section.trajectory?.point_error_m?.p95, " m")}</td><td>${e2eMetricText(section.cross_track_error_m?.p95, " m")}</td><td>${e2eMetricText(section.inference_ms?.p95, " ms", 2)}</td><td>${esc((section.laps || []).join(", "))}</td></tr>`).join("")}</tbody></table>`
    : `<table><thead><tr><th>Section</th><th>Samples</th><th>Aggression</th><th>Steer rate p95</th><th>Lat. accel p95</th><th>CTE p95</th><th>Latency p95</th><th>Laps</th></tr></thead><tbody>${sections.map((section) => `<tr onclick="seekAnalysisTime(${Number(section.t_start) || 0})"><td>${esc(section.section)}</td><td>${esc(section.sample_count || section.pose_count || 0)}</td><td>${e2eMetricText(section.teacher_free?.score, "", 1)}</td><td>${e2eMetricText(section.teacher_free?.steering_rate_abs_per_s?.p95, " /s", 2)}</td><td>${e2eMetricText(section.teacher_free?.lateral_accel_abs_mps2?.p95, " m/s²", 2)}</td><td>${e2eMetricText(section.cross_track_error_m?.p95, " m")}</td><td>${e2eMetricText(section.pipeline_latency_ms?.p95 ?? section.inference_ms?.p95, " ms", 2)}</td><td>${esc((section.laps || []).join(", "))}</td></tr>`).join("")}</tbody></table>`;
  return `
    <section class="e2e-result-summary">
      <div class="e2e-result-heading"><strong>E2E evaluation</strong><span>${esc(e2e.mode)}${e2e.model ? ` / ${esc(shortName(e2e.model))}` : ""}</span></div>
      <div class="e2e-metric-grid">${cards.map(([label, value]) => `<div><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`).join("")}</div>
      ${teacherFree.score !== undefined ? `
        <div class="e2e-teacher-free">
          <div class="e2e-aggression-score"><span>Teacher-free Aggressiveness</span><strong>${e2eMetricText(teacherFree.score, " / 100", 1)}</strong><em class="${esc(teacherFree.classification || "")}">${esc(teacherFree.classification || "-")}</em><i><b style="width:${Math.max(0, Math.min(100, Number(teacherFree.score) || 0))}%"></b></i></div>
          <div class="e2e-metric-grid teacher-free-grid">
            ${[
              ["Steering rate p95", e2eMetricText(teacherFree.steering_rate_abs_per_s?.p95, " /s", 2)],
              ["Lateral accel p95", e2eMetricText(teacherFree.lateral_accel_abs_mps2?.p95, " m/s²", 2)],
              ["Long. accel p95", e2eMetricText(teacherFree.longitudinal_accel_abs_mps2?.p95, " m/s²", 2)],
              ["Jerk p95", e2eMetricText(teacherFree.jerk_abs_mps3?.p95, " m/s³", 2)],
              ["Steer saturation", e2eMetricText(Number(teacherFree.steering_saturation_rate || 0) * 100, "%", 1)],
              ["Oscillations", e2eMetricText(teacherFree.steering_oscillations_per_min, " /min", 1)],
              ["Aggressive samples", e2eMetricText(Number(teacherFree.aggressive_sample_rate || 0) * 100, "%", 1)],
              ["CTE p95", e2eMetricText(teacherFree.cross_track_error_m?.p95, " m", 3)],
            ].map(([label, value]) => `<div><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`).join("")}
          </div>
          <div class="field-hint">Score is the p95 weighted combination of steering/throttle rate, acceleration, jerk, lateral acceleration, and control saturation. Missing sensors are excluded from the weight normalization.</div>
        </div>` : ""}
      ${events.length ? `<div class="e2e-events"><strong>High-aggressiveness events</strong>${events.slice(0, 12).map((event) => `<button onclick="seekAnalysisTime(${Number(event.peak_t) || 0})"><span>${formatAnalysisClock(event.t_start)}–${formatAnalysisClock(event.t_end)} / ${esc(event.section || "-")}</span><b>${e2eMetricText(event.peak_score, "", 1)}</b><em>${esc((event.reasons || []).join(", ") || "combined load")}</em></button>`).join("")}</div>` : ""}
      ${sections.length ? `<details open><summary>Section metrics</summary><div class="e2e-section-table">${sectionTable}</div></details>` : ""}
      ${worst.length ? `<div class="e2e-worst"><strong>Worst samples</strong>${worst.map((item) => `<button onclick="seekAnalysisTime(${Number(item.t) || 0})"><span>${formatAnalysisClock(item.t)} / ${esc(item.section || "-")}</span><b>${Number.isFinite(Number(item.trajectory_ade_m)) ? `ADE ${Number(item.trajectory_ade_m).toFixed(3)} m` : Number.isFinite(Number(item.steering_error ?? item.steering_applied_error)) ? `steer ${Number(item.steering_error ?? item.steering_applied_error).toFixed(3)}` : `CTE ${Number(item.cross_track_error_m).toFixed(3)} m`}</b></button>`).join("")}</div>` : ""}
    </section>`;
}

function analysisSignalCard(label, id, value, labelId = "") {
  return `<div class="analysis-signal-card"><span ${labelId ? `id="${esc(labelId)}"` : ""}>${esc(label)}</span><strong id="${esc(id)}">${esc(value)}</strong></div>`;
}

function renderJetsonStatsPanel() {
  const stats = state.analysis.timeline?.jetson_stats;
  const series = stats?.series || [];
  if (!series.length) return "";
  const selected = new Set(analysisSelectedJetsonSeries().map((item) => item.id));
  const groups = [...new Set(series.map((item) => item.group).filter(Boolean))];
  return `
    <div class="analysis-jetson-panel">
      <div class="analysis-jetson-toolbar">
        <div><strong>Jetson stats</strong><span>${esc((stats.topics || []).join(", ") || "diagnostics")}</span></div>
        <div class="actions">
          <button onclick="setJetsonStatsPreset('load')">Load</button>
          <button onclick="setJetsonStatsPreset('thermal')">Thermal</button>
          <button onclick="setJetsonStatsPreset('power')">Power</button>
          <button onclick="setJetsonStatsPreset('clear')">Clear</button>
        </div>
      </div>
      <div class="analysis-jetson-selector">
        ${groups.map((group) => `
          <div class="analysis-jetson-group">
            <span>${esc(group)}</span>
            ${series.filter((item) => item.group === group).map((item) => `
              <label title="${esc(item.source || item.id)}">
                <input type="checkbox" ${selected.has(item.id) ? "checked" : ""} onchange="toggleJetsonStatsSeries(${js(item.id)})" />
                <i style="background:${esc(jetsonSeriesColor(item.id))}"></i>
                ${esc(item.label)}${item.unit ? ` <em>${esc(item.unit)}</em>` : ""}
              </label>
            `).join("")}
          </div>
        `).join("")}
      </div>
      <div class="analysis-chart-shell">
        <canvas id="analysis-jetson-canvas" class="analysis-jetson-canvas"></canvas>
      </div>
    </div>
  `;
}

function renderAnalysisSources() {
  const topics = state.analysis.detail?.topics || {};
  const trajectory = analysisTrajectory();
  const speed = analysisSpeedSeries()[0];
  const detection = state.analysis.timeline?.object_detection || state.analysis.detail?.object_detection || {};
  const detectionModel = detection?.metadata?.model?.onnx || detection?.metadata?.model_root || "";
  const rows = [
    ["Image", topics.image || state.analysis.detail?.image_topic],
    ["Control", topics.control || state.analysis.detail?.control_topic],
    ["Mode", topics.mode || state.analysis.detail?.mode_topic],
    [analysisSpeedSourceLabel(speed), speed?.source || topics.speed || trajectory.source],
    ["Trajectory", [
      trajectory.source,
      analysisLocalizationMethodLabel(trajectory.localizationMethod),
      trajectory.frameId ? `frame ${trajectory.frameId}` : "",
    ].filter(Boolean).join(" / ")],
    ["Object detection", [detection.source, detectionModel ? shortName(detectionModel) : ""].filter(Boolean).join(" / ")],
  ].filter(([, value]) => value);
  if (!rows.length) return "";
  return `<div class="analysis-source-row">${rows.map(([label, value]) => `<span><strong>${esc(label)}</strong>${esc(value)}</span>`).join("")}</div>`;
}

function analysisLocalizationMethodLabel(value) {
  return {
    vgl: "VGL + VSLAM",
    vslam_identity: "VSLAM map origin",
    vslam_identity_fallback: "VSLAM map-origin fallback",
    vslam_from_scratch: "VSLAM from scratch (no map)",
  }[String(value || "")] || "";
}

async function openBagAnalysis(path = "") {
  state.tab = "bag-analysis";
  if (!state.analysis.selectedMapPath) state.analysis.selectedMapPath = pipelineMap()?.path || "";
  if (!state.analysis.topicConfigPath) {
    const configs = state.cameraTopicConfigs || [];
    state.analysis.topicConfigPath = (configs.find((config) => config.recommended) || configs[0])?.path || "";
  }
  render();
  if (path) await selectAnalysisBag(path);
}

async function selectAnalysisBag(path) {
  const selectedPath = String(path || "");
  const changed = selectedPath !== state.analysis.selectedBagPath;
  const previousTopics = {
    image: state.analysis.imageTopic,
    control: state.analysis.controlTopic,
    mode: state.analysis.modeTopic,
    pose: state.analysis.poseTopic,
    speed: state.analysis.speedTopic,
  };
  state.analysis.selectedBagPath = selectedPath;
  state.analysis.bagDetail = null;
  state.analysis.bagDetailLoading = Boolean(selectedPath);
  if (changed) {
    state.analysis.imageTopic = "";
    state.analysis.controlTopic = "";
    state.analysis.modeTopic = "";
    state.analysis.poseTopic = "";
    state.analysis.speedTopic = "";
  }
  if (isAnalysisTab()) render();
  if (!selectedPath) {
    if (state.tab === "e2e-analysis") scheduleE2EPreflight({ immediate: true, force: true });
    else scheduleAnalysisPreflight({ immediate: true, force: true });
    return;
  }
  try {
    const detail = await api(apiPath("/api/rosbags/detail", { path: selectedPath }));
    if (state.analysis.selectedBagPath !== selectedPath) return;
    state.analysis.bagDetail = detail.rosbag || detail;
    if (changed) {
      const selectedConfig = cameraTopicConfigForTopics(analysisTopicRecords()) || defaultCameraTopicConfig();
      state.analysis.topicConfigPath = selectedConfig?.path || "";
    }
    for (const kind of ["image", "control", "mode", "pose", "speed"]) {
      const key = `${kind}Topic`;
      state.analysis[key] = changed ? preferredAnalysisTopic(kind, analysisTopics(kind)) : previousTopics[kind];
    }
    if (changed) {
      state.analysis.selectedImageTopics = state.analysis.imageTopic ? [state.analysis.imageTopic] : [];
      state.analysis.primaryImageTopic = state.analysis.imageTopic || "";
      state.e2ePipeline.imageTopic = state.analysis.imageTopic || "";
      state.e2ePipeline.controlTopic = analysisTopics("control").find((topic) => topic.name === "/teleop/control_cmd")?.name
        || state.analysis.controlTopic
        || "";
      state.e2ePipeline.odometryTopic = state.analysis.poseTopic || state.e2ePipeline.odometryTopic;
      const imuTopic = analysisTopicRecords().find((topic) => topic.type === "sensor_msgs/msg/Imu" && topic.name === "/sensors/imu")
        || analysisTopicRecords().find((topic) => topic.type === "sensor_msgs/msg/Imu");
      if (imuTopic?.name) {
        state.e2ePipeline.imuTopic = imuTopic.name;
        state.analysis.e2eImuTopic = imuTopic.name;
      }
      const rawRgb = analysisTopicRecords().find((topic) => topic.name === "/realsense/color/image_raw" && /sensor_msgs\/(msg\/)?Image$/.test(topic.type))
        || analysisTopicRecords().find((topic) => /sensor_msgs\/(msg\/)?Image$/.test(topic.type) && /(color|rgb)/i.test(topic.name))
        || analysisTopicRecords().find((topic) => /sensor_msgs\/(msg\/)?Image$/.test(topic.type));
      const cameraInfo = analysisTopicRecords().find((topic) => topic.name === "/realsense/color/camera_info" && /CameraInfo$/.test(topic.type))
        || analysisTopicRecords().find((topic) => /CameraInfo$/.test(topic.type) && rawRgb && topic.name.startsWith(rawRgb.name.replace(/\/image[^/]*$/, "")))
        || analysisTopicRecords().find((topic) => /CameraInfo$/.test(topic.type));
      state.analysis.objectDetectionImageTopic = rawRgb?.name || "";
      state.analysis.objectDetectionCameraInfoTopic = cameraInfo?.name || "";
    }
  } catch (error) {
    if (state.analysis.selectedBagPath === selectedPath) {
      state.analysis.bagDetail = { topics: [], error: error.message };
      toast(`Rosbag inspection failed: ${error.message}`, "error");
    }
  } finally {
    if (state.analysis.selectedBagPath === selectedPath) {
      state.analysis.bagDetailLoading = false;
      if (isAnalysisTab()) render();
      if (state.tab === "e2e-analysis") scheduleE2EPreflight({ immediate: true, force: true });
      else scheduleAnalysisPreflight({ immediate: true, force: true });
    }
  }
}

function updateAnalysisOption(key, value) {
  if (!(key in state.analysis)) return;
  if (key === "maxFps") {
    const number = Number(value);
    state.analysis.maxFps = Number.isFinite(number) ? Math.max(1, Math.min(60, Math.round(number))) : 15;
  } else if (key === "offlineObjectDetection") {
    state.analysis.offlineObjectDetection = Boolean(value);
  } else if ([
    "objectDetectionSourceWidth", "objectDetectionSourceHeight",
    "objectDetectionNetworkWidth", "objectDetectionNetworkHeight",
    "objectDetectionMaxFps", "objectDetectionConfidence",
    "objectDetectionNms", "objectDetectionReplayRate",
  ].includes(key)) {
    state.analysis[key] = Number(value);
  } else {
    state.analysis[key] = String(value ?? "");
  }
  if (state.tab === "e2e-analysis") scheduleE2EPreflight();
  else {
    const payload = analysisPreflightPayload();
    bindAnalysisPreflight(payload);
    scheduleAnalysisPreflight();
  }
  if (key === "selectedMapPath" && state.analysis.timeline) updateAnalysisConsistencyDom();
  if (key === "offlineObjectDetection" && isAnalysisTab()) render();
}

function bindAnalysisPreflight(payload) {
  const token = preflightToken("analyze-rosbag", payload);
  const executionToken = preflightExecutionToken("analyze-rosbag", payload);
  const mapResourceToken = preflightMapResourceToken(payload);
  ["analysis-preflight", "analysis-start", "analysis-preflight-reason"].forEach((id) => {
    const element = $(id);
    if (!element) return;
    const targets = element.dataset.preflightRole ? [element] : [...element.querySelectorAll("[data-preflight-role]")];
    targets.forEach((target) => {
      target.dataset.preflightToken = token;
      target.dataset.preflightExecutionToken = executionToken;
      target.dataset.preflightMapResourceToken = mapResourceToken;
    });
  });
  updatePreflightDom("analyze-rosbag", payload);
}

function scheduleAnalysisPreflight(options = {}) {
  if (state.tab !== "bag-analysis") return;
  clearTimeout(analysisPreflightTimer);
  const check = () => {
    const payload = analysisPreflightPayload();
    bindAnalysisPreflight(payload);
    requestPreflight("analyze-rosbag", payload, { force: Boolean(options.force) });
  };
  if (options.immediate) check();
  else analysisPreflightTimer = setTimeout(check, 250);
}

async function refreshAnalysisList() {
  const payload = await api("/api/analyses");
  state.analysis.analyses = normalizeAnalysisList(payload);
  return state.analysis.analyses;
}

async function refreshAnalysisData() {
  try {
    await Promise.all([
      refreshAnalysisList(),
      state.analysis.selectedBagPath ? selectAnalysisBag(state.analysis.selectedBagPath) : Promise.resolve(),
    ]);
    if (isAnalysisTab()) render();
  } catch (error) {
    toast(`Analysis refresh failed: ${error.message}`, "error");
  }
}

function updateAnalysisListDom() {
  const target = $("analysis-job-list");
  if (target) target.innerHTML = renderAnalysisList();
}

async function startBagAnalysis() {
  const payload = analysisPreflightPayload();
  const topics = [payload.primary_image_topic, ...(payload.image_topics || [])].filter(Boolean);
  if (!confirmAction({
    title: "Start rosbag analysis?",
    target: payload.rosbag || "No rosbag selected",
    detail: `Mode: ${payload.preset || "telemetry"}\nImage topics: ${topics.length ? [...new Set(topics)].join(", ") : "not selected"}`,
  })) return;
  if (!beginAction("analysis:start", "Starting analysis")) return;
  if (!acquirePreflightExecution("analyze-rosbag", payload)) {
    endAction("analysis:start");
    return;
  }
  try {
    if (!(await confirmPreflight("analyze-rosbag", payload))) return;
    const result = await api("/api/analyses", { method: "POST", body: JSON.stringify(payload) });
    if (result.preflight) cachePreflightResult("analyze-rosbag", payload, result.preflight);
    if (result.task) {
      rememberStartedTask(result.task, "analyze-rosbag", payload);
      state.selectedTaskId = result.task.task_id;
    }
    const record = normalizeAnalysisRecord(result.analysis || result.job || result);
    const id = analysisRecordId(record);
    if (id) {
      pauseAnalysisPlayback();
      state.analysis.analyses = [record, ...state.analysis.analyses.filter((item) => analysisRecordId(item) !== id)];
      state.analysis.selectedId = id;
      state.analysis.detail = null;
      state.analysis.timeline = null;
      state.analysis.mapDetail = null;
      state.analysis.currentTime = 0;
      resetAnalysisFrameRenderState();
    }
    updateAnalysisListDom();
    const viewer = $("analysis-viewer-body");
    if (viewer) viewer.innerHTML = renderAnalysisViewer();
    updateTaskChrome();
    toast("Rosbag analysis started");
  } catch (error) {
    if (error?.payload?.active_task?.task_id) {
      rememberStartedTask(error.payload.active_task, "analyze-rosbag", payload);
      refreshVisiblePreflightDom();
      toast(error.payload.error || "Another offline analysis is already running.", "error");
      return;
    }
    if (capturePreflightError("analyze-rosbag", payload, error)) {
      toast(preflightBlockingReason(preflightEntry("analyze-rosbag", payload)), "error");
      return;
    }
    toast(`Analysis start failed: ${error.message}`, "error");
  } finally {
    releasePreflightExecution("analyze-rosbag", payload);
    endAction("analysis:start", { renderAfter: state.tab === "bag-analysis" });
  }
}

async function openAnalysisResult(id) {
  const selectedId = String(id || "");
  if (!selectedId) return;
  const selectedRecord = state.analysis.analyses.find((item) => analysisRecordId(item) === selectedId);
  const previousResult = {
    selectedId: state.analysis.selectedId,
    detail: state.analysis.detail,
    timeline: state.analysis.timeline,
    mapDetail: state.analysis.mapDetail,
    currentTime: state.analysis.currentTime,
    visualFailures: state.analysis.visualFailures,
  };
  let loadError = null;
  pauseAnalysisPlayback();
  state.analysis.selectedId = selectedId;
  state.analysis.loadingResult = true;
  state.analysis.detail = null;
  state.analysis.timeline = null;
  state.analysis.mapDetail = null;
  state.analysis.currentTime = 0;
  state.analysis.visualFailures = {};
  resetAnalysisFrameRenderState();
  state.tab = analysisRecordKind(selectedRecord) === "e2e" ? "e2e-analysis" : "bag-analysis";
  render();
  try {
    const [detail, timeline] = await Promise.all([
      api(`/api/analyses/${encodeURIComponent(selectedId)}`),
      api(`/api/analyses/${encodeURIComponent(selectedId)}/timeline`),
    ]);
    if (state.analysis.selectedId !== selectedId) return;
    state.analysis.detail = normalizeAnalysisRecord(detail.analysis || detail);
    state.analysis.timeline = normalizeAnalysisTimeline(timeline.timeline || timeline);
    const mapPath = analysisResultMapPath();
    if (mapPath) {
      if (!state.analysis.selectedMapPath) state.analysis.selectedMapPath = mapPath;
      try {
        state.analysis.mapDetail = await api(apiPath("/api/maps/detail", { path: mapPath }));
      } catch (error) {
        state.analysis.mapDetail = null;
        state.analysis.timeline.map_load_error = error.message;
      }
    }
  } catch (error) {
    if (state.analysis.selectedId === selectedId) {
      loadError = error;
      if (previousResult.timeline) {
        state.analysis.selectedId = previousResult.selectedId;
        state.analysis.detail = previousResult.detail;
        state.analysis.timeline = previousResult.timeline;
        state.analysis.mapDetail = previousResult.mapDetail;
        state.analysis.currentTime = previousResult.currentTime;
        state.analysis.visualFailures = previousResult.visualFailures;
      }
    }
  } finally {
    if (state.analysis.selectedId === selectedId) {
      state.analysis.loadingResult = false;
      if (isAnalysisTab()) {
        render();
        if (state.analysis.timeline) {
          requestAnimationFrame(() => $("analysis-viewer-body")?.scrollIntoView({ behavior: "smooth", block: "start" }));
        }
      }
    } else if (state.analysis.selectedId === previousResult.selectedId) {
      state.analysis.loadingResult = false;
      if (isAnalysisTab()) render();
    }
    if (loadError) toast(`Analysis result is not ready: ${loadError.message}`, "error");
  }
}

function focusAnalysisResult(id) {
  const selectedId = String(id || "");
  if (selectedId && selectedId === state.analysis.selectedId && state.analysis.timeline) {
    $("analysis-viewer-body")?.scrollIntoView({ behavior: "smooth", block: "start" });
    return;
  }
  return openAnalysisResult(selectedId);
}

function reloadAnalysisResult() {
  return openAnalysisResult(state.analysis.selectedId);
}

function normalizeAnalysisTimeline(raw) {
  const timeline = raw && typeof raw === "object" ? { ...raw } : {};
  timeline.frames = analysisFrames(timeline);
  timeline.controls = normalizeTimedRecords(timeline.controls);
  timeline.comparison_controls = normalizeTimedRecords(timeline.comparison_controls);
  timeline.modes = normalizeTimedRecords(timeline.modes);
  timeline.speeds = normalizeTimedRecords(timeline.speeds);
  timeline.sections = normalizeTimedRecords(timeline.sections);
  timeline.e2e_diagnostics = normalizeTimedRecords(timeline.e2e_diagnostics);
  const e2e = timeline.e2e && typeof timeline.e2e === "object" ? timeline.e2e : {};
  timeline.e2e = {
    ...e2e,
    predictions: normalizeTimedRecords(e2e.predictions),
    recorded_applied_controls: normalizeTimedRecords(e2e.recorded_applied_controls || timeline.comparison_controls),
    latency: normalizeTimedRecords(e2e.latency || timeline.e2e_diagnostics),
    sections: Array.isArray(e2e.sections) ? e2e.sections : [],
    events: Array.isArray(e2e.events) ? e2e.events : [],
    metrics: e2e.metrics && typeof e2e.metrics === "object" ? e2e.metrics : {},
  };
  timeline.jetson_stats = normalizeJetsonStats(timeline.jetson_stats);
  const trajectory = timeline.trajectory && typeof timeline.trajectory === "object" ? timeline.trajectory : {};
  const trajectorySamples = normalizeTimedRecords(trajectory.samples || timeline.trajectory_samples);
  timeline.trajectory = {
    ...trajectory,
    frame_id: String(trajectory.frame_id || timeline.trajectory_frame_id || ""),
    source: String(trajectory.source || ""),
    samples: trajectorySamples,
    valid_samples: trajectorySamples.filter(
      (sample) => Number.isFinite(Number(sample.x)) && Number.isFinite(Number(sample.y)),
    ),
  };
  const jetsonSeriesSamples = timeline.jetson_stats.series.flatMap((series) => series.samples || []);
  const series = [timeline.frames, timeline.controls, timeline.comparison_controls, timeline.modes, timeline.speeds, trajectorySamples, jetsonSeriesSamples, timeline.e2e.predictions, timeline.e2e.latency];
  const lastTimes = series
    .map((records) => Number(records[records.length - 1]?.t))
    .filter(Number.isFinite);
  timeline._duration_s = Math.max(
    0,
    Number(timeline.duration_s) || 0,
    lastTimes.reduce((maximum, value) => Math.max(maximum, value), 0),
  );
  const speedValues = (timeline.speeds.length ? timeline.speeds : trajectorySamples)
    .map((item) => Number(item.value ?? item.speed_mps))
    .filter((value) => Number.isFinite(value) && value >= 0);
  timeline._speed_max = Math.max(
    1,
    speedValues.reduce((maximum, value) => Math.max(maximum, value), 0) * 1.08,
  );
  timeline._trajectory_speed_max = Math.max(
    1,
    trajectorySamples.reduce((maximum, sample) => {
      const value = Number(sample.speed_mps);
      return Number.isFinite(value) && value >= 0 ? Math.max(maximum, value) : maximum;
    }, 0),
  );
  timeline._trajectory_bounds = timeline.trajectory.valid_samples.reduce(
    (bounds, sample) => ({
      minX: Math.min(bounds.minX, Number(sample.x)),
      maxX: Math.max(bounds.maxX, Number(sample.x)),
      minY: Math.min(bounds.minY, Number(sample.y)),
      maxY: Math.max(bounds.maxY, Number(sample.y)),
    }),
    { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity },
  );
  timeline._normalized = true;
  if (!state.analysis.jetsonSelectedSeries?.length) {
    state.analysis.jetsonSelectedSeries = defaultJetsonSeriesSelection(timeline.jetson_stats.series);
  }
  return timeline;
}

function normalizeJetsonStats(raw) {
  const value = raw && typeof raw === "object" ? raw : {};
  const series = (Array.isArray(value.series) ? value.series : [])
    .map((item) => {
      if (!item || typeof item !== "object") return null;
      const samples = normalizeTimedRecords(item.samples)
        .map((sample) => ({ t: sample.t, value: Number(sample.value) }))
        .filter((sample) => Number.isFinite(sample.value));
      if (!samples.length) return null;
      const range = finiteNumberRange(samples, (sample) => sample.value);
      return {
        id: String(item.id || item.label || ""),
        label: String(item.label || item.id || ""),
        group: String(item.group || ""),
        unit: String(item.unit || ""),
        source: String(item.source || ""),
        key: String(item.key || ""),
        order: Number(item.order) || 99,
        min: Number.isFinite(Number(item.min)) ? Number(item.min) : range.min,
        max: Number.isFinite(Number(item.max)) ? Number(item.max) : range.max,
        samples,
      };
    })
    .filter((item) => item && item.id);
  series.sort((a, b) => (a.order - b.order) || a.label.localeCompare(b.label));
  return {
    topics: Array.isArray(value.topics) ? value.topics.map(String) : [],
    series,
  };
}

function defaultJetsonSeriesSelection(series) {
  const preferred = [
    "cpu/all/used",
    "gpu/gpu/Used",
    "mem/ram/Use",
    "temp/cpu/message",
    "temp/gpu/message",
    "power/VDD_IN/Power",
  ];
  const ids = [];
  preferred.forEach((id) => {
    if (series.some((item) => item.id === id)) ids.push(id);
  });
  if (ids.length) return ids.slice(0, 6);
  return series.slice(0, 4).map((item) => item.id);
}

function normalizeTimedRecords(raw) {
  const values = Array.isArray(raw) ? raw : [];
  return values
    .map((item) => {
      if (Array.isArray(item)) return { t: Number(item[0]), value: item[1] };
      return item && typeof item === "object" ? { ...item, t: Number(item.t ?? item.time ?? item.time_s) } : null;
    })
    .filter((item) => item && Number.isFinite(item.t))
    .sort((a, b) => a.t - b.t);
}

function finiteNumberRange(records, valueOf) {
  return (records || []).reduce((range, record) => {
    const raw = valueOf(record);
    const values = Array.isArray(raw) ? raw : [raw];
    values.forEach((value) => {
      const number = Number(value);
      if (!Number.isFinite(number)) return;
      range.min = Math.min(range.min, number);
      range.max = Math.max(range.max, number);
      range.count += 1;
    });
    return range;
  }, { min: Infinity, max: -Infinity, count: 0 });
}

function finiteNumberMaximum(records, valueOf, fallback = 0) {
  const range = finiteNumberRange(records, valueOf);
  return range.count ? Math.max(fallback, range.max) : fallback;
}

function analysisFrames(timeline = state.analysis.timeline || {}) {
  if (timeline._normalized && Array.isArray(timeline.frames)) return timeline.frames;
  const raw = timeline.frames || timeline.image_frames || timeline.media?.frames || [];
  return normalizeTimedRecords(raw)
    .map((frame) => ({ ...frame, path: String(frame.path || frame.asset || frame.file || "") }))
    .filter((frame) => frame.path);
}

function analysisTrajectory(timeline = state.analysis.timeline || {}) {
  const trajectory = timeline.trajectory && typeof timeline.trajectory === "object" ? timeline.trajectory : {};
  return {
    frameId: String(trajectory.frame_id || ""),
    source: String(trajectory.source || ""),
    localizationMethod: String(
      trajectory.localization_method || timeline.offline_localization?.method || "",
    ),
    samples: timeline._normalized && Array.isArray(trajectory.samples)
      ? trajectory.samples
      : normalizeTimedRecords(trajectory.samples),
    validSamples: timeline._normalized && Array.isArray(trajectory.valid_samples)
      ? trajectory.valid_samples
      : normalizeTimedRecords(trajectory.samples).filter(
        (sample) => Number.isFinite(Number(sample.x)) && Number.isFinite(Number(sample.y)),
      ),
  };
}

function analysisDuration(timeline = state.analysis.timeline || {}) {
  if (timeline._normalized && Number.isFinite(Number(timeline._duration_s))) {
    return Number(timeline._duration_s);
  }
  const explicit = Number(timeline.duration_s || state.analysis.detail?.duration_s || 0);
  const series = [
    analysisFrames(timeline),
    timeline.controls || [],
    timeline.modes || [],
    timeline.speeds || [],
    analysisTrajectory(timeline).samples,
  ];
  const lastValues = series
    .map((records) => Number(records[records.length - 1]?.t))
    .filter(Number.isFinite);
  return Math.max(0, explicit, lastValues.reduce((maximum, value) => Math.max(maximum, value), 0));
}

function analysisResultMapPath() {
  const timeline = state.analysis.timeline || {};
  const detail = state.analysis.detail || {};
  return String(timeline.map?.path || timeline.map_path || detail.map?.path || detail.map_path || detail.map_dir || "");
}

function analysisMapConsistency() {
  const timeline = state.analysis.timeline || {};
  const consistency = timeline.map?.consistency || timeline.map_consistency || {};
  const resultPath = analysisResultMapPath();
  const selectedPath = state.analysis.selectedMapPath;
  const mismatch = Boolean(
    (resultPath || selectedPath)
    && trimTrailingSlash(resultPath) !== trimTrailingSlash(selectedPath),
  );
  if (mismatch) {
    return {
      className: "missing",
      label: "map mismatch",
      message: `This result was generated with ${resultPath}. Re-run analysis to use ${selectedPath}; the viewer will not silently re-align it.`,
    };
  }
  if (!analysisMapFingerprintMatches()) {
    return {
      className: "missing",
      label: "map changed",
      message: "The selected Map contents changed after this analysis completed. Re-run analysis before overlaying the trajectory.",
    };
  }
  const rawStatus = String(consistency.status || consistency.state || "").toLowerCase();
  const inside = Number(consistency.inside_fraction ?? timeline.map?.inside_fraction);
  const label = Number.isFinite(inside) ? `${(inside * 100).toFixed(1)}% in map` : rawStatus || (resultPath ? "map pinned" : "no map");
  const className = ["failed", "mismatch", "invalid", "blocked"].includes(rawStatus) || (Number.isFinite(inside) && inside < 0.5)
    ? "missing"
    : ["warning", "partial", "unknown"].includes(rawStatus) || (Number.isFinite(inside) && inside < 0.9) ? "warning" : resultPath ? "ok" : "missing";
  return {
    className,
    label,
    message: String(consistency.message || (Number.isFinite(inside) ? `${(inside * 100).toFixed(1)}% of localized samples fall inside the selected map raster or lane extent.` : resultPath ? `Result is pinned to ${resultPath}.` : "No map was associated with this analysis.")),
  };
}

function updateAnalysisConsistencyDom() {
  const body = $("analysis-viewer-body");
  if (!body || !state.analysis.timeline) return;
  body.innerHTML = renderAnalysisViewer();
  mountAnalysisViewer();
}

function analysisTimelineIssues() {
  const timeline = state.analysis.timeline || {};
  const issues = [
    ...(Array.isArray(timeline.missing) ? timeline.missing : []),
    ...(Array.isArray(timeline.missing_data) ? timeline.missing_data : []),
    ...(Array.isArray(timeline.warnings) ? timeline.warnings : []),
    ...analysisRecordMissing(state.analysis.detail || {}),
  ].map(preflightText).filter(Boolean);
  if (!analysisFrames(timeline).length) issues.push("No supported image frames were extracted; telemetry playback remains available.");
  if (!(timeline.controls || []).length) issues.push("No control command samples were found.");
  if (!(timeline.modes || []).length) issues.push("No operation mode samples were found.");
  if (!(timeline.speeds || []).length && !analysisTrajectory(timeline).samples.some((sample) => Number.isFinite(Number(sample.speed_mps)))) issues.push("No measured or commanded speed samples were found.");
  if (!analysisTrajectory(timeline).samples.length && state.analysis.detail?.trajectory_mode !== "none") issues.push("No synchronized trajectory was produced. Check recorded poses or the offline localization log.");
  if (state.analysis.mapDetail && !analysisMapFingerprintMatches()) issues.push("The Map fingerprint no longer matches this result; the map overlay is suppressed.");
  else if (state.analysis.mapDetail && !analysisTrajectoryCanUseMap()) issues.push(`Trajectory frame '${analysisTrajectory(timeline).frameId}' has no verified transform to map; the map overlay is suppressed.`);
  if (timeline.map_load_error) issues.push(`Map background could not be loaded: ${timeline.map_load_error}`);
  return [...new Set(issues)];
}

function formatAnalysisClock(seconds) {
  const value = Math.max(0, Number(seconds) || 0);
  const hours = Math.floor(value / 3600);
  const minutes = Math.floor((value % 3600) / 60);
  const secs = value % 60;
  return `${hours ? `${String(hours).padStart(2, "0")}:` : ""}${String(minutes).padStart(2, "0")}:${secs.toFixed(2).padStart(5, "0")}`;
}

function stopAnalysisAnimationFrame() {
  if (state.analysis.rafId) cancelAnimationFrame(state.analysis.rafId);
  state.analysis.rafId = 0;
  state.analysis.lastTickMs = 0;
}

function mountAnalysisViewer() {
  if (!state.analysis.timeline || !$("analysis-viewer-body")) return;
  resetAnalysisFrameRenderState();
  updateAnalysisPlaybackDom(true);
  if (state.analysis.playing) startAnalysisAnimationFrame();
}

function startAnalysisAnimationFrame() {
  stopAnalysisAnimationFrame();
  if (!state.analysis.playing || !isAnalysisTab()) return;
  state.analysis.rafId = requestAnimationFrame(analysisPlaybackTick);
}

function analysisPlaybackTick(now) {
  if (!state.analysis.playing || !isAnalysisTab()) {
    stopAnalysisAnimationFrame();
    return;
  }
  if (!state.analysis.lastTickMs) state.analysis.lastTickMs = now;
  const elapsed = Math.max(0, Math.min(0.25, (now - state.analysis.lastTickMs) / 1000));
  state.analysis.lastTickMs = now;
  const duration = analysisDuration();
  state.analysis.currentTime = Math.min(duration, state.analysis.currentTime + elapsed * state.analysis.playbackRate);
  if (state.analysis.currentTime >= duration) state.analysis.playing = false;
  try {
    updateAnalysisPlaybackDom();
  } catch (error) {
    handleAnalysisPlaybackError(error);
  } finally {
    if (state.analysis.playing) state.analysis.rafId = requestAnimationFrame(analysisPlaybackTick);
    else stopAnalysisAnimationFrame();
  }
}

function toggleAnalysisPlayback() {
  if (!state.analysis.timeline) return;
  if (state.analysis.playing) {
    pauseAnalysisPlayback();
    return;
  }
  const duration = analysisDuration();
  if (!(duration > 0)) {
    toast("This analysis has no playable timeline.", "error");
    return;
  }
  if (state.analysis.currentTime >= duration) state.analysis.currentTime = 0;
  state.analysis.playing = true;
  state.analysis.lastTickMs = 0;
  try {
    updateAnalysisPlaybackDom(true);
    startAnalysisAnimationFrame();
  } catch (error) {
    handleAnalysisPlaybackError(error);
  }
}

function handleAnalysisPlaybackError(error) {
  state.analysis.playing = false;
  stopAnalysisAnimationFrame();
  const playButton = $("analysis-play-button");
  if (playButton) playButton.textContent = "Play";
  const message = error instanceof Error ? error.message : String(error || "unknown error");
  console.error("Analysis playback stopped", error);
  toast(`Playback stopped: ${message}`, "error");
}

function pauseAnalysisPlayback() {
  state.analysis.playing = false;
  stopAnalysisAnimationFrame();
  updateAnalysisPlaybackDom(true);
}

function seekAnalysisTime(value) {
  const duration = analysisDuration();
  state.analysis.currentTime = Math.max(0, Math.min(duration, Number(value) || 0));
  state.analysis.lastTickMs = 0;
  updateAnalysisPlaybackDom(true);
}

function seekAnalysisRelative(delta) {
  seekAnalysisTime(state.analysis.currentTime + Number(delta || 0));
}

function setAnalysisPlaybackRate(value) {
  const rate = Number(value);
  state.analysis.playbackRate = Number.isFinite(rate) ? Math.max(0.1, Math.min(8, rate)) : 1;
}

function timedRecordIndex(records, time) {
  let low = 0;
  let high = records.length - 1;
  let found = -1;
  while (low <= high) {
    const middle = (low + high) >> 1;
    if (Number(records[middle].t) <= time + 1.0e-9) {
      found = middle;
      low = middle + 1;
    } else {
      high = middle - 1;
    }
  }
  return found;
}

function timedRecordAt(records, time) {
  const index = timedRecordIndex(records || [], time);
  return index >= 0 ? records[index] : null;
}

function stepAnalysisFrame(direction) {
  const frames = analysisFrames();
  if (!frames.length) {
    seekAnalysisRelative(Number(direction) / Math.max(1, state.analysis.maxFps));
    return;
  }
  const current = timedRecordIndex(frames, state.analysis.currentTime);
  const index = Math.max(0, Math.min(frames.length - 1, current + (Number(direction) >= 0 ? 1 : -1)));
  pauseAnalysisPlayback();
  seekAnalysisTime(frames[index].t);
}

function analysisImageChannels() {
  const detailTopics = state.analysis.detail?.topics || {};
  const manifestImageTopics = Array.isArray(detailTopics.image_topics) ? detailTopics.image_topics : [];
  const channelSet = new Set(manifestImageTopics);
  const frames = analysisFrames();
  const sampleCount = Math.min(frames.length, 100);
  for (let i = 0; i < sampleCount; i += 1) {
    if (frames[i]?.channels) {
      Object.keys(frames[i].channels).forEach((k) => channelSet.add(k));
    }
  }
  if (detailTopics.image) channelSet.add(detailTopics.image);
  if (detailTopics.primary_image_topic) channelSet.add(detailTopics.primary_image_topic);

  return [...channelSet].filter(Boolean);
}

function analysisPrimaryImageTopic(channels = analysisImageChannels()) {
  const detailTopics = state.analysis.detail?.topics || {};
  return detailTopics.primary_image_topic || detailTopics.image || channels[0] || "";
}

function renderAnalysisCameraSelector() {
  const channels = analysisImageChannels();
  if (!channels.length) return "";

  const primaryTopic = analysisPrimaryImageTopic(channels);
  const currentChannel = state.analysis.selectedChannel || primaryTopic || channels[0];
  const viewMode = channels.length === 1 ? "single" : (state.analysis.imageViewMode || "single");

  return `
    <div class="analysis-camera-selector">
      <div class="analysis-camera-mode" role="group" aria-label="Image layout">
        ${[
          ["single", "Single"],
          ["strip", "Primary"],
          ["grid", "Grid"],
        ].map(([mode, label]) => `
          <button
            class="${viewMode === mode ? "active" : ""}"
            ${channels.length === 1 && mode !== "single" ? "disabled" : ""}
            onclick="setAnalysisImageViewMode(${js(mode)})"
          >${esc(label)}</button>
        `).join("")}
      </div>
      <label>
        <span>${viewMode === "single" ? "Channel" : "Primary"}</span>
        <select onchange="selectAnalysisCameraChannel(this.value)">
          ${channels.map((ch) => `<option value="${esc(ch)}" ${ch === currentChannel ? "selected" : ""}>${esc(ch)}${ch === primaryTopic ? " (clock)" : ""}</option>`).join("")}
        </select>
      </label>
    </div>
  `;
}

function setAnalysisImageViewMode(mode) {
  const normalized = ["single", "strip", "grid"].includes(mode) ? mode : "single";
  state.analysis.imageViewMode = normalized;
  resetAnalysisFrameRenderState();
  const viewer = $("analysis-viewer-body");
  if (viewer) {
    viewer.innerHTML = renderAnalysisViewer();
    mountAnalysisViewer();
  }
}

function selectAnalysisCameraChannel(channel) {
  state.analysis.selectedChannel = channel;
  resetAnalysisFrameRenderState();
  const viewer = $("analysis-viewer-body");
  if (viewer) {
    viewer.innerHTML = renderAnalysisViewer();
    mountAnalysisViewer();
  } else {
    updateAnalysisFrame(state.analysis.currentTime);
  }
}

function analysisAssetUrl(frame, channelTopic = null) {
  let path = String(frame?.path || "");
  if (channelTopic && frame?.channels && frame.channels[channelTopic]?.path) {
    path = String(frame.channels[channelTopic].path);
  }
  if (!path) return "";
  const relative = path.replace(/^\/+/, "").replace(/^frames\//, "");
  return `/api/analyses/${encodeURIComponent(state.analysis.selectedId)}/frames/${relative.split("/").map(encodeURIComponent).join("/")}`;
}

function resetAnalysisFrameRenderState() {
  state.analysis.renderedFrameIndex = -1;
  state.analysis.renderedFrameKey = "";
  state.analysis.renderedChannel = "";
  state.analysis.activeBufferImg = "";
  state.analysis.pendingFrameKey = "";
  state.analysis.frameRequestSerial += 1;
  state.analysis.channelRender = {};
}

function visibleAnalysisImage(imgA, imgB) {
  if (imgA?.classList.contains("visible")) return imgA;
  if (imgB?.classList.contains("visible")) return imgB;
  return null;
}

function analysisChannelPayloadAt(frames, index, channel, primaryTopic) {
  for (let cursor = index; cursor >= 0; cursor -= 1) {
    const frame = frames[cursor];
    if (!frame) continue;
    const channelPayload = frame.channels?.[channel];
    if (channelPayload?.path) {
      return {
        ...channelPayload,
        frameIndex: cursor,
        frameTime: Number(frame.t || 0),
        path: channelPayload.path,
        stale: cursor !== index,
      };
    }
    if (channel === primaryTopic && frame.path) {
      return {
        path: frame.path,
        width: frame.width,
        height: frame.height,
        delta_ms: 0,
        frameIndex: cursor,
        frameTime: Number(frame.t || 0),
        stale: cursor !== index,
      };
    }
  }
  return null;
}

function renderAnalysisMultiTiles(grid, channels, selectedChannel, primaryTopic, mode) {
  const ordered = [
    selectedChannel,
    ...channels.filter((channel) => channel !== selectedChannel),
  ].filter(Boolean);
  const layoutKey = `${mode}|${ordered.join("\n")}`;
  grid.className = `analysis-multi-image-grid visible ${mode === "strip" ? "strip" : "grid"}`;
  if (grid.dataset.layoutKey === layoutKey) return ordered;
  grid.dataset.layoutKey = layoutKey;
  grid.innerHTML = ordered.map((channel) => `
    <div class="analysis-channel-tile ${channel === selectedChannel ? "primary" : ""}" data-channel="${esc(channel)}">
      <div class="analysis-channel-images">
        <img data-buffer="a" alt="${esc(channel)} frame A" decoding="async" />
        <img data-buffer="b" alt="${esc(channel)} frame B" decoding="async" />
        <div class="analysis-channel-empty visible">Waiting for frame...</div>
      </div>
      <div class="analysis-channel-caption">
        <strong>${esc(channel)}</strong>
        <span>${channel === primaryTopic ? "clock" : ""}</span>
      </div>
    </div>
  `).join("");
  return ordered;
}

function updateAnalysisMultiFrame(frames, index, time, channels, selectedChannel, primaryTopic) {
  const grid = $("analysis-multi-image-grid");
  const imgA = $("analysis-frame-image-a");
  const imgB = $("analysis-frame-image-b");
  const empty = $("analysis-frame-empty");
  if (!grid || !empty) return;

  if (imgA) imgA.classList.remove("visible");
  if (imgB) imgB.classList.remove("visible");
  empty.classList.remove("visible");
  grid.classList.add("visible");

  const mode = state.analysis.imageViewMode === "grid" ? "grid" : "strip";
  const visibleChannels = renderAnalysisMultiTiles(
    grid,
    channels,
    selectedChannel || primaryTopic || channels[0],
    primaryTopic,
    mode,
  );

  visibleChannels.forEach((channel) => {
    const tile = [...grid.querySelectorAll(".analysis-channel-tile")]
      .find((element) => element.dataset.channel === channel);
    if (!tile) return;
    const payload = analysisChannelPayloadAt(frames, index, channel, primaryTopic);
    const emptyTile = tile.querySelector(".analysis-channel-empty");
    const captionMeta = tile.querySelector(".analysis-channel-caption span");
    const activeImg = tile.querySelector("img.visible");
    const targetImg = activeImg?.dataset.buffer === "b"
      ? tile.querySelector('img[data-buffer="a"]')
      : tile.querySelector('img[data-buffer="b"]');

    if (!payload) {
      if (!activeImg && emptyTile) emptyTile.classList.add("visible");
      if (captionMeta) captionMeta.textContent = channel === primaryTopic ? "clock" : "waiting";
      return;
    }

    const url = analysisAssetUrl({ path: payload.path });
    const renderState = state.analysis.channelRender[channel] || { serial: 0, key: "", pendingKey: "" };
    const key = `${payload.frameIndex}|${channel}|${url}`;
    if (captionMeta) {
      const age = Math.max(0, time - Number(payload.frameTime || 0));
      const delta = Number(payload.delta_ms || 0);
      const parts = [];
      if (channel === primaryTopic) parts.push("clock");
      if (payload.stale && age > 0.001) parts.push(`hold ${age.toFixed(2)}s`);
      else if (delta) parts.push(`${delta >= 0 ? "+" : ""}${delta}ms`);
      captionMeta.textContent = parts.join(" / ");
    }
    if (!targetImg || key === renderState.key || renderState.pendingKey) {
      state.analysis.channelRender[channel] = renderState;
      return;
    }

    renderState.serial = Number(renderState.serial || 0) + 1;
    renderState.pendingKey = key;
    state.analysis.channelRender[channel] = renderState;
    const requestSerial = renderState.serial;
    const loader = new Image();
    loader.decoding = "async";
    loader.onload = async () => {
      try {
        if (typeof loader.decode === "function") await loader.decode();
      } catch {
        // onload is enough on browsers that do not expose decode consistently.
      }
      const latest = state.analysis.channelRender[channel];
      if (!latest || latest.serial !== requestSerial || latest.pendingKey !== key) return;
      targetImg.src = url;
      targetImg.classList.add("visible");
      if (activeImg && activeImg !== targetImg) activeImg.classList.remove("visible");
      if (emptyTile) emptyTile.classList.remove("visible");
      latest.key = key;
      latest.pendingKey = "";
      latest.active = targetImg.dataset.buffer || "a";
    };
    loader.onerror = () => {
      const latest = state.analysis.channelRender[channel];
      if (!latest || latest.serial !== requestSerial || latest.pendingKey !== key) return;
      latest.pendingKey = "";
      if (!tile.querySelector("img.visible") && emptyTile) {
        emptyTile.textContent = "Frame unavailable";
        emptyTile.classList.add("visible");
      }
    };
    loader.src = url;
  });

  state.analysis.renderedFrameIndex = index;
  state.analysis.renderedFrameKey = `multi|${state.analysis.imageViewMode}|${index}`;
  state.analysis.renderedChannel = selectedChannel || "";

  const frameTime = $("analysis-frame-time");
  if (frameTime) {
    frameTime.textContent = `${formatAnalysisClock(time)} / frame ${index + 1} of ${frames.length}`;
  }
}

function updateAnalysisFrame(time, force = false) {
  const frames = analysisFrames();
  const imgA = $("analysis-frame-image-a");
  const imgB = $("analysis-frame-image-b");
  const empty = $("analysis-frame-empty");
  const grid = $("analysis-multi-image-grid");
  if (!empty) return;

  const index = timedRecordIndex(frames, time);
  if (index < 0) {
    if (imgA) imgA.classList.remove("visible");
    if (imgB) imgB.classList.remove("visible");
    if (grid) grid.classList.remove("visible");
    empty.textContent = frames.length ? "Waiting for the first image frame..." : "No image frames were extracted.";
    empty.classList.add("visible");
    state.analysis.renderedFrameIndex = -1;
    state.analysis.renderedFrameKey = "";
    state.analysis.renderedChannel = "";
    state.analysis.pendingFrameKey = "";
    return;
  }

  const detailTopics = state.analysis.detail?.topics || {};
  const channels = analysisImageChannels();
  const primaryTopic = analysisPrimaryImageTopic(channels);
  const firstFrameChannels = frames[0]?.channels ? Object.keys(frames[0].channels) : [];
  const selectedChannel = state.analysis.selectedChannel || primaryTopic || firstFrameChannels[0] || null;
  const viewMode = channels.length > 1 ? (state.analysis.imageViewMode || "single") : "single";

  if (viewMode !== "single") {
    updateAnalysisMultiFrame(frames, index, time, channels, selectedChannel, primaryTopic);
    return;
  }

  if (grid) grid.classList.remove("visible");

  const currentFrame = frames[index];
  const hasChannel = selectedChannel && (
    selectedChannel === primaryTopic
    || (currentFrame?.channels && Boolean(currentFrame.channels[selectedChannel]))
  );

  if (selectedChannel && !hasChannel) {
    // Keep current image visible to prevent blackout; update frame timestamp info
    state.analysis.renderedFrameIndex = index;
    state.analysis.renderedFrameKey = `${index}|${selectedChannel}|missing`;
    state.analysis.renderedChannel = selectedChannel;
    const frameTime = $("analysis-frame-time");
    if (frameTime) {
      frameTime.textContent = `${formatAnalysisClock(time)} / frame ${index + 1} of ${frames.length} (no new frame)`;
    }
    return;
  }

  const url = analysisAssetUrl(currentFrame, selectedChannel);
  const frameKey = `${index}|${selectedChannel || ""}|${url}`;
  if (state.analysis.pendingFrameKey && !force) {
    const frameTime = $("analysis-frame-time");
    if (frameTime) {
      frameTime.textContent = `${formatAnalysisClock(time)} / frame ${index + 1} of ${frames.length} (loading)`;
    }
    return;
  }
  if (
    url
    && frameKey !== state.analysis.renderedFrameKey
    && frameKey !== state.analysis.pendingFrameKey
  ) {
    const requestSerial = (state.analysis.frameRequestSerial || 0) + 1;
    state.analysis.frameRequestSerial = requestSerial;
    state.analysis.pendingFrameKey = frameKey;

    const activeImg = visibleAnalysisImage(imgA, imgB);
    const targetImg = activeImg === imgB ? imgA : imgB;

    if (targetImg) {
      const loader = new Image();
      loader.decoding = "async";
      loader.onload = async () => {
        try {
          if (typeof loader.decode === "function") await loader.decode();
        } catch {
          // Some browsers resolve onload after enough decode work; keep the loaded image.
        }
        if (requestSerial !== state.analysis.frameRequestSerial) return;
        if (frameKey !== state.analysis.pendingFrameKey) return;
        targetImg.onload = null;
        targetImg.onerror = null;
        targetImg.src = url;
        targetImg.classList.add("visible");
        if (activeImg && activeImg !== targetImg) {
          activeImg.classList.remove("visible");
        }
        empty.classList.remove("visible");
        state.analysis.activeBufferImg = targetImg === imgB ? "b" : "a";
        state.analysis.renderedFrameIndex = index;
        state.analysis.renderedFrameKey = frameKey;
        state.analysis.renderedChannel = selectedChannel;
        state.analysis.pendingFrameKey = "";

        const stage = targetImg.closest(".analysis-image-stage");
        if (stage) {
          stage.dataset.frameSize = `${loader.naturalWidth}×${loader.naturalHeight}`;
        }
      };
      loader.onerror = () => {
        if (requestSerial !== state.analysis.frameRequestSerial) return;
        if (frameKey !== state.analysis.pendingFrameKey) return;
        state.analysis.pendingFrameKey = "";
        if (!visibleAnalysisImage(imgA, imgB)) {
          empty.textContent = "This frame could not be loaded.";
          empty.classList.add("visible");
        }
      };
      loader.src = url;
    }

    // Preload next & previous 4 frames for zero-latency frame-by-frame stepping
    for (let offset = -2; offset <= 5; offset++) {
      if (offset === 0) continue;
      const targetIdx = index + offset;
      if (targetIdx >= 0 && targetIdx < frames.length) {
        const preloadUrl = analysisAssetUrl(frames[targetIdx], selectedChannel);
        if (preloadUrl) {
          const pImg = new Image();
          pImg.src = preloadUrl;
        }
      }
    }
  }

  const channelInfo = selectedChannel && currentFrame?.channels?.[selectedChannel];
  const deltaStr = channelInfo && Number.isFinite(channelInfo.delta_ms) && channelInfo.delta_ms !== 0
    ? ` (${channelInfo.delta_ms >= 0 ? "+" : ""}${channelInfo.delta_ms}ms)`
    : "";

  const frameTime = $("analysis-frame-time");
  if (frameTime) {
    frameTime.textContent = `${formatAnalysisClock(time)} / frame ${index + 1} of ${frames.length}${deltaStr}`;
  }
}

function analysisModeLabel(record) {
  if (!record) return "-";
  if (record.label) return String(record.label).toUpperCase();
  const value = Number(record.mode ?? record.value);
  return { 1: "AUTO", 2: "MANUAL", 3: "STOP", 4: "PROPO" }[value] || String(record.mode ?? record.value ?? "-");
}

function formatAnalysisValue(value, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(3)}${suffix}` : "-";
}

function updateAnalysisPlaybackDom(force = false) {
  if (!state.analysis.timeline || !isAnalysisTab()) return;
  const time = state.analysis.currentTime;
  const duration = analysisDuration();
  const seek = $("analysis-seek");
  if (seek) {
    seek.max = String(duration);
    seek.value = String(Math.min(duration, time));
  }
  const clock = $("analysis-clock-label");
  if (clock) clock.textContent = `${formatAnalysisClock(time)} / ${formatAnalysisClock(duration)}`;
  const playButton = $("analysis-play-button");
  if (playButton) playButton.textContent = state.analysis.playing ? "Pause" : "Play";
  updateAnalysisFrame(time, force);

  const timeline = state.analysis.timeline;
  const control = timedRecordAt(timeline.controls || [], time);
  const mode = timedRecordAt(timeline.modes || [], time);
  const trajectory = timedRecordAt(analysisTrajectory(timeline).samples, time);
  const speed = timedRecordAt(timeline.speeds || [], time);
  const e2ePrediction = timedRecordAt(timeline.e2e?.predictions || [], time);
  const e2eLatency = timedRecordAt(timeline.e2e?.latency || [], time);
  const recordedSection = timedRecordAt(timeline.sections || [], time);
  const speedLabel = $("analysis-label-speed");
  if (speedLabel) speedLabel.textContent = analysisSpeedSourceLabel(speed);
  const values = {
    "analysis-value-mode": analysisModeLabel(mode),
    "analysis-value-steering": formatAnalysisValue(control?.steering),
    "analysis-value-throttle": formatAnalysisValue(control?.throttle),
    "analysis-value-brake": formatAnalysisValue(control?.brake),
    "analysis-value-speed": formatAnalysisValue(
      speed?.value ?? speed?.speed_mps ?? trajectory?.speed_mps,
      analysisSpeedIsCommanded(speed) ? "" : " m/s",
    ),
    "analysis-value-e2e-steering": formatAnalysisValue(e2ePrediction?.steering_pred ?? e2ePrediction?.steering),
    "analysis-value-e2e-steering-error": formatAnalysisValue(e2ePrediction?.steering_error ?? e2ePrediction?.steering_applied_error),
    "analysis-value-e2e-latency": formatAnalysisValue(
      e2eLatency?.capture_to_command_ms ?? e2ePrediction?.pipeline_latency_ms ?? e2ePrediction?.total_ms,
      " ms",
    ),
    "analysis-value-e2e-aggression": formatAnalysisValue(e2ePrediction?.aggressiveness_score, " / 100"),
    "analysis-value-e2e-latacc": formatAnalysisValue(
      e2ePrediction?.lateral_accel_mps2 ?? trajectory?.lateral_accel_mps2,
      " m/s²",
    ),
    "analysis-value-e2e-section": String(e2ePrediction?.section || trajectory?.section || recordedSection?.section || "-"),
    "analysis-value-e2e-trajectory-ade": formatAnalysisValue(e2ePrediction?.trajectory_ade_m, " m"),
    "analysis-value-e2e-trajectory-fde": formatAnalysisValue(e2ePrediction?.trajectory_fde_m, " m"),
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = $(id);
    if (element) element.textContent = value;
  });

  const now = performance.now();
  if (force || now - state.analysis.lastVisualUpdateMs >= 32) {
    drawAnalysisVisual("timeline", drawAnalysisTimeline);
    drawAnalysisVisual("Jetson chart", drawJetsonStatsChart);
    drawAnalysisVisual("map", drawAnalysisMap);
    drawAnalysisVisual("E2E trajectory", drawE2ETrajectory);
    state.analysis.lastVisualUpdateMs = now;
  }
}

function seekAnalysisFromTimeline(event) {
  const canvas = event.currentTarget || $("analysis-timeline-canvas");
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left) / Math.max(1, rect.width)));
  seekAnalysisTime(fraction * analysisDuration());
}

function analysisCanvasContext(canvas, fallbackHeight) {
  if (!canvas) return null;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, Math.round(rect.width || canvas.parentElement?.clientWidth || 800));
  const height = Math.max(180, Math.round(rect.height || fallbackHeight));
  const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  const pixelWidth = Math.round(width * dpr);
  const pixelHeight = Math.round(height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) return null;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width, height };
}

function drawAnalysisVisual(name, draw) {
  state.analysis.visualFailures = state.analysis.visualFailures || {};
  if (state.analysis.visualFailures[name]) return;
  try {
    draw();
  } catch (error) {
    state.analysis.visualFailures[name] = true;
    console.error(`Analysis ${name} rendering failed`, error);
    const message = error instanceof Error ? error.message : String(error || "unknown error");
    toast(`${name} rendering disabled for this result: ${message}`, "error");
  }
}

function drawAnalysisTimeline() {
  const canvas = $("analysis-timeline-canvas");
  const timeline = state.analysis.timeline;
  if (!canvas || !timeline) return;
  const e2e = timeline.e2e?.mode ? timeline.e2e : null;
  const predictions = e2e?.predictions || [];
  const recordedAppliedControls = e2e?.recorded_applied_controls || [];
  const errorMax = finiteNumberMaximum(predictions, (item) => [
    Math.abs(Number(item.steering_error ?? item.steering_applied_error)),
    Math.abs(Number(item.throttle_error ?? item.throttle_applied_error)),
  ], 0.1) * 1.1;
  const latencyRecords = (e2e?.latency || []).length ? e2e.latency : predictions;
  const latencyMax = finiteNumberMaximum(latencyRecords, (item) => item.capture_to_command_ms ?? item.total_ms ?? item.inference_ms, 1) * 1.08;
  const dynamicsRecords = analysisTrajectory(timeline).samples || [];
  const dynamicsMax = finiteNumberMaximum(dynamicsRecords, (item) => [
    Math.abs(Number(item.longitudinal_accel_mps2)),
    Math.abs(Number(item.lateral_accel_mps2)),
  ], 1) * 1.08;
  const jerkMax = finiteNumberMaximum(dynamicsRecords, (item) => Math.abs(Number(item.jerk_mps3)), 1) * 1.08;
  const trajectoryTracks = [
    {
      label: "TRAJECTORY ADE m",
      min: 0,
      max: finiteNumberMaximum(predictions, (item) => item.trajectory_ade_m, 0.1) * 1.1,
      lines: [{ records: predictions, value: (item) => item.trajectory_ade_m, color: "#5aa8ff" }],
    },
    {
      label: "TRAJECTORY FDE m",
      min: 0,
      max: finiteNumberMaximum(predictions, (item) => item.trajectory_fde_m, 0.1) * 1.1,
      lines: [{ records: predictions, value: (item) => item.trajectory_fde_m, color: "#f0b35a" }],
    },
    {
      label: "LATENCY ms",
      min: 0,
      max: latencyMax,
      lines: [{ records: latencyRecords, value: (item) => item.capture_to_command_ms ?? item.total_ms ?? item.inference_ms, color: "#b28dff" }],
    },
    {
      label: analysisSpeedSourceLabel(analysisSpeedSeries()[0], true),
      min: 0,
      max: analysisSpeedMaximum(),
      lines: [{ records: analysisSpeedSeries(), value: (item) => item.value ?? item.speed_mps, color: "#e7b84b" }],
    },
  ];
  const controlTracks = [
    {
      label: "STEER GT/PRED",
      min: -1,
      max: 1,
      lines: [
        { records: predictions, value: (item) => item.steering_gt, color: "#d8dee9" },
        { records: predictions, value: (item) => item.steering_pred ?? item.steering, color: "#5aa8ff" },
        { records: recordedAppliedControls, value: (item) => item.steering, color: "#bd93f9" },
      ],
    },
    {
      label: "THROTTLE GT/PRED",
      min: 0,
      max: 1,
      lines: [
        { records: predictions, value: (item) => item.throttle_gt, color: "#d8dee9" },
        { records: predictions, value: (item) => item.throttle_pred ?? item.throttle, color: "#45c478" },
        { records: recordedAppliedControls, value: (item) => item.throttle, color: "#bd93f9" },
      ],
    },
    {
      label: "CONTROL ERROR",
      min: -errorMax,
      max: errorMax,
      lines: [
        { records: predictions, value: (item) => item.steering_error ?? item.steering_applied_error, color: "#ff6b6b" },
        { records: predictions, value: (item) => item.throttle_error ?? item.throttle_applied_error, color: "#f0b35a" },
      ],
    },
    {
      label: "AGGRESSIVENESS",
      min: 0,
      max: 100,
      lines: [{ records: predictions, value: (item) => item.aggressiveness_score, color: "#ff8c42" }],
    },
    {
      label: "ACCEL m/s²",
      min: -dynamicsMax,
      max: dynamicsMax,
      lines: [
        { records: dynamicsRecords, value: (item) => item.longitudinal_accel_mps2, color: "#4fc3c7" },
        { records: dynamicsRecords, value: (item) => item.lateral_accel_mps2, color: "#ff6b9d" },
      ],
    },
    {
      label: "JERK m/s³",
      min: -jerkMax,
      max: jerkMax,
      lines: [{ records: dynamicsRecords, value: (item) => item.jerk_mps3, color: "#f6c85f" }],
    },
    {
      label: "LATENCY ms",
      min: 0,
      max: latencyMax,
      lines: [{ records: latencyRecords, value: (item) => item.capture_to_command_ms ?? item.total_ms ?? item.inference_ms, color: "#b28dff" }],
    },
    {
      label: analysisSpeedSourceLabel(analysisSpeedSeries()[0], true),
      min: 0,
      max: analysisSpeedMaximum(),
      lines: [{ records: analysisSpeedSeries(), value: (item) => item.value ?? item.speed_mps, color: "#e7b84b" }],
    },
  ];
  const tracks = e2e
    ? (e2e.task === "trajectory" ? trajectoryTracks : controlTracks)
    : [
    { label: "STEER", min: -1, max: 1, lines: [{ records: timeline.controls || [], value: (item) => item.steering, color: "#5aa8ff" }] },
    { label: "PEDALS", min: 0, max: 1, lines: [{ records: timeline.controls || [], value: (item) => item.throttle, color: "#45c478" }, { records: timeline.controls || [], value: (item) => item.brake, color: "#f26d6d" }] },
    { label: analysisSpeedSourceLabel(analysisSpeedSeries()[0], true), min: 0, max: analysisSpeedMaximum(), lines: [{ records: analysisSpeedSeries(), value: (item) => item.value ?? item.speed_mps, color: "#e7b84b" }] },
  ];
  canvas.style.height = `${Math.max(310, 95 + tracks.length * 68)}px`;
  const prepared = analysisCanvasContext(canvas, Math.max(310, 95 + tracks.length * 68));
  if (!prepared) return;
  const { ctx, width, height } = prepared;
  const duration = Math.max(0.001, analysisDuration(timeline));
  const left = 70;
  const right = 14;
  const top = 14;
  const modeHeight = 28;
  const chartTop = top + modeHeight + 12;
  const chartHeight = Math.max(48, (height - chartTop - 20) / tracks.length);
  const plotWidth = Math.max(1, width - left - right);
  const toX = (time) => left + Math.max(0, Math.min(1, Number(time) / duration)) * plotWidth;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0a0d10";
  ctx.fillRect(0, 0, width, height);
  ctx.font = "11px ui-sans-serif, system-ui";
  ctx.textBaseline = "middle";

  const gridCount = Math.max(4, Math.min(12, Math.floor(plotWidth / 120)));
  for (let index = 0; index <= gridCount; index += 1) {
    const fraction = index / gridCount;
    const x = left + fraction * plotWidth;
    ctx.strokeStyle = "rgba(255,255,255,0.07)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, top);
    ctx.lineTo(x, height - 18);
    ctx.stroke();
    ctx.fillStyle = "#78838e";
    ctx.textAlign = index === 0 ? "left" : index === gridCount ? "right" : "center";
    ctx.fillText(formatAnalysisClock(fraction * duration), x, height - 8);
  }

  ctx.fillStyle = "#11171c";
  ctx.fillRect(left, top, plotWidth, modeHeight);
  ctx.fillStyle = "#98a2ad";
  ctx.textAlign = "right";
  ctx.fillText("MODE", left - 8, top + modeHeight / 2);
  const modes = timeline.modes || [];
  const modeColors = { AUTO: "#2f8f61", MANUAL: "#386fa4", STOP: "#8b3c43", PROPO: "#7a5c9b" };
  modes.forEach((record, index) => {
    const next = modes[index + 1];
    const x = toX(record.t);
    const endX = toX(next ? next.t : duration);
    const label = analysisModeLabel(record);
    ctx.fillStyle = modeColors[label] || "#555f69";
    ctx.fillRect(x, top, Math.max(1, endX - x), modeHeight);
    if (endX - x > 45) {
      ctx.fillStyle = "rgba(255,255,255,0.85)";
      ctx.textAlign = "left";
      ctx.fillText(label, x + 5, top + modeHeight / 2);
    }
  });

  tracks.forEach((track, trackIndex) => {
    const y = chartTop + trackIndex * chartHeight;
    const innerTop = y + 6;
    const innerHeight = chartHeight - 12;
    ctx.fillStyle = trackIndex % 2 ? "#0d1115" : "#0f1418";
    ctx.fillRect(left, y, plotWidth, chartHeight - 2);
    ctx.strokeStyle = "rgba(255,255,255,0.08)";
    ctx.beginPath();
    ctx.moveTo(left, y + chartHeight / 2);
    ctx.lineTo(width - right, y + chartHeight / 2);
    ctx.stroke();
    ctx.fillStyle = "#98a2ad";
    ctx.textAlign = "right";
    ctx.fillText(track.label, left - 8, y + chartHeight / 2);
    track.lines.forEach((line) => drawAnalysisSeries(ctx, line.records, line.value, line.color, toX, innerTop, innerHeight, track.min, track.max, plotWidth));
  });

  const cursorX = toX(state.analysis.currentTime);
  ctx.strokeStyle = "rgba(255,255,255,0.92)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(cursorX, top - 3);
  ctx.lineTo(cursorX, height - 18);
  ctx.stroke();
  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.moveTo(cursorX - 5, top - 4);
  ctx.lineTo(cursorX + 5, top - 4);
  ctx.lineTo(cursorX, top + 3);
  ctx.closePath();
  ctx.fill();
}

function drawAnalysisSeries(ctx, records, valueOf, color, toX, top, height, min, max, plotWidth) {
  if (!records?.length || max <= min) return;
  const maxPoints = Math.max(200, Math.floor(plotWidth * 2));
  const step = Math.max(1, Math.ceil(records.length / maxPoints));
  let started = false;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.7;
  ctx.lineJoin = "round";
  ctx.beginPath();
  for (let index = 0; index < records.length; index += step) {
    const record = records[index];
    const value = Number(valueOf(record));
    if (!Number.isFinite(value)) {
      started = false;
      continue;
    }
    const x = toX(record.t);
    const y = top + (1 - Math.max(0, Math.min(1, (value - min) / (max - min)))) * height;
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  }
  const last = records[records.length - 1];
  const lastValue = Number(valueOf(last));
  if (Number.isFinite(lastValue)) {
    const x = toX(last.t);
    const y = top + (1 - Math.max(0, Math.min(1, (lastValue - min) / (max - min)))) * height;
    if (!started) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();
}

function analysisSpeedSeries() {
  const direct = state.analysis.timeline?.speeds || [];
  return direct.length ? direct : analysisTrajectory().samples;
}

function analysisSpeedSourceLabel(record, compact = false) {
  const commanded = analysisSpeedIsCommanded(record);
  if (compact) return commanded ? "CMD RAW" : "SPEED";
  return commanded ? "Commanded speed (raw)" : "Vehicle speed";
}

function analysisSpeedIsCommanded(record) {
  const source = String(record?.source || state.analysis.timeline?.trajectory?.source || "");
  return source.startsWith("/commands/") || /motor\/speed/i.test(source) || /commanded/i.test(source);
}

function analysisSpeedMaximum() {
  const cached = Number(state.analysis.timeline?._speed_max);
  if (Number.isFinite(cached) && cached > 0) return cached;
  const values = analysisSpeedSeries().map((item) => Number(item.value ?? item.speed_mps)).filter((value) => Number.isFinite(value) && value >= 0);
  if (!values.length) return 1;
  return Math.max(1, values.reduce((maximum, value) => Math.max(maximum, value), 0) * 1.08);
}

function analysisJetsonSeries() {
  return state.analysis.timeline?.jetson_stats?.series || [];
}

function analysisSelectedJetsonSeries() {
  const series = analysisJetsonSeries();
  if (!series.length) return [];
  const selectedIds = Array.isArray(state.analysis.jetsonSelectedSeries)
    ? state.analysis.jetsonSelectedSeries
    : [];
  if (selectedIds.includes("__none__")) return [];
  const selected = series.filter((item) => selectedIds.includes(item.id));
  return selected.length ? selected : series.filter((item) => defaultJetsonSeriesSelection(series).includes(item.id));
}

function toggleJetsonStatsSeries(id) {
  const current = Array.isArray(state.analysis.jetsonSelectedSeries)
    ? state.analysis.jetsonSelectedSeries.filter((value) => value !== "__none__")
    : defaultJetsonSeriesSelection(analysisJetsonSeries());
  const index = current.indexOf(id);
  if (index >= 0) current.splice(index, 1);
  else current.push(id);
  state.analysis.jetsonSelectedSeries = current;
  const body = $("analysis-viewer-body");
  if (body) {
    body.innerHTML = renderAnalysisViewer();
    mountAnalysisViewer();
  }
}

function setJetsonStatsPreset(name) {
  const series = analysisJetsonSeries();
  const matches = {
    load: (item) => item.id === "cpu/all/used" || item.id === "gpu/gpu/Used" || item.id === "mem/ram/Use" || item.id === "mem/emc/Bandwidth",
    thermal: (item) => item.group === "temp" && /cpu|gpu|soc|tj/i.test(item.id),
    power: (item) => item.group === "power" && /VDD_IN|CPU_GPU|SOC/i.test(item.id) && /Power|Average/.test(item.id),
    clear: () => false,
  }[name] || (() => false);
  const selected = series.filter(matches).slice(0, 8).map((item) => item.id);
  state.analysis.jetsonSelectedSeries = selected.length ? selected : ["__none__"];
  const body = $("analysis-viewer-body");
  if (body) {
    body.innerHTML = renderAnalysisViewer();
    mountAnalysisViewer();
  }
}

function jetsonSeriesColor(id) {
  const palette = ["#5aa8ff", "#45c478", "#e7b84b", "#f26d6d", "#a779e9", "#4fc3c7", "#ff9f43", "#b8d36d"];
  let hash = 0;
  String(id).split("").forEach((char) => { hash = ((hash << 5) - hash + char.charCodeAt(0)) | 0; });
  return palette[Math.abs(hash) % palette.length];
}

function drawJetsonStatsChart() {
  const canvas = $("analysis-jetson-canvas");
  const prepared = analysisCanvasContext(canvas, 300);
  if (!prepared || !state.analysis.timeline) return;
  const { ctx, width, height } = prepared;
  const selected = analysisSelectedJetsonSeries();
  const duration = Math.max(0.001, analysisDuration());
  const left = 76;
  const right = 16;
  const top = 16;
  const bottom = 22;
  const plotWidth = Math.max(1, width - left - right);
  const groups = [...new Set(selected.map((item) => item.unit || item.group || "value"))];
  const trackHeight = Math.max(54, (height - top - bottom) / Math.max(1, groups.length));
  const toX = (time) => left + Math.max(0, Math.min(1, Number(time) / duration)) * plotWidth;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0a0d10";
  ctx.fillRect(0, 0, width, height);
  ctx.font = "11px ui-sans-serif, system-ui";
  ctx.textBaseline = "middle";

  if (!selected.length) {
    ctx.fillStyle = "#8793a0";
    ctx.textAlign = "center";
    ctx.fillText("Select Jetson metrics to plot", width / 2, height / 2);
    return;
  }

  const gridCount = Math.max(4, Math.min(12, Math.floor(plotWidth / 120)));
  for (let index = 0; index <= gridCount; index += 1) {
    const x = left + (index / gridCount) * plotWidth;
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.beginPath();
    ctx.moveTo(x, top);
    ctx.lineTo(x, height - bottom);
    ctx.stroke();
  }

  groups.forEach((unit, index) => {
    const trackSeries = selected.filter((item) => (item.unit || item.group || "value") === unit);
    const y = top + index * trackHeight;
    const innerTop = y + 8;
    const innerHeight = Math.max(24, trackHeight - 18);
    const valueRange = trackSeries.reduce((range, item) => {
      const itemRange = finiteNumberRange(item.samples, (sample) => sample.value);
      if (!itemRange.count) return range;
      range.min = Math.min(range.min, itemRange.min);
      range.max = Math.max(range.max, itemRange.max);
      range.count += itemRange.count;
      return range;
    }, { min: Infinity, max: -Infinity, count: 0 });
    let min = valueRange.count ? valueRange.min : 0;
    let max = valueRange.count ? valueRange.max : 1;
    if (unit === "%") {
      min = 0;
      max = Math.max(100, max);
    } else if (max <= min) {
      max = min + 1;
    } else {
      const pad = (max - min) * 0.08;
      min -= pad;
      max += pad;
    }
    ctx.fillStyle = index % 2 ? "#0d1115" : "#0f1418";
    ctx.fillRect(left, y, plotWidth, trackHeight - 3);
    ctx.strokeStyle = "rgba(255,255,255,0.08)";
    ctx.beginPath();
    ctx.moveTo(left, y + trackHeight / 2);
    ctx.lineTo(width - right, y + trackHeight / 2);
    ctx.stroke();
    ctx.fillStyle = "#98a2ad";
    ctx.textAlign = "right";
    ctx.fillText(unit, left - 9, y + trackHeight / 2);
    trackSeries.forEach((item) => {
      drawAnalysisSeries(ctx, item.samples, (sample) => sample.value, jetsonSeriesColor(item.id), toX, innerTop, innerHeight, min, max, plotWidth);
    });
  });

  const legendY = Math.max(12, height - 10);
  let legendX = left;
  selected.slice(0, 8).forEach((item) => {
    ctx.fillStyle = jetsonSeriesColor(item.id);
    ctx.fillRect(legendX, legendY - 5, 8, 8);
    ctx.fillStyle = "#c7d0da";
    ctx.textAlign = "left";
    const text = item.label.length > 24 ? `${item.label.slice(0, 23)}...` : item.label;
    ctx.fillText(text, legendX + 12, legendY);
    legendX += Math.min(190, 42 + text.length * 7);
  });

  const cursorX = toX(state.analysis.currentTime);
  ctx.strokeStyle = "rgba(255,255,255,0.9)";
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.moveTo(cursorX, top - 4);
  ctx.lineTo(cursorX, height - bottom + 3);
  ctx.stroke();
}

function analysisTrajectoryCanUseMap() {
  if (!state.analysis.mapDetail) return true;
  if (!analysisMapFingerprintMatches()) return false;
  const timeline = state.analysis.timeline || {};
  const trajectory = analysisTrajectory(timeline);
  const consistency = timeline.map?.consistency || timeline.map_consistency || {};
  return !trajectory.frameId
    || trajectory.frameId === "map"
    || consistency.aligned === true
    || consistency.transform_applied === true
    || consistency.overlay_ready === true;
}

function analysisMapFingerprintMatches() {
  const expected = String(
    state.analysis.timeline?.map?.fingerprint
      || state.analysis.detail?.map?.fingerprint
      || "",
  );
  const current = String(state.analysis.mapDetail?.map?.fingerprint || "");
  return !expected || !current || expected === current;
}

function analysisMapProjector(detail, samples, width, height) {
  if (detail && (detail.raster?.resolution_m_per_px || collectMapPoints(detail).length)) {
    return mapPointProjector(detail, width, height);
  }
  if (!samples.length) return () => [width / 2, height / 2];
  const cached = state.analysis.timeline?._trajectory_bounds;
  const bounds = cached && [cached.minX, cached.maxX, cached.minY, cached.maxY].every(Number.isFinite)
    ? cached
    : samples.reduce(
      (result, sample) => ({
        minX: Math.min(result.minX, Number(sample.x)),
        maxX: Math.max(result.maxX, Number(sample.x)),
        minY: Math.min(result.minY, Number(sample.y)),
        maxY: Math.max(result.maxY, Number(sample.y)),
      }),
      { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity },
    );
  const { minX, maxX, minY, maxY } = bounds;
  const pad = 28;
  const scale = Math.min(
    (width - pad * 2) / Math.max(1.0e-6, maxX - minX),
    (height - pad * 2) / Math.max(1.0e-6, maxY - minY),
  );
  return (point) => [pad + (Number(point[0]) - minX) * scale, height - pad - (Number(point[1]) - minY) * scale];
}

function analysisSpeedColor(speed, maxSpeed) {
  const fraction = Math.max(0, Math.min(1, Number(speed || 0) / Math.max(0.001, maxSpeed)));
  const hue = 210 - fraction * 210;
  return `hsl(${hue.toFixed(0)} 82% 60%)`;
}

function analysisAggressionColor(score) {
  const fraction = Math.max(0, Math.min(1, Number(score || 0) / 100));
  const hue = 135 - fraction * 135;
  return `hsl(${hue.toFixed(0)} 86% 57%)`;
}

function drawE2ETrajectory() {
  const canvas = $("analysis-e2e-trajectory-canvas");
  const timeline = state.analysis.timeline;
  if (!canvas || timeline?.e2e?.task !== "trajectory") return;
  const prepared = analysisCanvasContext(canvas, 360);
  if (!prepared) return;
  const { ctx, width, height } = prepared;
  const sample = timedRecordAt(timeline.e2e.predictions || [], state.analysis.currentTime);
  const predicted = Array.isArray(sample?.trajectory_pred) ? sample.trajectory_pred : [];
  const groundTruth = Array.isArray(sample?.trajectory_gt) ? sample.trajectory_gt : [];
  const all = [[0, 0], ...predicted, ...groundTruth].filter(
    (point) => Array.isArray(point) && point.length >= 2 && point.every((value) => Number.isFinite(Number(value))),
  );

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#080a0d";
  ctx.fillRect(0, 0, width, height);
  const pad = 42;
  const maxForward = Math.max(1, ...all.map((point) => Number(point[0]))) * 1.15;
  const lateralExtent = Math.max(0.75, ...all.map((point) => Math.abs(Number(point[1])))) * 1.2;
  const scale = Math.min(
    (height - pad * 1.7) / maxForward,
    (width - pad * 2) / (lateralExtent * 2),
  );
  const originX = width / 2;
  const originY = height - pad;
  const toPixel = (point) => [
    originX - Number(point[1]) * scale,
    originY - Number(point[0]) * scale,
  ];

  ctx.font = "10px ui-sans-serif, system-ui";
  ctx.textBaseline = "middle";
  for (let meter = 0; meter <= Math.ceil(maxForward); meter += 1) {
    const y = originY - meter * scale;
    ctx.strokeStyle = "rgba(255,255,255,0.08)";
    ctx.beginPath();
    ctx.moveTo(pad, y);
    ctx.lineTo(width - pad, y);
    ctx.stroke();
    ctx.fillStyle = "#71808d";
    ctx.textAlign = "right";
    ctx.fillText(`${meter}m`, pad - 6, y);
  }
  for (let meter = -Math.floor(lateralExtent); meter <= Math.floor(lateralExtent); meter += 1) {
    const x = originX - meter * scale;
    ctx.strokeStyle = meter === 0 ? "rgba(255,255,255,0.2)" : "rgba(255,255,255,0.06)";
    ctx.beginPath();
    ctx.moveTo(x, pad / 2);
    ctx.lineTo(x, originY);
    ctx.stroke();
  }

  const drawLocalPath = (points, color, widthPx) => {
    const pixels = [[0, 0], ...points].map(toPixel);
    if (pixels.length < 2) return;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = widthPx;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    pixels.forEach(([x, y], index) => index ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
    ctx.stroke();
    pixels.slice(1).forEach(([x, y]) => {
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, Math.PI * 2);
      ctx.fill();
    });
  };
  drawLocalPath(groundTruth, "#d8dee9", 3);
  drawLocalPath(predicted, "#5aa8ff", 3);

  ctx.save();
  ctx.translate(originX, originY);
  ctx.fillStyle = "#45c478";
  ctx.strokeStyle = "#071018";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, -13);
  ctx.lineTo(-7, 8);
  ctx.lineTo(0, 5);
  ctx.lineTo(7, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  if (!sample || (!predicted.length && !groundTruth.length)) {
    ctx.fillStyle = "#98a2ad";
    ctx.textAlign = "center";
    ctx.fillText("No trajectory prediction at this time", width / 2, height / 2);
  }
}

function drawAnalysisMap(backgroundImage = undefined) {
  const canvas = $("analysis-map-canvas");
  const prepared = analysisCanvasContext(canvas, 390);
  if (!prepared || !state.analysis.timeline) return;
  const { ctx, width, height } = prepared;
  const detail = state.analysis.mapDetail;
  const trajectory = analysisTrajectory();
  const samples = trajectory.validSamples;
  const imageUrl = detail?.raster?.image_url || detail?.preview_image_url || "";

  let image = backgroundImage;
  if (image === undefined && imageUrl) {
    const cached = mapPreviewImages.get(imageUrl);
    if (cached?.complete) image = cached.naturalWidth ? cached : null;
    else {
      const loading = cached || new Image();
      loading.onload = () => drawAnalysisVisual("map", () => drawAnalysisMap(loading));
      loading.onerror = () => drawAnalysisVisual("map", () => drawAnalysisMap(null));
      if (!cached) {
        mapPreviewImages.set(imageUrl, loading);
        loading.src = imageUrl;
      }
      image = null;
    }
  }

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#080a0d";
  ctx.fillRect(0, 0, width, height);
  if (image?.naturalWidth) {
    ctx.drawImage(image, 0, 0, width, height);
    ctx.fillStyle = "rgba(4, 7, 10, 0.18)";
    ctx.fillRect(0, 0, width, height);
  } else {
    drawGrid(ctx, width, height);
  }

  const toPixel = analysisMapProjector(detail, samples, width, height);
  if (detail) drawAnalysisMapReference(ctx, detail, toPixel);
  const canOverlay = analysisTrajectoryCanUseMap();
  if (canOverlay && samples.length >= 2) {
    const maxSpeed = Number(state.analysis.timeline?._trajectory_speed_max) || 1;
    const useAggression = Boolean(state.analysis.timeline?.e2e?.metrics?.teacher_free);
    const maxSegments = Math.max(500, Math.floor(width * 4));
    const step = Math.max(1, Math.ceil(samples.length / maxSegments));
    ctx.save();
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    for (let index = step; index < samples.length; index += step) {
      const previous = samples[Math.max(0, index - step)];
      const current = samples[index];
      const a = toPixel([previous.x, previous.y]);
      const b = toPixel([current.x, current.y]);
      if (![...a, ...b].every(Number.isFinite)) continue;
      ctx.strokeStyle = useAggression
        ? analysisAggressionColor((Number(previous.aggressiveness_score) + Number(current.aggressiveness_score)) * 0.5)
        : analysisSpeedColor((Number(previous.speed_mps) + Number(current.speed_mps)) * 0.5, maxSpeed);
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(b[0], b[1]);
      ctx.stroke();
    }
    ctx.restore();

    const current = timedRecordAt(samples, state.analysis.currentTime);
    if (current) {
      const [x, y] = toPixel([current.x, current.y]);
      const yaw = Number(current.yaw || 0);
      if (Number.isFinite(x) && Number.isFinite(y)) {
        const rasterYaw = detail?.raster?.resolution_m_per_px
          ? Number(detail.raster.origin_xy_yaw?.[2] || 0)
          : 0;
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(rasterYaw - yaw);
        ctx.fillStyle = "#ffffff";
        ctx.strokeStyle = "#081018";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(12, 0);
        ctx.lineTo(-7, -6);
        ctx.lineTo(-4, 0);
        ctx.lineTo(-7, 6);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
    }
  }

  const empty = $("analysis-map-empty");
  if (empty) {
    if (!samples.length) {
      empty.textContent = "No synchronized trajectory is available.";
      empty.classList.add("visible");
    } else if (!canOverlay) {
      empty.textContent = !analysisMapFingerprintMatches()
        ? "Map contents changed after analysis; re-run before overlaying the trajectory."
        : `Trajectory frame '${trajectory.frameId}' cannot be aligned to map without an explicit transform.`;
      empty.classList.add("visible");
    } else if (!detail) {
      empty.textContent = "Map background unavailable; trajectory is fitted to its own extent.";
      empty.classList.add("visible");
    } else {
      empty.classList.remove("visible");
    }
  }
}

function drawAnalysisMapReference(ctx, detail, toPixel) {
  const lanes = detail.hd_map?.lanes || [];
  lanes.forEach((lane) => {
    drawPolyline(ctx, (lane.left_bound || []).map(toPixel), "rgba(69,196,120,0.72)", 2, lane.closed_loop);
    drawPolyline(ctx, (lane.right_bound || []).map(toPixel), "rgba(216,120,216,0.72)", 2, lane.closed_loop);
    drawPolyline(ctx, (lane.centerline || []).map(toPixel), "rgba(231,200,75,0.7)", lane.primary ? 2.5 : 1.5, lane.closed_loop);
  });
  if (!lanes.length) drawPolyline(ctx, (detail.centerline_csv?.points || []).map(toPixel), "rgba(90,168,255,0.72)", 2, false);
  drawPolyline(ctx, (detail.raceline_csv?.points || []).map(toPixel), "rgba(255,109,109,0.7)", 2, false);
}

function renderMaps() {
  const items = filteredSortedItems(state.maps, "maps");
  return `
    <div class="page">
      <div class="map-workspace-layout">
        <section class="panel map-list-panel">
          <div class="panel-header"><h2>Local Maps</h2><span class="spacer"></span><button onclick="refreshAll()">Scan</button></div>
          <div class="panel-body">
            ${renderListControls("maps", items.length, state.maps.length, "Search map name or path")}
            ${state.maps.length
              ? (items.length ? renderDateGroupedList("maps", items, mapList) : `<div class="empty">No maps match the current filter.</div>`)
              : `<div class="empty">No map folders under ${esc(state.config?.map_root || "")}</div>`}
          </div>
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

function mapList(items = state.maps) {
  return `
    <div class="map-list">
      ${items
        .map((map) => {
          const selected = state.selectedMapPath === map.path;
          const artifactKeys = ["cuvgl_map", "cuvslam_map", "hd_map", "centerline_csv", "raceline_csv", "line_preview"];
          const title = mapDisplayName(map);
          return `
            <article class="map-list-item ${selected ? "selected" : ""}">
              <div class="map-list-main">
                <strong title="${esc(map.name)}">${esc(title)}</strong>
                ${title !== map.name ? `<span class="map-list-subtitle" title="${esc(map.name)}">${esc(map.name)}</span>` : ""}
                <div class="path" title="${esc(map.path)}">${esc(map.path)}</div>
                <div class="chips">${artifactKeys
                  .map((key) => `<span class="chip ${map.artifacts[key]?.exists ? "ok" : "missing"}">${artifactLabel(key)}</span>`)
                  .join("")}</div>
              </div>
              <div class="map-list-actions">
                ${map.complete_runtime_bundle ? `<span class="status success">runtime ready</span>` : `<span class="status failed">incomplete</span>`}
                <button class="primary" onclick="openMapWorkspace(${js(map.path)})">${selected ? "Viewing" : "Open"}</button>
                <button onclick="copyText(${js(map.path)})">Copy</button>
                <button onclick="copyHdMapEditorCommand(${js(map.path)})">Editor Cmd</button>
                ${renderMapStageButton("prepare-hd-raster", map.path, "Raster")}
                ${renderMapStageButton("generate-raceline", map.path, "Raceline")}
                ${renderMapStageButton("generate-preview", map.path, "Preview")}
                <button onclick="fillTransferLocal(${js(map.path)})">Transfer</button>
                <button class="danger" onclick="deleteMapFolder(${js(map.path)})">Delete</button>
              </div>
              <div class="map-list-preflight-grid">
                ${renderMapStageReadiness("prepare-hd-raster", map.path, "Raster", { micro: true })}
                ${renderMapStageReadiness("generate-raceline", map.path, "Raceline", { micro: true })}
                ${renderMapStageReadiness("generate-preview", map.path, "Preview", { micro: true })}
              </div>
            </article>`;
        })
        .join("")}
    </div>
  `;
}

function mapDisplayName(map) {
  return map?.display_name || map?.name || "map";
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
          <h3>${esc(mapDisplayName(detail.map))}</h3>
          <div class="path" title="${esc(detail.map.path)}">${esc(detail.map.path)}</div>
        </div>
        <div class="actions">
          ${renderMapStageButton("prepare-hd-raster", detail.map.path, "Raster")}
          <button onclick="copyHdMapEditorCommand(${js(detail.map.path)})">Editor Cmd</button>
          ${renderMapStageButton("generate-raceline", detail.map.path, "Raceline")}
          ${renderMapStageButton("generate-preview", detail.map.path, "Preview")}
          <button onclick="fillTransferLocal(${js(detail.map.path)})">Transfer</button>
        </div>
      </div>
      <div class="map-stage-readiness-grid">
        ${renderMapStageReadiness("prepare-hd-raster", detail.map.path, "Landmark raster")}
        ${renderMapStageReadiness("generate-raceline", detail.map.path, "Raceline generation")}
        ${renderMapStageReadiness("generate-preview", detail.map.path, "Preview generation")}
      </div>
      <div class="map-preview-grid">
        <div class="map-preview-shell">
          <canvas
            id="map-preview-canvas"
            width="900"
            height="620"
            onpointerdown="handleMapEditorPointerDown(event)"
            onpointermove="handleMapEditorPointerMove(event)"
            onpointerup="handleMapEditorPointerUp(event)"
            onpointerleave="handleMapEditorPointerUp(event)"
            ondblclick="handleMapEditorDoubleClick(event)"
            oncontextmenu="handleMapEditorContextMenu(event)"
            onwheel="handleMapEditorWheel(event)"
          ></canvas>
        </div>
        <aside class="map-side-panel">
          ${renderHdRasterOptions(detail)}
          ${renderHdMapEditor(detail)}
          ${renderHdMapVersions(detail)}
          ${renderRacelineClearance(detail)}
          ${renderSectionGateEditor(detail)}
          ${renderLayerToggles()}
          ${renderMapInspector(detail)}
        </aside>
      </div>
      ${renderSimulationPanel(detail)}
    </div>
  `;
}

function renderHdRasterOptions(detail) {
  const options = state.hdRasterGeneration;
  const snapshotReady = Boolean(detail.map?.artifacts?.snapshot?.exists);
  const mode = options.cropMode || (options.pathCropDistanceM > 0 ? "path" : "percent");
  return `
    <div class="inspector-block hd-raster-block">
      <div class="inspector-title-row">
        <h4>HD Raster</h4>
        <span class="${snapshotReady ? "ok" : "warn"}">${snapshotReady ? "Ready" : "Need snapshot"}</span>
      </div>
      <div class="form-grid">
        <div class="field">
          <label for="hd-raster-crop-mode">Crop mode</label>
          <select id="hd-raster-crop-mode" onchange="updateHdRasterCropMode(this.value)">
            <option value="path" ${mode === "path" ? "selected" : ""}>Track focus</option>
            <option value="percent" ${mode === "percent" ? "selected" : ""}>Percentile</option>
            <option value="off" ${mode === "off" ? "selected" : ""}>Off</option>
          </select>
        </div>
        ${mode === "path"
          ? `
            <div class="field">
              <label for="hd-raster-path-crop-distance">Distance from path (m)</label>
              <input
                id="hd-raster-path-crop-distance"
                type="number"
                min="0.01"
                step="0.1"
                value="${esc(options.pathCropDistanceM)}"
                oninput="updateHdRasterNumber('pathCropDistanceM', this)"
              />
            </div>`
          : mode === "percent"
          ? `
            <div class="field">
              <label for="hd-raster-auto-crop-percentile">Central percentile (%)</label>
              <input
                id="hd-raster-auto-crop-percentile"
                type="number"
                min="0.001"
                max="100"
                step="0.1"
                value="${esc(options.autoCropPercentile)}"
                oninput="updateHdRasterNumber('autoCropPercentile', this)"
              />
            </div>`
          : `
            <div class="field">
              <label>Crop value</label>
              <input value="All points" disabled />
            </div>`}
        <div class="field-hint full">
          ${mode === "path"
            ? `Raster keeps landmarks within ${options.pathCropDistanceM} m of the VSLAM path.`
            : mode === "off" || options.autoCropPercentile >= 100
            ? "Raster bounds use all landmark points."
            : `Raster bounds use the central ${options.autoCropPercentile}% of landmark points.`}
        </div>
        <div class="actions full">
          ${renderMapStageButton("prepare-hd-raster", detail.map.path, "Generate Raster", { className: "primary" })}
        </div>
      </div>
    </div>
  `;
}

function updateHdRasterCropMode(mode) {
  const normalized = ["path", "percent", "off"].includes(mode) ? mode : "path";
  state.hdRasterGeneration.cropMode = normalized;
  if (normalized === "off") {
    state.hdRasterGeneration.pathCropDistanceM = 0.0;
    state.hdRasterGeneration.autoCropPercentile = 100.0;
  } else if (normalized === "path") {
    state.hdRasterGeneration.pathCropDistanceM = state.hdRasterGeneration.pathCropDistanceM > 0
      ? state.hdRasterGeneration.pathCropDistanceM
      : 1.5;
    state.hdRasterGeneration.autoCropPercentile = 100.0;
  } else {
    state.hdRasterGeneration.pathCropDistanceM = 0.0;
    state.hdRasterGeneration.autoCropPercentile = state.hdRasterGeneration.autoCropPercentile < 100
      ? state.hdRasterGeneration.autoCropPercentile
      : 99.0;
  }
  refreshHdRasterOptions();
}

function updateHdRasterNumber(field, input) {
  const value = Number(String(input?.value ?? "").trim());
  const min = Number(input?.min || 0);
  const max = input?.max === "" || input?.max == null ? Infinity : Number(input.max);
  const valid = Number.isFinite(value) && value >= min && value <= max;
  input?.setCustomValidity(valid ? "" : "Enter a finite value in range");
  if (!valid) return;
  if (field === "pathCropDistanceM") {
    state.hdRasterGeneration.cropMode = "path";
    state.hdRasterGeneration.pathCropDistanceM = value;
    state.hdRasterGeneration.autoCropPercentile = 100.0;
  } else if (field === "autoCropPercentile") {
    state.hdRasterGeneration.cropMode = "percent";
    state.hdRasterGeneration.pathCropDistanceM = 0.0;
    state.hdRasterGeneration.autoCropPercentile = value;
  }
  refreshHdRasterOptions({ renderAfter: false });
}

function refreshHdRasterOptions(options = {}) {
  if (state.selectedMapDetail?.map?.path) {
    invalidateMapPreflights(state.selectedMapDetail.map.path);
    if (options.renderAfter === false) scheduleVisiblePreflights({ force: true });
    else render();
  }
}

function hdRasterGenerationPayload() {
  const mode = state.hdRasterGeneration.cropMode || "path";
  return {
    auto_crop_percentile: mode === "percent" ? state.hdRasterGeneration.autoCropPercentile : 100.0,
    auto_crop_min_retained_ratio: state.hdRasterGeneration.autoCropMinRetainedRatio,
    path_crop_distance_m: mode === "path" ? state.hdRasterGeneration.pathCropDistanceM : 0.0,
    path_crop_min_retained_ratio: state.hdRasterGeneration.pathCropMinRetainedRatio,
  };
}

function renderSimulationPanel(detail) {
  ensureSimulationState(detail);
  const sim = state.simulation;
  const path = simulationPathPoints(detail);
  const ready = path.length >= 2;
  const status = !ready ? "Need path" : sim.playing ? "Running" : sim.time > 0 ? "Paused" : "Ready";
  const sourceOptions = [
    ["raceline", "Raceline"],
    ["centerline", "Centerline"],
  ];
  return `
    <section class="simulation-panel">
      <div class="simulation-header">
        <div>
          <h4>Simulation</h4>
          <span id="simulation-status" class="${ready ? (sim.playing ? "running" : "ok") : "warn"}">${esc(status)}</span>
        </div>
        <div class="simulation-actions">
          <button id="simulation-run-button" class="primary" onclick="toggleSimulationPlayback()" ${ready ? "" : "disabled"}>${sim.playing ? "Pause" : "Run"}</button>
          <button onclick="stepSimulationOnce()" ${ready ? "" : "disabled"}>Step</button>
          <button onclick="resetSimulation()">Reset</button>
        </div>
      </div>
      <div class="simulation-body">
        <div class="simulation-stage">
          <canvas id="simulation-canvas" width="920" height="520"></canvas>
        </div>
        <aside class="simulation-controls">
          <div class="simulation-control-grid">
            <div class="field full">
              <label for="simulation-source">Path source</label>
              <select id="simulation-source" onchange="setSimulationSource(this.value)">
                ${sourceOptions.map(([value, label]) => `<option value="${value}" ${sim.source === value ? "selected" : ""}>${esc(label)}</option>`).join("")}
              </select>
            </div>
            ${simulationNumberInput("targetSpeedMps", "Target speed (m/s)", 0, 0.1)}
            ${simulationNumberInput("wheelbaseM", "Wheelbase (m)", 0.01, 0.01)}
            ${simulationNumberInput("minLookaheadM", "Min lookahead (m)", 0.01, 0.05)}
            ${simulationNumberInput("maxLookaheadM", "Max lookahead (m)", 0.01, 0.05)}
            ${simulationNumberInput("lookaheadGainS", "Lookahead gain (s)", 0, 0.05)}
            ${simulationNumberInput("maxSteeringRad", "Max steering (rad)", 0.01, 0.01)}
            ${simulationNumberInput("maxAccelMps2", "Max accel (m/s^2)", 0.01, 0.1)}
            ${simulationNumberInput("maxDecelMps2", "Max decel (m/s^2)", 0.01, 0.1)}
          </div>
          <div class="simulation-metrics" id="simulation-metrics">
            ${renderSimulationMetrics()}
          </div>
        </aside>
      </div>
    </section>
  `;
}

function simulationNumberInput(key, label, min, step) {
  const value = state.simulation.settings[key];
  return `
    <div class="field">
      <label for="simulation-${esc(key)}">${esc(label)}</label>
      <input
        id="simulation-${esc(key)}"
        type="number"
        min="${esc(min)}"
        step="${esc(step)}"
        value="${esc(value)}"
        oninput="updateSimulationSetting(${js(key)}, this)"
      />
    </div>
  `;
}

function renderSimulationMetrics() {
  const sim = state.simulation;
  return `
    ${statTile("time", `${sim.time.toFixed(2)} s`)}
    ${statTile("speed", `${sim.speed.toFixed(2)} m/s`)}
    ${statTile("steer", `${sim.steeringRad.toFixed(3)} rad`)}
    ${statTile("error", `${sim.crossTrackErrorM.toFixed(3)} m`)}
    ${statTile("max error", `${sim.maxCrossTrackErrorM.toFixed(3)} m`)}
    ${statTile("distance", `${sim.distance.toFixed(1)} m`)}
  `;
}

function racelineEnvelopeLabel(vehicleWidthM, safetyMarginM) {
  return `${(vehicleWidthM + 2 * safetyMarginM).toFixed(3)} m total`;
}

function renderRacelineClearance(detail) {
  const options = state.racelineGeneration;
  const centerlineReady = Boolean(detail.map?.artifacts?.centerline_csv?.exists);
  return `
    <div class="inspector-block">
      <div class="inspector-title-row">
        <h4>Raceline Clearance</h4>
        <span class="${centerlineReady ? "ok" : "warn"}">${centerlineReady ? "Ready" : "Need centerline"}</span>
      </div>
      <div class="form-grid">
        <div class="field">
          <label for="raceline-vehicle-width">Vehicle width (m)</label>
          <input
            id="raceline-vehicle-width"
            type="number"
            min="0"
            step="0.001"
            value="${esc(options.vehicleWidthM)}"
            oninput="updateRacelineGeneration('vehicleWidthM', this)"
          />
        </div>
        <div class="field">
          <label for="raceline-safety-margin">Boundary margin / side (m)</label>
          <input
            id="raceline-safety-margin"
            type="number"
            min="0"
            step="0.001"
            value="${esc(options.safetyMarginM)}"
            oninput="updateRacelineGeneration('safetyMarginM', this)"
          />
        </div>
        <div id="raceline-envelope" class="field-hint raceline-envelope-hint full">
          Effective optimizer envelope: ${esc(racelineEnvelopeLabel(options.vehicleWidthM, options.safetyMarginM))}
        </div>
        <div class="field">
          <label for="raceline-max-speed">Max speed (m/s)</label>
          <input
            id="raceline-max-speed"
            type="number"
            min="0.001"
            step="0.1"
            value="${esc(options.maxSpeedMps)}"
            oninput="updateRacelineGeneration('maxSpeedMps', this)"
          />
        </div>
        <div class="field">
          <label for="raceline-min-speed">Min speed (m/s)</label>
          <input
            id="raceline-min-speed"
            type="number"
            min="0"
            step="0.1"
            value="${esc(options.minSpeedMps)}"
            oninput="updateRacelineGeneration('minSpeedMps', this)"
          />
        </div>
        <div class="field">
          <label for="raceline-lateral-accel">Lateral accel limit (m/s^2)</label>
          <input
            id="raceline-lateral-accel"
            type="number"
            min="0.001"
            step="0.1"
            value="${esc(options.lateralAccelLimitMps2)}"
            oninput="updateRacelineGeneration('lateralAccelLimitMps2', this)"
          />
        </div>
        <div class="field">
          <label for="raceline-accel-limit">Accel limit (m/s^2)</label>
          <input
            id="raceline-accel-limit"
            type="number"
            min="0.001"
            step="0.1"
            value="${esc(options.accelLimitMps2)}"
            oninput="updateRacelineGeneration('accelLimitMps2', this)"
          />
        </div>
        <div class="field">
          <label for="raceline-decel-limit">Decel limit (m/s^2)</label>
          <input
            id="raceline-decel-limit"
            type="number"
            min="0.001"
            step="0.1"
            value="${esc(options.decelLimitMps2)}"
            oninput="updateRacelineGeneration('decelLimitMps2', this)"
          />
        </div>
        <div class="actions full">
          ${renderMapStageButton("generate-raceline", detail.map.path, "Generate Raceline", { className: "primary" })}
        </div>
      </div>
    </div>
  `;
}

function updateRacelineGeneration(field, input) {
  const allowed = [
    "vehicleWidthM",
    "safetyMarginM",
    "maxSpeedMps",
    "minSpeedMps",
    "lateralAccelLimitMps2",
    "accelLimitMps2",
    "decelLimitMps2",
  ];
  if (!allowed.includes(field)) return;
  const raw = String(input?.value ?? "").trim();
  const value = Number(raw);
  const mustBePositive = ["maxSpeedMps", "lateralAccelLimitMps2", "accelLimitMps2", "decelLimitMps2"].includes(field);
  const valid = raw !== "" && Number.isFinite(value) && (mustBePositive ? value > 0 : value >= 0);
  input?.setCustomValidity(valid ? "" : mustBePositive ? "Enter a finite value greater than 0" : "Enter a finite value greater than or equal to 0");
  if (valid) state.racelineGeneration[field] = value;

  const vehicleInput = $("raceline-vehicle-width");
  const marginInput = $("raceline-safety-margin");
  const maxSpeedInput = $("raceline-max-speed");
  const minSpeedInput = $("raceline-min-speed");
  const lateralAccelInput = $("raceline-lateral-accel");
  const accelLimitInput = $("raceline-accel-limit");
  const decelLimitInput = $("raceline-decel-limit");
  const vehicleWidthM = Number(vehicleInput?.value);
  const safetyMarginM = Number(marginInput?.value);
  const maxSpeedMps = Number(maxSpeedInput?.value);
  const minSpeedMps = Number(minSpeedInput?.value);
  const lateralAccelLimitMps2 = Number(lateralAccelInput?.value);
  const accelLimitMps2 = Number(accelLimitInput?.value);
  const decelLimitMps2 = Number(decelLimitInput?.value);
  const inputsValid =
    String(vehicleInput?.value ?? "").trim() !== "" &&
    String(marginInput?.value ?? "").trim() !== "" &&
    String(maxSpeedInput?.value ?? "").trim() !== "" &&
    String(minSpeedInput?.value ?? "").trim() !== "" &&
    String(lateralAccelInput?.value ?? "").trim() !== "" &&
    String(accelLimitInput?.value ?? "").trim() !== "" &&
    String(decelLimitInput?.value ?? "").trim() !== "" &&
    Number.isFinite(vehicleWidthM) &&
    Number.isFinite(safetyMarginM) &&
    Number.isFinite(maxSpeedMps) &&
    Number.isFinite(minSpeedMps) &&
    Number.isFinite(lateralAccelLimitMps2) &&
    Number.isFinite(accelLimitMps2) &&
    Number.isFinite(decelLimitMps2) &&
    vehicleWidthM >= 0 &&
    safetyMarginM >= 0 &&
    maxSpeedMps > 0 &&
    minSpeedMps >= 0 &&
    minSpeedMps <= maxSpeedMps &&
    lateralAccelLimitMps2 > 0 &&
    accelLimitMps2 > 0 &&
    decelLimitMps2 > 0;
  const envelope = $("raceline-envelope");
  if (envelope) {
    envelope.textContent = inputsValid
      ? `Effective optimizer envelope: ${racelineEnvelopeLabel(vehicleWidthM, safetyMarginM)}`
      : "Raceline dimensions and speed profile must be finite and physically usable.";
  }
  if (inputsValid && state.selectedMapDetail?.map?.path) {
    state.racelineGeneration.vehicleWidthM = vehicleWidthM;
    state.racelineGeneration.safetyMarginM = safetyMarginM;
    state.racelineGeneration.maxSpeedMps = maxSpeedMps;
    state.racelineGeneration.minSpeedMps = minSpeedMps;
    state.racelineGeneration.lateralAccelLimitMps2 = lateralAccelLimitMps2;
    state.racelineGeneration.accelLimitMps2 = accelLimitMps2;
    state.racelineGeneration.decelLimitMps2 = decelLimitMps2;
    scheduleRacelinePreflight(state.selectedMapDetail.map.path);
  }
}

function racelineGenerationPayload() {
  const vehicleInput = $("raceline-vehicle-width");
  const marginInput = $("raceline-safety-margin");
  const maxSpeedInput = $("raceline-max-speed");
  const minSpeedInput = $("raceline-min-speed");
  const lateralAccelInput = $("raceline-lateral-accel");
  const accelLimitInput = $("raceline-accel-limit");
  const decelLimitInput = $("raceline-decel-limit");
  const rawVehicleWidth = vehicleInput
    ? String(vehicleInput.value).trim()
    : state.racelineGeneration.vehicleWidthM;
  const rawSafetyMargin = marginInput
    ? String(marginInput.value).trim()
    : state.racelineGeneration.safetyMarginM;
  const rawMaxSpeed = maxSpeedInput ? String(maxSpeedInput.value).trim() : state.racelineGeneration.maxSpeedMps;
  const rawMinSpeed = minSpeedInput ? String(minSpeedInput.value).trim() : state.racelineGeneration.minSpeedMps;
  const rawLateralAccel = lateralAccelInput
    ? String(lateralAccelInput.value).trim()
    : state.racelineGeneration.lateralAccelLimitMps2;
  const rawAccelLimit = accelLimitInput ? String(accelLimitInput.value).trim() : state.racelineGeneration.accelLimitMps2;
  const rawDecelLimit = decelLimitInput ? String(decelLimitInput.value).trim() : state.racelineGeneration.decelLimitMps2;
  const vehicleWidthM = Number(rawVehicleWidth);
  const safetyMarginM = Number(rawSafetyMargin);
  const maxSpeedMps = Number(rawMaxSpeed);
  const minSpeedMps = Number(rawMinSpeed);
  const lateralAccelLimitMps2 = Number(rawLateralAccel);
  const accelLimitMps2 = Number(rawAccelLimit);
  const decelLimitMps2 = Number(rawDecelLimit);
  if (
    rawVehicleWidth === "" ||
    rawSafetyMargin === "" ||
    rawMaxSpeed === "" ||
    rawMinSpeed === "" ||
    rawLateralAccel === "" ||
    rawAccelLimit === "" ||
    rawDecelLimit === "" ||
    !Number.isFinite(vehicleWidthM) ||
    !Number.isFinite(safetyMarginM) ||
    !Number.isFinite(maxSpeedMps) ||
    !Number.isFinite(minSpeedMps) ||
    !Number.isFinite(lateralAccelLimitMps2) ||
    !Number.isFinite(accelLimitMps2) ||
    !Number.isFinite(decelLimitMps2) ||
    vehicleWidthM < 0 ||
    safetyMarginM < 0 ||
    maxSpeedMps <= 0 ||
    minSpeedMps < 0 ||
    minSpeedMps > maxSpeedMps ||
    lateralAccelLimitMps2 <= 0 ||
    accelLimitMps2 <= 0 ||
    decelLimitMps2 <= 0
  ) {
    throw new Error("Raceline dimensions and speed profile must be finite and physically usable.");
  }
  state.racelineGeneration.vehicleWidthM = vehicleWidthM;
  state.racelineGeneration.safetyMarginM = safetyMarginM;
  state.racelineGeneration.maxSpeedMps = maxSpeedMps;
  state.racelineGeneration.minSpeedMps = minSpeedMps;
  state.racelineGeneration.lateralAccelLimitMps2 = lateralAccelLimitMps2;
  state.racelineGeneration.accelLimitMps2 = accelLimitMps2;
  state.racelineGeneration.decelLimitMps2 = decelLimitMps2;
  return {
    vehicle_width_m: vehicleWidthM,
    safety_margin_m: safetyMarginM,
    max_speed_mps: maxSpeedMps,
    min_speed_mps: minSpeedMps,
    lateral_accel_limit_mps2: lateralAccelLimitMps2,
    accel_limit_mps2: accelLimitMps2,
    decel_limit_mps2: decelLimitMps2,
  };
}

function renderLayerToggles() {
  const layers = [
    ["landmark", "Landmark"],
    ["left_bound", "Left bound"],
    ["right_bound", "Right bound"],
    ["centerline", "Centerline"],
    ["raceline", "Raceline"],
    ["odometry", "Odometry"],
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

function renderHdMapEditor(detail) {
  const editor = ensureMapEditor(detail);
  const lane = activeEditorLane();
  const rasterReady = mapEditorRasterReady(detail);
  const issue = editorLaneIssue(lane);
  const selected = Boolean(editor.selected);
  const assist = normalizedMapEditorAssistSettings();
  const canSmooth = canSmoothSelectedEditorRange();
  const status = !rasterReady ? "Raster required" : issue || (editor.dirty ? "Unsaved" : "Ready");
  const canSave = editor.enabled && rasterReady && !issue;
  return `
    <div class="inspector-block map-editor-block">
      <div class="inspector-title-row">
        <h4>HD Map Edit</h4>
        <span id="map-editor-status" class="${issue || !rasterReady ? "warn" : editor.dirty ? "dirty" : "ok"}">${esc(status)}</span>
      </div>
      <div class="editor-actions">
        <button class="${editor.enabled ? "primary" : ""}" onclick="toggleHdMapEditor()">${editor.enabled ? "Editing" : "Edit"}</button>
        <button id="map-editor-undo" onclick="undoMapEditor()" ${editor.enabled && editor.undoStack.length ? "" : "disabled"}>Undo</button>
        <button id="map-editor-redo" onclick="redoMapEditor()" ${editor.enabled && editor.redoStack.length ? "" : "disabled"}>Redo</button>
        <button id="map-editor-save" class="${actionBusy("hd-map:save") ? "is-busy" : ""}" onclick="saveHdMapFromEditor()" ${canSave ? "" : "disabled"} ${actionButtonAttrs("hd-map:save", "HD map is saving...")}>${esc(actionButtonLabel("hd-map:save", "Save", "Saving..."))}</button>
        <button id="map-editor-auto-center" onclick="regenerateEditorCenterlineFromBounds()" ${editor.enabled ? "" : "disabled"}>Auto Center</button>
        <button id="map-editor-delete" class="danger" onclick="deleteSelectedEditorPoint()" ${editor.enabled && selected ? "" : "disabled"}>Delete Pt</button>
      </div>
      <div class="editor-field-row">
        ${editorFieldButton("left_bound", "Left boundary")}
        ${editorFieldButton("right_bound", "Right boundary")}
        ${editorFieldButton("centerline", "Centerline")}
      </div>
      <div class="editor-assist-grid">
        <div class="field">
          <label for="map-editor-assist-range">Assist range (pts)</label>
          <input
            id="map-editor-assist-range"
            type="number"
            min="1"
            max="${MAX_MAP_EDITOR_ASSIST_RANGE_POINTS}"
            step="1"
            value="${esc(assist.rangePoints)}"
            oninput="updateMapEditorAssistNumber('rangePoints', this)"
            ${editor.enabled ? "" : "disabled"}
          />
        </div>
        <div class="field">
          <label for="map-editor-assist-spacing">Point spacing (m)</label>
          <input
            id="map-editor-assist-spacing"
            type="number"
            min="0.02"
            max="1"
            step="0.01"
            value="${esc(assist.spacingM)}"
            oninput="updateMapEditorAssistNumber('spacingM', this)"
            ${editor.enabled ? "" : "disabled"}
          />
        </div>
        <button id="map-editor-smooth" class="primary full" onclick="smoothSelectedEditorRange()" ${canSmooth ? "" : "disabled"}>Smooth Area</button>
        <div id="map-editor-assist-hint" class="field-hint full">${esc(mapEditorAssistHint())}</div>
      </div>
      <div class="editor-zoom-row">
        <button onclick="zoomMapEditor(0.75)">-</button>
        <button onclick="resetMapEditorZoom()">Fit</button>
        <button onclick="zoomMapEditor(1.3333333333)">+</button>
        <span id="map-editor-zoom-value">${esc(mapEditorZoomLabel())}</span>
      </div>
      <div class="editor-toggle-row">
        <label class="layer-toggle">
          <input id="map-editor-closed-loop" type="checkbox" ${lane.closed_loop ? "checked" : ""} onchange="toggleEditorClosedLoop(this.checked)" ${editor.enabled ? "" : "disabled"} />
          <span>Closed loop</span>
        </label>
        <label class="layer-toggle">
          <input id="map-editor-show-centerline" type="checkbox" ${editor.showCenterline ? "checked" : ""} onchange="toggleEditorCenterline(this.checked)" />
          <span>Center line</span>
        </label>
      </div>
      <div id="map-editor-counts" class="editor-counts">${renderEditorCounts(lane)}</div>
    </div>
  `;
}

function editorFieldButton(field, label) {
  const active = state.mapEditor.activeField === field;
  return `<button id="map-editor-field-${esc(field)}" class="${active ? "active" : ""}" onclick="setMapEditorField(${js(field)})" ${state.mapEditor.enabled ? "" : "disabled"}>${esc(label)}</button>`;
}

function renderHdMapVersions(detail) {
  const versions = detail.hd_map_versions?.versions || [];
  const activeId = detail.hd_map_versions?.active_id || "";
  const dirty = Boolean(detail.hd_map_versions?.working_copy_dirty);
  const hdReady = Boolean(detail.hd_map?.exists && detail.map?.artifacts?.centerline_csv?.exists);
  const status = dirty ? "Working copy" : activeId ? activeId : hdReady ? "Unsaved" : "Need HD map";
  return `
    <div class="inspector-block hd-version-block">
      <div class="inspector-title-row">
        <h4>HD Map Versions</h4>
        <span class="${dirty ? "dirty" : activeId ? "ok" : "warn"}">${esc(status)}</span>
      </div>
      <div class="version-save-row">
        <input id="hd-map-version-label" placeholder="optional label" ${hdReady ? "" : "disabled"} />
        <button class="primary ${actionBusy("hd-map-version:save") ? "is-busy" : ""}" onclick="saveHdMapVersion()" ${hdReady ? "" : "disabled"} ${actionButtonAttrs("hd-map-version:save", "HD map version is saving...")}>${esc(actionButtonLabel("hd-map-version:save", "Save Ver", "Saving..."))}</button>
      </div>
      <div class="version-list">
        ${versions.length
          ? versions.map((version) => renderHdMapVersionRow(version, detail.map.path)).join("")
          : `<div class="field-hint">No saved HD map versions yet.</div>`}
      </div>
    </div>
  `;
}

function renderHdMapVersionRow(version, mapPath) {
  const hasRaceline = Boolean(version.artifacts?.raceline_csv?.exists);
  const active = Boolean(version.active);
  const matches = Boolean(version.matches_active_files);
  const status = active ? (matches ? "active" : "edited") : hasRaceline ? "raceline" : "map only";
  return `
    <div class="version-row ${active ? "active" : ""}">
      <div>
        <strong>${esc(version.label || version.id)}</strong>
        <span>${esc(version.id)} / ${esc(status)}</span>
      </div>
      <button onclick="activateHdMapVersion(${js(mapPath)}, ${js(version.id)})" ${active && matches ? "disabled" : ""}>Use</button>
    </div>
  `;
}

async function saveHdMapVersion() {
  const detail = state.selectedMapDetail;
  if (!detail?.map?.path) return;
  const label = String($("hd-map-version-label")?.value || "").trim();
  if (!confirmAction({
    title: "Save a new HD map version?",
    target: detail.map.path,
    detail: label ? `Label: ${label}` : "No label set.",
  })) return;
  if (!beginAction("hd-map-version:save", "Saving HD map version")) return;
  try {
    const saved = await api("/api/maps/save-hd-map-version", {
      method: "POST",
      body: JSON.stringify({
        map_dir: detail.map.path,
        label,
      }),
    });
    state.selectedMapDetail = saved;
    state.selectedMapPath = saved.map.path;
    const mapIndex = state.maps.findIndex((item) => item.path === saved.map.path);
    if (mapIndex >= 0) state.maps[mapIndex] = saved.map;
    ensureMapEditor(saved, { force: true });
    state.mapEditor.enabled = false;
    invalidateMapPreflights(saved.map.path);
    toast("HD map version saved");
    render();
  } catch (error) {
    toast(`Version save failed: ${error.message}`, "error");
  } finally {
    endAction("hd-map-version:save", { renderAfter: state.tab === "maps" });
  }
}

async function activateHdMapVersion(mapPath, versionId) {
  if (!mapPath || !versionId) return;
  if (!confirmAction({
    title: "Activate this HD map version?",
    target: mapPath,
    detail: `Version: ${versionId}`,
  })) return;
  try {
    const saved = await api("/api/maps/activate-hd-map-version", {
      method: "POST",
      body: JSON.stringify({
        map_dir: mapPath,
        version_id: versionId,
      }),
    });
    state.selectedMapDetail = saved;
    state.selectedMapPath = saved.map.path;
    const mapIndex = state.maps.findIndex((item) => item.path === saved.map.path);
    if (mapIndex >= 0) state.maps[mapIndex] = saved.map;
    ensureMapEditor(saved, { force: true });
    state.mapEditor.enabled = false;
    invalidateMapPreflights(saved.map.path);
    toast(`Activated ${versionId}`);
    render();
  } catch (error) {
    toast(`Version activate failed: ${error.message}`, "error");
  }
}

function renderEditorCounts(lane) {
  return [
    `L ${lane.left_bound.length}`,
    `R ${lane.right_bound.length}`,
    `C ${lane.centerline.length}`,
  ].join(" / ");
}

function renderSectionGateEditor(detail) {
  const editor = ensureSectionEditor(detail);
  const lane = sectionEditorLane(detail);
  const ready = Boolean(detail.hd_map?.exists && lane?.centerline?.length >= 2);
  const selected = Boolean(editor.selectedGateId);
  const status = !detail.hd_map?.exists ? "Need HD map" : !ready ? "Need centerline" : editor.dirty ? "Unsaved" : "Ready";
  return `
    <div class="inspector-block section-editor-block">
      <div class="inspector-title-row">
        <h4>Section Gates</h4>
        <span id="section-editor-status" class="${ready ? (editor.dirty ? "dirty" : "ok") : "warn"}">${esc(status)}</span>
      </div>
      <div class="editor-actions">
        <button class="${editor.enabled ? "primary" : ""}" onclick="toggleSectionEditor()" ${ready ? "" : "disabled"}>${editor.enabled ? "Editing" : "Edit"}</button>
        <button id="section-editor-save" class="${actionBusy("section-gates:save") ? "is-busy" : ""}" onclick="saveSectionGatesFromEditor()" ${editor.enabled && ready ? "" : "disabled"} ${actionButtonAttrs("section-gates:save", "Section gates are saving...")}>${esc(actionButtonLabel("section-gates:save", "Save", "Saving..."))}</button>
        <button id="section-editor-delete" class="danger" onclick="deleteSelectedSectionGate()" ${editor.enabled && selected ? "" : "disabled"}>Delete Gate</button>
      </div>
      <div id="section-editor-counts" class="editor-counts">${renderSectionEditorCounts(detail)}</div>
    </div>
  `;
}

function renderSectionEditorCounts(detail) {
  const gates = sectionGatesForDetail(detail) || detail.hd_map?.section_gates || [];
  const gateCount = gates.length;
  const lane = sectionEditorLane(detail);
  const laneGateCount = gates.filter((gate) => !lane?.id || gate.lane_id === lane.id).length;
  const sectionCount = lane?.closed_loop ? laneGateCount : Math.max(0, laneGateCount - 1);
  const selected = state.sectionEditor.selectedGateId ? ` / selected ${state.sectionEditor.selectedGateId}` : "";
  return `G ${gateCount} / S ${sectionCount}${selected}`;
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
        ${statTile("odom pts", stats.odometry_points || 0)}
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
  pruneSelectedJetsonPullPaths(sequences);
  const selectedPullPaths = selectedJetsonPullPaths();
  const activePullPaths = jetsonActivePullPaths();
  const queue = state.jetsonTransfer;
  const queueRunning = Boolean(queue.running);
  const pullBusy = actionBusy("jetson:pull") || queueRunning;
  const currentPullPath = queue.currentPath || activePullPaths[queue.currentIndex] || "";
  const queueText = queueRunning
    ? `Pulling ${queue.currentIndex + 1} of ${queue.total}: ${shortName(currentPullPath)}`
    : actionBusy("jetson:pull")
      ? `Starting ${activePullPaths.length || selectedPullPaths.length || 1} pull transfer${(activePullPaths.length || selectedPullPaths.length) > 1 ? "s" : ""}`
      : selectedPullPaths.length
        ? `${selectedPullPaths.length} selected`
        : "No sequence selected";
  const primaryPullLabel = queueRunning
    ? `Pulling ${queue.currentIndex + 1}/${queue.total}`
    : actionButtonLabel("jetson:pull", selectedPullPaths.length > 1 ? "Pull Selected in Order" : "Pull Selected Sequence", "Starting Pull...");
  const sequenceOptions = sequences
    .map((sequence) => `<option value="${esc(sequence.path)}">${esc(sequence.name)} - ${esc(sequence.modified || sequence.path)}</option>`)
    .join("");
  const sequenceList = sequences
    .slice(0, 12)
    .map(
      (sequence) => {
        const rowState = jetsonPullRowState(sequence.path, selectedPullPaths);
        return `
        <div class="mini-row selectable-row">
          <div>
            <strong><label><input type="checkbox" ${selectedPullPaths.includes(sequence.path) ? "checked" : ""} ${pullBusy ? "disabled" : ""} onchange="toggleJetsonPullSelection(${js(sequence.path)}, this.checked)" /> ${esc(sequence.name)}</label></strong>
            <div class="path" title="${esc(sequence.path)}">${esc(sequence.path)}</div>
          </div>
          <div class="actions">
            <button onclick="useJetsonRosbag(${js(sequence.path)})" ${pullBusy ? "disabled" : ""}>Use</button>
            <button class="primary ${rowState.className}" onclick="pullJetsonRosbag(${js(sequence.path)})" ${rowState.disabled ? `disabled title="${esc(rowState.title)}"` : ""}>${esc(rowState.label)}</button>
          </div>
        </div>`;
      },
    )
    .join("");
  return `
    <div class="transfer-grid">
      <section class="transfer-card">
        <h3>Pull rosbag sequences</h3>
        <div class="form-grid">
          <div class="field full">
            <label>Discovered by metadata.yaml</label>
            <select id="pull-remote-select" onchange="useJetsonRosbag(this.value)" ${pullBusy ? "disabled" : ""}>
              <option value="">Select inspected sequence</option>
              ${sequenceOptions}
            </select>
          </div>
          <div class="field full"><label>From Jetson</label><input id="pull-remote" value="" placeholder="${esc(config.jetson_record_root || "")}/<sequence>" ${pullBusy ? "disabled" : ""} /></div>
          <div class="field full"><label>To notebook</label><input id="pull-local" value="${esc(config.record_root || "")}" ${pullBusy ? "disabled" : ""} /></div>
          <div class="transfer-queue-summary full">
            <span>${esc(queueText)}</span>
            <span class="spacer"></span>
            <button onclick="selectAllJetsonPulls()" ${sequences.length && !pullBusy ? "" : "disabled"}>Select All</button>
            <button onclick="clearJetsonPullSelection()" ${selectedPullPaths.length && !pullBusy ? "" : "disabled"}>Clear</button>
          </div>
          <div class="actions full">
            <button class="primary ${actionBusy("jetson:pull") ? "is-busy" : ""}" onclick="startJetsonPull()" ${queueRunning ? "disabled" : ""} ${actionButtonAttrs("jetson:pull", "Pull transfer is starting...")}>${esc(primaryPullLabel)}</button>
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
            <button class="primary ${actionBusy("jetson:push") ? "is-busy" : ""}" onclick="startJetsonPush()" ${actionButtonAttrs("jetson:push", "Push transfer is starting...")}>${esc(actionButtonLabel("jetson:push", "Start Push", "Starting Push..."))}</button>
            <button onclick="copyPushCommand()">Copy Command</button>
          </div>
        </div>
      </section>
    </div>
  `;
}

function renderTerminalPage() {
  const commandRunner = state.config?.custom_commands_enabled
    ? `
      <section class="panel">
        <div class="panel-header"><h2>Run Command</h2></div>
        <div class="panel-body">
          <div class="form-grid">
            <div class="field"><label>Title</label><input id="custom-title" value="Custom command" /></div>
            <div class="field"><label>Working directory</label><input id="custom-cwd" value="${esc(state.config?.repo_root || "")}" /></div>
            <div class="field full"><label>Command</label><textarea id="custom-command" placeholder="echo hello"></textarea></div>
            <div class="actions full"><button class="primary ${actionBusy("custom-command:run") ? "is-busy" : ""}" onclick="runCustomCommand()" ${actionButtonAttrs("custom-command:run", "Command is starting...")}>${esc(actionButtonLabel("custom-command:run", "Run", "Starting..."))}</button><button onclick="copyText($('custom-command').value)">Copy</button></div>
          </div>
        </div>
      </section>`
    : `
      <section class="panel">
        <div class="panel-header"><h2>Command Execution Disabled</h2></div>
        <div class="panel-body">
          <div class="notice">The Console does not accept arbitrary commands by default. Task history and logs remain available below.</div>
        </div>
      </section>`;
  return `
    <div class="page">
      ${commandRunner}
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
                  <button class="danger ${actionBusy(`task:stop:${task.task_id}`) ? "is-busy" : ""}" onclick="stopTask(${js(task.task_id)})" ${["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"} ${actionButtonAttrs(`task:stop:${task.task_id}`, "Stop request is being sent...")}>${esc(actionButtonLabel(`task:stop:${task.task_id}`, "Stop", "Stopping..."))}</button>
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
        <button class="danger ${actionBusy(`task:stop:${task?.task_id || ""}`) ? "is-busy" : ""}" onclick="stopTask(${js(task?.task_id || "")})" ${task && ["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"} ${actionButtonAttrs(`task:stop:${task?.task_id || ""}`, "Stop request is being sent...")}>${esc(actionButtonLabel(`task:stop:${task?.task_id || ""}`, "Stop", "Stopping..."))}</button>
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
            <button class="danger ${actionBusy(`task:stop:${task?.task_id || ""}`) ? "is-busy" : ""}" onclick="stopTask(${js(task?.task_id || "")})" ${task && ["running", "queued", "stopping"].includes(task.status) ? "" : "disabled"} ${actionButtonAttrs(`task:stop:${task?.task_id || ""}`, "Stop request is being sent...")}>${esc(actionButtonLabel(`task:stop:${task?.task_id || ""}`, "Stop", "Stopping..."))}</button>
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
  state.stream = new EventSource(`/api/tasks/${encodeURIComponent(taskId)}/stream?tail=${LIVE_LOG_INITIAL_TAIL_LINES}`);
  state.stream.onmessage = (event) => {
    const payload = JSON.parse(event.data);
    const chunk = payload.chunk || "";
    const append = Boolean(state.logText && chunk);
    appendLiveLog(chunk);
    if (payload.task) {
      const index = state.tasks.findIndex((item) => item.task_id === payload.task.task_id);
      const previous = index >= 0 ? state.tasks[index] : null;
      if (index >= 0) state.tasks[index] = payload.task;
      if (mapTaskFinishedSince(previous, payload.task)) {
        invalidatePreflightsForTask(payload.task);
        scheduleVisiblePreflights({ force: true });
        refreshSelectedMapData({ preserveViewport: true }).catch(() => {});
      }
      refreshVisiblePreflightDom();
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
    const text = await apiText(`/api/tasks/${encodeURIComponent(taskId)}/log`);
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
  const task = taskById(taskId);
  if (!confirmAction({
    title: "Stop this task?",
    target: task?.title || taskId,
    detail: "The Console will send a stop request to the task process group.",
    destructive: true,
  })) return;
  const actionKey = `task:stop:${taskId}`;
  if (!beginAction(actionKey, "Stopping task")) return;
  try {
    await api(`/api/tasks/${encodeURIComponent(taskId)}/stop`, { method: "POST", body: "{}" });
    await refreshAll();
    toast("Stop request sent");
  } catch (error) {
    toast(`Stop failed: ${error.message}`, "error");
  } finally {
    endAction(actionKey);
  }
}

async function runCustomCommand() {
  if (!state.config?.custom_commands_enabled) {
    toast("Custom command execution is disabled", "error");
    return;
  }
  const payload = {
    title: $("custom-title").value,
    kind: "custom",
    cwd: $("custom-cwd").value,
    command: $("custom-command").value,
  };
  if (!confirmAction({
    title: "Run this custom command?",
    target: payload.cwd,
    detail: payload.command,
    destructive: true,
  })) return;
  const actionKey = "custom-command:run";
  if (!beginAction(actionKey, "Starting command")) return;
  try {
    const result = await api("/api/tasks/run", { method: "POST", body: JSON.stringify(payload) });
    await refreshAll();
    selectTask(result.task.task_id);
  } catch (error) {
    toast(`Command failed: ${error.message}`, "error");
  } finally {
    endAction(actionKey);
  }
}

function readNumberInput(id, fallback) {
  const value = Number($(id)?.value);
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function readFpvForm(updateState = true) {
  const codec = $("fpv-codec")?.value || state.fpv.codec;
  const transport = $("fpv-transport")?.value || state.fpv.transport || "mjpeg";
  const next = {
    ...state.fpv,
    host: $("fpv-host") ? $("fpv-host").value.trim() : state.fpv.host,
    codec,
    width: readNumberInput("fpv-width", state.fpv.width),
    height: readNumberInput("fpv-height", state.fpv.height),
    fps: readNumberInput("fpv-fps", state.fpv.fps),
    port: readNumberInput("fpv-port", state.fpv.port),
    payload: codec === "mjpeg" ? 26 : readNumberInput("fpv-payload", state.fpv.payload),
    transport,
    displaySink: $("fpv-display-sink")?.value || state.fpv.displaySink,
    noDisplay: Boolean($("fpv-no-display")?.checked ?? state.fpv.noDisplay),
    browserStatus: state.fpv.browserStatus,
    webrtcPlaying: state.fpv.webrtcPlaying,
    webrtcClientState: state.fpv.webrtcClientState,
    webrtcClientError: state.fpv.webrtcClientError,
    webrtcStats: state.fpv.webrtcStats,
  };
  if (updateState) state.fpv = next;
  return next;
}

function handleFpvCodecChange() {
  const codec = $("fpv-codec")?.value || state.fpv.codec;
  const payload = $("fpv-payload");
  if (payload) {
    if (codec === "mjpeg") {
      payload.value = "26";
      payload.disabled = true;
      payload.title = "JPEG RTP uses payload type 26";
    } else {
      if (payload.value === "26") payload.value = "96";
      payload.disabled = false;
      payload.title = "";
    }
  }
  const transport = $("fpv-transport");
  if (transport && codec !== "h264" && transport.value === "webrtc") {
    transport.value = "mjpeg";
    toast("WebRTC passthrough currently supports H.264; switched to MJPEG compatibility mode");
  }
  updateFpvCommandPreview();
}

function handleFpvTransportChange() {
  const transport = $("fpv-transport")?.value || "mjpeg";
  const codec = $("fpv-codec")?.value || state.fpv.codec;
  if (transport === "webrtc" && codec !== "h264") {
    $("fpv-transport").value = "mjpeg";
    toast("Select H.264 before enabling WebRTC passthrough", "error");
  }
  readFpvForm();
  render();
}

function fpvDestinationIssue(fpv = state.fpv) {
  const host = String(fpv.host || "").trim();
  if (!host) return "Notebook receiver IP is required";
  if (/\s|[\x00-\x1f\x7f]/.test(host)) return "Receiver host must not contain whitespace";
  const lower = host.toLowerCase();
  if (lower === "localhost" || lower === "::1" || lower === "[::1]" || lower.startsWith("127.")) {
    return "Loopback addresses cannot send video to a remote notebook";
  }
  const port = Number(fpv.port);
  if (!Number.isInteger(port) || port < 1 || port > 65535) {
    return "RTP port must be between 1 and 65535";
  }
  return "";
}

function fpvBrowserStatusText(status = state.fpv.browserStatus || {}) {
  const transport = fpvStatusTransport(status);
  if (transport === "webrtc") {
    if (!status.webrtc?.available) {
      return `WebRTC unavailable · ${status.webrtc?.error || "required GStreamer components are missing"}`;
    }
    if (state.fpv.webrtcClientError) return `WebRTC error · ${state.fpv.webrtcClientError}`;
    if (status.running && !fpvWebRtcSessionIsOwned(status)) {
      return "WebRTC session is open in another page · stop it there or wait for its lease to expire";
    }
    if (fpvWebRtcPlaybackIsStalled(status)) {
      return `WebRTC playback stalled · RTP is still arriving · ${Number(status.rtp_packet_count || 0).toLocaleString()} packets received by Console`;
    }
    if (fpvBackendMediaIsStalled(status)) {
      return `RTP stalled · no packet for ${Number(fpvLastMediaAge(status)).toFixed(1)}s · ${Number(status.rtp_packet_count || 0).toLocaleString()} packets`;
    }
    if (status.running && !Number(status.rtp_packet_count || 0)) {
      const phase = status.webrtc?.phase || state.fpv.webrtcClientState;
      return `${phase === "connected" ? "WebRTC connected" : "WebRTC connecting"} · waiting for H.264 RTP on UDP ${status.settings?.port || state.fpv.port}`;
    }
    if (status.running) {
      const stats = state.fpv.webrtcStats || {};
      const decoded = Number(stats.framesDecoded || 0).toLocaleString();
      const received = Number(stats.packetsReceived || 0).toLocaleString();
      const lost = Number(stats.packetsLost || 0).toLocaleString();
      const jitter = Number.isFinite(Number(stats.averageJitterBufferMs))
        ? ` · jitter buffer ${Number(stats.averageJitterBufferMs).toFixed(1)}ms`
        : "";
      const age = Number.isFinite(Number(fpvLastMediaAge(status)))
        ? ` · last RTP ${Number(fpvLastMediaAge(status)).toFixed(1)}s ago`
        : "";
      return `WebRTC ${state.fpv.webrtcClientState || status.webrtc?.phase || "connecting"} · ${decoded} frames decoded · ${received} packets received · ${lost} lost${jitter}${age}`;
    }
  }
  if (!status.available) return "GStreamer is not available on the Console host";
  if (fpvBrowserIsStalled(status)) {
    return `RTP stalled · no new frame for ${Number(status.last_frame_age_s).toFixed(1)}s · ${Number(status.frame_count).toLocaleString()} frames received`;
  }
  if (status.running && !status.frame_count) {
    return `Waiting for RTP packets on UDP ${status.settings?.port || state.fpv.port}`;
  }
  if (status.frame_count) {
    const output = status.settings
      ? `${status.settings.output_width}x${status.settings.output_height}@${status.settings.output_fps}`
      : "browser stream";
    const age = Number.isFinite(Number(status.last_frame_age_s))
      ? ` · last frame ${Number(status.last_frame_age_s).toFixed(1)}s ago`
      : "";
    return `${Number(status.frame_count).toLocaleString()} frames · ${fmtBytes(status.jpeg_bytes)} · ${output}${age}`;
  }
  if (status.last_error) return "Receiver stopped with an error";
  return "Ready to receive on the notebook";
}

function fpvBrowserIsStalled(status = state.fpv.browserStatus || {}) {
  return fpvBackendMediaIsStalled(status) || fpvWebRtcPlaybackIsStalled(status);
}

function fpvBackendMediaIsStalled(status = state.fpv.browserStatus || {}) {
  const transport = fpvStatusTransport(status);
  const mediaCount = transport === "webrtc"
    ? Number(status.rtp_packet_count || 0)
    : Number(status.frame_count || 0);
  const mediaAge = fpvLastMediaAge(status);
  return Boolean(
    status.running
    && mediaCount > 0
    && Number.isFinite(Number(mediaAge))
    && Number(mediaAge) > 3,
  );
}

function fpvWebRtcPlaybackIsStalled(status = state.fpv.browserStatus || {}) {
  if (!fpvWebRtcSessionIsOwned(status)) return false;
  if (Number(status.rtp_packet_count || 0) <= 0 || fpvBackendMediaIsStalled(status)) return false;
  const lastProgressAt = Number(state.fpv.webrtcLastProgressAtMs || state.fpv.webrtcRtpObservedAtMs || 0);
  return lastProgressAt > 0 && Date.now() - lastProgressAt > 3000;
}

function fpvWebRtcSessionIsOwned(status = state.fpv.browserStatus || {}) {
  return Boolean(
    status.running
    && fpvStatusTransport(status) === "webrtc"
    && fpvPeerConnection
    && fpvPeerConnection.connectionState !== "closed"
    && fpvPeerSessionId
    && fpvPeerSessionId === status.session_id,
  );
}

function fpvReceiverCanAutoStop(status = state.fpv.browserStatus || {}) {
  if (!status.running) return false;
  return fpvStatusTransport(status) !== "webrtc" || fpvWebRtcSessionIsOwned(status);
}

function fpvBrowserStallMessage(status = state.fpv.browserStatus || {}) {
  if (fpvWebRtcPlaybackIsStalled(status)) {
    return "WebRTC video stopped while RTP is still arriving";
  }
  const age = Number(fpvLastMediaAge(status));
  const mediaLabel = fpvStatusTransport(status) === "webrtc" ? "RTP packet" : "decoded frame";
  return Number.isFinite(age) ? `No new ${mediaLabel} for ${age.toFixed(1)}s` : "RTP input stopped";
}

function noteFpvWebRtcRtpObserved(status = state.fpv.browserStatus || {}) {
  if (
    status.running
    && fpvStatusTransport(status) === "webrtc"
    && Number(status.rtp_packet_count || 0) > 0
    && !state.fpv.webrtcRtpObservedAtMs
  ) {
    state.fpv.webrtcRtpObservedAtMs = Date.now();
  }
}

function fpvLastMediaAge(status = state.fpv.browserStatus || {}) {
  const transport = fpvStatusTransport(status);
  return transport === "webrtc" ? status.last_packet_age_s : status.last_frame_age_s;
}

function fpvStatusTransport(status = state.fpv.browserStatus || {}) {
  if (!status.running) return state.fpv.transport || "mjpeg";
  return status.settings?.transport || status.transport || state.fpv.transport || "mjpeg";
}

function fpvBrowserPayload(fpv = state.fpv) {
  return {
    codec: fpv.codec,
    width: fpv.width,
    height: fpv.height,
    fps: fpv.fps,
    port: fpv.port,
    payload: fpv.payload,
    transport: fpv.transport || "mjpeg",
  };
}

async function startBrowserFpv() {
  if (state.fpv.starting || state.fpv.browserStatus?.running) return;
  const external = state.tasks.find((task) => task.kind === "fpv-viewer" && ["queued", "running", "stopping"].includes(task.status));
  if (external) {
    toast("Stop the external RTP viewer before starting the browser view", "error");
    return;
  }
  const fpv = readFpvForm();
  if (!confirmAction({
    title: `Start ${fpv.transport === "webrtc" ? "WebRTC" : "MJPEG"} browser receiver?`,
    target: `UDP ${fpv.port} / ${fpv.codec} ${fpv.width}x${fpv.height}@${fpv.fps}`,
    detail: "Keep other viewers stopped while the browser owns this RTP port.",
  })) return;
  closeFpvPeerConnection();
  const lifecycleGeneration = fpvLifecycleGeneration;
  resetFpvWebRtcPlaybackState();
  state.fpv.starting = true;
  state.fpv.webrtcClientState = "closed";
  state.fpv.webrtcClientError = "";
  if (state.tab === "fpv") render();
  let startedSessionId = "";
  try {
    const result = await api("/api/fpv/start", {
      method: "POST",
      body: JSON.stringify(fpvBrowserPayload(fpv)),
    });
    startedSessionId = result.fpv?.session_id || "";
    if (lifecycleGeneration !== fpvLifecycleGeneration) {
      await stopFpvSessionSilently(startedSessionId);
      return;
    }
    state.fpv.browserStatus = result.fpv || {};
    state.fpv.starting = false;
    noteFpvWebRtcRtpObserved(state.fpv.browserStatus);
    if (state.tab === "fpv") render();
    if (fpv.transport === "webrtc") {
      const connected = await connectBrowserFpvWebRtc(startedSessionId, lifecycleGeneration);
      if (!connected || lifecycleGeneration !== fpvLifecycleGeneration) {
        await stopFpvSessionSilently(startedSessionId);
        return;
      }
      toast("WebRTC receiver started");
    } else {
      toast("MJPEG browser receiver started");
    }
  } catch (error) {
    if (lifecycleGeneration !== fpvLifecycleGeneration) {
      await stopFpvSessionSilently(startedSessionId);
      return;
    }
    const message = error instanceof Error ? error.message : String(error);
    const cleanedUp = await stopFailedFpvSession(startedSessionId, lifecycleGeneration, message);
    if (cleanedUp) toast(`Could not start browser receiver: ${message}`, "error");
  }
}

async function stopBrowserFpv({ silent = false, renderAfter = true } = {}) {
  const previousStatus = state.fpv.browserStatus || {};
  const sessionId = state.fpv.browserStatus?.session_id || "";
  if (!silent && sessionId && !confirmAction({
    title: "Stop browser RTP receiver?",
    target: `UDP ${previousStatus.settings?.port || state.fpv.port}`,
    detail: "The embedded FPV view will stop receiving frames.",
  })) return;
  closeFpvPeerConnection();
  const lifecycleGeneration = fpvLifecycleGeneration;
  resetFpvWebRtcPlaybackState();
  state.fpv.starting = false;
  state.fpv.webrtcClientState = "closed";
  state.fpv.webrtcClientError = "";
  state.fpv.browserStatus = stoppedFpvBrowserStatus(previousStatus);
  if (!sessionId) {
    if (renderAfter && state.tab === "fpv") render();
    return;
  }
  try {
    const result = await api("/api/fpv/stop", {
      method: "POST",
      body: JSON.stringify({ session_id: sessionId }),
    });
    if (lifecycleGeneration !== fpvLifecycleGeneration) return;
    state.fpv.browserStatus = result.fpv || { available: true, running: false };
    if (renderAfter && state.tab === "fpv") render();
    if (!silent) toast("Browser RTP receiver stopped");
  } catch (error) {
    if (lifecycleGeneration !== fpvLifecycleGeneration) return;
    state.fpv.webrtcClientError = `Could not stop browser receiver: ${error.message}`;
    if (renderAfter && state.tab === "fpv") render();
    if (!silent) toast(`Could not stop browser receiver: ${error.message}`, "error");
  }
}

async function stopFpvSessionSilently(sessionId) {
  if (!sessionId) return null;
  try {
    return await api("/api/fpv/stop", {
      method: "POST",
      body: JSON.stringify({ session_id: sessionId }),
    });
  } catch {
    return null;
  }
}

async function stopFailedFpvSession(sessionId, lifecycleGeneration, message) {
  if (lifecycleGeneration !== fpvLifecycleGeneration) return false;
  const previousStatus = state.fpv.browserStatus || {};
  closeFpvPeerConnection();
  const cleanupGeneration = fpvLifecycleGeneration;
  resetFpvWebRtcPlaybackState();
  state.fpv.starting = false;
  state.fpv.webrtcClientState = "failed";
  state.fpv.webrtcClientError = message;
  state.fpv.browserStatus = stoppedFpvBrowserStatus(previousStatus);
  updateFpvBrowserStatusDom();
  const result = await stopFpvSessionSilently(sessionId);
  if (cleanupGeneration !== fpvLifecycleGeneration) return false;
  if (result?.fpv) state.fpv.browserStatus = result.fpv;
  state.fpv.webrtcClientError = message;
  if (state.tab === "fpv") render();
  return true;
}

function stoppedFpvBrowserStatus(status = {}) {
  return {
    ...status,
    running: false,
    session_id: "",
    last_error: "",
  };
}

function resetFpvWebRtcPlaybackState() {
  state.fpv.webrtcPlaying = false;
  state.fpv.webrtcStats = null;
  state.fpv.webrtcLastProgressAtMs = 0;
  state.fpv.webrtcLastFramesDecoded = 0;
  state.fpv.webrtcLastVideoTime = 0;
  state.fpv.webrtcRtpObservedAtMs = 0;
}

function fpvMediaElement() {
  return $("fpv-browser-video") || $("fpv-browser-image");
}

function closeFpvPeerConnection({ invalidate = true } = {}) {
  if (invalidate) fpvLifecycleGeneration += 1;
  const connection = fpvPeerConnection;
  fpvPeerConnection = null;
  fpvPeerSessionId = "";
  fpvRemoteStream = null;
  const video = $("fpv-browser-video");
  if (video) video.srcObject = null;
  if (!connection) return;
  connection.ontrack = null;
  connection.onconnectionstatechange = null;
  connection.oniceconnectionstatechange = null;
  try {
    connection.close();
  } catch {
    // A connection which already failed may throw while closing on older browsers.
  }
}

function fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration) {
  return Boolean(
    lifecycleGeneration === fpvLifecycleGeneration
    && connection === fpvPeerConnection
    && sessionId === fpvPeerSessionId,
  );
}

function attachFpvRemoteStream() {
  const video = $("fpv-browser-video");
  if (!video || !fpvRemoteStream) return;
  if (video.srcObject !== fpvRemoteStream) video.srcObject = fpvRemoteStream;
  video.play().catch(() => {});
}

function waitForFpvIceGathering(connection, timeoutMs = 8000) {
  if (connection.iceGatheringState === "complete") return Promise.resolve();
  return new Promise((resolve, reject) => {
    const timeout = window.setTimeout(() => {
      connection.removeEventListener("icegatheringstatechange", onChange);
      reject(new Error("browser ICE gathering timed out"));
    }, timeoutMs);
    const onChange = () => {
      if (connection.iceGatheringState !== "complete") return;
      window.clearTimeout(timeout);
      connection.removeEventListener("icegatheringstatechange", onChange);
      resolve();
    };
    connection.addEventListener("icegatheringstatechange", onChange);
  });
}

async function connectBrowserFpvWebRtc(sessionId, lifecycleGeneration) {
  if (!window.RTCPeerConnection) throw new Error("this browser does not support WebRTC");
  if (!sessionId) throw new Error("WebRTC session was not created");
  if (lifecycleGeneration !== fpvLifecycleGeneration) return false;

  const connection = new RTCPeerConnection({ iceServers: [], bundlePolicy: "max-bundle" });
  fpvPeerConnection = connection;
  fpvPeerSessionId = sessionId;
  state.fpv.webrtcClientState = "connecting";

  connection.ontrack = (event) => {
    if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return;
    fpvRemoteStream = event.streams[0] || new MediaStream([event.track]);
    attachFpvRemoteStream();
  };
  connection.onconnectionstatechange = () => {
    if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return;
    state.fpv.webrtcClientState = connection.connectionState;
    if (connection.connectionState === "failed") {
      state.fpv.webrtcPlaying = false;
      void stopFailedFpvSession(sessionId, lifecycleGeneration, "WebRTC connection failed");
      return;
    }
    if (connection.connectionState === "closed") {
      state.fpv.webrtcPlaying = false;
    }
    updateFpvBrowserStatusDom();
  };
  connection.oniceconnectionstatechange = () => {
    if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return;
    if (connection.iceConnectionState === "failed") {
      void stopFailedFpvSession(sessionId, lifecycleGeneration, "ICE connection failed");
    }
  };

  const offerResult = await api("/api/fpv/webrtc/offer", {
    method: "POST",
    body: JSON.stringify({ session_id: sessionId }),
  });
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  const offer = offerResult.offer || {};
  if (offer.type !== "offer" || !offer.sdp) throw new Error("Console returned an invalid WebRTC offer");
  await connection.setRemoteDescription(offer);
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  const answer = await connection.createAnswer();
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  await connection.setLocalDescription(answer);
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  await waitForFpvIceGathering(connection);
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  const local = connection.localDescription;
  if (!local?.type || !local?.sdp) throw new Error("browser did not create a complete WebRTC answer");
  const answerResult = await api("/api/fpv/webrtc/answer", {
    method: "POST",
    body: JSON.stringify({
      session_id: sessionId,
      type: local.type,
      sdp: local.sdp,
    }),
  });
  if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return false;
  if (answerResult.fpv) state.fpv.browserStatus = answerResult.fpv;
  updateFpvBrowserStatusDom();
  return true;
}

function handleFpvBrowserVideoPlaying() {
  state.fpv.webrtcPlaying = true;
  state.fpv.webrtcClientError = "";
  state.fpv.webrtcLastProgressAtMs = Date.now();
  const video = $("fpv-browser-video");
  if (video) state.fpv.webrtcLastVideoTime = Number(video.currentTime || 0);
  const placeholder = $("fpv-video-placeholder");
  if (placeholder) placeholder.classList.remove("visible");
  updateFpvBrowserStatusDom();
}

function handleFpvBrowserVideoWaiting() {
  state.fpv.webrtcPlaying = false;
  updateFpvBrowserStatusDom();
}

function handleFpvBrowserImageLoad() {
  const image = $("fpv-browser-image");
  if (image) image.classList.add("loaded");
  const placeholder = $("fpv-video-placeholder");
  if (placeholder) placeholder.classList.remove("visible");
}

function handleFpvBrowserImageError() {
  const placeholder = $("fpv-video-placeholder");
  if (placeholder) {
    placeholder.classList.add("visible");
    const title = placeholder.querySelector("strong");
    if (title) title.textContent = "The browser stream stopped";
  }
}

function updateFpvBrowserStatusDom() {
  const status = state.fpv.browserStatus || {};
  const badge = $("fpv-browser-badge");
  const stalled = fpvBrowserIsStalled(status);
  const transport = fpvStatusTransport(status);
  const usesWebRtc = transport === "webrtc";
  const unownedWebRtc = usesWebRtc && status.running && !fpvWebRtcSessionIsOwned(status);
  const hasMedia = usesWebRtc
    ? Boolean(state.fpv.webrtcPlaying || Number(state.fpv.webrtcStats?.framesDecoded || 0) > 0)
    : Number(status.frame_count || 0) > 0;
  const clientFailed = Boolean(state.fpv.webrtcClientError);
  if (badge) {
    badge.className = `status ${clientFailed || status.last_error ? "failed" : unownedWebRtc ? "running" : stalled ? "stopping" : status.running ? "running" : "idle"}`;
    badge.textContent = clientFailed || status.last_error
      ? "ERROR"
      : status.running
        ? (unownedWebRtc ? "OTHER TAB" : stalled ? "STALLED" : hasMedia ? "LIVE" : usesWebRtc && state.fpv.webrtcClientState === "connecting" ? "CONNECTING" : "WAITING")
        : "STOPPED";
  }
  const stats = $("fpv-browser-stats");
  if (stats) stats.textContent = fpvBrowserStatusText(status);
  const error = $("fpv-browser-error");
  if (error) {
    error.textContent = state.fpv.webrtcClientError || status.last_error || "";
    error.hidden = !state.fpv.webrtcClientError && !status.last_error;
  }
  const placeholder = $("fpv-video-placeholder");
  if (placeholder) {
    const waiting = status.running && !hasMedia;
    placeholder.classList.toggle("visible", Boolean(!status.running || waiting || stalled || clientFailed));
    const title = placeholder.querySelector("strong");
    if (title) {
      title.textContent = clientFailed
        ? state.fpv.webrtcClientError
        : unownedWebRtc
          ? "WebRTC session is open in another page"
          : stalled
            ? fpvBrowserStallMessage(status)
            : waiting
              ? `${usesWebRtc && state.fpv.webrtcClientState === "connecting" ? "Connecting WebRTC and waiting" : "Waiting"} for RTP on UDP ${status.settings?.port || state.fpv.port}...`
              : !status.running
                ? "Browser receiver is stopped"
                : "";
    }
  }
}

async function pollFpvBrowserStatus() {
  if (fpvHeartbeatBusy || state.tab !== "fpv" || !state.fpv.browserStatus?.running) return;
  fpvHeartbeatBusy = true;
  const previousSession = state.fpv.browserStatus.session_id;
  const lifecycleGeneration = fpvLifecycleGeneration;
  try {
    const shouldRenewLease = fpvStatusTransport(state.fpv.browserStatus) !== "webrtc"
      || fpvWebRtcSessionIsOwned(state.fpv.browserStatus);
    if (shouldRenewLease) {
      await api("/api/fpv/heartbeat", {
        method: "POST",
        body: JSON.stringify({ session_id: previousSession }),
      });
    }
    if (
      lifecycleGeneration !== fpvLifecycleGeneration
      || previousSession !== state.fpv.browserStatus?.session_id
    ) return;
    const result = await api("/api/fpv/status");
    if (
      lifecycleGeneration !== fpvLifecycleGeneration
      || previousSession !== state.fpv.browserStatus?.session_id
    ) return;
    const next = result.fpv || {};
    state.fpv.browserStatus = next;
    noteFpvWebRtcRtpObserved(next);
    if (!next.running || next.session_id !== previousSession) {
      closeFpvPeerConnection();
      resetFpvWebRtcPlaybackState();
      render();
      return;
    }
    await refreshFpvWebRtcStats();
    updateFpvBrowserStatusDom();
  } catch {
    if (lifecycleGeneration !== fpvLifecycleGeneration) return;
    const result = await api("/api/fpv/status").catch(() => null);
    if (lifecycleGeneration !== fpvLifecycleGeneration) return;
    if (result?.fpv) {
      state.fpv.browserStatus = result.fpv;
      noteFpvWebRtcRtpObserved(result.fpv);
      if (!result.fpv.running) {
        closeFpvPeerConnection();
        resetFpvWebRtcPlaybackState();
        render();
      }
    }
  } finally {
    fpvHeartbeatBusy = false;
  }
}

async function refreshFpvWebRtcStats() {
  const connection = fpvPeerConnection;
  const sessionId = fpvPeerSessionId;
  const lifecycleGeneration = fpvLifecycleGeneration;
  if (!connection || connection.connectionState === "closed") return;
  try {
    const reports = await connection.getStats();
    if (!fpvPeerIsCurrent(connection, sessionId, lifecycleGeneration)) return;
    let inbound = null;
    reports.forEach((report) => {
      if (report.type === "inbound-rtp" && (report.kind === "video" || report.mediaType === "video")) {
        inbound = report;
      }
    });
    if (!inbound) return;
    const emitted = Number(inbound.jitterBufferEmittedCount || 0);
    const hasDecodedFrameCounter = inbound.framesDecoded != null && Number.isFinite(Number(inbound.framesDecoded));
    const framesDecoded = hasDecodedFrameCounter ? Number(inbound.framesDecoded) : 0;
    const video = $("fpv-browser-video");
    const videoTime = Number(video?.currentTime || 0);
    if (
      framesDecoded > Number(state.fpv.webrtcLastFramesDecoded || 0)
      || (
        !hasDecodedFrameCounter
        && videoTime > Number(state.fpv.webrtcLastVideoTime || 0) + 0.01
      )
    ) {
      state.fpv.webrtcLastProgressAtMs = Date.now();
    }
    state.fpv.webrtcLastFramesDecoded = framesDecoded;
    state.fpv.webrtcLastVideoTime = videoTime;
    state.fpv.webrtcStats = {
      framesDecoded,
      framesDropped: Number(inbound.framesDropped || 0),
      packetsReceived: Number(inbound.packetsReceived || 0),
      packetsLost: Number(inbound.packetsLost || 0),
      bytesReceived: Number(inbound.bytesReceived || 0),
      jitterSeconds: Number(inbound.jitter || 0),
      averageJitterBufferMs: emitted > 0
        ? (Number(inbound.jitterBufferDelay || 0) / emitted) * 1000
        : 0,
    };
  } catch {
    // Stats are diagnostic only; a browser may temporarily reject during ICE changes.
  }
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
  const issue = fpvDestinationIssue(fpv);
  if (issue) return `# ${issue}. Select an IP candidate or enter it above.`;
  return [
    "ros2 launch jetpilot_system_launch bringup.launch.py",
    "enable_sensor_kit:=true",
    "sensor_kit_enable_rtp_stream:=true",
    `sensor_kit_rtp_host:=${sh(fpv.host)}`,
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
  if (!state.config?.custom_commands_enabled) {
    toast("Starting the viewer is disabled; copy and run the command locally", "error");
    return;
  }
  if (state.fpv.browserStatus?.running) {
    toast("Stop the browser RTP view before starting the external viewer", "error");
    return;
  }
  const fpv = readFpvForm();
  const command = buildFpvReceiverCommand(fpv);
  if (!confirmAction({
    title: "Open external RTP viewer?",
    target: `UDP ${fpv.port} / ${fpv.codec} ${fpv.width}x${fpv.height}@${fpv.fps}`,
    detail: command,
  })) return;
  if (!beginAction("fpv:external", "Starting external viewer")) return;
  try {
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
  } catch (error) {
    toast(`FPV viewer failed: ${error.message}`, "error");
  } finally {
    endAction("fpv:external");
  }
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
  const issue = fpvDestinationIssue(fpv);
  if (issue) {
    toast(issue, "error");
    return;
  }
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
  const destinationNotice = $("fpv-destination-notice");
  if (destinationNotice) {
    const issue = fpvDestinationIssue(fpv);
    destinationNotice.textContent = issue;
    destinationNotice.hidden = !issue;
  }
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

async function selectMapBuildRosbag(path) {
  const selectedPath = String(path || "");
  const manualInput = $("build-topic-config");
  if (manualInput) manualInput.value = "";
  if (!selectedPath) {
    updateCameraTopicPreview();
    scheduleMapBuildPreflight({ immediate: true, force: true });
    return;
  }

  try {
    const detail = await api(apiPath("/api/rosbags/detail", { path: selectedPath }));
    if ($("build-rosbag")?.value !== selectedPath) return;
    const selectedConfig = (
      cameraTopicConfigForTopics((detail.rosbag || detail)?.topics || [])
      || defaultCameraTopicConfig()
    );
    const configSelect = $("build-topic-config-select");
    if (configSelect) configSelect.value = selectedConfig?.path || "";
    updateCameraTopicPreview();
  } catch (error) {
    toast(`Camera config detection failed: ${error.message}`, "error");
  } finally {
    if ($("build-rosbag")?.value === selectedPath) {
      scheduleMapBuildPreflight({ immediate: true, force: true });
    }
  }
}

async function useRosbag(path) {
  state.tab = "map-builder";
  render();
  $("build-rosbag").value = path;
  fillMapDir();
  await selectMapBuildRosbag(path);
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
  scheduleMapBuildPreflight();
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
  scheduleMapBuildPreflight();
}

async function startMapBuild() {
  const payload = mapBuildPreflightPayload();
  const mapDir = payload.map_dir;
  if (!mapDir) {
    window.alert("Map name is required.");
    return;
  }
  if (!confirmAction({
    title: "Start VGL/VSLAM map build?",
    target: mapDir,
    detail: `Rosbag: ${payload.rosbag || "not selected"}\nSteps: ${payload.steps || "default"}`,
  })) return;
  if (!beginAction("map-build:start", "Starting map build")) return;
  if (!acquirePreflightExecution("map-build", payload)) {
    endAction("map-build:start");
    return;
  }
  try {
    if (!(await confirmPreflight("map-build", payload))) return;
    const result = await api("/api/maps/build-vgl-vslam", { method: "POST", body: JSON.stringify(payload) });
    if (result.preflight) cachePreflightResult("map-build", payload, result.preflight);
    rememberStartedTask(result.task, "map-build", payload);
    await refreshAll();
    selectTask(result.task.task_id);
  } catch (error) {
    if (captureMapTaskConflict(error)) return;
    if (capturePreflightError("map-build", payload, error)) {
      toast(preflightBlockingReason(preflightEntry("map-build", payload)), "error");
      return;
    }
    toast(`Map build failed: ${error.message}`, "error");
  } finally {
    releasePreflightExecution("map-build", payload);
    endAction("map-build:start", { renderAfter: state.tab === "map-builder" || state.tab === "dashboard" });
  }
}

function copyMapBuildCommand() {
  const rosbag = $("build-rosbag")?.value || "<rosbag>";
  const mapDir = outputMapDir({ placeholder: true });
  const topicConfig = selectedCameraTopicConfig();
  const steps = $("build-steps")?.value || "edex compute_poses cuvgl";
  const topicArg = topicConfig ? ` --topic-config ${sh(topicConfig)}` : "";
  copyText(`jetpilot_map build-vgl-vslam --rosbag ${sh(rosbag)} --map-dir ${sh(mapDir)} --steps ${sh(steps)}${topicArg}`);
}

function mapNameFromPath(mapPath) {
  const parts = trimTrailingSlash(mapPath).split("/").filter(Boolean);
  return parts[parts.length - 1] || "map";
}

function hdMapEditorCommand(mapPath) {
  const cleanPath = trimTrailingSlash(mapPath);
  const mapName = mapNameFromPath(cleanPath);
  const python = state.config?.python_bin || "python3";
  const pythonWs = trimTrailingSlash(state.config?.python_ws || "python_ws");
  return [
    sh(python),
    sh(`${pythonWs}/map_tools/hd_map_editor.py`),
    "--map-yaml",
    sh(`${cleanPath}/vslam_landmarks.yaml`),
    "--output",
    sh(`${cleanPath}/${mapName}_hd_map.yaml`),
    "--centerline-output",
    sh(`${cleanPath}/${mapName}_hd_map_centerline.csv`),
  ].join(" ");
}

function copyHdMapEditorCommand(mapPath) {
  copyText(hdMapEditorCommand(mapPath), "HD map editor command copied");
}

function cloneMapPoint(point) {
  return [Number(point?.[0] || 0), Number(point?.[1] || 0)];
}

function cloneMapPolyline(points = []) {
  return points
    .filter((point) => Array.isArray(point) && point.length >= 2)
    .map(cloneMapPoint)
    .filter((point) => Number.isFinite(point[0]) && Number.isFinite(point[1]));
}

function cloneEditorLane(lane = defaultEditorLane()) {
  return {
    id: lane.id || "lane_001",
    primary: lane.primary !== false,
    closed_loop: lane.closed_loop !== false,
    left_bound: cloneMapPolyline(lane.left_bound || []),
    right_bound: cloneMapPolyline(lane.right_bound || []),
    centerline: cloneMapPolyline(lane.centerline || []),
  };
}

function defaultEditorLane() {
  return {
    id: "lane_001",
    primary: true,
    closed_loop: true,
    left_bound: [],
    right_bound: [],
    centerline: [],
  };
}

function laneForEditorFromDetail(detail) {
  const lanes = detail?.hd_map?.lanes || [];
  const source = lanes.find((lane) => lane.primary) || lanes[0];
  if (!source) return defaultEditorLane();
  return cloneEditorLane({ ...source, id: source.id || "lane_001", primary: true });
}

function ensureMapEditor(detail, options = {}) {
  const mapPath = detail?.map?.path || "";
  if (!mapPath) return state.mapEditor;
  const sameMap = state.mapEditor.mapPath === mapPath;
  if (options.force || state.mapEditor.mapPath !== mapPath || !state.mapEditor.lanes.length) {
    const lane = laneForEditorFromDetail(detail);
    state.mapEditor = {
      ...state.mapEditor,
      mapPath,
      dirty: false,
      selected: null,
      dragging: null,
      zoom: sameMap ? state.mapEditor.zoom : 1,
      primaryLaneId: lane.id,
      lanes: [lane],
      undoStack: [],
      redoStack: [],
      dragSnapshot: null,
    };
  }
  return state.mapEditor;
}

function activeEditorLane() {
  if (!state.mapEditor.lanes.length) state.mapEditor.lanes = [defaultEditorLane()];
  return state.mapEditor.lanes[0];
}

function captureMapEditorSnapshot() {
  return {
    lanes: state.mapEditor.lanes.map(cloneEditorLane),
    primaryLaneId: state.mapEditor.primaryLaneId,
    activeField: state.mapEditor.activeField,
    selected: state.mapEditor.selected ? { ...state.mapEditor.selected } : null,
    dirty: state.mapEditor.dirty,
  };
}

function restoreMapEditorSnapshot(snapshot) {
  if (!snapshot) return;
  state.mapEditor.lanes = (snapshot.lanes || []).map(cloneEditorLane);
  if (!state.mapEditor.lanes.length) state.mapEditor.lanes = [defaultEditorLane()];
  state.mapEditor.primaryLaneId = snapshot.primaryLaneId || state.mapEditor.lanes[0].id;
  state.mapEditor.activeField = MAP_EDITOR_FIELDS.includes(snapshot.activeField)
    ? snapshot.activeField
    : "left_bound";
  state.mapEditor.selected = snapshot.selected ? { ...snapshot.selected } : null;
  state.mapEditor.dragging = null;
  state.mapEditor.dragSnapshot = null;
  state.mapEditor.dirty = Boolean(snapshot.dirty);
}

function rememberMapEditorState() {
  state.mapEditor.undoStack.push(captureMapEditorSnapshot());
  if (state.mapEditor.undoStack.length > 200) state.mapEditor.undoStack.shift();
  state.mapEditor.redoStack = [];
}

function undoMapEditor() {
  if (!state.mapEditor.enabled || !state.mapEditor.undoStack.length) return;
  state.mapEditor.redoStack.push(captureMapEditorSnapshot());
  restoreMapEditorSnapshot(state.mapEditor.undoStack.pop());
  updateMapEditorChrome();
  drawMapPreview();
}

function redoMapEditor() {
  if (!state.mapEditor.enabled || !state.mapEditor.redoStack.length) return;
  state.mapEditor.undoStack.push(captureMapEditorSnapshot());
  restoreMapEditorSnapshot(state.mapEditor.redoStack.pop());
  updateMapEditorChrome();
  drawMapPreview();
}

function cloneSectionGate(gate) {
  return {
    id: gate.id || "gate_001",
    lane_id: gate.lane_id || "",
    s_m: Number(gate.s_m || 0),
    line: cloneMapPolyline(gate.line || []).slice(0, 2),
  };
}

function ensureSectionEditor(detail, options = {}) {
  const mapPath = detail?.map?.path || "";
  if (!mapPath) return state.sectionEditor;
  if (options.force || state.sectionEditor.mapPath !== mapPath) {
    state.sectionEditor = {
      ...state.sectionEditor,
      mapPath,
      dirty: false,
      selectedGateId: "",
      gates: (detail.hd_map?.section_gates || []).map(cloneSectionGate),
    };
  }
  return state.sectionEditor;
}

function sectionEditorLane(detail) {
  const lanes = detail?.hd_map?.lanes || [];
  return lanes.find((lane) => lane.primary) || lanes[0] || null;
}

function sectionGatesForDetail(detail) {
  if (!state.sectionEditor.enabled || state.sectionEditor.mapPath !== detail?.map?.path) return null;
  return state.sectionEditor.gates;
}

function updateSectionEditorChrome() {
  const detail = state.selectedMapDetail;
  if (!detail || state.sectionEditor.mapPath !== detail.map?.path) return;
  const lane = sectionEditorLane(detail);
  const ready = Boolean(detail.hd_map?.exists && lane?.centerline?.length >= 2);
  const status = $("section-editor-status");
  if (status) {
    status.textContent = !detail.hd_map?.exists ? "Need HD map" : !ready ? "Need centerline" : state.sectionEditor.dirty ? "Unsaved" : "Ready";
    status.className = ready ? (state.sectionEditor.dirty ? "dirty" : "ok") : "warn";
  }
  const save = $("section-editor-save");
  if (save) save.disabled = !(state.sectionEditor.enabled && ready);
  const del = $("section-editor-delete");
  if (del) del.disabled = !(state.sectionEditor.enabled && state.sectionEditor.selectedGateId);
  const counts = $("section-editor-counts");
  if (counts) counts.textContent = renderSectionEditorCounts(detail);
}

function toggleSectionEditor() {
  if (!state.selectedMapDetail) return;
  ensureSectionEditor(state.selectedMapDetail);
  state.sectionEditor.enabled = !state.sectionEditor.enabled;
  if (state.sectionEditor.enabled) {
    state.mapEditor.enabled = false;
    state.mapEditor.dragging = null;
    state.mapEditor.selected = null;
    state.mapLayers.centerline = true;
    state.mapLayers.section_gates = true;
    state.mapLayers.section_labels = true;
  }
  state.sectionEditor.selectedGateId = "";
  render();
}

function nextSectionGateId() {
  const existing = new Set(state.sectionEditor.gates.map((gate) => gate.id));
  let index = 1;
  while (existing.has(`gate_${String(index).padStart(3, "0")}`)) index += 1;
  return `gate_${String(index).padStart(3, "0")}`;
}

function projectPointToLane(point, lane) {
  const centerline = lane?.centerline || [];
  if (centerline.length < 2) return null;
  let best = null;
  let sBefore = 0;
  const segmentCount = lane.closed_loop && centerline.length >= 3 ? centerline.length : centerline.length - 1;
  for (let index = 0; index < segmentCount; index += 1) {
    const start = centerline[index];
    const end = centerline[(index + 1) % centerline.length];
    const vx = Number(end[0]) - Number(start[0]);
    const vy = Number(end[1]) - Number(start[1]);
    const segmentLength = Math.hypot(vx, vy);
    if (segmentLength <= 1.0e-9) continue;
    const rawT = ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / (segmentLength * segmentLength);
    const t = Math.max(0, Math.min(1, rawT));
    const projected = [Number(start[0]) + vx * t, Number(start[1]) + vy * t];
    const distance = pointDistance(point, projected);
    if (!best || distance < best.distance) {
      best = { point: projected, distance, s_m: sBefore + segmentLength * t, direction: [vx / segmentLength, vy / segmentLength] };
    }
    sBefore += segmentLength;
  }
  return best;
}

function gateLineForLaneProjection(lane, projection) {
  const laneLength = polylineWorldLength(lane.centerline || [], lane.closed_loop);
  const fraction = laneLength > 1.0e-9 ? projection.s_m / laneLength : 0;
  if ((lane.left_bound || []).length >= 2 && (lane.right_bound || []).length >= 2) {
    return [
      samplePolylineAt(lane.left_bound, fraction, lane.closed_loop),
      samplePolylineAt(lane.right_bound, fraction, lane.closed_loop),
    ];
  }
  const width = 0.5;
  const normal = [-projection.direction[1], projection.direction[0]];
  return [
    [projection.point[0] + normal[0] * width, projection.point[1] + normal[1] * width],
    [projection.point[0] - normal[0] * width, projection.point[1] - normal[1] * width],
  ];
}

function nearestSectionGate(detail, pixel, hitRadius) {
  const gates = sectionGatesForDetail(detail) || [];
  const canvas = $("map-preview-canvas");
  if (!canvas) return null;
  const toPixel = mapPointProjector(detail, canvas.width, canvas.height);
  let best = null;
  for (const gate of gates) {
    const line = (gate.line || []).map(toPixel);
    if (line.length < 2) continue;
    const midpoint = [(line[0][0] + line[1][0]) * 0.5, (line[0][1] + line[1][1]) * 0.5];
    const distance = pointDistance(pixel, midpoint);
    if (distance <= hitRadius && (!best || distance < best.distance)) best = { gate, distance };
  }
  return best?.gate || null;
}

function handleSectionEditorPointerDown(event) {
  const detail = state.selectedMapDetail;
  if (!detail || !state.sectionEditor.enabled || state.sectionEditor.mapPath !== detail.map?.path) return false;
  if (event.button != null && event.button !== 0) return false;
  event.preventDefault();
  const { canvas, point, hitRadius } = canvasEventInfo(event);
  const nearest = nearestSectionGate(detail, point, hitRadius * 1.3);
  if (nearest) {
    state.sectionEditor.selectedGateId = nearest.id;
    updateSectionEditorChrome();
    drawMapPreview();
    return true;
  }
  const world = mapPixelToWorld(detail, canvas.width, canvas.height, point);
  const lane = sectionEditorLane(detail);
  const projection = world && lane ? projectPointToLane(world, lane) : null;
  if (!projection || !lane) return false;
  const resolution = Number(detail.raster?.resolution_m_per_px || 0);
  const maxDistanceM = Math.max(0.25, hitRadius * (resolution || 0.02) * 2.0);
  if (projection.distance > maxDistanceM) return false;
  const gate = {
    id: nextSectionGateId(),
    lane_id: lane.id || detail.hd_map?.primary_lane_id || "lane_001",
    s_m: projection.s_m,
    line: gateLineForLaneProjection(lane, projection),
  };
  state.sectionEditor.gates.push(gate);
  state.sectionEditor.gates.sort((a, b) => Number(a.s_m || 0) - Number(b.s_m || 0));
  state.sectionEditor.selectedGateId = gate.id;
  state.sectionEditor.dirty = true;
  updateSectionEditorChrome();
  drawMapPreview();
  return true;
}

function deleteSelectedSectionGate() {
  if (!state.selectedMapDetail || !state.sectionEditor.enabled || !state.sectionEditor.selectedGateId) return;
  state.sectionEditor.gates = state.sectionEditor.gates.filter((gate) => gate.id !== state.sectionEditor.selectedGateId);
  state.sectionEditor.selectedGateId = "";
  state.sectionEditor.dirty = true;
  updateSectionEditorChrome();
  drawMapPreview();
}

async function saveSectionGatesFromEditor() {
  const detail = state.selectedMapDetail;
  if (!detail) return;
  ensureSectionEditor(detail);
  if (!confirmAction({
    title: "Save section gates?",
    target: detail.map.path,
    detail: `${state.sectionEditor.gates.length} gate(s) will be written.`,
  })) return;
  if (!beginAction("section-gates:save", "Saving section gates")) return;
  try {
    const saved = await api("/api/maps/save-section-gates", {
      method: "POST",
      body: JSON.stringify({
        map_dir: detail.map.path,
        section_gates: state.sectionEditor.gates,
      }),
    });
    state.selectedMapDetail = saved;
    state.selectedMapPath = saved.map.path;
    ensureSectionEditor(saved, { force: true });
    state.sectionEditor.enabled = true;
    state.sectionEditor.dirty = false;
    toast("Section gates saved");
    render();
  } catch (error) {
    toast(`Section gate save failed: ${error.message}`, "error");
  } finally {
    endAction("section-gates:save", { renderAfter: state.tab === "maps" });
  }
}

function editorLanesForDetail(detail) {
  if (!state.mapEditor.enabled || state.mapEditor.mapPath !== detail?.map?.path) return null;
  return state.mapEditor.lanes;
}

function mapEditorRasterReady(detail) {
  const raster = detail?.raster || {};
  return Boolean(raster.resolution_m_per_px && raster.width && raster.height);
}

function clampMapEditorZoom(value) {
  return Math.max(0.25, Math.min(32, Number(value) || 1));
}

function mapEditorZoomLabel() {
  return `${Math.round((state.mapEditor.zoom || 1) * 100)}%`;
}

function editorLaneIssue(lane) {
  if (!lane) return "No lane";
  const boundPoints = lane.closed_loop ? 3 : 2;
  const centerlinePoints = lane.closed_loop ? 3 : 2;
  if ((lane.left_bound || []).length < boundPoints) return `Left needs ${boundPoints}`;
  if ((lane.right_bound || []).length < boundPoints) return `Right needs ${boundPoints}`;
  if ((lane.centerline || []).length < centerlinePoints) return `Center needs ${centerlinePoints}`;
  return "";
}

function normalizedMapEditorAssistSettings() {
  const rangePoints = Math.max(
    1,
    Math.min(MAX_MAP_EDITOR_ASSIST_RANGE_POINTS, Math.round(Number(state.mapEditor.assistRangePoints) || DEFAULT_MAP_EDITOR_ASSIST_RANGE_POINTS)),
  );
  const spacingM = Math.max(
    0.02,
    Math.min(1, Number(state.mapEditor.assistSpacingM) || DEFAULT_MAP_EDITOR_ASSIST_SPACING_M),
  );
  state.mapEditor.assistRangePoints = rangePoints;
  state.mapEditor.assistSpacingM = spacingM;
  return { rangePoints, spacingM };
}

function mapEditorAssistHint() {
  const selected = state.mapEditor.selected;
  const lane = activeEditorLane();
  if (!state.mapEditor.enabled) return "Enable editing, then select a boundary point.";
  if (!selected) return "Select a line point near the curve to smooth.";
  const count = lane[selected.field]?.length || 0;
  const label = selected.field === "left_bound" ? "left" : selected.field === "right_bound" ? "right" : "center";
  return `${label} point ${selected.index + 1}/${count}: smoothing uses neighboring points and keeps Undo available.`;
}

function canSmoothSelectedEditorRange() {
  const detail = state.selectedMapDetail;
  const selected = state.mapEditor.selected;
  if (!detail || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path || !selected) return false;
  if (!MAP_EDITOR_FIELDS.includes(selected.field)) return false;
  const lane = activeEditorLane();
  const points = lane[selected.field] || [];
  if (!points[selected.index]) return false;
  if (points.length < (lane.closed_loop ? 4 : 3)) return false;
  const assist = normalizedMapEditorAssistSettings();
  const range = selectedEditorRange(points, selected.index, assist.rangePoints, lane.closed_loop);
  return range.segment.length >= (range.closedWhole && lane.closed_loop ? 4 : 3);
}

function updateMapEditorAssistNumber(field, input) {
  const raw = String(input?.value ?? "").trim();
  const value = Number(raw);
  const isRange = field === "rangePoints";
  const min = isRange ? 1 : 0.02;
  const max = isRange ? MAX_MAP_EDITOR_ASSIST_RANGE_POINTS : 1;
  const valid = raw !== "" && Number.isFinite(value) && value >= min && value <= max;
  input?.setCustomValidity(valid ? "" : `Enter a value from ${min} to ${max}`);
  if (!valid) return;
  if (isRange) state.mapEditor.assistRangePoints = Math.round(value);
  else if (field === "spacingM") state.mapEditor.assistSpacingM = value;
  updateMapEditorChrome();
}

function setMapEditorField(field) {
  if (!MAP_EDITOR_FIELDS.includes(field)) return;
  state.mapEditor.activeField = field;
  updateMapEditorChrome();
}

function toggleHdMapEditor() {
  if (!state.selectedMapDetail) return;
  ensureMapEditor(state.selectedMapDetail);
  state.mapEditor.enabled = !state.mapEditor.enabled;
  if (state.mapEditor.enabled) {
    state.sectionEditor.enabled = false;
    state.sectionEditor.selectedGateId = "";
  }
  state.mapEditor.selected = null;
  state.mapEditor.dragging = null;
  render();
}

function toggleEditorClosedLoop(checked) {
  if (!state.selectedMapDetail || !state.mapEditor.enabled) return;
  ensureMapEditor(state.selectedMapDetail);
  rememberMapEditorState();
  const lane = activeEditorLane();
  lane.closed_loop = Boolean(checked);
  regenerateEditorCenterline(lane);
  markMapEditorDirty();
  updateMapEditorChrome();
  drawMapPreview();
}

function toggleEditorCenterline(checked) {
  state.mapEditor.showCenterline = Boolean(checked);
  updateMapEditorChrome();
  drawMapPreview();
}

function markMapEditorDirty() {
  state.mapEditor.dirty = true;
  updateMapEditorChrome();
}

function updateMapEditorChrome() {
  const detail = state.selectedMapDetail;
  if (!detail || state.mapEditor.mapPath !== detail.map?.path) return;
  const lane = activeEditorLane();
  const rasterReady = mapEditorRasterReady(detail);
  const issue = editorLaneIssue(lane);
  const selected = Boolean(state.mapEditor.selected);
  const status = $("map-editor-status");
  if (status) {
    status.textContent = !rasterReady ? "Raster required" : issue || (state.mapEditor.dirty ? "Unsaved" : "Ready");
    status.className = issue || !rasterReady ? "warn" : state.mapEditor.dirty ? "dirty" : "ok";
  }
  const save = $("map-editor-save");
  if (save) save.disabled = !(state.mapEditor.enabled && rasterReady && !issue);
  const del = $("map-editor-delete");
  if (del) del.disabled = !(state.mapEditor.enabled && selected);
  const undo = $("map-editor-undo");
  if (undo) undo.disabled = !(state.mapEditor.enabled && state.mapEditor.undoStack.length);
  const redo = $("map-editor-redo");
  if (redo) redo.disabled = !(state.mapEditor.enabled && state.mapEditor.redoStack.length);
  const autoCenter = $("map-editor-auto-center");
  if (autoCenter) autoCenter.disabled = !(state.mapEditor.enabled && (lane.left_bound || []).length >= 2 && (lane.right_bound || []).length >= 2);
  const smooth = $("map-editor-smooth");
  if (smooth) smooth.disabled = !canSmoothSelectedEditorRange();
  const assist = normalizedMapEditorAssistSettings();
  const assistRange = $("map-editor-assist-range");
  if (assistRange) {
    assistRange.disabled = !state.mapEditor.enabled;
    assistRange.value = String(assist.rangePoints);
  }
  const assistSpacing = $("map-editor-assist-spacing");
  if (assistSpacing) {
    assistSpacing.disabled = !state.mapEditor.enabled;
    assistSpacing.value = String(assist.spacingM);
  }
  const assistHint = $("map-editor-assist-hint");
  if (assistHint) assistHint.textContent = mapEditorAssistHint();
  for (const field of MAP_EDITOR_FIELDS) {
    const button = $(`map-editor-field-${field}`);
    if (button) {
      button.disabled = !state.mapEditor.enabled;
      button.classList.toggle("active", state.mapEditor.activeField === field);
    }
  }
  const closedLoop = $("map-editor-closed-loop");
  if (closedLoop) {
    closedLoop.checked = Boolean(lane.closed_loop);
    closedLoop.disabled = !state.mapEditor.enabled;
  }
  const showCenterline = $("map-editor-show-centerline");
  if (showCenterline) showCenterline.checked = Boolean(state.mapEditor.showCenterline);
  const counts = $("map-editor-counts");
  if (counts) counts.textContent = renderEditorCounts(lane);
  const zoom = $("map-editor-zoom-value");
  if (zoom) zoom.textContent = mapEditorZoomLabel();
}

function mapCanvasFitScale(canvas, width, height) {
  const shell = canvas?.parentElement;
  if (!shell || !width || !height) return 1;
  const styles = window.getComputedStyle(shell);
  const paddingX = Number.parseFloat(styles.paddingLeft || "0") + Number.parseFloat(styles.paddingRight || "0");
  const paddingY = Number.parseFloat(styles.paddingTop || "0") + Number.parseFloat(styles.paddingBottom || "0");
  const contentWidth = Math.max(260, shell.clientWidth - paddingX);
  const contentHeight = Math.max(260, window.innerHeight * 0.68 - paddingY);
  return Math.max(0.01, Math.min(contentWidth / width, contentHeight / height, 1));
}

function applyMapCanvasDisplay(canvas, width, height) {
  const fitScale = mapCanvasFitScale(canvas, width, height);
  const displayScale = fitScale * clampMapEditorZoom(state.mapEditor.zoom || 1);
  canvas.style.width = `${Math.max(160, Math.round(width * displayScale))}px`;
  canvas.style.height = `${Math.max(120, Math.round(height * displayScale))}px`;
}

function mapEditorAnchorFromPoint(clientX, clientY) {
  const canvas = $("map-preview-canvas");
  const shell = canvas?.parentElement;
  if (!canvas || !shell) return null;
  const rect = canvas.getBoundingClientRect();
  const shellRect = shell.getBoundingClientRect();
  if (!rect.width || !rect.height) return null;
  return {
    sourceX: (clientX - rect.left) * canvas.width / rect.width,
    sourceY: (clientY - rect.top) * canvas.height / rect.height,
    viewportX: clientX - shellRect.left,
    viewportY: clientY - shellRect.top,
  };
}

function shellViewportCenter(shell) {
  const rect = shell.getBoundingClientRect();
  return {
    x: rect.left + shell.clientWidth * 0.5,
    y: rect.top + shell.clientHeight * 0.5,
  };
}

function mapEditorViewportCenterAnchor() {
  const canvas = $("map-preview-canvas");
  const shell = canvas?.parentElement;
  if (!canvas || !shell) return null;
  const center = shellViewportCenter(shell);
  return mapEditorAnchorFromPoint(center.x, center.y);
}

function scrollContentOffset(element, scroller) {
  const elementRect = element.getBoundingClientRect();
  const scrollerRect = scroller.getBoundingClientRect();
  return {
    x: elementRect.left - scrollerRect.left + scroller.scrollLeft,
    y: elementRect.top - scrollerRect.top + scroller.scrollTop,
  };
}

function restoreMapEditorZoomAnchor(anchor) {
  if (!anchor) return;
  const canvas = $("map-preview-canvas");
  const shell = canvas?.parentElement;
  if (!canvas || !shell) return;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const offset = scrollContentOffset(canvas, shell);
  const targetLeft = offset.x + anchor.sourceX * rect.width / canvas.width - anchor.viewportX;
  const targetTop = offset.y + anchor.sourceY * rect.height / canvas.height - anchor.viewportY;
  shell.scrollLeft = Math.max(0, Math.min(targetLeft, shell.scrollWidth - shell.clientWidth));
  shell.scrollTop = Math.max(0, Math.min(targetTop, shell.scrollHeight - shell.clientHeight));
}

function setMapEditorZoom(value, options = {}) {
  state.mapEditor.zoom = clampMapEditorZoom(value);
  drawMapPreview();
  requestAnimationFrame(() => {
    if (options.resetScroll) {
      const shell = $("map-preview-canvas")?.parentElement;
      if (shell) {
        shell.scrollLeft = 0;
        shell.scrollTop = 0;
      }
    } else {
      restoreMapEditorZoomAnchor(options.anchor || null);
    }
    updateMapEditorChrome();
  });
}

function zoomMapEditor(factor) {
  setMapEditorZoom((state.mapEditor.zoom || 1) * Number(factor || 1), {
    anchor: mapEditorViewportCenterAnchor(),
  });
}

function resetMapEditorZoom() {
  setMapEditorZoom(1, { resetScroll: true });
}

function handleMapEditorWheel(event) {
  if (!state.selectedMapDetail || state.mapEditor.mapPath !== state.selectedMapDetail.map?.path) return;
  if (!event.ctrlKey && !event.metaKey && !event.altKey) return;
  event.preventDefault();
  const factor = Math.exp(-event.deltaY * 0.0015);
  setMapEditorZoom((state.mapEditor.zoom || 1) * factor, {
    anchor: mapEditorAnchorFromPoint(event.clientX, event.clientY),
  });
}

function pointDistance(a, b) {
  return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]));
}

function polylineWorldLength(points, closedLoop) {
  if (points.length < 2) return 0;
  let total = 0;
  for (let index = 1; index < points.length; index += 1) total += pointDistance(points[index], points[index - 1]);
  if (closedLoop && points.length >= 3) total += pointDistance(points[0], points[points.length - 1]);
  return total;
}

function samplePolylineAt(points, fraction, closedLoop) {
  if (!points.length) return [0, 0];
  if (points.length === 1) return cloneMapPoint(points[0]);
  const segments = [];
  const segmentCount = closedLoop && points.length >= 3 ? points.length : points.length - 1;
  let total = 0;
  for (let index = 0; index < segmentCount; index += 1) {
    const start = points[index];
    const end = points[(index + 1) % points.length];
    const length = pointDistance(start, end);
    segments.push({ start, end, length });
    total += length;
  }
  if (total <= 1.0e-9) return cloneMapPoint(points[0]);
  const normalized = closedLoop ? ((fraction % 1) + 1) % 1 : Math.max(0, Math.min(1, fraction));
  let target = normalized * total;
  for (const segment of segments) {
    if (target <= segment.length || segment === segments[segments.length - 1]) {
      const ratio = segment.length <= 1.0e-9 ? 0 : target / segment.length;
      return [
        segment.start[0] + (segment.end[0] - segment.start[0]) * ratio,
        segment.start[1] + (segment.end[1] - segment.start[1]) * ratio,
      ];
    }
    target -= segment.length;
  }
  return cloneMapPoint(points[points.length - 1]);
}

function openRightBoundForEditor(left, right) {
  if (left.length < 2 || right.length < 2) return right;
  const forward = pointDistance(left[0], right[0]) + pointDistance(left[left.length - 1], right[right.length - 1]);
  const reversed = pointDistance(left[0], right[right.length - 1]) + pointDistance(left[left.length - 1], right[0]);
  return reversed < forward ? [...right].reverse() : right;
}

function closedRightBoundAlignment(left, right) {
  if (left.length < 3 || right.length < 3) return { points: right, offset: 0 };
  const probeCount = 64;
  const fractions = Array.from({ length: probeCount }, (_, index) => index / probeCount);
  const leftProbe = fractions.map((fraction) => samplePolylineAt(left, fraction, true));
  let best = { points: right, offset: 0, score: Number.POSITIVE_INFINITY };
  for (const candidate of [right, [...right].reverse()]) {
    for (let shift = 0; shift < probeCount; shift += 1) {
      const offset = shift / probeCount;
      let score = 0;
      for (let index = 0; index < fractions.length; index += 1) {
        const point = samplePolylineAt(candidate, fractions[index] + offset, true);
        const dx = leftProbe[index][0] - point[0];
        const dy = leftProbe[index][1] - point[1];
        score += dx * dx + dy * dy;
      }
      if (score < best.score) best = { points: candidate, offset, score };
    }
  }
  return best;
}

function dedupePolyline(points) {
  const result = [];
  for (const point of points) {
    const last = result[result.length - 1];
    if (!last || pointDistance(last, point) > 1.0e-6) result.push(point);
  }
  return result;
}

function resamplePolyline(points, count, closedLoop) {
  const safeCount = Math.max(2, Math.floor(Number(count) || 2));
  if (closedLoop) {
    return Array.from({ length: safeCount }, (_, index) => samplePolylineAt(points, index / safeCount, true));
  }
  return Array.from({ length: safeCount }, (_, index) => samplePolylineAt(points, index / Math.max(1, safeCount - 1), false));
}

function interpolateMapPoint(a, b, ratio) {
  return [
    Number(a[0]) + (Number(b[0]) - Number(a[0])) * ratio,
    Number(a[1]) + (Number(b[1]) - Number(a[1])) * ratio,
  ];
}

function chaikinSmoothOpenPolyline(points, iterations = 2) {
  let result = cloneMapPolyline(points);
  for (let pass = 0; pass < iterations; pass += 1) {
    if (result.length < 3) return result;
    const next = [cloneMapPoint(result[0])];
    for (let index = 0; index < result.length - 1; index += 1) {
      const start = result[index];
      const end = result[index + 1];
      next.push(interpolateMapPoint(start, end, 0.25));
      next.push(interpolateMapPoint(start, end, 0.75));
    }
    next.push(cloneMapPoint(result[result.length - 1]));
    result = dedupePolyline(next);
  }
  return result;
}

function chaikinSmoothClosedPolyline(points, iterations = 2) {
  let result = cloneMapPolyline(points);
  for (let pass = 0; pass < iterations; pass += 1) {
    if (result.length < 4) return result;
    const next = [];
    for (let index = 0; index < result.length; index += 1) {
      const start = result[index];
      const end = result[(index + 1) % result.length];
      next.push(interpolateMapPoint(start, end, 0.25));
      next.push(interpolateMapPoint(start, end, 0.75));
    }
    result = dedupePolyline(next);
  }
  return result;
}

function resamplePolylineBySpacing(points, spacingM, closedLoop) {
  const clean = dedupePolyline(cloneMapPolyline(points));
  const minimum = closedLoop ? 3 : 2;
  if (clean.length < minimum) return clean;
  const length = polylineWorldLength(clean, closedLoop);
  if (length <= 1.0e-9) return clean;
  const targetCount = Math.max(
    minimum,
    clean.length,
    Math.ceil(length / Math.max(0.02, spacingM)) + (closedLoop ? 0 : 1),
  );
  return dedupePolyline(resamplePolyline(clean, Math.min(MAX_MAP_EDITOR_ASSIST_RESAMPLE_POINTS, targetCount), closedLoop));
}

function smoothEditorPolylineForAssist(points, spacingM, closedLoop) {
  const smoothed = closedLoop
    ? chaikinSmoothClosedPolyline(points, 2)
    : chaikinSmoothOpenPolyline(points, 2);
  return resamplePolylineBySpacing(smoothed, spacingM, closedLoop);
}

function wrapPolylineIndex(index, length) {
  return ((index % length) + length) % length;
}

function selectedEditorRange(points, selectedIndex, rangePoints, closedLoop) {
  const count = points.length;
  if (closedLoop) {
    const rangeCount = Math.min(count, rangePoints * 2 + 1);
    if (rangeCount >= count) {
      return {
        closedWhole: true,
        startIndex: 0,
        count,
        segment: cloneMapPolyline(points),
      };
    }
    const startIndex = wrapPolylineIndex(selectedIndex - rangePoints, count);
    return {
      closedWhole: false,
      startIndex,
      count: rangeCount,
      segment: Array.from({ length: rangeCount }, (_, offset) => cloneMapPoint(points[(startIndex + offset) % count])),
    };
  }

  const startIndex = Math.max(0, selectedIndex - rangePoints);
  const endIndex = Math.min(count - 1, selectedIndex + rangePoints);
  return {
    closedWhole: false,
    startIndex,
    endIndex,
    count: endIndex - startIndex + 1,
    segment: points.slice(startIndex, endIndex + 1).map(cloneMapPoint),
  };
}

function replaceEditorRange(points, range, replacement, closedLoop) {
  const cleanReplacement = dedupePolyline(cloneMapPolyline(replacement));
  if (range.closedWhole) return cleanReplacement;
  if (!closedLoop) {
    return [
      ...points.slice(0, range.startIndex).map(cloneMapPoint),
      ...cleanReplacement,
      ...points.slice(range.endIndex + 1).map(cloneMapPoint),
    ];
  }
  const rest = [];
  for (let offset = range.count; offset < points.length; offset += 1) {
    rest.push(cloneMapPoint(points[(range.startIndex + offset) % points.length]));
  }
  return [...cleanReplacement, ...rest];
}

function nearestPolylineIndex(points, targetPoint) {
  let bestIndex = 0;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (let index = 0; index < points.length; index += 1) {
    const distance = pointDistance(points[index], targetPoint);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index;
    }
  }
  return bestIndex;
}

function dtwPointPairs(left, right) {
  const n = left.length;
  const m = right.length;
  if (!n || !m) return [];
  const band = Math.max(18, Math.floor(Math.max(n, m) * 0.22), Math.abs(n - m) + 2);
  let previous = new Float64Array(m).fill(Number.POSITIVE_INFINITY);
  let current = new Float64Array(m).fill(Number.POSITIVE_INFINITY);
  const backtrack = new Uint8Array(n * m);

  for (let i = 0; i < n; i += 1) {
    current.fill(Number.POSITIVE_INFINITY);
    const minJ = Math.max(0, i - band);
    const maxJ = Math.min(m - 1, i + band);
    for (let j = minJ; j <= maxJ; j += 1) {
      const cost = pointDistance(left[i], right[j]);
      if (i === 0 && j === 0) {
        current[j] = cost;
        continue;
      }
      const diagonal = i > 0 && j > 0 ? previous[j - 1] : Number.POSITIVE_INFINITY;
      const up = i > 0 ? previous[j] : Number.POSITIVE_INFINITY;
      const side = j > 0 ? current[j - 1] : Number.POSITIVE_INFINITY;
      let best = diagonal;
      let move = 0;
      if (up < best) {
        best = up;
        move = 1;
      }
      if (side < best) {
        best = side;
        move = 2;
      }
      current[j] = cost + best;
      backtrack[i * m + j] = move;
    }
    [previous, current] = [current, previous];
  }

  const pairs = [];
  let i = n - 1;
  let j = m - 1;
  while (i >= 0 && j >= 0) {
    pairs.push([left[i], right[j]]);
    if (i === 0 && j === 0) break;
    const move = backtrack[i * m + j];
    if (move === 0) {
      i -= 1;
      j -= 1;
    } else if (move === 1) {
      i -= 1;
    } else {
      j -= 1;
    }
  }
  return pairs.reverse();
}

function centerlineFromDtwBounds(lane, targetCount) {
  const dtwCount = Math.max(24, Math.min(512, targetCount));
  const leftSamples = resamplePolyline(lane.left_bound, dtwCount, lane.closed_loop);
  const alignment = lane.closed_loop
    ? closedRightBoundAlignment(lane.left_bound, lane.right_bound)
    : { points: openRightBoundForEditor(lane.left_bound, lane.right_bound), offset: 0 };
  const rightSamples = lane.closed_loop
    ? Array.from({ length: dtwCount }, (_, index) => samplePolylineAt(alignment.points, index / dtwCount + alignment.offset, true))
    : resamplePolyline(alignment.points, dtwCount, false);
  const centerline = dtwPointPairs(leftSamples, rightSamples).map(([left, right]) => [
    (left[0] + right[0]) * 0.5,
    (left[1] + right[1]) * 0.5,
  ]);
  const deduped = dedupePolyline(centerline);
  if (deduped.length <= targetCount) return deduped;
  return dedupePolyline(resamplePolyline(deduped, targetCount, lane.closed_loop));
}

function regenerateEditorCenterline(lane) {
  lane.left_bound = cloneMapPolyline(lane.left_bound);
  lane.right_bound = cloneMapPolyline(lane.right_bound);
  const minimum = lane.closed_loop ? 3 : 2;
  if (lane.left_bound.length < minimum || lane.right_bound.length < minimum) {
    lane.centerline = [];
    return;
  }
  const leftLength = polylineWorldLength(lane.left_bound, lane.closed_loop);
  const rightLength = polylineWorldLength(lane.right_bound, lane.closed_loop);
  const averageLength = 0.5 * (leftLength + rightLength);
  const spacingCount = averageLength > 1.0e-9 ? Math.ceil(averageLength / 0.1) + (lane.closed_loop ? 0 : 1) : 0;
  const sampleCount = Math.max(lane.closed_loop ? 8 : 2, lane.left_bound.length, lane.right_bound.length, spacingCount);
  const count = Math.max(2, Math.min(2000, sampleCount));
  lane.centerline = centerlineFromDtwBounds(lane, count);
}

function canvasEventInfo(event) {
  const canvas = event.currentTarget || $("map-preview-canvas");
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / Math.max(1, rect.width);
  const scaleY = canvas.height / Math.max(1, rect.height);
  return {
    canvas,
    point: [(event.clientX - rect.left) * scaleX, (event.clientY - rect.top) * scaleY],
    hitRadius: 12 * Math.max(scaleX, scaleY),
  };
}

function mapPixelToWorld(detail, width, height, pixel) {
  const raster = detail?.raster || {};
  if (!raster.resolution_m_per_px || !raster.width || !raster.height) return null;
  const origin = raster.origin_xy_yaw || [0, 0, 0];
  const scaleX = width / raster.width;
  const scaleY = height / raster.height;
  const gridX = pixel[0] / scaleX;
  const gridY = (raster.height - 1) - pixel[1] / scaleY;
  const localX = gridX * raster.resolution_m_per_px;
  const localY = gridY * raster.resolution_m_per_px;
  const yaw = Number(origin[2] || 0);
  const cos = Math.cos(yaw);
  const sin = Math.sin(yaw);
  const world = [
    Number(origin[0] || 0) + cos * localX - sin * localY,
    Number(origin[1] || 0) + sin * localX + cos * localY,
  ];
  return world.every(Number.isFinite) ? world : null;
}

function nearestEditorPoint(detail, pixel, hitRadius) {
  const lane = activeEditorLane();
  const canvas = $("map-preview-canvas");
  if (!canvas) return null;
  const toPixel = mapPointProjector(detail, canvas.width, canvas.height);
  let best = null;
  for (const field of MAP_EDITOR_FIELDS) {
    for (let index = 0; index < lane[field].length; index += 1) {
      const candidate = toPixel(lane[field][index]);
      const distance = pointDistance(candidate, pixel);
      if (distance <= hitRadius && (!best || distance < best.distance)) {
        best = { field, index, distance };
      }
    }
  }
  return best ? { field: best.field, index: best.index } : null;
}

function pointSegmentDistance(point, start, end) {
  const dx = Number(end[0]) - Number(start[0]);
  const dy = Number(end[1]) - Number(start[1]);
  const lengthSq = dx * dx + dy * dy;
  if (lengthSq <= 1.0e-9) return pointDistance(point, start);
  const t = Math.max(0, Math.min(1, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / lengthSq));
  return pointDistance(point, [start[0] + dx * t, start[1] + dy * t]);
}

function nearestEditorSegment(detail, field, pixel) {
  const lane = activeEditorLane();
  const points = lane[field] || [];
  const canvas = $("map-preview-canvas");
  if (!canvas || points.length < 2) return null;
  const toPixel = mapPointProjector(detail, canvas.width, canvas.height);
  const segmentCount = lane.closed_loop && points.length >= 3 ? points.length : points.length - 1;
  let best = null;
  for (let index = 0; index < segmentCount; index += 1) {
    const start = toPixel(points[index]);
    const end = toPixel(points[(index + 1) % points.length]);
    const distance = pointSegmentDistance(pixel, start, end);
    if (!best || distance < best.distance) best = { index, distance };
  }
  return best;
}

function insertEditorPoint(lane, field, world, detail, pixel) {
  if (!lane[field]) lane[field] = [];
  const points = lane[field];
  if (points.length < 2) {
    points.push(world);
    return points.length - 1;
  }
  const segment = nearestEditorSegment(detail, field, pixel);
  const insertIndex = segment
    ? (lane.closed_loop && segment.index === points.length - 1 ? points.length : segment.index + 1)
    : points.length;
  points.splice(insertIndex, 0, world);
  return insertIndex;
}

function handleMapEditorPointerDown(event) {
  if (state.sectionEditor.enabled && handleSectionEditorPointerDown(event)) return;
  const detail = state.selectedMapDetail;
  if (!detail || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path) return;
  if (!mapEditorRasterReady(detail)) return;
  if (event.button != null && event.button !== 0) return;
  event.preventDefault();
  ensureMapEditor(detail);
  const { canvas, point, hitRadius } = canvasEventInfo(event);
  const lane = activeEditorLane();
  const nearest = nearestEditorPoint(detail, point, hitRadius);
  if (nearest) {
    state.mapEditor.dragSnapshot = captureMapEditorSnapshot();
    state.mapEditor.activeField = nearest.field;
    state.mapEditor.selected = nearest;
    state.mapEditor.dragging = nearest;
  } else {
    const world = mapPixelToWorld(detail, canvas.width, canvas.height, point);
    if (!world) return;
    rememberMapEditorState();
    const field = state.mapEditor.activeField;
    const index = insertEditorPoint(lane, field, world, detail, point);
    state.mapEditor.selected = { field, index };
    state.mapEditor.dragging = { ...state.mapEditor.selected };
    state.mapEditor.dragSnapshot = null;
    if (field !== "centerline") regenerateEditorCenterline(lane);
    markMapEditorDirty();
  }
  if (canvas.setPointerCapture && event.pointerId != null) canvas.setPointerCapture(event.pointerId);
  updateMapEditorChrome();
  drawMapPreview();
}

function handleMapEditorPointerMove(event) {
  const detail = state.selectedMapDetail;
  const drag = state.mapEditor.dragging;
  if (!detail || !drag || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path) return;
  event.preventDefault();
  const { canvas, point } = canvasEventInfo(event);
  const world = mapPixelToWorld(detail, canvas.width, canvas.height, point);
  if (!world) return;
  const lane = activeEditorLane();
  if (!lane[drag.field]?.[drag.index]) return;
  if (state.mapEditor.dragSnapshot) {
    state.mapEditor.undoStack.push(state.mapEditor.dragSnapshot);
    if (state.mapEditor.undoStack.length > 200) state.mapEditor.undoStack.shift();
    state.mapEditor.redoStack = [];
    state.mapEditor.dragSnapshot = null;
  }
  lane[drag.field][drag.index] = world;
  if (drag.field !== "centerline") regenerateEditorCenterline(lane);
  markMapEditorDirty();
  drawMapPreview();
}

function handleMapEditorPointerUp(event) {
  if (!state.mapEditor.dragging) return;
  state.mapEditor.dragging = null;
  state.mapEditor.dragSnapshot = null;
  const canvas = event.currentTarget || $("map-preview-canvas");
  if (canvas?.releasePointerCapture && event.pointerId != null) {
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch {
      // Pointer capture may already be released by the browser.
    }
  }
  updateMapEditorChrome();
  drawMapPreview();
}

function deleteEditorPoint(target) {
  if (!target) return false;
  const lane = activeEditorLane();
  if (!lane[target.field] || !lane[target.field][target.index]) return false;
  rememberMapEditorState();
  lane[target.field].splice(target.index, 1);
  state.mapEditor.selected = null;
  state.mapEditor.dragging = null;
  if (target.field !== "centerline") regenerateEditorCenterline(lane);
  markMapEditorDirty();
  return true;
}

function deleteSelectedEditorPoint() {
  if (!state.selectedMapDetail || !state.mapEditor.enabled) return;
  if (deleteEditorPoint(state.mapEditor.selected)) {
    updateMapEditorChrome();
    drawMapPreview();
  }
}

function smoothSelectedEditorRange() {
  const detail = state.selectedMapDetail;
  if (!detail || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path) return;
  if (!canSmoothSelectedEditorRange()) {
    toast("Select a boundary point with enough neighbors first.", "error");
    return;
  }

  const lane = activeEditorLane();
  const selected = { ...state.mapEditor.selected };
  const points = lane[selected.field] || [];
  const selectedPoint = cloneMapPoint(points[selected.index]);
  const assist = normalizedMapEditorAssistSettings();
  const range = selectedEditorRange(points, selected.index, assist.rangePoints, lane.closed_loop);
  const minimum = range.closedWhole ? (lane.closed_loop ? 4 : 3) : 3;
  if (range.segment.length < minimum) {
    toast("Selected area is too short to smooth.", "error");
    return;
  }

  const smoothed = smoothEditorPolylineForAssist(range.segment, assist.spacingM, Boolean(range.closedWhole));
  if (smoothed.length < minimum) {
    toast("Smooth result was too short; try a larger range.", "error");
    return;
  }

  rememberMapEditorState();
  lane[selected.field] = replaceEditorRange(points, range, smoothed, lane.closed_loop);
  state.mapEditor.selected = {
    field: selected.field,
    index: nearestPolylineIndex(lane[selected.field], selectedPoint),
  };
  state.mapEditor.activeField = selected.field;
  state.mapEditor.dragging = null;
  state.mapEditor.dragSnapshot = null;
  if (selected.field !== "centerline") regenerateEditorCenterline(lane);
  markMapEditorDirty();
  toast(`Smoothed ${range.segment.length} pts to ${smoothed.length} pts`);
  updateMapEditorChrome();
  drawMapPreview();
}

function regenerateEditorCenterlineFromBounds() {
  const detail = state.selectedMapDetail;
  if (!detail || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path) return;
  const lane = activeEditorLane();
  const minimum = lane.closed_loop ? 3 : 2;
  if ((lane.left_bound || []).length < minimum || (lane.right_bound || []).length < minimum) {
    toast(`Left and right boundaries need at least ${minimum} points.`, "error");
    return;
  }
  rememberMapEditorState();
  regenerateEditorCenterline(lane);
  state.mapEditor.selected = null;
  state.mapEditor.dragging = null;
  state.mapEditor.dragSnapshot = null;
  markMapEditorDirty();
  updateMapEditorChrome();
  drawMapPreview();
}

function deleteNearestEditorPoint(event) {
  const detail = state.selectedMapDetail;
  if (!detail || !state.mapEditor.enabled || state.mapEditor.mapPath !== detail.map?.path) return;
  const { point, hitRadius } = canvasEventInfo(event);
  const nearest = nearestEditorPoint(detail, point, hitRadius * 1.3);
  if (deleteEditorPoint(nearest)) {
    updateMapEditorChrome();
    drawMapPreview();
  }
}

function handleMapEditorDoubleClick(event) {
  if (state.sectionEditor.enabled) {
    event.preventDefault();
    deleteSelectedSectionGate();
    return;
  }
  if (!state.mapEditor.enabled) return;
  event.preventDefault();
  deleteNearestEditorPoint(event);
}

function handleMapEditorContextMenu(event) {
  if (state.sectionEditor.enabled) {
    event.preventDefault();
    deleteSelectedSectionGate();
    return;
  }
  if (!state.mapEditor.enabled) return;
  event.preventDefault();
  deleteNearestEditorPoint(event);
}

async function saveHdMapFromEditor() {
  const detail = state.selectedMapDetail;
  if (!detail) return;
  ensureMapEditor(detail);
  const lane = activeEditorLane();
  const issue = editorLaneIssue(lane);
  if (issue) {
    toast(issue, "error");
    return;
  }
  if (!confirmAction({
    title: "Save HD map lines?",
    target: detail.map.path,
    detail: `Lane: ${state.mapEditor.primaryLaneId}\nLeft/right bounds and centerline files will be updated.`,
    destructive: true,
  })) return;
  if (!beginAction("hd-map:save", "Saving HD map")) return;
  const wasEnabled = state.mapEditor.enabled;
  try {
    const saved = await api("/api/maps/save-hd-map", {
      method: "POST",
      body: JSON.stringify({
        map_dir: detail.map.path,
        primary_lane_id: state.mapEditor.primaryLaneId,
        lanes: state.mapEditor.lanes,
      }),
    });
    state.selectedMapDetail = saved;
    state.selectedMapPath = saved.map.path;
    const mapIndex = state.maps.findIndex((item) => item.path === saved.map.path);
    if (mapIndex >= 0) state.maps[mapIndex] = saved.map;
    ensureMapEditor(saved, { force: true });
    state.mapEditor.enabled = wasEnabled;
    state.mapEditor.dirty = false;
    state.mapEditor.undoStack = [];
    state.mapEditor.redoStack = [];
    state.mapEditor.dragSnapshot = null;
    invalidateMapPreflights(saved.map.path);
    toast("HD map saved");
    render();
  } catch (error) {
    toast(`HD map save failed: ${error.message}`, "error");
  } finally {
    endAction("hd-map:save", { renderAfter: state.tab === "maps" });
  }
}

async function runMapStage(stage, mapDir) {
  const endpoint = `/api/maps/${stage}`;
  const body = { map_dir: mapDir };
  const actionKey = `map-stage:${stage}:${mapDir}`;
  if (stage === "prepare-hd-raster") {
    Object.assign(body, hdRasterGenerationPayload());
  }
  if (stage === "generate-raceline") {
    try {
      Object.assign(body, racelineGenerationPayload());
    } catch (error) {
      toast(error.message, "error");
      return;
    }
  }
  const stageLabel = {
    "prepare-hd-raster": "Generate HD raster?",
    "generate-raceline": "Generate raceline?",
    "generate-preview": "Generate preview?",
  }[stage] || "Run map stage?";
  if (!confirmAction({
    title: stageLabel,
    target: mapDir,
    detail: "The Console will start a map task and lock this map folder while it runs.",
  })) return;
  if (!beginAction(actionKey, "Starting map stage")) return;
  bindMapStagePreflight(stage, mapDir, body);
  if (!acquirePreflightExecution(stage, body)) {
    endAction(actionKey);
    return;
  }
  try {
    if (!(await confirmPreflight(stage, body))) return;
    const result = await api(endpoint, { method: "POST", body: JSON.stringify(body) });
    if (result.preflight) cachePreflightResult(stage, body, result.preflight);
    rememberStartedTask(result.task, stage, body);
    await refreshAll();
    selectTask(result.task.task_id);
  } catch (error) {
    if (captureMapTaskConflict(error)) return;
    if (capturePreflightError(stage, body, error)) {
      toast(preflightBlockingReason(preflightEntry(stage, body)), "error");
      return;
    }
    toast(`Map stage failed: ${error.message}`, "error");
  } finally {
    releasePreflightExecution(stage, body);
    endAction(actionKey, { renderAfter: state.tab === "maps" || state.tab === "dashboard" });
  }
}

async function openMapWorkspace(path) {
  state.tab = "maps";
  state.selectedMapPath = path;
  state.selectedMapDetail = null;
  render();
  try {
    const detail = await api(apiPath("/api/maps/detail", { path }));
    state.selectedMapDetail = detail;
    state.selectedMapPath = detail.map.path;
    const index = state.maps.findIndex((item) => item.path === path || item.path === detail.map.path);
    if (index >= 0) state.maps[index] = detail.map;
    render();
  } catch (error) {
    toast(`Map load failed: ${error.message}`, "error");
    render();
  }
}

function selectedMapReplacement(path) {
  return state.maps.find((item) => item.path === path)
    || state.maps.find((item) => item.path.startsWith(`${path}/`))
    || state.maps.find((item) => path.startsWith(`${item.path}/`));
}

async function refreshSelectedMapData(options = {}) {
  if (!state.selectedMapPath || selectedMapRefreshInFlight) return;
  selectedMapRefreshInFlight = true;
  const preserveAnchor = options.preserveViewport ? mapEditorViewportCenterAnchor() : null;
  try {
    const maps = await api("/api/maps/local");
    state.maps = maps.maps || [];
    const replacement = selectedMapReplacement(state.selectedMapPath);
    if (replacement) state.selectedMapPath = replacement.path;
    if (!state.selectedMapPath) return;

    const detail = await api(apiPath("/api/maps/detail", { path: state.selectedMapPath }));
    state.selectedMapDetail = detail;
    state.selectedMapPath = detail.map.path;
    const index = state.maps.findIndex((item) => item.path === detail.map.path);
    if (index >= 0) state.maps[index] = detail.map;

    if (options.render === false) {
      updateTaskChrome();
      updateMapEditorChrome();
      drawMapPreview();
      scheduleVisiblePreflights({ force: true });
    } else {
      render();
      if (preserveAnchor) {
        requestAnimationFrame(() => requestAnimationFrame(() => restoreMapEditorZoomAnchor(preserveAnchor)));
      }
    }
  } catch (error) {
    console.warn("Selected map refresh failed", error);
  } finally {
    selectedMapRefreshInFlight = false;
  }
}

function refreshSelectedMap() {
  if (!state.selectedMapPath) return;
  invalidateMapPreflights(state.selectedMapPath);
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
    try {
      const raster = detail.raster || {};
      const naturalWidth = image?.naturalWidth || raster.width || 900;
      const naturalHeight = image?.naturalHeight || raster.height || 620;
      canvas.width = Math.max(320, naturalWidth);
      canvas.height = Math.max(240, naturalHeight);
      applyMapCanvasDisplay(canvas, canvas.width, canvas.height);
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
    } catch (error) {
      console.warn("Map preview draw failed", error);
      const ctx = canvas.getContext("2d");
      canvas.width = Math.max(320, canvas.width || 900);
      canvas.height = Math.max(240, canvas.height || 620);
      ctx.fillStyle = "#0b0d10";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#f2c94c";
      ctx.font = "14px ui-sans-serif, system-ui";
      ctx.fillText("Map preview could not draw this edit. Check the point data and try undo/delete.", 18, 28);
    }
  };
  if (imageUrl) {
    const cached = mapPreviewImages.get(imageUrl);
    if (cached?.complete) {
      draw(cached.naturalWidth ? cached : null);
      return;
    }
    const image = cached || new Image();
    image.onload = () => draw(image);
    image.onerror = () => draw(null);
    if (!cached) {
      mapPreviewImages.set(imageUrl, image);
      image.src = imageUrl;
    }
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
  const editorLanes = editorLanesForDetail(detail);
  const lanes = editorLanes || detail.hd_map?.lanes || [];
  const showCenterline = state.mapLayers.centerline && (!editorLanes || state.mapEditor.showCenterline);
  if (state.mapLayers.odometry && detail.odometry?.points?.length) {
    drawPolyline(ctx, detail.odometry.points.map(toPixel), "rgba(47, 128, 237, 0.86)", 2.5, false);
  }
  for (const lane of lanes) {
    if (state.mapLayers.left_bound) drawPolyline(ctx, (lane.left_bound || []).map(toPixel), "#45c478", 3, lane.closed_loop);
    if (state.mapLayers.right_bound) drawPolyline(ctx, (lane.right_bound || []).map(toPixel), "#d878d8", 3, lane.closed_loop);
    if (showCenterline) drawPolyline(ctx, (lane.centerline || []).map(toPixel), "#e7c84b", lane.primary ? 4 : 2, lane.closed_loop);
  }
  if (editorLanes) {
    drawEditorPointHandles(ctx, detail, toPixel);
  }
  if (!editorLanes && state.mapLayers.centerline) {
    drawPolyline(ctx, (detail.centerline_csv?.points || []).map(toPixel), "#5aa8ff", 2, false);
  }
  if (state.mapLayers.raceline) {
    drawPolyline(ctx, (detail.raceline_csv?.points || []).map(toPixel), "#ff6d6d", 3, false);
  }
  if (state.mapLayers.section_gates) {
    const gates = sectionGatesForDetail(detail) || detail.hd_map?.section_gates || [];
    for (const gate of gates) {
      const line = (gate.line || []).map(toPixel);
      const selected = state.sectionEditor.enabled && gate.id === state.sectionEditor.selectedGateId;
      drawPolyline(ctx, line, selected ? "#57c7c2" : "#ffffff", selected ? 4 : 2, false);
      if (state.mapLayers.section_labels && line.length >= 2) {
        const x = (line[0][0] + line[1][0]) * 0.5;
        const y = (line[0][1] + line[1][1]) * 0.5;
        drawLabel(ctx, gate.id, x + 6, y - 6);
      }
    }
  }
}

function drawEditorPointHandles(ctx, detail, toPixel) {
  if (!state.mapEditor.enabled || state.mapEditor.mapPath !== detail?.map?.path) return;
  const lane = activeEditorLane();
  const selected = state.mapEditor.selected;
  const fields = [
    ["left_bound", "#45c478"],
    ["right_bound", "#d878d8"],
    ["centerline", "#e7c84b"],
  ];
  ctx.save();
  for (const [field, color] of fields) {
    const points = lane[field] || [];
    for (let index = 0; index < points.length; index += 1) {
      const [x, y] = toPixel(points[index]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      const isSelected = selected?.field === field && selected?.index === index;
      ctx.beginPath();
      ctx.fillStyle = color;
      ctx.strokeStyle = isSelected ? "#ffffff" : "rgba(8, 10, 12, 0.86)";
      ctx.lineWidth = isSelected ? 3 : 2;
      ctx.arc(x, y, isSelected ? 6 : 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }
  ctx.restore();
}

function drawPolyline(ctx, points, color, width, closed) {
  const clean = points.filter((point) => Array.isArray(point) && Number.isFinite(point[0]) && Number.isFinite(point[1]));
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
  points.push(...(detail.odometry?.points || []));
  return points.filter((point) => Array.isArray(point) && point.length >= 2);
}

function finitePoint(point) {
  if (Array.isArray(point) && point.length >= 2) {
    const x = Number(point[0]);
    const y = Number(point[1]);
    if (Number.isFinite(x) && Number.isFinite(y)) return { x, y };
  }
  if (point && typeof point === "object") {
    const x = Number(point.x);
    const y = Number(point.y);
    if (Number.isFinite(x) && Number.isFinite(y)) return { x, y };
  }
  return null;
}

function normalizePoints(points = []) {
  return points.map(finitePoint).filter(Boolean);
}

function primaryLane(detail) {
  const lanes = detail?.hd_map?.lanes || [];
  return lanes.find((lane) => lane.primary) || lanes[0] || null;
}

function simulationPathPoints(detail = state.selectedMapDetail) {
  if (!detail) return [];
  if (state.simulation.source === "raceline") {
    const raceline = normalizePoints(detail.raceline_csv?.points || []);
    if (raceline.length >= 2) return raceline;
  }
  const laneCenterline = normalizePoints(primaryLane(detail)?.centerline || []);
  if (laneCenterline.length >= 2) return laneCenterline;
  return normalizePoints(detail.centerline_csv?.points || []);
}

function simulationPathClosed(points) {
  if (!points || points.length < 3) return false;
  const first = points[0];
  const last = points[points.length - 1];
  return Math.hypot(first.x - last.x, first.y - last.y) <= 0.35;
}

function ensureSimulationState(detail = state.selectedMapDetail, options = {}) {
  const sim = state.simulation;
  const mapPath = detail?.map?.path || "";
  const needsReset = options.force || sim.mapPath !== mapPath;
  if (needsReset) {
    sim.mapPath = mapPath;
    resetSimulationStateFromPath(detail);
  }
}

function resetSimulationStateFromPath(detail = state.selectedMapDetail) {
  const sim = state.simulation;
  stopSimulationLoop();
  const path = simulationPathPoints(detail);
  sim.playing = false;
  sim.rafId = 0;
  sim.lastTickMs = 0;
  sim.time = 0;
  sim.distance = 0;
  sim.lapCount = 0;
  sim.lastProgress = 0;
  sim.nearestIndex = 0;
  sim.targetIndex = 0;
  sim.targetPoint = null;
  sim.steeringRad = 0;
  sim.throttle = 0;
  sim.brake = 0;
  sim.crossTrackErrorM = 0;
  sim.maxCrossTrackErrorM = 0;
  sim.trajectory = [];
  sim.speed = 0;
  if (path.length >= 2) {
    sim.x = path[0].x;
    sim.y = path[0].y;
    sim.yaw = Math.atan2(path[1].y - path[0].y, path[1].x - path[0].x);
    sim.trajectory.push({ x: sim.x, y: sim.y });
  } else {
    sim.x = 0;
    sim.y = 0;
    sim.yaw = 0;
  }
}

function resetSimulation() {
  ensureSimulationState(state.selectedMapDetail, { force: true });
  updateSimulationChrome();
  drawSimulationPreview();
}

function stopSimulationLoop() {
  if (state.simulation.rafId) cancelAnimationFrame(state.simulation.rafId);
  state.simulation.rafId = 0;
  state.simulation.playing = false;
  state.simulation.lastTickMs = 0;
}

function toggleSimulationPlayback() {
  const path = simulationPathPoints();
  if (path.length < 2) return;
  state.simulation.playing = !state.simulation.playing;
  state.simulation.lastTickMs = 0;
  updateSimulationChrome();
  if (state.simulation.playing) {
    state.simulation.rafId = requestAnimationFrame(simulationPlaybackTick);
  } else {
    stopSimulationLoop();
    updateSimulationChrome();
  }
}

function simulationPlaybackTick(timestampMs) {
  const sim = state.simulation;
  if (!sim.playing) return;
  if (!sim.lastTickMs) sim.lastTickMs = timestampMs;
  const elapsedS = Math.min(0.08, Math.max(0, (timestampMs - sim.lastTickMs) / 1000));
  sim.lastTickMs = timestampMs;
  const dt = Math.max(0.005, Math.min(sim.settings.dtS, elapsedS || sim.settings.dtS));
  const steps = Math.max(1, Math.ceil(elapsedS / dt));
  const stepDt = elapsedS > 0 ? elapsedS / steps : dt;
  for (let index = 0; index < steps; index += 1) stepSimulation(stepDt);
  drawSimulationPreview();
  updateSimulationMetricsDom();
  sim.rafId = requestAnimationFrame(simulationPlaybackTick);
}

function stepSimulationOnce() {
  if (state.simulation.playing) return;
  stepSimulation(0.1);
  drawSimulationPreview();
  updateSimulationChrome();
}

function updateSimulationSetting(key, input) {
  if (!Object.prototype.hasOwnProperty.call(state.simulation.settings, key)) return;
  const value = Number(String(input?.value ?? "").trim());
  const valid = Number.isFinite(value) && value >= Number(input?.min || 0);
  input?.setCustomValidity(valid ? "" : "Enter a finite value in range");
  if (!valid) return;
  state.simulation.settings[key] = value;
  if (key === "minLookaheadM" && state.simulation.settings.maxLookaheadM < value) {
    state.simulation.settings.maxLookaheadM = value;
    const maxInput = $("simulation-maxLookaheadM");
    if (maxInput) maxInput.value = value;
  }
  if (key === "maxLookaheadM" && value < state.simulation.settings.minLookaheadM) {
    state.simulation.settings.minLookaheadM = value;
    const minInput = $("simulation-minLookaheadM");
    if (minInput) minInput.value = value;
  }
}

function setSimulationSource(source) {
  state.simulation.source = source === "centerline" ? "centerline" : "raceline";
  resetSimulation();
}

function stepSimulation(dt) {
  const detail = state.selectedMapDetail;
  const path = simulationPathPoints(detail);
  const sim = state.simulation;
  const settings = sim.settings;
  if (path.length < 2 || !Number.isFinite(dt) || dt <= 0) return;

  const tracking = computePurePursuitTarget(path, {
    x: sim.x,
    y: sim.y,
    yaw: sim.yaw,
    speed: sim.speed,
    settings,
  });
  if (!tracking) return;

  const speedError = settings.targetSpeedMps - sim.speed;
  const accelCommand = speedError >= 0
    ? Math.min(settings.maxAccelMps2, speedError * 1.8)
    : Math.max(-settings.maxDecelMps2, speedError * 2.4);
  const drag = settings.dragPerS * sim.speed;
  const accel = accelCommand - drag;
  const previousSpeed = sim.speed;
  sim.speed = Math.max(0, sim.speed + accel * dt);
  sim.steeringRad = tracking.steeringRad;
  sim.throttle = Math.max(0, accelCommand / Math.max(1e-6, settings.maxAccelMps2));
  sim.brake = Math.max(0, -accelCommand / Math.max(1e-6, settings.maxDecelMps2));

  const averageSpeed = (previousSpeed + sim.speed) * 0.5;
  sim.x += averageSpeed * Math.cos(sim.yaw) * dt;
  sim.y += averageSpeed * Math.sin(sim.yaw) * dt;
  sim.yaw = normalizeAngle(sim.yaw + (averageSpeed / Math.max(1e-6, settings.wheelbaseM)) * Math.tan(sim.steeringRad) * dt);
  sim.time += dt;
  sim.distance += Math.max(0, averageSpeed * dt);
  sim.nearestIndex = tracking.nearestIndex;
  sim.targetIndex = tracking.targetIndex;
  sim.targetPoint = tracking.targetPoint;
  sim.crossTrackErrorM = tracking.crossTrackErrorM;
  sim.maxCrossTrackErrorM = Math.max(sim.maxCrossTrackErrorM, tracking.crossTrackErrorM);
  if (tracking.progress < sim.lastProgress - 0.55 && simulationPathClosed(path)) sim.lapCount += 1;
  sim.lastProgress = tracking.progress;

  const previous = sim.trajectory[sim.trajectory.length - 1];
  if (!previous || Math.hypot(previous.x - sim.x, previous.y - sim.y) > 0.03) {
    sim.trajectory.push({ x: sim.x, y: sim.y });
    if (sim.trajectory.length > 3000) sim.trajectory.shift();
  }
}

function computePurePursuitTarget(path, input) {
  const { x, y, yaw, speed, settings } = input;
  let nearestIndex = 0;
  let nearestDistance = Infinity;
  let progressDistance = 0;
  let progressAtNearest = 0;
  for (let index = 0; index < path.length; index += 1) {
    const distance = Math.hypot(path[index].x - x, path[index].y - y);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearestIndex = index;
      progressAtNearest = progressDistance;
    }
    if (index + 1 < path.length) {
      progressDistance += Math.hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
    }
  }
  const totalLength = Math.max(progressDistance, 1e-6);
  const closed = simulationPathClosed(path);
  const lookahead = Math.max(
    settings.minLookaheadM,
    Math.min(settings.maxLookaheadM, settings.minLookaheadM + Math.abs(speed) * settings.lookaheadGainS),
  );
  let targetIndex = nearestIndex;
  let travelled = 0;
  const maximumSegments = closed ? path.length : Math.max(1, path.length - 1 - nearestIndex);
  for (let count = 0; count < maximumSegments; count += 1) {
    const current = targetIndex;
    let next = current + 1;
    if (next >= path.length) next = closed ? 0 : path.length - 1;
    travelled += Math.hypot(path[next].x - path[current].x, path[next].y - path[current].y);
    targetIndex = next;
    if (travelled >= lookahead || (!closed && targetIndex >= path.length - 1)) break;
  }
  const targetPoint = path[targetIndex];
  const dx = targetPoint.x - x;
  const dy = targetPoint.y - y;
  const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
  const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
  const denominator = Math.max(1e-6, localX * localX + localY * localY);
  const curvature = 2 * localY / denominator;
  const steeringRad = Math.max(
    -settings.maxSteeringRad,
    Math.min(settings.maxSteeringRad, Math.atan(settings.wheelbaseM * curvature)),
  );
  return {
    nearestIndex,
    targetIndex,
    targetPoint,
    steeringRad,
    crossTrackErrorM: nearestDistance,
    progress: progressAtNearest / totalLength,
  };
}

function normalizeAngle(angle) {
  let output = angle;
  while (output > Math.PI) output -= Math.PI * 2;
  while (output < -Math.PI) output += Math.PI * 2;
  return output;
}

function updateSimulationChrome() {
  const ready = simulationPathPoints(state.selectedMapDetail).length >= 2;
  const sim = state.simulation;
  const status = !ready ? "Need path" : sim.playing ? "Running" : sim.time > 0 ? "Paused" : "Ready";
  const statusEl = $("simulation-status");
  if (statusEl) {
    statusEl.textContent = status;
    statusEl.className = ready ? (sim.playing ? "running" : "ok") : "warn";
  }
  const runButton = $("simulation-run-button");
  if (runButton) {
    runButton.textContent = sim.playing ? "Pause" : "Run";
    runButton.disabled = !ready;
  }
  updateSimulationMetricsDom();
}

function updateSimulationMetricsDom() {
  const metrics = $("simulation-metrics");
  if (metrics) metrics.innerHTML = renderSimulationMetrics();
}

function drawSimulationPreview() {
  const canvas = $("simulation-canvas");
  const detail = state.selectedMapDetail;
  if (!canvas || !detail) return;
  ensureSimulationState(detail);
  const ctx = canvas.getContext("2d");
  const width = canvas.width || 920;
  const height = canvas.height || 520;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#080a0d";
  ctx.fillRect(0, 0, width, height);
  drawGrid(ctx, width, height);
  const toPixel = mapPointProjector(detail, width, height);
  const lanes = detail.hd_map?.lanes || [];
  for (const lane of lanes) {
    drawPolyline(ctx, normalizePoints(lane.left_bound || []).map((point) => toPixel([point.x, point.y])), "rgba(69,196,120,0.65)", 2, lane.closed_loop);
    drawPolyline(ctx, normalizePoints(lane.right_bound || []).map((point) => toPixel([point.x, point.y])), "rgba(216,120,216,0.65)", 2, lane.closed_loop);
    drawPolyline(ctx, normalizePoints(lane.centerline || []).map((point) => toPixel([point.x, point.y])), "rgba(231,200,75,0.72)", lane.primary ? 3 : 1.5, lane.closed_loop);
  }
  drawPolyline(ctx, normalizePoints(detail.centerline_csv?.points || []).map((point) => toPixel([point.x, point.y])), "rgba(90,168,255,0.72)", 2, false);
  drawPolyline(ctx, normalizePoints(detail.raceline_csv?.points || []).map((point) => toPixel([point.x, point.y])), "rgba(255,109,109,0.9)", 3, false);

  const sim = state.simulation;
  drawPolyline(ctx, sim.trajectory.map((point) => toPixel([point.x, point.y])), "#ffffff", 2.5, false);
  if (sim.targetPoint) {
    const [tx, ty] = toPixel([sim.targetPoint.x, sim.targetPoint.y]);
    ctx.save();
    ctx.strokeStyle = "#57c7c2";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tx, ty, 8, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
  drawSimulationVehicle(ctx, detail, toPixel);

  const path = simulationPathPoints(detail);
  if (path.length < 2) {
    ctx.save();
    ctx.fillStyle = "#f2c94c";
    ctx.font = "14px ui-sans-serif, system-ui";
    ctx.fillText("Generate or edit a centerline/raceline before simulation.", 18, 28);
    ctx.restore();
  }
}

function drawSimulationVehicle(ctx, detail, toPixel) {
  const sim = state.simulation;
  const [x, y] = toPixel([sim.x, sim.y]);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  const rasterYaw = detail?.raster?.resolution_m_per_px
    ? Number(detail.raster.origin_xy_yaw?.[2] || 0)
    : 0;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(rasterYaw - sim.yaw);
  ctx.fillStyle = "#f2f5f8";
  ctx.strokeStyle = "#050709";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(16, 0);
  ctx.lineTo(-10, -8);
  ctx.lineTo(-6, 0);
  ctx.lineTo(-10, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
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
  try {
    const result = await api("/api/jetson/inspect", {
      method: "POST",
      body: JSON.stringify(target),
    });
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
  if (state.jetsonTransfer.running || actionBusy("jetson:pull")) return;
  if ($("pull-remote")) $("pull-remote").value = path || "";
  if ($("pull-remote-select") && path) $("pull-remote-select").value = path;
  if (path) setJetsonPullSelection([path], { renderAfter: false });
}

function pullJetsonRosbag(path) {
  useJetsonRosbag(path);
  return startJetsonPull();
}

async function startTransfer(direction, paths = null) {
  const target = jetsonTarget();
  const payload = {
    host: paths?.host || target.host,
    user: paths?.user || target.user,
    remote_path: paths?.remote || "",
    local_path: paths?.local || "",
  };
  const endpoint = direction === "jetson-to-local" ? "/api/transfers/jetson-to-local" : "/api/transfers/local-to-jetson";
  const result = await api(endpoint, { method: "POST", body: JSON.stringify(payload) });
  await refreshAll();
  selectTask(result.task.task_id);
  return result.task;
}

function selectedJetsonPullPaths() {
  return [...new Set((state.jetsonTransfer.selectedPullPaths || []).filter(Boolean))];
}

function jetsonActivePullPaths() {
  return [...new Set((state.jetsonTransfer.activePullPaths || []).filter(Boolean))];
}

function jetsonPullRowState(path, selectedPullPaths = selectedJetsonPullPaths()) {
  const queue = state.jetsonTransfer || {};
  const activePaths = jetsonActivePullPaths();
  const activeIndex = activePaths.indexOf(path);
  const pullBusy = actionBusy("jetson:pull") || queue.running;
  if (pullBusy && activeIndex >= 0) {
    if (queue.running && activeIndex < queue.currentIndex) {
      return { className: "", disabled: true, label: "Done", title: "This selected sequence has already been pulled in the current queue." };
    }
    if (queue.running && activeIndex === queue.currentIndex) {
      return { className: "is-busy", disabled: true, label: "Pulling...", title: "This selected sequence is currently being pulled." };
    }
    if (queue.running) {
      return { className: "", disabled: true, label: "Queued", title: "This selected sequence is waiting in the current pull queue." };
    }
    return { className: "is-busy", disabled: true, label: "Starting...", title: "This selected sequence is included in the pull that is starting." };
  }
  if (pullBusy) {
    return { className: "", disabled: true, label: "Pull", title: "Another selected sequence is being pulled." };
  }
  return {
    className: "",
    disabled: false,
    label: selectedPullPaths.includes(path) ? "Pull This" : "Pull",
    title: "",
  };
}

function setJetsonPullSelection(paths, options = {}) {
  state.jetsonTransfer.selectedPullPaths = [...new Set((paths || []).filter(Boolean))];
  if (options.renderAfter !== false) render();
}

function toggleJetsonPullSelection(path, checked) {
  if (state.jetsonTransfer.running || actionBusy("jetson:pull")) return;
  const selected = new Set(selectedJetsonPullPaths());
  if (checked) selected.add(path);
  else selected.delete(path);
  setJetsonPullSelection([...selected]);
}

function selectAllJetsonPulls() {
  if (state.jetsonTransfer.running || actionBusy("jetson:pull")) return;
  setJetsonPullSelection(jetsonRosbagSequences().map((sequence) => sequence.path));
}

function clearJetsonPullSelection() {
  if (state.jetsonTransfer.running || actionBusy("jetson:pull")) return;
  setJetsonPullSelection([]);
}

function pruneSelectedJetsonPullPaths(sequences) {
  if (!state.jetsonTransfer.selectedPullPaths.length || !sequences.length) return;
  const known = new Set(sequences.map((sequence) => sequence.path));
  state.jetsonTransfer.selectedPullPaths = state.jetsonTransfer.selectedPullPaths.filter((path) => known.has(path));
}

async function waitForTransferTask(taskId) {
  while (taskId) {
    await sleep(1500);
    const payload = await api(`/api/tasks/${encodeURIComponent(taskId)}`);
    const task = payload.task;
    if (!task || !isActiveTask(task)) return task;
    const index = state.tasks.findIndex((item) => item.task_id === task.task_id);
    if (index >= 0) state.tasks[index] = task;
    updateTaskChrome();
  }
  return null;
}

async function startJetsonPull() {
  const paths = selectedJetsonPullPaths();
  const manualPath = pullRemotePath();
  const pullPaths = paths.length ? paths : manualPath ? [manualPath] : [];
  const target = jetsonTarget();
  const localPath = pullLocalPath();
  if (!pullPaths.length) {
    window.alert("Select or enter at least one Jetson rosbag sequence first.");
    return null;
  }
  if (!confirmAction({
    title: pullPaths.length > 1 ? "Pull selected rosbag sequences?" : "Pull this rosbag sequence?",
    target: `${target.user}@${target.host}`,
    detail: `From:\n${pullPaths.join("\n")}\n\nTo:\n${localPath}`,
  })) return null;
  state.jetsonTransfer.activePullPaths = pullPaths;
  state.jetsonTransfer.currentPath = pullPaths[0] || "";
  if (!beginAction("jetson:pull", "Starting Jetson pull")) {
    state.jetsonTransfer.activePullPaths = [];
    state.jetsonTransfer.currentPath = "";
    return null;
  }
  state.jetsonTransfer.running = true;
  state.jetsonTransfer.currentIndex = 0;
  state.jetsonTransfer.total = pullPaths.length;
  render();
  try {
    let lastTask = null;
    for (let index = 0; index < pullPaths.length; index += 1) {
      state.jetsonTransfer.currentIndex = index;
      state.jetsonTransfer.currentPath = pullPaths[index];
      render();
      lastTask = await startTransfer("jetson-to-local", {
        host: target.host,
        user: target.user,
        remote: pullPaths[index],
        local: localPath,
      });
      const finished = await waitForTransferTask(lastTask.task_id);
      if (finished && finished.status !== "success") {
        toast(`Transfer stopped at ${shortName(pullPaths[index])}: ${finished.status}`, "error");
        break;
      }
    }
    await refreshAll();
    if (lastTask) selectTask(lastTask.task_id);
    return lastTask;
  } catch (error) {
    toast(`Transfer failed: ${error.message}`, "error");
    return null;
  } finally {
    state.jetsonTransfer.running = false;
    state.jetsonTransfer.activePullPaths = [];
    state.jetsonTransfer.currentPath = "";
    state.jetsonTransfer.currentIndex = 0;
    state.jetsonTransfer.total = 0;
    endAction("jetson:pull", { renderAfter: false });
    render();
  }
}

function startJetsonPush() {
  const paths = {
    remote: $("push-remote").value,
    local: $("push-local").value,
  };
  const target = jetsonTarget();
  if (!confirmAction({
    title: "Push map bundle to Jetson?",
    target: `${target.user}@${target.host}`,
    detail: `From notebook:\n${paths.local || "not selected"}\n\nTo Jetson:\n${paths.remote || "not selected"}`,
    destructive: true,
  })) return null;
  if (!beginAction("jetson:push", "Starting Jetson push")) return null;
  return startTransfer("local-to-jetson", paths)
    .catch((error) => {
      toast(`Transfer failed: ${error.message}`, "error");
      return null;
    })
    .finally(() => endAction("jetson:push"));
}

function copyPullCommand() {
  const paths = selectedJetsonPullPaths();
  const manualPath = pullRemotePath();
  const pullPaths = paths.length ? paths : manualPath ? [manualPath] : [];
  if (!pullPaths.length) {
    window.alert("Select or enter at least one Jetson rosbag sequence first.");
    return;
  }
  const target = jetsonTarget();
  copyText(
    pullPaths
      .map((path) => `rsync -avhP ${sh(`${target.user}@${target.host}:${path}`)} ${sh(trimTrailingSlash(pullLocalPath()) + "/")}`)
      .join("\n"),
    pullPaths.length > 1 ? "rsync commands copied" : "rsync command copied",
  );
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
window.clearWebCache = clearWebCache;
window.updateListControl = updateListControl;
window.toggleDateGroup = toggleDateGroup;
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
window.selectDashboardWorkflow = selectDashboardWorkflow;
window.runWorkflowAction = runWorkflowAction;
window.selectPipelineMap = selectPipelineMap;
window.startFpvViewer = startFpvViewer;
window.startBrowserFpv = startBrowserFpv;
window.stopBrowserFpv = stopBrowserFpv;
window.handleFpvBrowserImageLoad = handleFpvBrowserImageLoad;
window.handleFpvBrowserImageError = handleFpvBrowserImageError;
window.handleFpvCodecChange = handleFpvCodecChange;
window.copyFpvReceiverCommand = copyFpvReceiverCommand;
window.copyFpvJetsonCommand = copyFpvJetsonCommand;
window.setFpvHost = setFpvHost;
window.updateFpvCommandPreview = updateFpvCommandPreview;
window.fillMapDir = fillMapDir;
window.useRosbag = useRosbag;
window.selectMapBuildRosbag = selectMapBuildRosbag;
window.openBagAnalysis = openBagAnalysis;
window.selectAnalysisBag = selectAnalysisBag;
window.updateAnalysisOption = updateAnalysisOption;
window.scheduleAnalysisPreflight = scheduleAnalysisPreflight;
window.scheduleE2EPreflight = scheduleE2EPreflight;
window.updateE2EOption = updateE2EOption;
window.updateE2EPipelineOption = updateE2EPipelineOption;
window.createE2EDataset = createE2EDataset;
window.trainE2EModel = trainE2EModel;
window.exportE2EOnnx = exportE2EOnnx;
window.deployE2EModel = deployE2EModel;
window.useE2ERunForOfflineEval = useE2ERunForOfflineEval;
window.updateObjectDetectionPipelineOption = updateObjectDetectionPipelineOption;
window.validateObjectDetectionDataset = validateObjectDetectionDataset;
window.trainObjectDetectionModel = trainObjectDetectionModel;
window.exportObjectDetectionOnnx = exportObjectDetectionOnnx;
window.deployObjectDetectionModel = deployObjectDetectionModel;
window.useObjectDetectionRunForBagAnalysis = useObjectDetectionRunForBagAnalysis;
window.refreshAnalysisData = refreshAnalysisData;
window.startBagAnalysis = startBagAnalysis;
window.startE2EAnalysis = startE2EAnalysis;
window.openAnalysisResult = openAnalysisResult;
window.focusAnalysisResult = focusAnalysisResult;
window.reloadAnalysisResult = reloadAnalysisResult;
window.deleteAnalysis = deleteAnalysis;
window.deleteRosbag = deleteRosbag;
window.toggleRosbagFavorite = toggleRosbagFavorite;
window.renameRosbag = renameRosbag;
window.restoreRosbag = restoreRosbag;
window.deleteTrashedRosbag = deleteTrashedRosbag;
window.deleteMapFolder = deleteMapFolder;
window.toggleAnalysisPlayback = toggleAnalysisPlayback;
window.seekAnalysisTime = seekAnalysisTime;
window.seekAnalysisRelative = seekAnalysisRelative;
window.setAnalysisPlaybackRate = setAnalysisPlaybackRate;
window.setAnalysisImageViewMode = setAnalysisImageViewMode;
window.selectAnalysisCameraChannel = selectAnalysisCameraChannel;
window.stepAnalysisFrame = stepAnalysisFrame;
window.seekAnalysisFromTimeline = seekAnalysisFromTimeline;
window.toggleJetsonStatsSeries = toggleJetsonStatsSeries;
window.setJetsonStatsPreset = setJetsonStatsPreset;
window.selectedCameraTopicConfig = selectedCameraTopicConfig;
window.applyCameraTopicConfig = applyCameraTopicConfig;
window.copySelectedCameraTopicConfig = copySelectedCameraTopicConfig;
window.updateCameraTopicPreview = updateCameraTopicPreview;
window.updateMapDirPreview = updateMapDirPreview;
window.scheduleMapBuildPreflight = scheduleMapBuildPreflight;
window.retryPreflightToken = retryPreflightToken;
window.startMapBuild = startMapBuild;
window.copyMapBuildCommand = copyMapBuildCommand;
window.runMapStage = runMapStage;
window.updateHdRasterCropMode = updateHdRasterCropMode;
window.updateHdRasterNumber = updateHdRasterNumber;
window.updateRacelineGeneration = updateRacelineGeneration;
window.toggleSimulationPlayback = toggleSimulationPlayback;
window.stepSimulationOnce = stepSimulationOnce;
window.resetSimulation = resetSimulation;
window.updateSimulationSetting = updateSimulationSetting;
window.setSimulationSource = setSimulationSource;
window.openMapWorkspace = openMapWorkspace;
window.refreshSelectedMap = refreshSelectedMap;
window.toggleMapLayer = toggleMapLayer;
window.toggleHdMapEditor = toggleHdMapEditor;
window.setMapEditorField = setMapEditorField;
window.undoMapEditor = undoMapEditor;
window.redoMapEditor = redoMapEditor;
window.toggleEditorClosedLoop = toggleEditorClosedLoop;
window.toggleEditorCenterline = toggleEditorCenterline;
window.updateMapEditorAssistNumber = updateMapEditorAssistNumber;
window.smoothSelectedEditorRange = smoothSelectedEditorRange;
window.regenerateEditorCenterlineFromBounds = regenerateEditorCenterlineFromBounds;
window.saveHdMapVersion = saveHdMapVersion;
window.activateHdMapVersion = activateHdMapVersion;
window.toggleSectionEditor = toggleSectionEditor;
window.deleteSelectedSectionGate = deleteSelectedSectionGate;
window.saveSectionGatesFromEditor = saveSectionGatesFromEditor;
window.zoomMapEditor = zoomMapEditor;
window.resetMapEditorZoom = resetMapEditorZoom;
window.handleMapEditorPointerDown = handleMapEditorPointerDown;
window.handleMapEditorPointerMove = handleMapEditorPointerMove;
window.handleMapEditorPointerUp = handleMapEditorPointerUp;
window.handleMapEditorDoubleClick = handleMapEditorDoubleClick;
window.handleMapEditorContextMenu = handleMapEditorContextMenu;
window.handleMapEditorWheel = handleMapEditorWheel;
window.deleteSelectedEditorPoint = deleteSelectedEditorPoint;
window.saveHdMapFromEditor = saveHdMapFromEditor;
window.fillTransferLocal = fillTransferLocal;
window.inspectJetson = inspectJetson;
window.copyJetsonInspect = copyJetsonInspect;
window.setJetsonHost = setJetsonHost;
window.startTransfer = startTransfer;
window.useJetsonRosbag = useJetsonRosbag;
window.pullJetsonRosbag = pullJetsonRosbag;
window.toggleJetsonPullSelection = toggleJetsonPullSelection;
window.selectAllJetsonPulls = selectAllJetsonPulls;
window.clearJetsonPullSelection = clearJetsonPullSelection;
window.startJetsonPull = startJetsonPull;
window.startJetsonPush = startJetsonPush;
window.copyPullCommand = copyPullCommand;
window.copyPushCommand = copyPushCommand;
window.copyHdMapEditorCommand = copyHdMapEditorCommand;

refreshAll().catch((error) => {
  $("app").innerHTML = `<div class="content"><div class="notice">Failed to load JetPilot Console: ${esc(error.message)}</div></div>`;
});

setInterval(() => {
  api("/api/tasks")
    .then(async (data) => {
      const nextTasks = data.tasks || [];
      const pipelineTaskFinished = nextTasks.some((task) => {
        if (!(isE2EPipelineTask(task) || isObjectDetectionPipelineTask(task)) || !isFinishedTask(task)) return false;
        const previous = state.tasks.find((item) => item.task_id === task.task_id);
        return !previous || previous.status !== task.status || previous.ended_at !== task.ended_at;
      });
      if (pipelineTaskFinished) {
        state.tasks = nextTasks;
        await refreshAll();
        return;
      }
      if (isAnalysisTab()) {
        state.tasks = nextTasks;
        const previousSelected = state.analysis.analyses.find((item) => analysisRecordId(item) === state.analysis.selectedId);
        await refreshAnalysisList().catch(() => state.analysis.analyses);
        updateAnalysisListDom();
        updateTaskChrome();
        refreshVisiblePreflightDom();
        const selected = state.analysis.analyses.find((item) => analysisRecordId(item) === state.analysis.selectedId);
        if (
          selected
          && analysisRecordStatus(selected) === "success"
          && analysisRecordStatus(previousSelected) !== "success"
          && !state.analysis.timeline
        ) {
          await openAnalysisResult(state.analysis.selectedId);
        }
        return;
      }
      const finishedMapTasks = nextTasks.filter((task) =>
        mapTaskFinishedSince(state.tasks.find((item) => item.task_id === task.task_id), task),
      );
      const mapTaskFinished = finishedMapTasks.length > 0;
      finishedMapTasks.forEach(invalidatePreflightsForTask);
      if (shouldRefreshMapsAfterTaskPoll(state.tasks, nextTasks)) {
        state.tasks = nextTasks;
        await refreshAll();
        return;
      }
      state.tasks = nextTasks;
      if (state.tab === "fpv" && state.fpv.browserStatus?.running && fpvMediaElement()) {
        updateTaskChrome();
        updateFpvBrowserStatusDom();
        return;
      }
      refreshVisiblePreflightDom();
      if (mapTaskFinished && state.selectedMapPath) {
        await refreshSelectedMapData({ preserveViewport: state.tab === "maps" });
        return;
      }
      if (state.tab === "maps" && state.selectedMapDetail) {
        updateTaskChrome();
        return;
      }
      if (!isEditingField() && !logHistoryIsBeingRead()) render();
      else updateTaskChrome();
    })
    .catch(() => {});
}, 5000);

setInterval(() => {
  pollFpvBrowserStatus();
}, 2000);

window.addEventListener("keydown", (event) => {
  if (!state.mapEditor.enabled || state.tab !== "maps" || isEditingField()) return;
  const key = String(event.key || "").toLowerCase();
  if ((event.metaKey || event.ctrlKey) && key === "z") {
    event.preventDefault();
    if (event.shiftKey) redoMapEditor();
    else undoMapEditor();
  } else if ((event.metaKey || event.ctrlKey) && key === "y") {
    event.preventDefault();
    redoMapEditor();
  } else if (["backspace", "delete"].includes(key) && state.mapEditor.selected) {
    event.preventDefault();
    deleteSelectedEditorPoint();
  }
});

window.addEventListener("beforeunload", () => {
  if (!state.fpv.starting && !fpvReceiverCanAutoStop(state.fpv.browserStatus)) return;
  const sessionId = state.fpv.browserStatus?.session_id || "";
  closeFpvPeerConnection();
  if (!sessionId || !navigator.sendBeacon) return;
  const body = new Blob(
    [JSON.stringify({ session_id: sessionId })],
    { type: "application/json" },
  );
  navigator.sendBeacon("/api/fpv/stop", body);
});
