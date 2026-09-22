// JetPilot Console: training workflow. Shared state is initialized by app.js.
const E2E_PIPELINE_TASK_KINDS = [
  "e2e-preprocess",
  "e2e-train",
  "e2e-export-onnx",
  "e2e-deploy",
];

function isE2EPipelineTask(task) {
  return E2E_PIPELINE_TASK_KINDS.includes(task?.kind);
}

function nextAvailableOutputName(name, occupied) {
  if (!name || !occupied.has(name)) return name;
  const match = name.match(/^(.*)_v([0-9]+)$/);
  const base = match ? match[1] : name;
  let version = match ? Number(match[2]) + 1 : 1;
  if (!Number.isSafeInteger(version)) version = 1;
  let candidate;
  do {
    const suffix = `_v${version++}`;
    candidate = `${base.slice(0, 64 - suffix.length)}${suffix}`;
  } while (occupied.has(candidate));
  return candidate;
}

function compactLocalDateTime(date = new Date()) {
  const pad = (value) => String(value).padStart(2, "0");
  return `${pad(date.getMonth() + 1)}${pad(date.getDate())}-${pad(date.getHours())}${pad(date.getMinutes())}`;
}

function suggestedE2EDatasetName() {
  return `e2e_dataset_${compactLocalDateTime()}`;
}

function suggestedE2ERunName(experiment) {
  const selected = experiment || state.e2ePipeline.experiments.find(
    (item) => item.id === state.e2ePipeline.experiment,
  );
  const target = state.e2ePipeline.outputTarget || selected?.target || "control";
  const suggested = selected?.suggested_run_names?.[target];
  if (suggested) return suggested;
  let prefix = selected?.name_prefix || selected?.id || "model";
  if (target === "steer" && prefix.includes("-control")) {
    prefix = prefix.replace("-control", "-steer-only");
  }
  return `${prefix}_${compactLocalDateTime()}`.slice(0, 64);
}

function applyE2ENameSuggestion(kind) {
  if (kind === "dataset") {
    state.e2ePipeline.datasetName = suggestedE2EDatasetName();
  } else if (kind === "run") {
    state.e2ePipeline.runName = suggestedE2ERunName();
  }
  render();
}

function e2eExperimentOptionLabel(experiment) {
  const family = String(experiment?.family || "model").toUpperCase();
  const label = String(experiment?.label || experiment?.id || "")
    .replace(/^(Control|Steering only) · /, "");
  return `[${family}] ${label}`;
}

function refreshSuggestedOutputNames() {
  const e2e = state.e2ePipeline;
  const detection = state.objectDetectionPipeline;
  const shared = state.sharedVitPipeline;
  const fields = [
    [e2e, "datasetName", "datasetRoot", "datasets", "occupiedDatasetNames", "e2e-preprocess"],
    [e2e, "runName", "runRoot", "runs", "occupiedRunNames", "e2e-train"],
    [detection, "runName", "runRoot", "runs", "occupiedRunNames", "object-detection-train"],
    [shared, "detectionRunName", "detectionRunRoot", "detectionRuns", "occupiedDetectionNames", "shared-vit-detection-train"],
    [shared, "modelName", "modelRoot", "models", "occupiedModelNames", "shared-vit-export"],
  ];
  for (const [pipeline, field, rootField, recordsField, namesField, taskKind] of fields) {
    // Resume intentionally targets the existing training directory.
    if (pipeline === detection && detection.trainingMode === "resume") continue;
    const occupied = new Set([
      ...(pipeline[namesField] || []),
      ...pipeline[recordsField].map((item) => item.name),
    ]);
    const prefix = `${pipeline[rootField].replace(/\/$/, "")}/`;
    for (const task of state.tasks) {
      if (task.kind !== taskKind) continue;
      for (const artifact of task.artifacts || []) {
        const path = String(artifact.path || "");
        if (pipeline[rootField] && path.startsWith(prefix)) {
          occupied.add(path.slice(prefix.length).split("/")[0]);
        }
      }
    }
    pipeline[field] = nextAvailableOutputName(pipeline[field], occupied);
  }
}

function updateE2EPipelineOption(key, value) {
  if (!(key in state.e2ePipeline)) return;
  const numberKeys = [
    "inputWidth", "inputHeight", "maxControlDtSec", "jpegQuality", "batchSize",
    "wamContextLength", "wamFutureHorizon", "wamFutureStride",
    "maxOdometryDtSec", "trajectoryPoints", "trajectoryHorizonSec", "trajectoryScaleM",
    "imuWindowSec", "imuSamples",
    "eventBins", "eventWindowMs", "eventStrideMs", "sampleHz", "eventSampleHz", "rolloutSteps",
    "numWorkers", "epochs", "learningRate", "finetuneEpochs",
    "finetuneLearningRate", "valFraction", "fraction", "weightDecay", "seed",
  ];
  if (numberKeys.includes(key)) state.e2ePipeline[key] = Number(value);
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
    if (dataset && (
      current?.task !== dataset.task
      || String(current?.modality || "image") !== String(dataset.modality || "image")
    )) {
      state.e2ePipeline.experiment = state.e2ePipeline.experiments.find(
        (item) => item.task === dataset.task
          && String(item.modality || "image") === String(dataset.modality || "image"),
      )?.id || "";
      const replacement = state.e2ePipeline.experiments.find((item) => item.id === state.e2ePipeline.experiment);
      const targets = replacement?.output_targets || [replacement?.target || "control"];
      if (!targets.includes(state.e2ePipeline.outputTarget)) state.e2ePipeline.outputTarget = targets[0] || "control";
      state.e2ePipeline.runName = suggestedE2ERunName();
    }
  }
  if (key === "datasetModality" && ["event_tensor", "rgb_event_async"].includes(state.e2ePipeline.datasetModality)) {
    state.e2ePipeline.datasetTask = "control";
    if (state.e2ePipeline.datasetModality === "event_tensor") {
      state.e2ePipeline.sampleHz = Math.min(
        250,
        1000 / Math.max(Number(state.e2ePipeline.eventStrideMs) || 4, 0.1),
      );
    } else {
      state.e2ePipeline.sampleHz = 30;
      state.e2ePipeline.eventSampleHz = 250;
      state.e2ePipeline.rolloutSteps = 32;
      state.e2ePipeline.batchSize = 4;
    }
  }
  if (key === "experiment") {
    const experiment = state.e2ePipeline.experiments.find((item) => item.id === state.e2ePipeline.experiment);
    const targets = experiment?.output_targets || [experiment?.target || "control"];
    if (!targets.includes(state.e2ePipeline.outputTarget)) state.e2ePipeline.outputTarget = targets[0] || "control";
    if (experiment?.recommended_batch_size) state.e2ePipeline.batchSize = experiment.recommended_batch_size;
    state.e2ePipeline.runName = suggestedE2ERunName(experiment);
  }
  if (key === "outputTarget") state.e2ePipeline.runName = suggestedE2ERunName();
  if (key === "runDir") {
    state.e2ePipeline.deployPreset = recommendedE2EDeployPreset(selectedE2ERun());
  }
  if (key === "deployRunDir") {
    const run = selectedE2EDeployRun();
    state.e2ePipeline.deployPreset = recommendedE2EDeployPreset(run);
    state.e2ePipeline.deployName = run?.name || "";
  }
  render();
}

function updateE2EPipelineList(key, select) {
  if (!(key in state.e2ePipeline)) return;
  state.e2ePipeline[key] = Array.from(select.selectedOptions || []).map((option) => option.value);
  if (key === "trainDatasetDirs") {
    state.e2ePipeline.datasetDir = state.e2ePipeline[key][0] || "";
    updateE2EPipelineOption("datasetDir", state.e2ePipeline.datasetDir);
    return;
  }
  render();
}

function e2eSelectedRosbagPaths() {
  const selected = state.e2ePipeline.datasetBagPaths;
  if (state.e2ePipeline.datasetBagSelectionTouched || selected.length) return [...selected];
  return state.analysis.selectedBagPath ? [state.analysis.selectedBagPath] : [];
}

function e2eRosbagGroups(items = state.rosbags) {
  const groups = new Map();
  items.forEach((item) => {
    const parts = rosbagRelativePath(item).split("/").filter(Boolean);
    parts.pop();
    const groupPath = parts.join("/");
    if (!groups.has(groupPath)) groups.set(groupPath, []);
    groups.get(groupPath).push(item);
  });
  return [...groups.entries()].map(([path, rosbags]) => ({ path, rosbags }));
}

function setE2ERosbagSelection(nextSelection) {
  const selected = new Set(nextSelection);
  state.e2ePipeline.datasetBagSelectionTouched = true;
  state.e2ePipeline.datasetBagPaths = state.rosbags
    .map((item) => String(item.path || ""))
    .filter((path) => path && selected.has(path));
  render();
}

function updateE2ERosbagSelection(path, checked) {
  const selected = new Set(e2eSelectedRosbagPaths());
  if (checked) selected.add(path);
  else selected.delete(path);
  setE2ERosbagSelection(selected);
}

function updateE2ERosbagGroup(groupPath, checked) {
  const selected = new Set(e2eSelectedRosbagPaths());
  const group = e2eRosbagGroups().find((item) => item.path === groupPath);
  (group?.rosbags || []).forEach((item) => {
    const path = String(item.path || "");
    if (!path) return;
    if (checked) selected.add(path);
    else selected.delete(path);
  });
  setE2ERosbagSelection(selected);
}

function clearE2ERosbagSelection() {
  setE2ERosbagSelection([]);
}

function toggleE2ERosbagGroup(groupPath, open) {
  state.e2ePipeline.collapsedRosbagGroups[groupPath] = !open;
}

function renderE2ERosbagPicker() {
  const selected = new Set(e2eSelectedRosbagPaths());
  const groups = e2eRosbagGroups();
  if (!groups.length) return `<div class="empty compact">Rosbagがありません。</div>`;
  return `
    <div class="e2e-rosbag-picker">
      <div class="e2e-rosbag-picker-summary">
        <span><strong>${selected.size}</strong> / ${state.rosbags.length} selected</span>
        <button type="button" class="link-button" onclick="clearE2ERosbagSelection()" ${selected.size ? "" : "disabled"}>全解除</button>
      </div>
      <div class="e2e-rosbag-groups">
        ${groups.map((group) => {
          const groupPaths = group.rosbags.map((item) => String(item.path || "")).filter(Boolean);
          const selectedCount = groupPaths.filter((path) => selected.has(path)).length;
          const collapsed = Boolean(state.e2ePipeline.collapsedRosbagGroups[group.path]);
          const groupLabel = group.path
            ? `record / ${group.path.split("/").filter(Boolean).join(" / ")}`
            : "record";
          return `
            <details class="e2e-rosbag-group" ${collapsed ? "" : "open"} ontoggle="toggleE2ERosbagGroup(${js(group.path)}, this.open)">
              <summary><span class="e2e-rosbag-group-path" title="${esc(groupLabel)}">${esc(groupLabel)}</span><strong>${selectedCount}/${groupPaths.length}</strong></summary>
              <div class="e2e-rosbag-group-actions">
                <label><input type="checkbox" ${selectedCount === groupPaths.length ? "checked" : ""} onchange="updateE2ERosbagGroup(${js(group.path)}, this.checked)" />グループ内をすべて選択</label>
              </div>
              <div class="e2e-rosbag-checklist">
                ${group.rosbags.map((item) => {
                  const path = String(item.path || "");
                  const label = String(item.display_name || item.label || item.name || shortName(path));
                  return `<label class="e2e-rosbag-choice" title="${esc(path)}"><input type="checkbox" value="${esc(path)}" ${selected.has(path) ? "checked" : ""} onchange="updateE2ERosbagSelection(${js(path)}, this.checked)" /><span>${esc(label)}</span></label>`;
                }).join("")}
              </div>
            </details>`;
        }).join("")}
      </div>
    </div>`;
}

function recommendedE2EDeployPreset(run) {
  if (!run) return "";
  if (run.architecture?.stateful_step) return "";
  const dataset = state.e2ePipeline.datasets.find((item) => item.path === run.dataset_dir);
  const topic = dataset?.image_topic || run.image_topic || "";
  const modality = dataset?.modality || run.modality || "image";
  if (modality === "rgb_event_async") return "rgb_event_async_control";
  const sensor = modality === "event_tensor" ? "event_tensor" : topic.endsWith("/event_image") ? "event" : "camera";
  const target = run.task === "trajectory" ? "trajectory" : run.steering_only ? "steering" : "control";
  return `${sensor}_${target}`;
}

function selectedE2EDataset() {
  const path = state.e2ePipeline.trainDatasetDirs[0] || state.e2ePipeline.datasetDir;
  return state.e2ePipeline.datasets.find((item) => item.path === path) || null;
}

function selectedE2ERun() {
  return state.e2ePipeline.runs.find((item) => item.path === state.e2ePipeline.runDir) || null;
}

function selectedE2EDeployRun() {
  return state.e2ePipeline.runs.find(
    (item) => item.path === state.e2ePipeline.deployRunDir,
  ) || null;
}

function selectedE2EDeployProfile() {
  return state.e2ePipeline.deployProfiles.find((item) => item.id === state.e2ePipeline.deployProfile) || null;
}

function renderE2ELossCurve(run) {
  const history = Array.isArray(run?.metrics?.history)
    ? run.metrics.history
    : Array.isArray(run?.progress?.history) ? run.progress.history : [];
  const values = history.map((item) => Number(item.validation_loss)).filter(Number.isFinite);
  if (!values.length) return "";
  const width = 280;
  const height = 64;
  const low = Math.min(...values);
  const high = Math.max(...values);
  const span = Math.max(high - low, 1e-9);
  const points = values.map((value, index) => {
    const x = values.length === 1 ? width / 2 : index * width / (values.length - 1);
    const y = height - 4 - (value - low) / span * (height - 8);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(" ");
  return `<div class="e2e-loss-curve"><small>Validation loss · ${values.length} epochs · best ${esc(low.toPrecision(4))}</small><svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Validation loss"><polyline fill="none" stroke="currentColor" stroke-width="2" points="${points}" /></svg></div>`;
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
  const rosbags = e2eSelectedRosbagPaths();
  return startE2EPipelineTask(
    "e2e-pipeline:dataset",
    "/api/e2e/datasets/create",
    "Create E2E dataset",
    pipeline.datasetName,
    {
      rosbag: rosbags[0] || "",
      rosbags,
      dataset_name: pipeline.datasetName,
      image_topic: pipeline.imageTopic || state.analysis.imageTopic,
      control_topic: pipeline.controlTopic,
      odometry_topic: pipeline.odometryTopic,
      imu_topic: pipeline.imuTopic,
      task: pipeline.datasetTask,
      modality: pipeline.datasetModality,
      input_width: pipeline.inputWidth,
      input_height: pipeline.inputHeight,
      max_control_dt_sec: pipeline.maxControlDtSec,
      timestamp_source: pipeline.timestampSource,
      max_odometry_dt_sec: pipeline.maxOdometryDtSec,
      trajectory_points: pipeline.trajectoryPoints,
      trajectory_horizon_sec: pipeline.trajectoryHorizonSec,
      trajectory_scale_m: pipeline.trajectoryScaleM,
      imu_window_sec: pipeline.imuWindowSec,
      imu_samples: pipeline.imuSamples,
      jpeg_quality: pipeline.jpegQuality,
      event_topic: pipeline.eventTopic,
      event_bins: pipeline.eventBins,
      event_window_ms: pipeline.eventWindowMs,
      event_stride_ms: pipeline.eventStrideMs,
      event_polarity_layout: pipeline.eventPolarityLayout,
      event_temporal_interpolation: pipeline.eventTemporalInterpolation,
      sample_hz: pipeline.sampleHz,
      event_sample_hz: pipeline.eventSampleHz,
      rollout_steps: pipeline.rolloutSteps,
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
      dataset_dir: pipeline.trainDatasetDirs[0] || pipeline.datasetDir,
      dataset_dirs: pipeline.trainDatasetDirs.length
        ? pipeline.trainDatasetDirs : [pipeline.datasetDir].filter(Boolean),
      validation_dataset_dirs: pipeline.splitMode === "explicit"
        ? pipeline.validationDatasetDirs : [],
      split_mode: pipeline.splitMode,
      run_name: pipeline.runName,
      experiment: pipeline.experiment,
      output_target: pipeline.outputTarget,
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
      rgb_feature_mode: pipeline.experiment === "async_rgb_evs_dinov3_control"
        ? pipeline.rgbFeatureMode : "image",
      wam_context_length: pipeline.wamContextLength,
      wam_future_horizon: pipeline.wamFutureHorizon,
      wam_future_stride: pipeline.wamFutureStride,
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
  const run = selectedE2EDeployRun();
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
      deploy_name: pipeline.deployName,
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
  state.e2eWorkspace = "evaluate";
  render();
  requestAnimationFrame(() => $("e2e-offline-eval")?.scrollIntoView({ behavior: "smooth", block: "start" }));
  toast(`Offline evaluation model selected: ${run.name}`);
}

function e2ePipelineStageState() {
  const pipeline = state.e2ePipeline;
  const run = selectedE2EDeployRun();
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
  const deployRun = selectedE2EDeployRun();
  const recommendedPreset = recommendedE2EDeployPreset(deployRun);
  const deployment = pipeline.deployPresets.find((item) => item.id === recommendedPreset);
  if (deployRun) pipeline.deployPreset = recommendedPreset;
  const profile = selectedE2EDeployProfile();
  const selectedExperiment = pipeline.experiments.find((item) => item.id === pipeline.experiment);
  const hasTwoStages = selectedExperiment?.stages === 2;
  const pipelineTasks = state.tasks.filter(isE2EPipelineTask).slice(0, 8);
  const activeDatasetTask = pipelineTasks.find(
    (task) => task.kind === "e2e-preprocess" && isActiveTask(task),
  );
  const imageTopic = pipeline.imageTopic || state.analysis.imageTopic;
  const experiments = dataset
    ? pipeline.experiments.filter(
      (item) => item.task === dataset.task
        && String(item.modality || "image") === String(dataset.modality || "image"),
    )
    : pipeline.experiments;
  const outputTargets = selectedExperiment?.output_targets
    || [selectedExperiment?.target || "control"];
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
            <div class="field"><label>Rosbags（複数選択可）</label>${renderE2ERosbagPicker()}<div class="field-hint">親pathごとに選択できます。複数選択時はbagごとに独立したdatasetを連続生成し、topic構成は同一である必要があります。</div></div>
            <div class="field"><label>Dataset name</label><input value="${esc(pipeline.datasetName)}" onchange="updateE2EPipelineOption('datasetName', this.value)" /><div class="field-hint">${esc(pipeline.datasetRoot)} · <button type="button" class="link-button" onclick="applyE2ENameSuggestion('dataset')">現在日時で再提案</button></div></div>
            <div class="field"><label>Learning task</label><select onchange="updateE2EPipelineOption('datasetTask', this.value)">${[["control","Control (steering + throttle)"],["trajectory","Trajectory (future odometry)"]].map(([value,label]) => `<option value="${value}" ${pipeline.datasetTask === value ? "selected" : ""}>${label}</option>`).join("")}</select></div>
            <div class="field"><label>Input representation</label><select onchange="updateE2EPipelineOption('datasetModality', this.value)">${[["image","RGB / 3ch image"],["event_tensor","Raw EVS 20ch tensor"],["rgb_event_async","Async RGB + Raw EVS 20ch"]].map(([value,label]) => `<option value="${value}" ${pipeline.datasetModality === value ? "selected" : ""}>${label}</option>`).join("")}</select></div>
            <div class="field"><label>${pipeline.datasetModality === "event_tensor" ? "Reference image topic（clockには不使用）" : pipeline.datasetModality !== "image" ? "RGB reference topic" : "Image topic"}</label><select onchange="updateE2EPipelineOption('imageTopic', this.value)">${analysisTopicOptions("image", imageTopic)}</select><div class="field-hint">${pipeline.datasetModality === "event_tensor" ? "TensorはRaw EVSの固定strideで生成します。このtopicは記録元の参照情報としてのみ保存します。" : pipeline.datasetModality === "rgb_event_async" ? "Frozen DINOv3でRGB stateを作り、100〜250 Hzに揺らしたEVS更新を学習します。250 Hz抽出を推奨します。" : ""}</div></div>
            ${pipeline.datasetModality !== "image" ? `<div class="field"><label>Raw EventPacket topic</label><input value="${esc(pipeline.eventTopic)}" onchange="updateE2EPipelineOption('eventTopic', this.value)" /></div><div class="e2e-compact-fields"><label>Bins<input type="number" min="1" max="64" value="${esc(pipeline.eventBins)}" onchange="updateE2EPipelineOption('eventBins', this.value)" /></label><label>Window (ms)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.eventWindowMs)}" onchange="updateE2EPipelineOption('eventWindowMs', this.value)" /></label><label>Stride (ms)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.eventStrideMs)}" onchange="updateE2EPipelineOption('eventStrideMs', this.value)" /></label><label>${pipeline.datasetModality === "event_tensor" ? "Tensor Hz" : "RGB Hz"}<input type="number" min="0.1" max="250" step="0.1" value="${esc(pipeline.sampleHz)}" onchange="updateE2EPipelineOption('sampleHz', this.value)" /></label>${pipeline.datasetModality === "rgb_event_async" ? `<label>EVS update Hz<input type="number" min="1" max="250" value="${esc(pipeline.eventSampleHz)}" onchange="updateE2EPipelineOption('eventSampleHz', this.value)" /></label><label>Max rollout<input type="number" min="1" max="64" value="${esc(pipeline.rolloutSteps)}" onchange="updateE2EPipelineOption('rolloutSteps', this.value)" /></label>` : ""}</div>${pipeline.datasetModality === "event_tensor" ? `<div class="field-hint">固定イベントクロックでTensorを生成し、各時点以前の最新teacher controlを割り当てます。Stride 4 msでは250 Hzです。</div>` : ""}<div class="e2e-compact-fields"><label>Channel layout<select onchange="updateE2EPipelineOption('eventPolarityLayout', this.value)">${[["polarity_major","polarity major (+B, -B)"],["time_major","time major (+,- per bin)"]].map(([value,label]) => `<option value="${value}" ${pipeline.eventPolarityLayout === value ? "selected" : ""}>${label}</option>`).join("")}</select></label><label>Interpolation<select onchange="updateE2EPipelineOption('eventTemporalInterpolation', this.value)">${[["none","None (CUDA compatible)"],["linear","Linear (CPU runtime)"]].map(([value,label]) => `<option value="${value}" ${pipeline.eventTemporalInterpolation === value ? "selected" : ""}>${label}</option>`).join("")}</select></label></div>` : ""}
            <div class="field"><label>Teacher control</label><select onchange="updateE2EPipelineOption('controlTopic', this.value)">${analysisTopicOptions("control", pipeline.controlTopic)}</select></div>
            <div class="field"><label>Odometry for trajectory GT</label><input value="${esc(pipeline.odometryTopic)}" onchange="updateE2EPipelineOption('odometryTopic', this.value)" /></div>
            <div class="field"><label>IMU topic</label><input value="${esc(pipeline.imuTopic)}" onchange="updateE2EPipelineOption('imuTopic', this.value)" /></div>
            <div class="e2e-compact-fields"><label>Width<input type="number" min="32" value="${esc(pipeline.inputWidth)}" onchange="updateE2EPipelineOption('inputWidth', this.value)" /></label><label>Height<input type="number" min="32" value="${esc(pipeline.inputHeight)}" onchange="updateE2EPipelineOption('inputHeight', this.value)" /></label><label>Max Δt (s)<input type="number" min="0.001" step="0.01" value="${esc(pipeline.maxControlDtSec)}" onchange="updateE2EPipelineOption('maxControlDtSec', this.value)" /></label></div>
            ${pipeline.datasetTask === "trajectory" ? `<div class="e2e-compact-fields"><label>Points<input type="number" min="2" value="${esc(pipeline.trajectoryPoints)}" onchange="updateE2EPipelineOption('trajectoryPoints', this.value)" /></label><label>Horizon (s)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.trajectoryHorizonSec)}" onchange="updateE2EPipelineOption('trajectoryHorizonSec', this.value)" /></label><label>Scale (m)<input type="number" min="0.1" step="0.1" value="${esc(pipeline.trajectoryScaleM)}" onchange="updateE2EPipelineOption('trajectoryScaleM', this.value)" /></label></div>` : ""}
            <details><summary>Alignment & IMU</summary><div class="field"><label>Alignment clock</label><select onchange="updateE2EPipelineOption('timestampSource', this.value)">${[["bag", "Bag recording time (same as Offline Analysis)"], ["header", "Header stamp (synchronized sensors only)"]].map(([value, label]) => `<option value="${value}" ${pipeline.timestampSource === value ? "selected" : ""}>${esc(label)}</option>`).join("")}</select></div><div class="e2e-compact-fields"><label>Odom Δt<input type="number" min="0.001" step="0.01" value="${esc(pipeline.maxOdometryDtSec)}" onchange="updateE2EPipelineOption('maxOdometryDtSec', this.value)" /></label><label>IMU window (s)<input type="number" min="0.01" step="0.1" value="${esc(pipeline.imuWindowSec)}" onchange="updateE2EPipelineOption('imuWindowSec', this.value)" /></label><label>IMU samples<input type="number" min="1" value="${esc(pipeline.imuSamples)}" onchange="updateE2EPipelineOption('imuSamples', this.value)" /></label></div></details>
            <button class="primary ${actionBusy("e2e-pipeline:dataset") ? "is-busy" : ""}" onclick="createE2EDataset()" ${e2eSelectedRosbagPaths().length && imageTopic && (pipeline.datasetTask === "trajectory" ? pipeline.odometryTopic : pipeline.controlTopic) ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:dataset", "Dataset creation is starting...")}>${esc(actionButtonLabel("e2e-pipeline:dataset", "Create dataset", "Starting..."))}</button>
            ${activeDatasetTask ? renderTaskProgress(activeDatasetTask) : ""}
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>02</span><div><strong>Train model</strong><small>Adjust repeatable training parameters</small></div></header>
            <div class="field"><label>Dataset split</label><select onchange="updateE2EPipelineOption('splitMode', this.value)">${[["temporal","簡易：train dataset内を時系列分割"],["explicit","厳密：train / validationを別datasetに固定"]].map(([value,label]) => `<option value="${value}" ${value === pipeline.splitMode ? "selected" : ""}>${label}</option>`).join("")}</select></div>
            <div class="field"><label>Train datasets（複数選択可）</label><select multiple size="5" onchange="updateE2EPipelineList('trainDatasetDirs', this)">${pipeline.datasets.map((item) => `<option value="${esc(item.path)}" ${pipeline.trainDatasetDirs.includes(item.path) ? "selected" : ""}>${esc(item.modality === "event_tensor" ? `${item.input_channels}ch EVS` : item.modality === "rgb_event_async" ? "RGB+20ch EVS" : item.task)} · ${esc(item.name)} — ${esc(item.sample_count)} samples</option>`).join("")}</select><div class="field-hint">簡易モードでは、選択した各datasetの末尾をvalidationに使います。</div></div>
            ${pipeline.splitMode === "explicit" ? `<div class="field"><label>Validation datasets（Trainとは別）</label><select multiple size="5" onchange="updateE2EPipelineList('validationDatasetDirs', this)">${pipeline.datasets.filter((item) => !pipeline.trainDatasetDirs.includes(item.path)).map((item) => `<option value="${esc(item.path)}" ${pipeline.validationDatasetDirs.includes(item.path) ? "selected" : ""}>${esc(item.modality === "event_tensor" ? `${item.input_channels}ch EVS` : item.modality === "rgb_event_async" ? "RGB+20ch EVS" : item.task)} · ${esc(item.name)} — ${esc(item.sample_count)} samples</option>`).join("")}</select><div class="field-hint">このdatasetは学習更新に一切使用せず、model選択用のvalidationだけに使います。</div></div>` : ""}
            <div class="field"><label>Model architecture / training</label><select onchange="updateE2EPipelineOption('experiment', this.value)">${experiments.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.experiment ? "selected" : ""}>${esc(e2eExperimentOptionLabel(item))}</option>`).join("")}</select></div>
            <div class="field"><label>Model output</label><select onchange="updateE2EPipelineOption('outputTarget', this.value)">${outputTargets.map((value) => `<option value="${esc(value)}" ${value === pipeline.outputTarget ? "selected" : ""}>${esc(value === "steer" ? "Steer only（throttleは0固定）" : value === "control" ? "Control（steering + throttle）" : "Trajectory")}</option>`).join("")}</select><div class="field-hint">モデル構造とは独立して学習出力を選択します。</div></div>
            <div class="field"><label>Model / run name</label><input value="${esc(pipeline.runName)}" onchange="updateE2EPipelineOption('runName', this.value)" /><div class="field-hint">${esc(pipeline.runRoot)} · architecture変更時に自動更新 · <button type="button" class="link-button" onclick="applyE2ENameSuggestion('run')">現在日時で再提案</button></div></div>
            <div class="e2e-compact-fields"><label>Epochs<input type="number" min="1" value="${esc(pipeline.epochs)}" onchange="updateE2EPipelineOption('epochs', this.value)" /></label><label>Learning rate<input type="number" min="0.00000001" step="0.0001" value="${esc(pipeline.learningRate)}" onchange="updateE2EPipelineOption('learningRate', this.value)" /></label><label>Batch<input type="number" min="1" value="${esc(pipeline.batchSize)}" onchange="updateE2EPipelineOption('batchSize', this.value)" /></label></div>
            ${selectedExperiment?.id === "async_rgb_evs_dinov3_control" ? `<div class="field"><label>RGB backbone計算</label><select onchange="updateE2EPipelineOption('rgbFeatureMode', this.value)">${[["cache","特徴キャッシュ（高速・推奨）"],["image","RGBから毎epoch計算（従来モード）"]].map(([value,label]) => `<option value="${value}" ${value === pipeline.rgbFeatureMode ? "selected" : ""}>${label}</option>`).join("")}</select><div class="field-hint">特徴キャッシュは固定DINOv3出力をdataset内に一度だけ保存します。学習結果とTensorRT exportの構造は変わりません。</div></div>` : ""}
            ${selectedExperiment?.family === "wam" ? `<div class="e2e-compact-fields"><label>Context frames<input type="number" min="1" max="32" value="${esc(pipeline.wamContextLength)}" onchange="updateE2EPipelineOption('wamContextLength', this.value)" /></label><label>Future steps<input type="number" min="1" max="32" value="${esc(pipeline.wamFutureHorizon)}" onchange="updateE2EPipelineOption('wamFutureHorizon', this.value)" /></label><label>Future stride<input type="number" min="1" max="10" value="${esc(pipeline.wamFutureStride)}" onchange="updateE2EPipelineOption('wamFutureStride', this.value)" /></label></div><div class="field-hint">未来DINO latentと未来のsteering/throttleを同時に学習します。TensorRT出力は明示的なhidden stateを持つ1-step graphです。</div>` : ""}
            ${hasTwoStages ? `<div class="e2e-compact-fields"><label>Fine-tune epochs<input type="number" min="1" value="${esc(pipeline.finetuneEpochs)}" onchange="updateE2EPipelineOption('finetuneEpochs', this.value)" /></label><label>Fine-tune LR<input type="number" min="0.00000001" step="0.0001" value="${esc(pipeline.finetuneLearningRate)}" onchange="updateE2EPipelineOption('finetuneLearningRate', this.value)" /></label></div>` : ""}
            <details><summary>Advanced parameters</summary><div class="e2e-compact-fields"><label>Data fraction<input type="number" min="0.001" max="1" step="0.05" value="${esc(pipeline.fraction)}" onchange="updateE2EPipelineOption('fraction', this.value)" /></label><label>Validation<input type="number" min="0.01" max="0.9" step="0.05" value="${esc(pipeline.valFraction)}" onchange="updateE2EPipelineOption('valFraction', this.value)" /></label><label>Workers<input type="number" min="0" value="${esc(pipeline.numWorkers)}" onchange="updateE2EPipelineOption('numWorkers', this.value)" /></label><label>Weight decay<input type="number" min="0" max="1" step="0.0001" value="${esc(pipeline.weightDecay)}" onchange="updateE2EPipelineOption('weightDecay', this.value)" /></label><label>Seed<input type="number" min="0" value="${esc(pipeline.seed)}" onchange="updateE2EPipelineOption('seed', this.value)" /></label><label>Device<select onchange="updateE2EPipelineOption('device', this.value)">${[["","Auto"],["cuda","CUDA"],["mps","Apple MPS"],["cpu","CPU"]].map(([value,label]) => `<option value="${value}" ${pipeline.device === value ? "selected" : ""}>${label}</option>`).join("")}</select></label></div></details>
            <button class="primary ${actionBusy("e2e-pipeline:train") ? "is-busy" : ""}" onclick="trainE2EModel()" ${pipeline.trainDatasetDirs.length && (pipeline.splitMode !== "explicit" || pipeline.validationDatasetDirs.length) ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:train", "Training is starting...")}>${esc(actionButtonLabel("e2e-pipeline:train", "Start training", "Starting..."))}</button>
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>03</span><div><strong>Evaluate & export</strong><small>Checkpoint → ONNX → offline eval</small></div></header>
            <div class="field"><label>Training run</label><select onchange="updateE2EPipelineOption('runDir', this.value)"><option value="">Select run</option>${pipeline.runs.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.runDir ? "selected" : ""}>${esc(item.task)} · ${esc(item.name)} — ${esc(item.model || item.status)}</option>`).join("")}</select></div>
            ${run ? `<div class="e2e-run-summary"><span>${run.best_checkpoint ? "Checkpoint ready" : "No best checkpoint"}</span><span>${run.onnx_path ? "ONNX ready" : "ONNX not exported"}</span><small>${esc(run.path)}</small>${renderE2ELossCurve(run)}</div>` : `<div class="empty compact">Select a completed training run.</div>`}
            <div class="button-stack"><button class="primary ${actionBusy("e2e-pipeline:export") ? "is-busy" : ""}" onclick="exportE2EOnnx()" ${run?.best_checkpoint ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:export", "ONNX export is starting...")}>${esc(actionButtonLabel("e2e-pipeline:export", "Export ONNX", "Starting..."))}</button><button onclick="useE2ERunForOfflineEval()" ${run?.onnx_path ? "" : "disabled"}>Use in offline eval</button></div>
          </article>
          <article class="e2e-pipeline-stage">
            <header><span>04</span><div><strong>Deploy to Jetson</strong><small>Select ONNX → transfer model files</small></div></header>
            <div class="field"><label>ONNX model to deploy</label><select onchange="updateE2EPipelineOption('deployRunDir', this.value)"><option value="">Select exported run</option>${pipeline.runs.filter((item) => item.onnx_path).map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.deployRunDir ? "selected" : ""}>${esc(item.task)} · ${esc(item.name)}</option>`).join("")}</select></div>
            <div class="field"><label>Connection profile</label><select onchange="updateE2EPipelineOption('deployProfile', this.value)">${pipeline.deployProfiles.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployProfile ? "selected" : ""}>${esc(item.label)} — ${esc(item.host === "__manual__" ? "manual" : item.host)}</option>`).join("")}</select></div>
            <div class="e2e-compact-fields"><label>SSH user<input value="${esc(pipeline.deployUser || profile?.user || "")}" onchange="updateE2EPipelineOption('deployUser', this.value)" /></label><label>Host<input value="${esc(pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "")}" onchange="updateE2EPipelineOption('deployHost', this.value)" /></label></div>
            <div class="field"><label>Model name on Jetson</label><input value="${esc(pipeline.deployName)}" onchange="updateE2EPipelineOption('deployName', this.value)" /><div class="field-hint">${esc(deployment?.label || (deployRun ? "No compatible deployment preset" : "Select an exported run"))} · ${esc(deployment?.model_name || "")}</div></div>
            <div class="notice compact">TensorRT engineは転送しません。Jetsonの実行用Docker環境内で生成してください。</div>
            <div class="field-hint">${esc(pipeline.remoteRoot || profile?.remote_root || "")}</div>
            <button class="primary ${actionBusy("e2e-pipeline:deploy") ? "is-busy" : ""}" onclick="deployE2EModel()" ${deployRun?.onnx_path && pipeline.deployName && deployment ? "" : "disabled"} ${actionButtonAttrs("e2e-pipeline:deploy", "Deployment is starting...")}>${esc(actionButtonLabel("e2e-pipeline:deploy", "Transfer model", "Starting..."))}</button>
          </article>
        </div>
        <details class="e2e-pipeline-tasks" ${pipelineTasks.some(isActiveTask) ? "open" : ""}><summary>Pipeline tasks (${pipelineTasks.length})</summary>${renderTaskTable(pipelineTasks)}</details>
      </div>
    </section>`;
}

const SHARED_VIT_TASK_KINDS = [
  "shared-vit-detection-train",
  "shared-vit-export",
  "shared-vit-deploy",
];

function isSharedVitTask(task) {
  return SHARED_VIT_TASK_KINDS.includes(task?.kind);
}

function selectedSharedVitControlRun() {
  return state.sharedVitPipeline.controlRuns.find(
    (item) => item.path === state.sharedVitPipeline.controlRunDir,
  ) || null;
}

function selectedSharedVitDetectionRun() {
  return state.sharedVitPipeline.detectionRuns.find(
    (item) => item.path === state.sharedVitPipeline.detectionRunDir,
  ) || null;
}

function selectedSharedVitModel() {
  return state.sharedVitPipeline.models.find(
    (item) => item.path === state.sharedVitPipeline.modelDir,
  ) || null;
}

function selectedSharedVitProfile() {
  return state.sharedVitPipeline.deployProfiles.find(
    (item) => item.id === state.sharedVitPipeline.deployProfile,
  ) || null;
}

function sharedVitControlModality(run) {
  return String(run?.image_topic || "").endsWith("/event_image") ? "event_image" : "rgb";
}

function sharedVitName(kind, modality = state.sharedVitPipeline.modality) {
  const sensor = modality === "event_image" ? "evs" : "rgb";
  const role = kind === "detection" ? "det-head" : "shared";
  return `vit-dinov3-vits16-${sensor}-${role}_${compactLocalDateTime()}`.slice(0, 64);
}

function updateSharedVitOption(key, value) {
  const pipeline = state.sharedVitPipeline;
  if (!(key in pipeline)) return;
  const numberKeys = ["epochs", "batch", "workers", "learningRate", "weightDecay", "opset"];
  if (numberKeys.includes(key)) pipeline[key] = Number(value);
  else pipeline[key] = String(value ?? "");
  if (key === "backboneWeights") {
    const weight = pipeline.weights.find((item) => item.path === pipeline.backboneWeights);
    if (weight?.modality) {
      pipeline.modality = weight.modality;
      pipeline.detectionRunName = sharedVitName("detection", weight.modality);
      pipeline.modelName = sharedVitName("model", weight.modality);
    }
  }
  if (key === "modality") {
    pipeline.detectionRunName = sharedVitName("detection", pipeline.modality);
    pipeline.modelName = sharedVitName("model", pipeline.modality);
  }
  if (key === "detectionRunDir") {
    const run = selectedSharedVitDetectionRun();
    if (run?.modality) pipeline.modality = run.modality;
    if (run?.mean?.length === 3) pipeline.mean = run.mean.join(",");
    if (run?.std?.length === 3) pipeline.std = run.std.join(",");
  }
  if (key === "modelDir") {
    const model = selectedSharedVitModel();
    if (model?.name) pipeline.deployName = model.name;
  }
  if (key === "deployProfile") {
    const profile = selectedSharedVitProfile();
    if (profile) {
      pipeline.deployUser = profile.user || "";
      pipeline.deployHost = profile.host === "__manual__" ? "" : (profile.host || "");
      const root = String(profile.remote_root || "").replace(/\/$/, "");
      pipeline.remoteRoot = root.endsWith("/shared_vit") ? root : `${root}/shared_vit`;
    }
  }
  render();
}

function prepareSharedVitControlTraining(modality) {
  const pipeline = state.e2ePipeline;
  pipeline.experiment = modality === "event_image"
    ? "dinov3_vits16_eventstate_frozen_head"
    : "dinov3_vits16_frozen_head";
  pipeline.runName = suggestedE2ERunName();
  state.e2eWorkspace = "train";
  render();
  toast(`${modality === "event_image" ? "EVS" : "RGB"} frozen-backbone preset selected.`);
}

function trainSharedVitDetection() {
  const pipeline = state.sharedVitPipeline;
  return startE2EPipelineTask(
    "shared-vit:train-detection",
    "/api/shared-vit/detection-training/start",
    "Train shared ViT detection head",
    pipeline.detectionRunName,
    {
      dataset_yaml: pipeline.datasetYaml,
      backbone_weights: pipeline.backboneWeights,
      run_name: pipeline.detectionRunName,
      modality: pipeline.modality,
      mean: pipeline.mean,
      std: pipeline.std,
      epochs: pipeline.epochs,
      batch: pipeline.batch,
      workers: pipeline.workers,
      device: pipeline.device,
      learning_rate: pipeline.learningRate,
      weight_decay: pipeline.weightDecay,
    },
  );
}

function exportSharedVitModel() {
  const pipeline = state.sharedVitPipeline;
  const control = selectedSharedVitControlRun();
  const detection = selectedSharedVitDetectionRun();
  return startE2EPipelineTask(
    "shared-vit:export",
    "/api/shared-vit/export-onnx",
    "Assemble shared ViT ONNX",
    pipeline.modelName,
    {
      control_checkpoint: control?.best_checkpoint || "",
      detection_checkpoint: detection?.best_checkpoint || "",
      model_name: pipeline.modelName,
      opset: pipeline.opset,
    },
  );
}

function deploySharedVitModel() {
  const pipeline = state.sharedVitPipeline;
  const model = selectedSharedVitModel();
  const profile = selectedSharedVitProfile();
  const user = pipeline.deployUser || profile?.user || "";
  const host = pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "";
  return startE2EPipelineTask(
    "shared-vit:deploy",
    "/api/shared-vit/deploy",
    "Deploy shared ViT to Jetson",
    `${user}@${host}`,
    {
      model_path: model?.onnx_path || "",
      profile: pipeline.deployProfile,
      user,
      host,
      remote_root: pipeline.remoteRoot,
      deploy_name: pipeline.deployName,
    },
  );
}

function renderSharedVitPipeline() {
  const pipeline = state.sharedVitPipeline;
  const control = selectedSharedVitControlRun();
  const detection = selectedSharedVitDetectionRun();
  const model = selectedSharedVitModel();
  const profile = selectedSharedVitProfile();
  const controlModality = sharedVitControlModality(control);
  const compatibleModality = !control || !detection?.modality || controlModality === detection.modality;
  const tasks = state.tasks.filter(isSharedVitTask).slice(0, 10);
  const user = pipeline.deployUser || profile?.user || "";
  const host = pipeline.deployHost || (profile?.host === "__manual__" ? "" : profile?.host) || "";
  return `
    <section class="panel e2e-pipeline-panel">
      <div class="panel-header"><h2>Shared ViT · Control + Detection</h2><span class="spacer"></span><button onclick="refreshAll()">Refresh artifacts</button></div>
      <div class="panel-body">
        <div class="notice compact">DINOv3 ViT-S/16 backboneを1回だけ推論し、Control headとYOLO互換Detection headへ同じ特徴を渡します。入力は固定212 × 120、backboneは学習しません。</div>
        <div class="e2e-pipeline-progress">
          ${[
            ["Backbone", pipeline.weights.length ? `${pipeline.weights.length} weight(s)` : "weight required", Boolean(pipeline.weights.length)],
            ["Control", pipeline.controlRuns.some((item) => item.best_checkpoint) ? "head ready" : "not trained", pipeline.controlRuns.some((item) => item.best_checkpoint)],
            ["Detection", pipeline.detectionRuns.some((item) => item.best_checkpoint) ? "head ready" : "not trained", pipeline.detectionRuns.some((item) => item.best_checkpoint)],
            ["Shared ONNX", pipeline.models.length ? "exported" : "not exported", Boolean(pipeline.models.length)],
            ["Jetson", state.tasks.some((item) => item.kind === "shared-vit-deploy" && item.status === "success") ? "deployed" : "not deployed", state.tasks.some((item) => item.kind === "shared-vit-deploy" && item.status === "success")],
          ].map(([label, detail, done], index) => `<div class="${done ? "done" : ""}"><span>${index + 1}</span><strong>${esc(label)}</strong><small>${esc(detail)}</small></div>`).join("")}
        </div>
        <div class="e2e-pipeline-grid">
          <article class="e2e-pipeline-stage">
            <header><span>01</span><div><strong>Backbone + Control head</strong><small>DINOv3 / EventState weight and frozen training</small></div></header>
            <div class="field"><label>Backbone weights</label><select onchange="updateSharedVitOption('backboneWeights', this.value)"><option value="">No weights installed</option>${pipeline.weights.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.backboneWeights ? "selected" : ""}>${esc(item.modality === "event_image" ? "EVS" : "RGB")} · ${esc(item.name)}</option>`).join("")}</select><div class="field-hint">配置先: ${esc(pipeline.weightRoot)}</div></div>
            <div class="e2e-compact-fields"><label>Modality<select onchange="updateSharedVitOption('modality', this.value)">${[["rgb","RGB"],["event_image","EVS / EventState"]].map(([value,label]) => `<option value="${value}" ${pipeline.modality === value ? "selected" : ""}>${label}</option>`).join("")}</select></label><label>Input<input value="212 × 120" disabled /></label></div>
            <div class="button-stack"><button onclick="prepareSharedVitControlTraining('rgb')">Open RGB control training</button><button onclick="prepareSharedVitControlTraining('event_image')">Open EVS control training</button></div>
            <div class="field"><label>Control run for assembly</label><select onchange="updateSharedVitOption('controlRunDir', this.value)"><option value="">Select frozen ViT run</option>${pipeline.controlRuns.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.controlRunDir ? "selected" : ""}>${esc(sharedVitControlModality(item) === "event_image" ? "EVS" : "RGB")} · ${esc(item.name)} — ${item.best_checkpoint ? "best.pt" : "incomplete"}</option>`).join("")}</select></div>
          </article>

          <article class="e2e-pipeline-stage">
            <header><span>02</span><div><strong>Train Detection head</strong><small>Separate dataset; frozen shared backbone</small></div></header>
            <div class="field"><label>Detection dataset</label><select onchange="updateSharedVitOption('datasetYaml', this.value)"><option value="">No dataset found</option>${pipeline.datasets.map((item) => { const path = objectDetectionDatasetPath(item); return `<option value="${esc(path)}" ${path === pipeline.datasetYaml ? "selected" : ""}>${esc(item.name || shortName(path))}${item.valid === false ? " — invalid" : ""}</option>`; }).join("")}</select></div>
            <div class="field"><label>Run name</label><input value="${esc(pipeline.detectionRunName)}" onchange="updateSharedVitOption('detectionRunName', this.value)" /><div class="field-hint">${esc(pipeline.detectionRunRoot)}</div></div>
            <div class="e2e-compact-fields"><label>Epochs<input type="number" min="1" value="${esc(pipeline.epochs)}" onchange="updateSharedVitOption('epochs', this.value)" /></label><label>Batch<input type="number" min="1" value="${esc(pipeline.batch)}" onchange="updateSharedVitOption('batch', this.value)" /></label><label>Device<select onchange="updateSharedVitOption('device', this.value)">${[["cuda","CUDA"],["mps","Apple MPS"],["cpu","CPU"]].map(([value,label]) => `<option value="${value}" ${pipeline.device === value ? "selected" : ""}>${label}</option>`).join("")}</select></label></div>
            <details><summary>Normalization & optimizer</summary><div class="field"><label>Mean (RGB order)</label><input value="${esc(pipeline.mean)}" onchange="updateSharedVitOption('mean', this.value)" /></div><div class="field"><label>Std (RGB order)</label><input value="${esc(pipeline.std)}" onchange="updateSharedVitOption('std', this.value)" /></div><div class="e2e-compact-fields"><label>LR<input type="number" min="0.00000001" step="0.0001" value="${esc(pipeline.learningRate)}" onchange="updateSharedVitOption('learningRate', this.value)" /></label><label>Weight decay<input type="number" min="0" step="0.0001" value="${esc(pipeline.weightDecay)}" onchange="updateSharedVitOption('weightDecay', this.value)" /></label><label>Workers<input type="number" min="0" value="${esc(pipeline.workers)}" onchange="updateSharedVitOption('workers', this.value)" /></label></div></details>
            <button class="primary ${actionBusy("shared-vit:train-detection") ? "is-busy" : ""}" onclick="trainSharedVitDetection()" ${pipeline.backboneWeights && pipeline.datasetYaml && pipeline.detectionRunName ? "" : "disabled"} ${actionButtonAttrs("shared-vit:train-detection", "Detection training is starting...")}>${esc(actionButtonLabel("shared-vit:train-detection", "Start head-only training", "Starting..."))}</button>
          </article>

          <article class="e2e-pipeline-stage">
            <header><span>03</span><div><strong>Assemble one ONNX</strong><small>One backbone pass; two TensorRT outputs</small></div></header>
            <div class="field"><label>Detection run</label><select onchange="updateSharedVitOption('detectionRunDir', this.value)"><option value="">Select detection run</option>${pipeline.detectionRuns.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.detectionRunDir ? "selected" : ""}>${esc(item.modality === "event_image" ? "EVS" : "RGB")} · ${esc(item.name)} — ${item.best_checkpoint ? "best.pt" : "incomplete"}</option>`).join("")}</select></div>
            ${control && detection ? `<div class="notice ${compatibleModality ? "compact" : "error compact"}">${compatibleModality ? `Compatible modality: ${esc(controlModality)}` : `Modality mismatch: Control=${esc(controlModality)}, Detection=${esc(detection.modality)}`}. Export時にbackbone fingerprintとmean/stdも検証します。</div>` : ""}
            <div class="field"><label>Shared model name</label><input value="${esc(pipeline.modelName)}" onchange="updateSharedVitOption('modelName', this.value)" /><div class="field-hint">${esc(pipeline.modelRoot)}</div></div>
            <div class="e2e-compact-fields"><label>ONNX opset<input type="number" min="11" max="20" value="${esc(pipeline.opset)}" onchange="updateSharedVitOption('opset', this.value)" /></label><label>Outputs<input value="control, detections" disabled /></label></div>
            <button class="primary ${actionBusy("shared-vit:export") ? "is-busy" : ""}" onclick="exportSharedVitModel()" ${control?.best_checkpoint && detection?.best_checkpoint && compatibleModality ? "" : "disabled"} ${actionButtonAttrs("shared-vit:export", "Assembly is starting...")}>${esc(actionButtonLabel("shared-vit:export", "Assemble & export ONNX", "Starting..."))}</button>
          </article>

          <article class="e2e-pipeline-stage">
            <header><span>04</span><div><strong>Deploy shared model</strong><small>One TensorRT engine + two ROS 2 decoders</small></div></header>
            <div class="field"><label>Assembled model</label><select onchange="updateSharedVitOption('modelDir', this.value)"><option value="">Select shared model</option>${pipeline.models.map((item) => `<option value="${esc(item.path)}" ${item.path === pipeline.modelDir ? "selected" : ""}>${esc(item.modality || "unknown")} · ${esc(item.name)}</option>`).join("")}</select></div>
            <div class="field"><label>Connection profile</label><select onchange="updateSharedVitOption('deployProfile', this.value)"><option value="">Select profile</option>${pipeline.deployProfiles.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployProfile ? "selected" : ""}>${esc(item.label || item.id)} — ${esc(item.host === "__manual__" ? "manual" : item.host)}</option>`).join("")}</select></div>
            <div class="e2e-compact-fields"><label>SSH user<input value="${esc(user)}" onchange="updateSharedVitOption('deployUser', this.value)" /></label><label>Host<input value="${esc(host)}" onchange="updateSharedVitOption('deployHost', this.value)" /></label></div>
            <div class="field"><label>Remote model root</label><input value="${esc(pipeline.remoteRoot)}" onchange="updateSharedVitOption('remoteRoot', this.value)" /></div>
            <div class="field"><label>Deploy name</label><input value="${esc(pipeline.deployName)}" onchange="updateSharedVitOption('deployName', this.value)" /></div>
            <div class="notice compact">TensorRT build is disabled. This step only transfers the shared ONNX model and metadata.</div>
            <button class="primary ${actionBusy("shared-vit:deploy") ? "is-busy" : ""}" onclick="deploySharedVitModel()" ${model?.onnx_path && user && host ? "" : "disabled"} ${actionButtonAttrs("shared-vit:deploy", "Deployment is starting...")}>${esc(actionButtonLabel("shared-vit:deploy", "Transfer shared model", "Starting..."))}</button>
            <div class="field-hint">ROS 2: <code>enable_shared_vit_inference:=true</code>. E2E/YOLO単独推論とは同時に有効化しません。</div>
          </article>
        </div>
        <details class="e2e-pipeline-tasks" ${tasks.some(isActiveTask) ? "open" : ""}><summary>Shared ViT tasks (${tasks.length})</summary>${renderTaskTable(tasks)}</details>
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
  const booleanKeys = ["simplify"];
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
          <div class="notice compact">Training and ONNX export run on this Notebook. Deployment transfers the exported files without building TensorRT <code>model.plan</code>.</div>
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
              <header><span>04</span><div><strong>Deploy to Jetson</strong><small>Transfer exported ONNX files only</small></div></header>
              <div class="field"><label>Connection profile</label><select onchange="updateObjectDetectionPipelineOption('deployProfile', this.value)"><option value="">Select profile</option>${pipeline.deployProfiles.map((item) => `<option value="${esc(item.id)}" ${item.id === pipeline.deployProfile ? "selected" : ""}>${esc(item.label || item.id)} — ${esc(item.host === "__manual__" ? "manual" : item.host)}</option>`).join("")}</select></div>
              <div class="e2e-compact-fields"><label>SSH user<input value="${esc(user)}" onchange="updateObjectDetectionPipelineOption('deployUser', this.value)" /></label><label>Host<input value="${esc(host)}" onchange="updateObjectDetectionPipelineOption('deployHost', this.value)" /></label></div>
              <div class="field"><label>Remote model root</label><input value="${esc(pipeline.remoteRoot || profile?.remote_root || "")}" onchange="updateObjectDetectionPipelineOption('remoteRoot', this.value)" /></div>
              <div class="field"><label>Model name</label><input value="${esc(pipeline.deployName)}" onchange="updateObjectDetectionPipelineOption('deployName', this.value)" /><div class="field-hint">Installed as &lt;remote root&gt;/${esc(pipeline.deployName || "model")}/model.onnx.</div></div>
              <div class="notice compact">TensorRT build is disabled. This step only transfers the ONNX model and metadata.</div>
              <button class="primary ${actionBusy("object-detection:deploy") ? "is-busy" : ""}" onclick="deployObjectDetectionModel()" ${run?.onnx_path && host && user ? "" : "disabled"} ${actionButtonAttrs("object-detection:deploy", "Deployment is starting...")}>${esc(actionButtonLabel("object-detection:deploy", "Transfer model", "Starting..."))}</button>
            </article>
          </div>
          <details class="e2e-pipeline-tasks" ${pipelineTasks.some(isActiveTask) ? "open" : ""}><summary>Object-detection tasks (${pipelineTasks.length})</summary>${renderTaskTable(pipelineTasks)}</details>
        </div>
      </section>
    </div>`;
}
