// JetPilot Console: transfer workflow. Shared state is initialized by app.js.
function fillTransferLocal(path) {
  state.jetsonTransfer.pushLocalPath = String(path || "");
  state.tab = "jetson";
  render();
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

function toggleJetsonPullGroup(groupPath, checked) {
  if (state.jetsonTransfer.running || actionBusy("jetson:pull")) return;
  const selected = new Set(selectedJetsonPullPaths());
  const group = jetsonRosbagGroups().find((item) => item.path === groupPath);
  (group?.sequences || []).forEach((sequence) => {
    if (checked) selected.add(sequence.path);
    else selected.delete(sequence.path);
  });
  setJetsonPullSelection([...selected]);
}

function toggleJetsonPullGroupOpen(groupPath, open) {
  state.jetsonTransfer.collapsedPullGroups[groupPath] = !open;
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
  state.jetsonTransfer.pushLocalPath = paths.local;
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

function jetsonRosbagRelativePath(path) {
  const fullPath = String(path || "").replace(/\/+$/g, "");
  const recordRoot = String(
    state.jetsonTarget?.record_root || state.config?.jetson_record_root || "",
  ).replace(/\/+$/g, "");
  if (recordRoot && fullPath.startsWith(`${recordRoot}/`)) {
    return fullPath.slice(recordRoot.length + 1);
  }
  return fullPath.split("/").filter(Boolean).slice(-2).join("/");
}

function jetsonRosbagGroups(sequences = jetsonRosbagSequences()) {
  const groups = new Map();
  sequences.forEach((sequence) => {
    const parts = jetsonRosbagRelativePath(sequence.path).split("/").filter(Boolean);
    parts.pop();
    const groupPath = parts.join("/");
    if (!groups.has(groupPath)) groups.set(groupPath, []);
    groups.get(groupPath).push(sequence);
  });
  return [...groups.entries()].map(([path, groupedSequences]) => ({
    path,
    sequences: groupedSequences,
  }));
}

function jetsonRosbagGroupLabel(groupPath) {
  return groupPath
    ? `record / ${groupPath.split("/").filter(Boolean).join(" / ")}`
    : "record";
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
