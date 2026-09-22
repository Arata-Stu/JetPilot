// JetPilot Console: preflight workflow. Shared state is initialized by app.js.
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
