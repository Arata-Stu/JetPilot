// JetPilot Console: list workflow. Shared state is initialized by app.js.
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
