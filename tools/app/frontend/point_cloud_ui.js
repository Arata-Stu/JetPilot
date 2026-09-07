/* The editable HD map always uses the original raster coordinate system. */
function newPointCloudView(key = '') {
  return { key, mode: false, data: null, loading: false, error: '', low: 0, high: 0,
    style: 'density', cellSizeM: 0.05, contrast: 1.8, densityCache: null };
}
let pointCloudView = newPointCloudView();
let pointCloudRedraw = 0;
function pointCloudState(detail) {
  const key = `${detail.map.path}|${detail.point_cloud_url || ''}`;
  if (pointCloudView.key !== key) pointCloudView = newPointCloudView(key);
  return pointCloudView;
}
function pointCloudActive(detail) {
  const view = pointCloudState(detail);
  return view.mode && !!view.data;
}
function renderPointCloudControls(detail) {
  const view = pointCloudState(detail);
  const available = !!detail.point_cloud_url;
  return `<div class="inspector-block" id="point-cloud-controls">
    <strong>点群の密度・高さ</strong>
    <p class="field-hint">背景だけを切り替えます。編集中の線の位置は変わりません。</p>
    <label><input type="checkbox" ${view.mode ? 'checked' : ''} ${!available || view.loading ? 'disabled' : ''} onchange="togglePointCloud(this.checked)"> 高さ付き点群を表示</label>
    ${!available ? '<p class="field-hint">この地図には高さ付き点群がありません。Rasterを一度再生成してください。</p>' : ''}
    ${view.loading ? '<p role="status">点群を読み込み中…</p>' : ''}
    ${view.error ? `<p role="alert">${esc(view.error)}</p><button onclick="togglePointCloud(true)">再試行</button>` : ''}
    ${view.data ? `<div ${view.mode ? '' : 'hidden'}>
      <label>表示 <select aria-label="点群の表示方法" onchange="setPointCloudStyle(this.value)">
        <option value="density" ${view.style === 'density' ? 'selected' : ''}>密度で濃淡</option>
        <option value="points" ${view.style === 'points' ? 'selected' : ''}>従来の点表示</option>
      </select></label>
      ${view.style === 'density' ? `<p class="field-hint">疎い点は薄く、密集した輪郭は明るく表示します。濃淡は表示用点群の密度で、障害物の占有率ではありません。</p>
      <label>集計幅 <output id="point-cloud-cellSizeM-value">${Math.round(view.cellSizeM * 100)} cm</output>
        <input style="width:100%" aria-label="密度の集計幅" type="range" min="0.02" max="0.2" step="0.01" value="${view.cellSizeM}" oninput="setPointCloudDensityOption('cellSizeM',this.value)"></label>
      <label>コントラスト <output id="point-cloud-contrast-value">${view.contrast.toFixed(1)}</output>
        <input style="width:100%" aria-label="密度のコントラスト" type="range" min="0.5" max="3" step="0.1" value="${view.contrast}" oninput="setPointCloudDensityOption('contrast',this.value)"></label>` : ''}
      <p class="field-hint">地図座標のZ（m）です。床からの高さではありません。</p>
      ${['low','high'].map((key, i) => `<label>${i ? '上限' : '下限'} <output id="point-cloud-${key}-value">${view[key].toFixed(3)} m</output>
      <input style="width:100%" aria-label="点群の高さ${i ? '上限' : '下限'}" id="point-cloud-${key}" type="range" min="${view.min}" max="${view.max}" step="any" value="${view[key]}" oninput="setPointCloudHeight('${key}',this.value)"></label>`).join('')}
      <button onclick="resetPointCloudHeight()">全高さに戻す</button>
      <p class="field-hint" id="point-cloud-count"></p>
      ${view.data.total_points > view.data.points.length ? `<p class="field-hint">表示用に${view.data.total_points.toLocaleString()}点から${view.data.points.length.toLocaleString()}点へ間引いています。元のスナップショットは保持しています。</p>` : ''}
    </div>` : ''}
  </div>`;
}
function refreshPointCloudControls() {
  const detail = state.selectedMapDetail;
  const root = document.getElementById('point-cloud-controls');
  if (detail && root) root.outerHTML = renderPointCloudControls(detail);
  drawMapPreview();
}
function validatePointCloud(data, raster) {
  if (data.issue) throw new Error(data.issue);
  if (data.version !== 1 || !Array.isArray(data.points) || data.points.length > 250000) throw new Error('点群データの形式が不正です。Rasterを再生成してください。');
  const a = data.raster || {}, b = raster || {};
  for (const key of ['width', 'height', 'resolution_m_per_px']) {
    if (!Number.isFinite(a[key]) || !Number.isFinite(Number(b[key])) || Math.abs(a[key] - Number(b[key])) > 1e-8) throw new Error('点群と編集地図の座標設定が一致しません。Rasterを再生成してください。');
  }
  if (!Array.isArray(a.origin_xy_yaw) || a.origin_xy_yaw.length !== 3 || a.origin_xy_yaw.some((v,i) => !Number.isFinite(v) || !Number.isFinite(Number(b.origin_xy_yaw?.[i])) || Math.abs(v - Number(b.origin_xy_yaw?.[i])) > 1e-8)) throw new Error('点群と編集地図の原点が一致しません。');
  if (!data.points.length) throw new Error('スナップショットに表示可能な点群がありません。');
  if (data.points.some(p => !Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite))) throw new Error('点群に無効な座標が含まれています。');
  return data;
}
async function togglePointCloud(enabled) {
  const detail = state.selectedMapDetail;
  if (!detail) return;
  const view = pointCloudState(detail);
  if (!enabled) { view.mode = false; refreshPointCloudControls(); return; }
  if (view.data) { view.mode = true; refreshPointCloudControls(); return; }
  view.loading = true; view.error = ''; refreshPointCloudControls();
  try {
    const data = validatePointCloud(await api(detail.point_cloud_url), detail.raster);
    if (pointCloudView !== view) return;
    view.data = data;
    view.min = Infinity; view.max = -Infinity;
    for (const p of data.points) { view.min = Math.min(view.min, p[2]); view.max = Math.max(view.max, p[2]); }
    view.low = view.min; view.high = view.max; view.mode = true;
  } catch (error) {
    view.error = `点群を表示できません：${error.message}`; view.mode = false;
  } finally {
    view.loading = false;
    if (pointCloudView === view) refreshPointCloudControls();
  }
}
function setPointCloudHeight(key, value) {
  const view = pointCloudView, n = Number(value);
  if (!view.data || !Number.isFinite(n)) return;
  view[key] = Math.max(view.min, Math.min(view.max, n));
  if (view.low > view.high) view[key === 'low' ? 'high' : 'low'] = view[key];
  for (const k of ['low','high']) {
    const input = document.getElementById(`point-cloud-${k}`);
    const output = document.getElementById(`point-cloud-${k}-value`);
    if (input) input.value = view[k];
    if (output) output.textContent = `${view[k].toFixed(3)} m`;
  }
  if (!pointCloudRedraw) pointCloudRedraw = requestAnimationFrame(() => { pointCloudRedraw = 0; drawMapPreview(); });
}
function resetPointCloudHeight() {
  pointCloudView.low = pointCloudView.min; pointCloudView.high = pointCloudView.max;
  refreshPointCloudControls();
}

function setPointCloudStyle(style) {
  if (!['density', 'points'].includes(style)) return;
  pointCloudView.style = style;
  refreshPointCloudControls();
}

function setPointCloudDensityOption(key, value) {
  const limits = {cellSizeM: [0.02, 0.2], contrast: [0.5, 3]};
  const range = limits[key], n = Number(value);
  if (!range || !Number.isFinite(n)) return;
  pointCloudView[key] = Math.max(range[0], Math.min(range[1], n));
  const output = document.getElementById(`point-cloud-${key}-value`);
  if (output) output.textContent = key === 'cellSizeM'
    ? `${Math.round(pointCloudView[key] * 100)} cm` : pointCloudView[key].toFixed(1);
  if (!pointCloudRedraw) pointCloudRedraw = requestAnimationFrame(() => { pointCloudRedraw = 0; drawMapPreview(); });
}

function pointCloudDensity(detail, view) {
  const cached = view.densityCache;
  if (cached && cached.data === view.data && cached.low === view.low &&
      cached.high === view.high && cached.cellSizeM === view.cellSizeM) return cached;
  // Bin in fixed map coordinates, not screen pixels: zoom does not change density.
  const raster = detail.raster;
  const project = mapPointProjector(detail, raster.width, raster.height);
  const cellPx = view.cellSizeM / raster.resolution_m_per_px;
  const columns = Math.ceil(raster.width / cellPx);
  const cells = new Map();
  let count = 0;
  for (const p of view.data.points) {
    if (p[2] < view.low || p[2] > view.high) continue;
    const [x, y] = project(p);
    if (x < 0 || y < 0 || x >= raster.width || y >= raster.height) continue;
    const col = Math.floor(x / cellPx), row = Math.floor(y / cellPx);
    const key = row * columns + col;
    cells.set(key, (cells.get(key) || 0) + 1);
    count++;
  }
  const histogram = new Map();
  for (const n of cells.values()) histogram.set(n, (histogram.get(n) || 0) + 1);
  let accumulated = 0, ceiling = 4;
  // A few extreme clusters should not make all other boundaries disappear.
  for (const [n, frequency] of [...histogram].sort((a, b) => a[0] - b[0])) {
    accumulated += frequency;
    if (accumulated >= cells.size * 0.98) { ceiling = Math.max(4, n); break; }
  }
  view.densityCache = {data: view.data, low: view.low, high: view.high,
    cellSizeM: view.cellSizeM, cells, columns, cellPx, ceiling, count};
  return view.densityCache;
}

function pointCloudDensityAlpha(count, ceiling, contrast) {
  return Math.pow(Math.min(1, Math.log1p(count) / Math.log1p(ceiling)), contrast);
}

function drawPointCloud(ctx, detail, width, height) {
  if (!pointCloudActive(detail)) return;
  const view = pointCloudView;
  const project = mapPointProjector(detail, width, height);
  let count = 0;
  if (view.style === 'density') {
    const density = pointCloudDensity(detail, view);
    count = density.count;
    if (state.mapLayers.landmark) {
      const sx = width / detail.raster.width, sy = height / detail.raster.height;
      for (const [key, n] of density.cells) {
        const x = (key % density.columns) * density.cellPx;
        const y = Math.floor(key / density.columns) * density.cellPx;
        const alpha = pointCloudDensityAlpha(n, density.ceiling, view.contrast);
        ctx.fillStyle = `rgba(169,197,217,${alpha.toFixed(3)})`;
        ctx.fillRect(x * sx, y * sy,
          Math.min(density.cellPx, detail.raster.width - x) * sx,
          Math.min(density.cellPx, detail.raster.height - y) * sy);
      }
    }
  } else {
    ctx.fillStyle = '#a9c5d9';
    for (const p of view.data.points) {
      if (p[2] < view.low || p[2] > view.high) continue;
      const [x,y] = project(p);
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      count++;
      if (state.mapLayers.landmark) ctx.fillRect(x, y, 1.5, 1.5);
    }
  }
  const output = document.getElementById('point-cloud-count');
  if (output) output.textContent = `画面内・選択高さの点：${count.toLocaleString()} / ${view.data.points.length.toLocaleString()}${state.mapLayers.landmark ? '' : '（Landmarkレイヤー非表示）'}`;
}
