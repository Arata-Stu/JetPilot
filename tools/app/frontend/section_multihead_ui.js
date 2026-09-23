// Section Multihead: reusable datasets -> all-head training -> per-head bag evaluation.
const sectionMultihead = {
  assets: { maps: [], datasets: [], runs: [], analyses: [], weights: [] },
  map: "", sections: [], bags: [], mode: "recorded", labelSource: "auto",
  imageTopic: "/realsense/color/image_raw", controlTopic: "/teleop/control_cmd",
  sectionTopic: "/localization/current_section", poseTopic: "/visual_slam/tracking/odometry",
  timestampSource: "header", throttle: 0.2, filterThrottle: "", localizationMap: "",
  localizationMode: "origin", cameraTopicConfig: "", replayRate: 0.5, datasetName: "", runName: "", analysisName: "",
  backbone: "", epochs: 30, batchSize: 32, workers: 4, device: "cuda", learningRate: 0.001,
  heads: [ { name: "straight", dataset: "", throttle: 0.3, generic: false },
           { name: "curve", dataset: "", throttle: 0.2, generic: false },
           { name: "generic", dataset: "", throttle: 0.2, generic: true } ],
  run: "", analysisHead: "", user: "", host: "", buildEngine: true,
  remoteRoot: "/workspaces/ros2_ws/models/e2e/section_multihead", busy: false, message: "",
};
function smEscape(value) { return esc(String(value ?? "")); }
function smOptions(records, selected, label = "name") {
  return '<option value="">選択してください</option>' + records.map(r =>
    `<option value="${smEscape(r.path)}" ${r.path === selected ? "selected" : ""}>${smEscape(r[label] || r.path)}</option>`).join("");
}
function smSet(key, value) { sectionMultihead[key] = value; }
function smSelectMap(value) {
  sectionMultihead.map = value; sectionMultihead.sections = [];
  sectionMultihead.localizationMap = value.slice(0, value.lastIndexOf("/")); render();
}
function smToggle(key, value, checked) {
  const values = new Set(sectionMultihead[key]); checked ? values.add(value) : values.delete(value);
  sectionMultihead[key] = [...values];
}
function smDataset(index, value) {
  const h = sectionMultihead.heads[index]; h.dataset = value;
  const d = sectionMultihead.assets.datasets.find(d => d.path === value);
  if (d) h.throttle = d.recommended_throttle; render();
}
function smAddHead() {
  sectionMultihead.heads.push({ name: `head_${sectionMultihead.heads.length + 1}`, dataset: "", throttle: .2, generic: false }); render();
}
function smRun(value) {
  sectionMultihead.run = value;
  sectionMultihead.analysisHead = sectionMultihead.assets.runs.find(r => r.path === value)?.heads?.[0]?.name || ""; render();
}
async function refreshSectionMultihead() {
  try {
    sectionMultihead.assets = await api("/api/section-multihead/pipeline");
    sectionMultihead.user ||= sectionMultihead.assets.default_user || "";
    sectionMultihead.host ||= sectionMultihead.assets.default_host || "";
    const suffix = compactLocalDateTime();
    sectionMultihead.datasetName ||= `section_dataset_${suffix}`;
    sectionMultihead.runName ||= `section_multihead_${suffix}`;
    sectionMultihead.analysisName ||= `head_analysis_${suffix}`;
    if (state.e2eWorkspace === "section-multihead") render();
  } catch (error) { sectionMultihead.message = error.message; render(); }
}
async function smStart(action) {
  const s = sectionMultihead;
  const common = { rosbags: s.bags, image_topic: s.imageTopic, control_topic: s.controlTopic,
    timestamp_source: s.timestampSource };
  const bodies = {
    preprocess: { ...common, dataset_name: s.datasetName, map: s.map, sections: s.sections,
      mode: s.mode, label_source: s.labelSource, section_topic: s.sectionTopic, pose_topic: s.poseTopic,
      throttle: Number(s.throttle), filter_throttle: s.filterThrottle === "" ? null : Number(s.filterThrottle),
      localization_map: s.localizationMap, localization_mode: s.localizationMode, camera_topic_config: s.cameraTopicConfig, replay_rate: Number(s.replayRate) },
    train: { run_name: s.runName, backbone_weights: s.backbone, heads: s.heads,
      epochs: Number(s.epochs), batch_size: Number(s.batchSize), workers: Number(s.workers),
      learning_rate: Number(s.learningRate), device: s.device },
    export: { run: s.run },
    analyze: { ...common, run: s.run, head: s.analysisHead, analysis_name: s.analysisName },
    deploy: { run: s.run, user: s.user, host: s.host, remote_root: s.remoteRoot, build_engine: s.buildEngine },
  };
  s.busy = true; render();
  try {
    const result = await api(`/api/section-multihead/${action}`, { method: "POST", body: JSON.stringify(bodies[action]) });
    s.message = `開始しました: ${result.task?.title || action}。進捗と停止は下のタスク一覧で確認できます。`;
    await refreshAll();
  } catch (error) { s.message = error.message; }
  finally { s.busy = false; await refreshSectionMultihead(); }
}
function smInput(label, key, type = "text", extra = "") {
  return `<label>${label}<input type="${type}" value="${smEscape(sectionMultihead[key])}" ${extra} onchange="smSet('${key}', this.value)"></label>`;
}
function smPlot(report) {
  const points = report.preview || [];
  if (!points.length) return "";
  const line = key => points.map((p,i) => `${((p.sample_index ?? i) * 760 / Math.max(1,report.count-1)).toFixed(1)},${(80-70*Math.max(-1,Math.min(1,p[key]))).toFixed(1)}`).join(" ");
  return `<svg viewBox="0 0 760 160" role="img" aria-label="操舵の教師値とhead推論値の比較" style="width:100%;max-height:200px"><path d="M0 80H760" stroke="#888"/><polyline points="${line("target")}" fill="none" stroke="#76c7c0" stroke-width="1.5"/><polyline points="${line("prediction")}" fill="none" stroke="#ffb45c" stroke-width="1.5"/></svg><small>緑: 教師操舵 / 橙: head推論。横軸はサンプル順、縦軸は操舵 −1〜1。全結果は predictions.csv に保存。</small>`;
}
function renderSectionMultihead() {
  const s = sectionMultihead, a = s.assets;
  const map = a.maps.find(m => m.path === s.map);
  const run = a.runs.find(r => r.path === s.run);
  const disabled = s.busy ? "disabled" : "";
  const tasks = (state.tasks || []).filter(t => String(t.kind || "").startsWith("section-multihead-"));
  return `<section class="panel"><div class="panel-header"><h2>Section Multihead</h2><button onclick="refreshSectionMultihead()">一覧を更新</button></div>
    <div class="panel-body"><p>DINOv3を固定し、各headをそれぞれのdatasetで学習します。全headを1つのTensorRTモデルとして配備します。</p>
    ${s.message ? `<p role="status">${smEscape(s.message)}</p>` : ""}
    <h3>1. データセット作成</h3><div class="form-grid">
    <label>HD map<select onchange="smSelectMap(this.value)">${smOptions(a.maps,s.map)}</select></label>
    ${smInput("データセット名", "datasetName")}
    <label>区間の割り当て<select onchange="smSet('mode',this.value);render()"><option value="recorded" ${s.mode === "recorded" ? "selected" : ""}>bag内のsection・位置・TFを使う</option><option value="offline" ${s.mode === "offline" ? "selected" : ""}>保存地図でオフラインVSLAM</option></select></label>
    ${smInput("このdatasetに対応する実行時throttle", "throttle", "number", 'min="0" max="1" step="0.01"')}
    ${smInput("画像topic", "imageTopic")}${smInput("教師操作topic", "controlTopic")}
    <label>時刻<select onchange="smSet('timestampSource',this.value)"><option value="header" ${s.timestampSource === "header" ? "selected" : ""}>header（offline VSLAMでは必須）</option><option value="bag" ${s.timestampSource === "bag" ? "selected" : ""}>bag記録時刻</option></select></label>
    ${smInput("抽出するthrottle（空欄なら境界の加減速も含む）", "filterThrottle", "number", 'min="0" max="1" step="0.01"')}
    ${s.mode === "offline" ? `${smInput("定位用mapディレクトリ（同じ座標系）", "localizationMap")}${smInput("カメラtopic設定ファイル（空欄なら既定）", "cameraTopicConfig")}<label>再定位方式<select onchange="smSet('localizationMode',this.value)"><option value="origin" ${s.localizationMode === "origin" ? "selected" : ""}>地図原点からの再定位</option><option value="vgl" ${s.localizationMode === "vgl" ? "selected" : ""}>VGLで初期位置を推定</option></select></label>${smInput("bag再生倍率", "replayRate", "number", 'min="0.05" max="2" step="0.05"')}` : `<label>記録済みラベル<select onchange="smSet('labelSource',this.value)"><option value="auto" ${s.labelSource === "auto" ? "selected" : ""}>section優先、なければ位置・TF</option><option value="section" ${s.labelSource === "section" ? "selected" : ""}>section topic</option><option value="pose" ${s.labelSource === "pose" ? "selected" : ""}>位置・TFから地図上で判定</option></select></label>${smInput("section topic", "sectionTopic")}${smInput("位置topic", "poseTopic")}`}
    </div><fieldset><legend>抽出section（複数選択）</legend>
    <button onclick="sectionMultihead.sections = [...(sectionMultihead.assets.maps.find(m=>m.path===sectionMultihead.map)?.sections || [])];render()">全sectionを選択</button>
    ${(map?.sections || []).map(id => `<label style="display:inline-block;margin:8px"><input type="checkbox" value="${smEscape(id)}" ${s.sections.includes(id) ? "checked" : ""} onchange="smToggle('sections',this.value,this.checked)">${smEscape(id)}</label>`).join("") || "地図を選ぶとsectionを自動取得します。"}</fieldset>
    <fieldset><legend>rosbag（前処理・解析で共通、最大32個）</legend>
    <select multiple size="6" style="width:100%" onchange="sectionMultihead.bags = [...this.selectedOptions].map(o=>o.value)">${(state.rosbags || []).map(b => `<option value="${smEscape(b.path)}" ${s.bags.includes(b.path) ? "selected" : ""}>${smEscape(b.name || b.path)}</option>`).join("")}</select></fieldset>
    <p>複数bagを1つのdatasetにまとめます。汎用head用には全sectionを選択してください。未判定・古いラベルは除外し、件数を保存します。</p>
    <button ${disabled} onclick="smStart('preprocess')">データセットを作成</button>
    <h3>2. 全headを学習</h3><div class="form-grid">
    ${smInput("学習名", "runName")}<label>DINOv3重み<select onchange="smSet('backbone',this.value)">${smOptions(a.weights,s.backbone)}</select></label>
    ${smInput("各headのepoch数", "epochs", "number", 'min="1"')}${smInput("バッチサイズ", "batchSize", "number", 'min="1"')}
    ${smInput("デバイス", "device")}${smInput("データ読込workers", "workers", "number", 'min="0"')}${smInput("学習率", "learningRate", "number", 'min="0.00000001" step="0.0001"')}
    </div><table><thead><tr><th>head名</th><th>dataset / section</th><th>throttle</th><th>汎用</th><th></th></tr></thead><tbody>
    ${s.heads.map((h,i) => `<tr><td><input value="${smEscape(h.name)}" onchange="sectionMultihead.heads[${i}].name=this.value"></td><td><select onchange="smDataset(${i},this.value)">${smOptions(a.datasets,h.dataset)}</select><small>${smEscape(a.datasets.find(d=>d.path===h.dataset)?.sections?.join(", ") || "")}</small></td><td><input type="number" min="0" max="1" step="0.01" value="${h.throttle}" onchange="sectionMultihead.heads[${i}].throttle=Number(this.value)"></td><td><input type="radio" name="sm-generic" ${h.generic ? "checked" : ""} onchange="sectionMultihead.heads.forEach((h,j)=>h.generic=j===${i});render()"></td><td><button onclick="sectionMultihead.heads.splice(${i},1);render()">削除</button></td></tr>`).join("")}</tbody></table>
    <button onclick="smAddHead()">headを追加</button><button ${disabled} onclick="smStart('train')">全headを学習してONNXを書き出す</button>
    <p>各datasetをバッチごとに順番に学習します。共有ViTは更新しません。汎用以外のhead間でsectionの重複は不可です。</p>
    <h3>3. head別rosbag解析・Jetson転送</h3><div class="form-grid"><label>学習済みモデル<select onchange="smRun(this.value)">${smOptions(a.runs,s.run)}</select></label>
    <label>解析head<select onchange="smSet('analysisHead',this.value)">${(run?.heads || []).map(h=>`<option value="${smEscape(h.name)}" ${s.analysisHead===h.name ? "selected" : ""}>${smEscape(h.name)}</option>`).join("")}</select></label>${smInput("解析名", "analysisName")}</div>
    <p>選択したheadを上で選んだbagの全フレームに適用し、教師操舵との差を比較します。自動head切り替えの評価ではありません。</p>
    <button ${disabled} onclick="smStart('analyze')">選択headで解析</button><button ${disabled} onclick="smStart('export')">ONNXを再出力</button>
    ${run ? `<p>検証MAE: ${smEscape(JSON.stringify(run.best_mae || {}))} / ONNX: ${run.has_onnx ? "出力済み" : "未出力"}</p>` : ""}
    <div class="form-grid">${smInput("Jetsonユーザー", "user")}${smInput("Jetsonホスト", "host")}${smInput("転送先モデルルート", "remoteRoot")}</div>
    <label><input type="checkbox" ${s.buildEngine ? "checked" : ""} onchange="smSet('buildEngine',this.checked)">転送後にJetsonでTensorRT engineを生成</label><button ${disabled} onclick="smStart('deploy')">Jetsonへ転送</button>
    <h3>解析結果</h3>${a.analyses.map(r=>`<details><summary>${smEscape(r.name)} — ${smEscape(r.head)} / ${r.count}件 / MAE ${Number(r.mae).toFixed(4)} / RMSE ${Number(r.rmse).toFixed(4)}</summary>${smPlot(r)}<p>${smEscape(r.path)}</p></details>`).join("") || "まだ解析結果がありません。"}
    <h3>タスク</h3>${tasks.length ? renderTaskTable(tasks) : "実行中のタスクはありません。"}
    </div></section>`;
}
