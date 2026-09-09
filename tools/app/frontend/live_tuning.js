/* Live traffic contains pose/state JSON only. All editing uses the local workspace. */
const tuning = {enabled:false, connected:false, epoch:0, busy:false, timer:null,
  host:'', user:'', mapPath:'', mapId:'', status:null, receivedAt:0,
  line:'centerline', speed:1, sectionSpeeds:{}, preview:null, previewDetail:null,
  active:null, trail:[], error:'', editorLayers:null};

function tuningVisible() {
  return tuning.enabled && state.tab === 'maps' && state.selectedMapPath === tuning.mapPath;
}
function endLiveTuning() {
  disconnectTuning();
  tuning.enabled = false;
  if (tuning.editorLayers) state.mapLayers = {...tuning.editorLayers};
  tuning.editorLayers = null;
}
function toggleLiveTuning() {
  if (tuning.enabled) {
    endLiveTuning();
  } else {
    if (hasUnsavedMapEdits()) {
      toast('地図の変更を保存してから実車調整へ進んでください。', 'error');
      return;
    }
    stopSimulationLoop();
    const target = jetsonTarget();
    if(tuning.mapPath!==state.selectedMapPath) Object.assign(tuning,{line:'centerline',speed:1,sectionSpeeds:{},status:null,mapId:''});
    Object.assign(tuning, {enabled:true, mapPath:state.selectedMapPath,
      host:target.host, user:target.user, preview:null, active:null, trail:[], error:''});
    tuning.editorLayers = {...state.mapLayers};
    state.mapLayers = {...state.mapLayers, landmark:false, odometry:false, left_bound:true,
      right_bound:true, centerline:true, raceline:true, custom_line:true, section_gates:true, section_labels:true};
  }
  render();
}
function renderLiveTuning(detail) {
  if (!tuning.enabled || tuning.mapPath !== detail.map.path) return '';
  const lines = [['centerline','センターライン'],['raceline','レースライン'],
    ...(detail.custom_lines || []).map(line => [`custom:${line.id}`, `Custom · ${line.name || line.id}`])];
  const sections = (detail.hd_map?.sections || []).filter(section => section.lane_id === detail.hd_map.primary_lane_id);
  const knownSections = new Set(sections.map(section=>section.id));
  const removed = Object.keys(tuning.sectionSpeeds).filter(id=>!knownSections.has(id));
  if(removed.length) {
    for(const id of removed) delete tuning.sectionSpeeds[id];
    tuning.preview=null;
    tuning.error=`削除された区間の速度指定を解除しました：${removed.join(', ')}。新しい区間を確認してプレビューしてください。`;
  }
  return `<section class="live-tuning-panel">
    <h3>実車調整モード</h3>
    <p>自己位置と走行ラインを確認し、速度を調整します。形状や区間を変更する場合は、上のボタンから地図編集へ戻ってください。</p>
    <div class="actions">
      <label>Jetson <input aria-label="調整先Jetson" value="${esc(tuning.host)}" ${tuning.connected ? 'disabled' : ''} onchange="tuning.host=this.value"></label>
      <label>ユーザー <input aria-label="Jetsonユーザー" value="${esc(tuning.user)}" ${tuning.connected ? 'disabled' : ''} onchange="tuning.user=this.value"></label>
      <button onclick="connectTuning()" ${tuning.connected || tuning.busy ? 'disabled' : ''}>接続</button>
      <button onclick="disconnectTuning();refreshTuningPanel()" ${!tuning.connected ? 'disabled' : ''}>切断</button>
    </div>
    <div class="actions">
      <label>走行に使うライン <select aria-label="実車の走行ライン" onchange="selectTuningLine(this.value)">${lines.map(([id,label]) => `<option value="${esc(id)}" ${tuning.line===id?'selected':''}>${esc(label)}</option>`).join('')}</select></label>
      <label>全体の目標上限（m/s） <input aria-label="実車調整の全体速度" type="number" min="0.1" max="3" step="0.1" value="${tuning.speed}" onchange="tuning.speed=Number(this.value);tuning.preview=null;refreshTuningPanel()"></label>
    </div>
    <details><summary>区間ごとの速度上限（空欄は全体値）</summary><div class="tuning-speeds">${sections.map(section => `<label>${esc(section.id)} <input aria-label="${esc(section.id)}の速度" type="number" min="0.1" max="3" step="0.1" value="${esc(tuning.sectionSpeeds[section.id] ?? '')}" onchange="setTuningSectionSpeed(${js(section.id)},this.value)"></label>`).join('') || '<p>地図編集のTopologyで区間を追加・保存してください。</p>'}</div></details>
    <button onclick="selectTuningLine(tuning.line)">保存済みラインの速度を読み直す</button>
    <div class="actions">
      <button onclick="prepareTuning()" ${tuning.busy?'disabled':''}>保存済み編集をプレビュー</button>
      <button id="tuning-apply" onclick="applyTuning(false)">この版を実車に適用</button>
      <button id="tuning-rollback" onclick="applyTuning(true)">直前の版に戻す</button>
      <button onclick="tuning.trail=[];drawMapPreview()">軌跡を消去</button>
    </div>
    <div id="tuning-status" role="status"></div>
    <p>適用はSTOP・1秒以上の停車時のみ。切断や画面を離れて4秒で調整経路の配信許可が失効します。再接続前にSTOPへ切り替えてください。</p>
  </section>`;
}
function setTuningSectionSpeed(id, value) {
  if (value === '') delete tuning.sectionSpeeds[id];
  else tuning.sectionSpeeds[id] = Number(value);
  tuning.preview=null;
  refreshTuningPanel();
}
function selectTuningLine(value) {
  const item = (state.selectedMapDetail?.custom_lines || []).find(line => `custom:${line.id}` === value);
  tuning.line=value; tuning.preview=null;
  tuning.speed=item?.default_speed_mps ?? 1;
  tuning.sectionSpeeds={...(item?.section_speeds_mps || {})};
  render();
}
function tuningRequest(action, extra={}) {
  return api(`/api/tuning/${action}`, {method:'POST', body:JSON.stringify({
    host:tuning.host,user:tuning.user,map_dir:tuning.mapPath,...extra})});
}
function disconnectTuning() {
  tuning.epoch++; tuning.connected=false; tuning.receivedAt=0;
  clearTimeout(tuning.timer); tuning.timer=null;
  tuning.trail=[];
}
async function connectTuning() {
  if (tuning.busy) return;
  const epoch=++tuning.epoch;
  tuning.busy=true; tuning.error=''; refreshTuningPanel();
  try {
    const status=await tuningRequest('connect');
    if(epoch!==tuning.epoch) return;
    if(status.map_id!==status.local_map_id) throw new Error('NotebookとJetsonの自己位置推定用マップが一致しません。');
    tuning.mapId=status.local_map_id; tuning.status=status; tuning.connected=true;
    tuning.receivedAt=Date.now();
    const active=await tuningRequest('active');
    if(epoch!==tuning.epoch) return;
    tuning.active=active.snapshot;
    tuning.timer=setTimeout(()=>pollTuning(epoch),1000);
  } catch(error) { if(epoch===tuning.epoch) {tuning.error=error.message;tuning.connected=false;} }
  finally { tuning.busy=false; render(); }
}
async function pollTuning(epoch) {
  if(epoch!==tuning.epoch || !tuning.connected) return;
  if(!tuningVisible()) {disconnectTuning();return;}
  try {
    const status=await tuningRequest('status');
    if(epoch!==tuning.epoch) return;
    if(status.map_id!==tuning.mapId) throw new Error('Jetsonのマップが変わりました。再接続してください。');
    tuning.status=status; tuning.receivedAt=Date.now(); tuning.error='';
    if(status.pose && status.localization==='localized') {
      tuning.trail.push([status.pose.x,status.pose.y]);
      if(tuning.trail.length>1200) tuning.trail.shift();
    }
    if(status.revision !== (tuning.active?.revision || '')) {
      const active=await tuningRequest('active');
      if(epoch!==tuning.epoch) return;
      tuning.active=active.snapshot;
    }
  } catch(error) { if(epoch===tuning.epoch) {tuning.error=error.message;disconnectTuning();} }
  finally {
    refreshTuningPanel(); drawMapPreview();
    if(epoch===tuning.epoch && tuning.connected) tuning.timer=setTimeout(()=>pollTuning(epoch),1000);
  }
}
async function prepareTuning() {
  if(tuning.busy) return;
  if(hasUnsavedMapEdits()) {toast('地図・ライン・区間の編集を保存してください。','error');return;}
  const detail=state.selectedMapDetail;
  const selection=JSON.stringify([tuning.line,tuning.speed,tuning.sectionSpeeds]);
  tuning.busy=true; tuning.error=''; tuning.preview=null; refreshTuningPanel();
  try {
    const result=await tuningRequest('prepare',{line:tuning.line,speed_mps:tuning.speed,section_speeds_mps:tuning.sectionSpeeds});
    if(detail!==state.selectedMapDetail || hasUnsavedMapEdits() || selection!==JSON.stringify([tuning.line,tuning.speed,tuning.sectionSpeeds])) throw new Error('編集内容が変わりました。もう一度プレビューしてください。');
    tuning.preview=result.snapshot; tuning.previewDetail=detail;
  } catch(error) {tuning.error=error.message;}
  finally {tuning.busy=false;refreshTuningPanel();drawMapPreview();}
}
function tuningApplyIssue(rollback=false) {
  if(tuning.busy) return '処理中';
  if(!tuning.connected || Date.now()-tuning.receivedAt>3000) return '接続状態を確認してください。';
  if(tuning.status?.apply_issue) return tuning.status.apply_issue;
  if(rollback) return tuning.status?.can_rollback ? '' : '戻せる版がありません。';
  if(!tuning.preview || tuning.previewDetail!==state.selectedMapDetail || hasUnsavedMapEdits()) return '編集を保存してプレビューしてください。';
  if(tuning.preview.map_id!==tuning.mapId) return 'マップが一致しません。';
  return '';
}
async function applyTuning(rollback) {
  const issue=tuningApplyIssue(rollback);
  if(issue) {toast(issue,'error');return;}
  const epoch=tuning.epoch;
  tuning.busy=true;tuning.error='';refreshTuningPanel();
  try {
    const status=await tuningRequest(rollback?'rollback':'apply',{
      revision:tuning.preview?.revision,expected_revision:tuning.status.revision});
    if(epoch!==tuning.epoch)return;
    tuning.status=status;tuning.receivedAt=Date.now();
    const active=await tuningRequest('active');
    if(epoch!==tuning.epoch)return;
    tuning.active=active.snapshot;
    toast('Jetsonが適用した版を確認しました。');
  } catch(error) {tuning.error=`適用結果を確認できません。状態を再確認してください：${error.message}`;}
  finally {tuning.busy=false;refreshTuningPanel();drawMapPreview();}
}
function refreshTuningPanel() {
  const el=$('tuning-status');
  if(!el)return;
  const rosDetailsOpen=$('tuning-ros-status')?.open || false;
  const status=tuning.status;
  const fresh=tuning.connected && Date.now()-tuning.receivedAt<3000;
  const mode={1:'AUTO',2:'MANUAL',3:'STOP',4:'PROPO'}[status?.mode] || '不明';
  const preview=tuning.previewDetail===state.selectedMapDetail ? tuning.preview : null;
  const canvas=$('map-preview-canvas');
  const position=fresh && status?.pose && canvas && state.selectedMapDetail
    ? mapPointProjector(state.selectedMapDetail,canvas.width,canvas.height)([status.pose.x,status.pose.y]) : null;
  const outside=position && (position[0]<0 || position[1]<0 || position[0]>canvas.width || position[1]>canvas.height);
  el.innerHTML=`<p>${fresh?'接続中':'未接続・受信停止'}${fresh && !status?.lease_live ? '（走行許可失効：STOPで停車してください）' : ''} · ${esc(fresh?status?.localization:'位置情報なし')} · ${esc(mode)} · ${fresh && status?.speed_mps!=null ? Number(status.speed_mps).toFixed(2)+' m/s':'速度不明'}</p>
    <p>実車：${esc(status?.line || '未適用')} / ${esc(status?.revision?.slice(0,12) || '—')}<br>プレビュー：${esc(preview?.line || '未作成')} / ${esc(preview?.revision?.slice(0,12) || '—')}${preview ? ` · ${preview.points.length}点 · 最大 ${Math.max(...preview.points.map(p=>p[5])).toFixed(2)} m/s` : ''}</p>
    ${fresh && !status?.pose ? `<p role="status">${esc(status?.pose_issue || '自己位置のTFを待っています。')}</p>` : ''}
    ${fresh && status?.localization!=='localized' ? `<p>自己位置推定の確定待ちです。センサー・localizationの起動と初期位置を確認してください。</p>` : ''}
    ${fresh && status?.pose ? `<p>実車位置：x=${Number(status.pose.x).toFixed(2)} m / y=${Number(status.pose.y).toFixed(2)} m</p>` : ''}
    ${outside ? '<p>自己位置は地図の描画範囲外です。開いている地図と地図の原点・範囲を確認してください。</p>' : ''}
    ${fresh && status?.ros ? `<details id="tuning-ros-status" ${rosDetailsOpen?'open':''}><summary>ROS受信状況</summary><p>Domain ${esc(status.ros.domain_id)} · ${esc(status.ros.rmw)}<br>Map: ${esc(status.ros.map_dir)}</p>${Object.entries(status.ros.publishers || {}).map(([topic,count])=>`<p>${esc(topic)}：配信元 ${Number(count)}</p>`).join('')}</details>` : ''}
    <p>白：実車の適用ライン　青：プレビュー　橙：自己位置・走行軌跡</p>
    ${tuning.error?`<p role="alert">${esc(tuning.error)}</p>`:''}
    <p>${esc(tuningApplyIssue())}</p>`;
  for(const [id,rollback] of [['tuning-apply',false],['tuning-rollback',true]]) {
    const button=$(id); if(button){button.disabled=Boolean(tuningApplyIssue(rollback));button.title=tuningApplyIssue(rollback);}
  }
}
function drawTuningOverlay(ctx,detail,width,height) {
  if(!tuningVisible())return;
  const project=mapPointProjector(detail,width,height), scale=canvasCssPixelScale(ctx);
  if(tuning.active?.map_id===tuning.mapId) drawPolyline(ctx,tuning.active.points.map(p=>project([p[1],p[2]])),'#ffffff',2,tuning.active.closed,scale);
  if(tuning.preview && tuning.previewDetail===detail) drawPolyline(ctx,tuning.preview.points.map(p=>project([p[1],p[2]])),'#55aaff',3,tuning.preview.closed,scale);
  const status=tuning.status;
  if(!tuning.connected || Date.now()-tuning.receivedAt>3000 || status?.localization!=='localized' || !status.pose)return;
  drawPolyline(ctx,tuning.trail.map(project),'#ffa94d',2,false,scale);
  const pose=status.pose;
  const start=project([pose.x,pose.y]);
  const heading=project([pose.x+Math.cos(pose.yaw),pose.y+Math.sin(pose.yaw)]);
  const dx=heading[0]-start[0], dy=heading[1]-start[1], length=Math.hypot(dx,dy);
  if(!start.every(Number.isFinite) || !Number.isFinite(length) || length===0)return;
  // Keep the heading visible at every map zoom; its origin remains the measured position.
  const end=[start[0]+dx/length*28*scale,start[1]+dy/length*28*scale];
  drawCanvasArrow(ctx,start,end,'#ffa94d','実車',scale);
}

setInterval(() => {
  if(tuningVisible() && tuning.connected && Date.now()-tuning.receivedAt>3000) {
    refreshTuningPanel(); drawMapPreview();
  }
},1000);
