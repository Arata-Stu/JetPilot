/* Shared Bag Analysis overlays and the live HD-map editor camera pane. */
const mapCameraView = {mapPath:'', id:'', timeline:null, channels:[], channel:'', time:0, playing:false, playbackSerial:0, serial:0, imageSerial:0, image:null, frame:null, key:'', pending:'', error:'', z:0};

function cameraProjectionMapIssue(timeline, detail) {
  if (!timeline?.camera_projection) return '投影情報がありません。CameraInfoとTFを含むbagを再解析してください。';
  if (!detail || timeline.map?.path !== detail.map?.path) return '解析に使用したMapと表示中のMapが一致しません。';
  const expected=timeline.camera_projection.localization_fingerprint;
  if (!expected || !detail.localization_fingerprint) return '自己位置推定Mapの対応を確認できません。';
  if (expected !== detail.localization_fingerprint) return '自己位置推定Mapが変更されています。bagを再解析してください。';
  return '';
}

function cameraOverlayLines(detail, draft=false) {
  const lanes=draft && state.mapEditor.mapPath===detail?.map?.path ? state.mapEditor.lanes : (detail?.hd_map?.lanes || []);
  const lines=[];
  for(const lane of lanes) for(const [field,color] of [['left_bound','#45ed88'],['right_bound','#f280e9'],['centerline','#ffe65c']]) {
    lines.push({points:lane[field] || [], closed:lane.closed_loop, color});
  }
  if(detail?.raceline_csv?.points?.length) {
    const primary=lanes.find(l=>l.primary) || lanes[0];
    lines.push({points:detail.raceline_csv.points, closed:Boolean(primary?.closed_loop), color:'#56d9ff'});
  }
  if(draft && state.customLineEditor.mapPath===detail?.map?.path && state.customLineEditor.workingLine) {
    const line=state.customLineEditor.workingLine;
    lines.push({points:customLineCoordinates(line), closed:line.closed_loop, color:'#ffa954'});
  }
  return lines;
}

function paintCameraOverlay(ctx, width, height, timeline, payload, detail, draft=false, z=0) {
  let reason=cameraProjectionMapIssue(timeline,detail);
  const projection=payload?.projection;
  if (!reason && (!projection || projection.issue)) reason=projection?.issue || 'この画像には投影情報がありません。';
  const model=timeline?.camera_projection?.models?.[projection?.model_id];
  if (!reason) reason=CameraProjection.issue(model,projection.image_geometry,width,height);
  if (reason) return reason;
  const lines=cameraOverlayLines(detail,draft);
  let drawn=0;
  ctx.save();
  ctx.lineWidth=Math.max(2,width/400);
  ctx.lineCap='round';
  for(const line of lines) {
    const segments=CameraProjection.segments(line.points,line.closed,model,projection.camera_from_map,projection.image_geometry,width,height,z);
    ctx.strokeStyle=line.color;
    ctx.beginPath();
    for(const [a,b] of segments) {ctx.moveTo(...a);ctx.lineTo(...b);}
    ctx.stroke();
    drawn+=segments.length;
  }
  ctx.restore();
  return drawn ? '投影中：左境界 緑 / 右境界 桃 / 中央 黄 / Raceline 水色 / Custom 橙' : '投影可能なラインが画角内にありません。';
}

function drawAnalysisCameraOverlays() {
  const root=$('analysis-viewer-body');
  if(!root) return;
  const messages=[];
  for(const canvas of root.querySelectorAll('.analysis-camera-overlay')) {
    const parent=canvas.parentElement;
    const image=parent.querySelector('img.visible');
    const multi=parent.querySelector('#analysis-multi-image-grid.visible');
    const width=parent.clientWidth,height=parent.clientHeight;
    canvas.width=Math.max(1,Math.round(width));canvas.height=Math.max(1,Math.round(height));
    const ctx=canvas.getContext('2d');
    if(!ctx || multi || !image || !image.naturalWidth || state.analysis.cameraOverlay===false) continue;
    const index=Number(image.dataset.projectionFrameIndex),topic=image.dataset.projectionChannel;
    if(!Number.isInteger(index) || !topic) continue;
    const frame=state.analysis.timeline?.frames?.[index];
    const payload=frame?.channels?.[topic];
    const scale=Math.min(width/image.naturalWidth,height/image.naturalHeight);
    ctx.save();
    ctx.translate((width-image.naturalWidth*scale)/2,(height-image.naturalHeight*scale)/2);
    ctx.scale(scale,scale);
    let reason;
    if(state.analysis.selectedMapPath && state.analysis.selectedMapPath !== state.analysis.mapDetail?.map?.path) reason='選択Mapと解析Mapが一致しません。';
    else reason=paintCameraOverlay(ctx,image.naturalWidth,image.naturalHeight,state.analysis.timeline,payload,state.analysis.mapDetail,false,Number(state.analysis.projectionHeightM || 0));
    ctx.restore();
    canvas.title=reason;
    messages.push(`${topic}: ${reason}`);
  }
  const label=$('analysis-projection-status');
  if(label) label.textContent=state.analysis.cameraOverlay===false ? 'HD Map投影は非表示です。' : messages.join(' / ') || '画像の読み込み待ち';
}

function setAnalysisCameraOverlay(enabled) {
  state.analysis.cameraOverlay=Boolean(enabled);drawAnalysisCameraOverlays();
}
function setCameraProjectionHeight(value, target) {
  const z=Number(value);
  if(!Number.isFinite(z) || Math.abs(z)>10) return;
  if(target==='map') {mapCameraView.z=z;updateMapCameraView();}
  else {state.analysis.projectionHeightM=z;drawAnalysisCameraOverlays();}
}

function ensureMapCameraView(detail) {
  if(mapCameraView.mapPath !== detail.map.path) {
    Object.assign(mapCameraView,{mapPath:detail.map.path,id:'',timeline:null,channels:[],channel:'',time:0,playing:false,image:null,frame:null,key:'',pending:'',failedKey:'',error:'',z:0,serial:mapCameraView.serial+1,imageSerial:mapCameraView.imageSerial+1});
  }
}
function renderMapCameraView(detail) {
  ensureMapCameraView(detail);
  const records=state.analysis.analyses.filter(record=>analysisRecordStatus(record)==='success' && (record.map?.path || record.map_path)===detail.map.path);
  const view=mapCameraView;
  return `<section class="inspector-block map-camera-pane">
    <h4>カメラ画像で配置を確認</h4>
    <label>このMapの解析動画<select aria-label="投影用の解析動画" onchange="loadMapCameraAnalysis(this.value)"><option value="">解析結果を選択</option>${records.map(record=>`<option value="${esc(analysisRecordId(record))}" ${view.id===analysisRecordId(record)?'selected':''}>${esc(record.name || shortName(record.rosbag) || analysisRecordId(record))}</option>`).join('')}</select></label>
    ${!records.length?'<div class="field-hint">Bag AnalysisでこのMapを選んで解析すると、動画を選択できます。</div>':''}
    ${view.timeline?`<select aria-label="投影するカメラ" onchange="setMapCameraChannel(this.value)">${view.channels.map(topic=>`<option value="${esc(topic)}" ${topic===view.channel?'selected':''}>${esc(topic)}</option>`).join('')}</select>`:''}
    <canvas id="map-camera-canvas" width="640" height="360" aria-label="編集中HD Mapのカメラ投影"></canvas>
    <div class="editor-actions"><button id="map-camera-play" onclick="toggleMapCameraPlayback()" ${view.timeline?'':'disabled'}>${view.playing?'一時停止':'再生'}</button><button onclick="stepMapCameraFrame(-1)" ${view.timeline?'':'disabled'}>前のコマ</button><button onclick="stepMapCameraFrame(1)" ${view.timeline?'':'disabled'}>次のコマ</button></div>
    <input id="map-camera-seek" type="range" min="0" max="${view.timeline?.duration_s || 0}" step="0.01" value="${view.time}" oninput="seekMapCameraTime(this.value)" aria-label="投影動画の再生位置" ${view.timeline?'':'disabled'} />
    <div id="map-camera-clock" class="field-hint"></div>
    <label>投影高さ（Map Z / m）<input type="number" min="-10" max="10" step="0.01" value="${view.z}" onchange="setCameraProjectionHeight(this.value, 'map')" /></label>
    <div id="map-camera-status" class="field-hint" role="status">${esc(view.error || '解析動画を選択してください。')}</div>
    <div class="field-hint">未保存の境界・centerline・Custom Lineも画像に反映します。平面上の配置確認用で、壁との接触や遮蔽は判定しません。</div>
  </section>`;
}

async function loadMapCameraAnalysis(id) {
  const view=mapCameraView, path=state.selectedMapDetail?.map?.path;
  const serial=++view.serial;
  view.imageSerial++;
  Object.assign(view,{id,timeline:null,channels:[],image:null,frame:null,pending:'',failedKey:'',key:'',playing:false,time:0,error:id?'動画を読み込んでいます…':''});
  render();
  if(!id) return;
  try {
    const response=await api(`/api/analyses/${encodeURIComponent(id)}/timeline`);
    if(serial!==view.serial || path!==state.selectedMapDetail?.map?.path) return;
    const timeline=normalizeAnalysisTimeline(response.timeline || response);
    if(timeline.map?.path !== path) throw new Error('別のMapを使用した解析結果です。');
    view.timeline=timeline;
    view.channels=[...new Set(timeline.frames.flatMap(frame=>Object.keys(frame.channels || {})))];
    // Older results can still show their video with an explicit reanalysis message.
    if(!view.channels.length) view.channels=[timeline.camera_projection?.primary_topic || 'primary'];
    view.channel=timeline.camera_projection?.primary_topic || view.channels[0];
    if(!view.channels.includes(view.channel)) view.channel=view.channels[0];
    view.error='';
  } catch(error) {if(serial===view.serial) view.error=error.message;}
  if(serial===view.serial) render();
}
function setMapCameraChannel(channel) {
  if(!mapCameraView.channels.includes(channel)) return;
  mapCameraView.channel=channel;mapCameraView.imageSerial++;mapCameraView.pending='';mapCameraView.image=null;mapCameraView.key='';updateMapCameraView();
}
function seekMapCameraTime(value) {
  const view=mapCameraView;
  view.time=Math.max(0,Math.min(Number(view.timeline?.duration_s)||0,Number(value)||0));
  updateMapCameraView();
}
function stepMapCameraFrame(direction) {
  const view=mapCameraView, frames=view.timeline?.frames || [];
  if(!frames.length) return;
  view.playing=false;
  const i=Math.max(0,Math.min(frames.length-1,timedRecordIndex(frames,view.time)+direction));
  seekMapCameraTime(frames[i].t);
}
function toggleMapCameraPlayback() {
  const view=mapCameraView;
  if(!view.timeline) return;
  view.playing=!view.playing;
  const playbackSerial=++view.playbackSerial;
  if(view.playing) {
    if(view.time>=view.timeline.duration_s) view.time=0;
    let last=performance.now();
    const serial=view.serial;
    const tick=(now)=>{
      if(playbackSerial!==view.playbackSerial) return;
      if(!view.playing || state.tab!=='maps' || serial!==view.serial || view.mapPath!==state.selectedMapDetail?.map?.path) {view.playing=false;return;}
      view.time=Math.min(view.timeline.duration_s,view.time+Math.min(0.2,(now-last)/1000));last=now;
      if(view.time>=view.timeline.duration_s) view.playing=false;
      updateMapCameraView();
      if(view.playing) requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }
  updateMapCameraView();
}
function updateMapCameraView() {
  const canvas=$('map-camera-canvas'),view=mapCameraView,detail=state.selectedMapDetail;
  if(!canvas || !detail || detail.map.path!==view.mapPath) return;
  const label=$('map-camera-status'),clock=$('map-camera-clock'),seek=$('map-camera-seek'),play=$('map-camera-play');
  if(play) play.textContent=view.playing?'一時停止':'再生';
  if(seek) seek.value=String(view.time);
  const frames=view.timeline?.frames || [],index=timedRecordIndex(frames,view.time);
  const frame=frames[index];
  const payload=frame?.channels?.[view.channel] || (view.channels.length===1?frame:null);
  const key=payload?.path?`${view.id}|${view.channel}|${payload.path}`:'';
  if(key && key!==view.key && !view.pending && key!==view.failedKey) {
    const image=new Image(),serial=++view.imageSerial;
    view.pending=key;
    image.onload=()=>{
      if(serial!==view.imageSerial || view.mapPath!==state.selectedMapDetail?.map?.path) return;
      view.image=image;view.frame=payload;view.imageTime=frame.t;view.key=key;view.pending='';view.error='';
      updateMapCameraView();
    };
    image.onerror=()=>{if(serial===view.imageSerial){view.pending='';view.failedKey=key;view.error='動画フレームを読み込めません。';if(label)label.textContent=view.error;}};
    const path=String(payload.path).replace(/^\/+/, '').replace(/^frames\//,'');
    image.src=`/api/analyses/${encodeURIComponent(view.id)}/frames/${path.split('/').map(encodeURIComponent).join('/')}`;
  }
  const ctx=canvas.getContext('2d');
  if(!ctx) return;
  if(view.image) {
    canvas.width=view.image.naturalWidth;canvas.height=view.image.naturalHeight;
    ctx.drawImage(view.image,0,0);
    const reason=paintCameraOverlay(ctx,canvas.width,canvas.height,view.timeline,view.frame,detail,true,view.z);
    if(label) label.textContent=view.error || reason;
    if(clock) clock.textContent=`画像 ${formatAnalysisClock(view.imageTime)} / 再生位置 ${formatAnalysisClock(view.time)}${view.pending?'（読み込み中）':''}`;
  } else {
    ctx.clearRect(0,0,canvas.width,canvas.height);
    if(label) label.textContent=view.error || (view.timeline?'画像の読み込み待ち':'解析動画を選択してください。');
  }
}
