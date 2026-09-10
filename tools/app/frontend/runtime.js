/* SSH operations are explicit. Opening this page never starts a remote process. */
const runtimeDefaults = {host: '192.168.11.190', user: 'tamiya', container: 'isaac_ros_dev_container', container_user: 'admin', host_workspace: '/home/tamiya/workspaces/JetPilot', screen: 'jetpilot-web', session: 'jetpilot-web', bringup: '/workspaces/scripts/bringup.sh', preset: 'record', sensor: 'realsense', vehicle: 'jpbb', rgb_fps: 30, infra_fps: 60, evs_window_ms: 50, evs_stride_ms: 10, fixed: false, throttle: 0.2, model: '', map: '', bag: ''};
const runtimeHostChoices = ['192.168.11.190', '10.42.0.1', '192.168.55.1'];
const runtimeWorkspaceChoices = ['/home/tamiya/workspaces/JetPilot', '/home/tamiya'];
let runtimeConfig = {...runtimeDefaults};
try { Object.assign(runtimeConfig, JSON.parse(localStorage.getItem('jetpilot-runtime-v1') || '{}')); } catch (_) {}
for (const key of ['host', 'user', 'container', 'container_user', 'host_workspace']) {
  if (!String(runtimeConfig[key] || '').trim()) runtimeConfig[key] = runtimeDefaults[key];
}
let runtimeBusy = false;
let runtimeResult = null;
let runtimeError = '';
let runtimeLastUpdate = '';
let runtimeBag = {state:'unknown'};
let runtimeBagTime = '';
let runtimeBagBusy = false;
let runtimeTune = {node:'/e2e_control_decoder', parameter:'fixed_throttle', value:0.2, sensor:'realsense', stream:'rgb', fps:30, evs_window_ms:50, evs_stride_ms:10};
let runtimeTuneMessage = '';

function runtimeChange(key, value, redraw = false) {
  runtimeConfig[key] = value;
  localStorage.setItem('jetpilot-runtime-v1', JSON.stringify(runtimeConfig));
  runtimeResult = null; // Old target's state must never be shown as the new target's state.
  runtimeLastUpdate = '';
  runtimeBag = {state:'unknown'}; runtimeBagTime = '';
  runtimeTuneMessage = '';
  render();
}
function runtimeResetConnectionDefaults() {
  for (const key of ['host', 'user', 'container', 'container_user', 'host_workspace']) {
    runtimeConfig[key] = runtimeDefaults[key];
  }
  localStorage.setItem('jetpilot-runtime-v1', JSON.stringify(runtimeConfig));
  runtimeResult = null;
  runtimeError = '';
  runtimeLastUpdate = '';
  runtimeBag = {state:'unknown'}; runtimeBagTime = '';
  render();
}
const runtimeControllerFields = [
 ['algorithm','制御方式'],['min_lookahead_m','最小lookahead（m）'],['max_lookahead_m','最大lookahead（m）'],
 ['lookahead_speed_gain_s','速度に応じたlookahead係数'],['max_steering_angle_rad','最大操舵角（rad）'],
 ['max_steering_command','ステア出力上限'],['map_lateral_error_gain','Map Pursuit：横偏差ゲイン'],
 ['mpc_path_error_weight','MPC：経路誤差の重み'],['mpc_heading_error_weight','MPC：方位誤差の重み'],['mpc_steering_weight','MPC：操舵の重み'],
 ['throttle_kp','速度PID：P'],['throttle_ki','速度PID：I'],['throttle_kd','速度PID：D'],
 ['throttle_feedforward','スロットルfeedforward'],['brake_kp','ブレーキP'],['max_throttle_command','スロットル上限'],
 ['max_brake_command','ブレーキ上限'],['max_target_speed_mps','目標速度上限（m/s）'],['max_steering_rate_per_s','ステア変化率上限']
];
function runtimeTuneTarget(node) {
 runtimeTune.node=node; runtimeTune.parameter=node==='/path_tracking_controller_node'?'algorithm':'fixed_throttle';
 runtimeTune.value=runtimeTune.parameter==='algorithm'?'pure_pursuit':0.2;
 runtimeTuneMessage=''; render();
}
function runtimeTuneField(name) {
 runtimeTune.parameter=name;runtimeTune.value=name==='algorithm'?'pure_pursuit':0;
 runtimeTuneMessage='現在値を取得してから調整してください';render();
}
function renderRuntime() {
  const field = (key, label, placeholder = '') => `<label>${esc(label)}<input value="${esc(runtimeConfig[key])}" placeholder="${esc(placeholder)}" onchange="runtimeChange('${key}',this.value)"></label>`;
  const select = (key, label, choices) => `<label>${esc(label)}<select onchange="runtimeChange('${key}',this.value,true)">${choices.map(([value, text]) => `<option value="${esc(value)}" ${String(runtimeConfig[key]) === String(value) ? 'selected' : ''}>${esc(text)}</option>`).join('')}</select></label>`;
  const button = (action, label, tone = '') => `<button class="${tone}" ${runtimeBusy ? 'disabled' : ''} onclick="runtimeAction('${action}')">${label}</button>`;
  const offline = runtimeConfig.preset.startsWith('offline-');
  const hasEvs = runtimeConfig.sensor.includes('event') || runtimeConfig.sensor.includes('silky');
  const evsHz = 1000 / Math.max(0.001, Number(runtimeConfig.evs_stride_ms) || 10);
  const states = {running: 'プロセス実行中（ROSの正常稼働はログで確認）', exited: '終了', not_started: '未起動'};
  const connectionState = runtimeBusy ? ['checking','接続確認中'] : runtimeError ? ['bad','接続エラー'] : runtimeResult?.container_running ? ['ok','接続済み'] : ['idle','未確認'];
  const bagState = {unknown:'未確認', recording:'● 記録中', idle:'停止中'}[runtimeBag.state] || '未確認';
  return `<section class="runtime-panel">
  <div class="runtime-hero">
    <div><span class="runtime-kicker">JETSON RUNTIME</span><h1>起動・運転</h1><p>Jetsonへ接続し、環境準備、車両起動、記録、走行中の調整を行います。</p></div>
    <div class="runtime-hero-status ${connectionState[0]}"><i></i><span>接続状態</span><strong>${connectionState[1]}</strong><small>${esc(runtimeConfig.host || 'Jetsonホスト未設定')}</small></div>
  </div>
  <div class="runtime-workflow">
  <section class="runtime-card runtime-connect-card">
    <header><span class="runtime-step">01</span><div><h2>Jetsonへ接続</h2><p>SSHとIsaac ROSコンテナを準備</p></div></header>
    <fieldset ${runtimeBusy ? 'disabled' : ''}><div class="runtime-grid runtime-grid-compact">
      ${field('host','Jetsonホスト','192.168.…')}${field('user','SSHユーザー')}
      <div class="runtime-wide runtime-quick-values"><span>よく使う接続先</span>${runtimeHostChoices.map(host => `<button type="button" class="ghost ${runtimeConfig.host === host ? 'active' : ''}" onclick="runtimeChange('host','${host}')">${host}</button>`).join('')}</div>
      <div class="runtime-wide">${field('host_workspace','Jetson側の作業ディレクトリ','isaac-ros activateを実行する場所')}</div>
      <div class="runtime-wide runtime-quick-values"><span>よく使う作業場所</span>${runtimeWorkspaceChoices.map(path => `<button type="button" class="ghost ${runtimeConfig.host_workspace === path ? 'active' : ''}" onclick="runtimeChange('host_workspace','${path}')">${path}</button>`).join('')}</div>
    </div><details><summary>詳細なDocker・セッション設定</summary><div class="runtime-grid runtime-grid-compact">${field('container','Dockerコンテナ名')}${field('container_user','コンテナ内ユーザー')}${field('screen','screen名')}${field('session','tmux名')}<div class="runtime-wide">${field('bringup','コンテナ内bringupパス')}</div></div></details></fieldset>
    <div class="runtime-actions">${button('prepare','1. 環境を準備','primary')} ${button('status','状態・ログ更新')} <button type="button" ${runtimeBusy ? 'disabled' : ''} onclick="runtimeResetConnectionDefaults()">接続設定を既定値へ</button></div>
    <p class="runtime-hint">screen内で<code>isaac-ros activate</code>を実行します。SSH鍵認証を使用します。</p>
  </section>

  <section class="runtime-card runtime-launch-card">
    <header><span class="runtime-step">02</span><div><h2>車両を起動</h2><p>用途・センサー・モデルを選択</p></div></header>
    <fieldset ${runtimeBusy ? 'disabled' : ''}><div class="runtime-grid runtime-grid-compact">
  ${select('preset','用途', [['record','データ収集'],['e2e','E2E走行'],['drive','通常走行：手動'],['runtime','通常走行：自己位置推定付き手動'],['competition','通常走行：ルールベース自動'],['tuning','通常走行：地図・ライン実車調整'],['offline-vslam','オフライン：地図なしVSLAM'],['offline-vslam-map','オフライン：保存地図VSLAM'],['offline-localization','オフライン：VGL・VSLAM']])}
  ${offline ? field('bag','bagのパス（コンテナ内）') : `${field('vehicle','車両プロファイル')}${select('sensor','センサー', [['realsense','RealSense'],['event-camera','EVS単独'],['realsense-silky','RealSense＋EVS'],['realsense-silky-flir','RealSense＋EVS＋FLIR']])}${runtimeConfig.sensor === 'event-camera' ? '' : ['rgb','infra'].map(stream => select(stream+'_fps',stream.toUpperCase()+' Hz',[[0,'OFF'],[30,'30 Hz'],[60,'60 Hz'],[90,'90 Hz']])).join('')}${hasEvs ? `<label>EVS蓄積窓（ms）<input type="number" min="1" max="1000" step="1" value="${esc(runtimeConfig.evs_window_ms)}" onchange="runtimeChange('evs_window_ms',this.value)"></label><label>EVSスライド幅（ms）<input type="number" min="1" max="1000" step="1" value="${esc(runtimeConfig.evs_stride_ms)}" onchange="runtimeChange('evs_stride_ms',this.value)"></label><div class="runtime-wide runtime-hint">直近${esc(runtimeConfig.evs_window_ms)} msのイベントから、${esc(runtimeConfig.evs_stride_ms)} msごと（約${evsHz.toFixed(1)} Hz）に画像を生成します。</div>` : ''}`}
  ${runtimeConfig.preset === 'record' ? `<label>操作方式<select onchange="runtimeChange('fixed',this.value==='fixed',true)"><option value="joy" ${!runtimeConfig.fixed?'selected':''}>Joy</option><option value="fixed" ${runtimeConfig.fixed?'selected':''}>固定スロットル（L2で停止）</option></select></label>`:''}
  ${runtimeConfig.preset === 'e2e' ? `<div class="runtime-wide">${field('model','モデルディレクトリ（コンテナ内）','/workspaces/ros2_ws/models/e2e/camera_steering')}</div>`:''}
  ${(runtimeConfig.preset === 'record' && runtimeConfig.fixed) || runtimeConfig.preset === 'e2e' ? field('throttle','固定モードで使うスロットル（0〜1）'):''}
  ${['runtime','competition','tuning','offline-vslam-map','offline-localization'].includes(runtimeConfig.preset) ? `<div class="runtime-wide">${field('map','地図のパス（コンテナ内）')}</div>`:''}
  </div></fieldset>
    <p class="runtime-hint">E2Eの制御方式はmetadataから自動設定します。走行操作はJoyを使用します。</p>
    <div class="runtime-actions">${button('preview','起動内容を確認')} ${button('start','2. bringupを起動','primary')} ${button('stop','bringupを終了','danger')}</div>
    <p class="runtime-warning">終了はブレーキではありません。Joyで停止してからbringupを終了してください。</p>
  </section>

  <section class="runtime-card runtime-record-card">
    <header><span class="runtime-step">03</span><div><h2>走行を記録</h2><p>rosbagの開始・停止と保存先を確認</p></div><span class="runtime-card-status ${runtimeBag.state === 'recording' ? 'recording' : ''}">${esc(bagState)}</span></header>
    ${offline ? `<div class="runtime-empty">オフライン用途では新しい記録を開始しません。</div>` : `<div class="runtime-actions">${button('record-start','● 記録開始','record')} ${button('record-stop','記録停止')}</div>`}
    <div class="runtime-record-detail"><strong>${esc(runtimeBag.current_uri || '記録先はまだありません')}</strong><span>${esc(runtimeBag.message || 'コンテナ接続後、約5秒ごとに状態を更新します。')}</span><small>${esc(runtimeBagTime)}</small></div>
    <button onclick="runtimeRefreshBag()" ${runtimeBusy || runtimeBagBusy ? 'disabled' : ''}>記録状態を更新</button>
  </section>

  <section class="runtime-card runtime-tune-card">
    <header><span class="runtime-step">04</span><div><h2>走行中に調整</h2><p>現在のノードへ一時的にparameterを適用</p></div></header>
    <fieldset ${runtimeBusy ? 'disabled' : ''}><div class="runtime-grid runtime-grid-compact">
  <label>対象<select onchange="runtimeTuneTarget(this.value)"><option value="/e2e_control_decoder" ${runtimeTune.node==='/e2e_control_decoder'?'selected':''}>E2E出力</option><option value="/teleop_cmd_node" ${runtimeTune.node==='/teleop_cmd_node'?'selected':''}>Joy出力</option><option value="/path_tracking_controller_node" ${runtimeTune.node==='/path_tracking_controller_node'?'selected':''}>Controller</option></select></label>
  <label>項目<select onchange="runtimeTuneField(this.value)">${(runtimeTune.node==='/path_tracking_controller_node'?runtimeControllerFields:[['fixed_throttle','固定スロットル'],['steering_offset','ステア offset（−1〜1）'],['steering_scale','ステア scale（0〜3）'],...(runtimeTune.node==='/teleop_cmd_node'?[['throttle_scale','Joyスロットル倍率']]:[])]).map(([key,label])=>`<option value="${key}" ${runtimeTune.parameter===key?'selected':''}>${label}</option>`).join('')}</select></label>
  ${runtimeTune.parameter==='algorithm'?`<label>制御方式<select onchange="runtimeTune.value=this.value">${['pure_pursuit','map_pursuit','kinematic_mpc'].map(name=>`<option ${runtimeTune.value===name?'selected':''}>${name}</option>`).join('')}</select></label>`:`<label>値<input type="number" step="0.01" value="${esc(runtimeTune.value)}" onchange="runtimeTune.value=Number(this.value)"></label>`}
  </div><div class="runtime-actions"><button onclick="runtimeTuningAction('param-get')">現在値を取得</button><button class="primary" onclick="runtimeTuningAction('param-set')">値を適用</button></div>
  <p class="runtime-hint">${runtimeTune.node==='/path_tracking_controller_node'?'方式変更は制御器を入れ替えます。速度PID変更時は積分状態をリセットします。lookaheadはPure／Map Pursuitで使用します。':'ステア出力＝入力 × scale ＋ offset。固定スロットル値は固定モード時に使用します。'}</p>
  <div class="runtime-grid runtime-grid-compact"><label>センサー調整<select onchange="runtimeTune.sensor=this.value;runtimeTuneMessage='';render()"><option value="realsense" ${runtimeTune.sensor==='realsense'?'selected':''}>RealSense</option><option value="evs" ${runtimeTune.sensor==='evs'?'selected':''}>EVS蓄積</option></select></label>
  ${runtimeTune.sensor==='realsense'?`<label>ストリーム<select onchange="runtimeTune.stream=this.value"><option value="rgb" ${runtimeTune.stream==='rgb'?'selected':''}>RGB</option><option value="infra" ${runtimeTune.stream==='infra'?'selected':''}>Infra</option></select></label><label>Hz<select onchange="runtimeTune.fps=Number(this.value)">${[30,60,90].map(fps=>`<option ${runtimeTune.fps===fps?'selected':''}>${fps}</option>`).join('')}</select></label>`:`<label>蓄積窓（ms）<input type="number" min="1" max="1000" step="1" value="${esc(runtimeTune.evs_window_ms)}" onchange="runtimeTune.evs_window_ms=Number(this.value)"></label><label>スライド幅（ms）<input type="number" min="1" max="1000" step="1" value="${esc(runtimeTune.evs_stride_ms)}" onchange="runtimeTune.evs_stride_ms=Number(this.value)"></label>`}</div>
  <div class="runtime-actions">${runtimeTune.sensor==='realsense'?`<button onclick="runtimeTuningAction('camera-get')">カメラ設定を取得</button><button onclick="runtimeTuningAction('camera-set')">Hzを適用</button>`:`<button onclick="runtimeTuningAction('evs-get')">EVS設定を取得</button><button onclick="runtimeTuningAction('evs-set')">蓄積設定を適用</button>`}</div>
  </fieldset><div class="runtime-inline-message" role="status">${esc(runtimeTuneMessage || '変更は現在のノードだけに適用され、次回起動設定は変わりません。')}</div>
  <button onclick="setTab('maps')">地図・ライン・区間速度の実車調整を開く</button>
  </section>
  </div>

  <section class="runtime-result ${runtimeError ? 'bad' : runtimeResult ? 'visible' : ''}">
    <div class="runtime-result-summary" role="status" aria-live="polite"><strong>${runtimeBusy?'操作中…':esc(runtimeResult?.message || '接続または起動操作を行うと、ここに状態とログが表示されます。')}</strong>${runtimeLastUpdate ? `<span>最終確認 ${esc(runtimeLastUpdate)}</span>` : ''}</div>
    ${runtimeError ? `<div class="runtime-error" role="alert">${esc(runtimeError)}</div>`:''}
    ${runtimeResult ? `<div class="runtime-state-chips"><span class="${runtimeResult.screen_running?'ok':''}">screen ${runtimeResult.screen_running ? '起動済み':'未起動'}</span><span class="${runtimeResult.container_running?'ok':''}">Docker ${runtimeResult.container_running ? '起動済み':'未起動'}</span><span>${esc(states[runtimeResult.state] || '')}${runtimeResult.exit_code != null ? ' · code '+esc(runtimeResult.exit_code):''}</span></div>${runtimeResult.command ? `<details><summary>実行コマンド</summary><pre>${esc(runtimeResult.command)}</pre></details>`:''}<details open><summary>bringupログ</summary><pre class="runtime-log">${esc(runtimeResult.log || 'ログはまだありません')}</pre></details><details><summary>環境準備ログ</summary><pre class="runtime-log">${esc(runtimeResult.environment_log || 'ログはまだありません')}</pre></details>`:''}
  </section>
  </section>`;
}
async function runtimeAction(action) {
  if (runtimeBusy) return;
  runtimeBusy = true; runtimeError = ''; render();
  try {
    runtimeResult = await api('/api/runtime/'+action, {method:'POST',body:JSON.stringify(runtimeConfig)});
    runtimeLastUpdate = new Date().toLocaleTimeString();
    runtimeRefreshBag();
  } catch (error) { runtimeError = error.message; }
  finally { runtimeBusy = false; if (state.tab === 'runtime') render(); }
}

async function runtimeRefreshBag() {
  if (runtimeBagBusy || !runtimeConfig.host || !runtimeConfig.container) return;
  runtimeBagBusy = true;
  const identity = JSON.stringify(runtimeConfig);
  try {
    const result = await api('/api/runtime/bag-status', {method:'POST',body:identity});
    if (identity !== JSON.stringify(runtimeConfig)) return;
    runtimeBag = result.bag;
    runtimeBagTime = new Date().toLocaleTimeString();
  } catch(error) {
    if (identity !== JSON.stringify(runtimeConfig)) return;
    runtimeBag = {state:'unknown',message:error.message}; runtimeBagTime = '';
  } finally {
    runtimeBagBusy = false;
    if (state.tab === 'runtime' && !document.querySelector('.runtime-panel input:focus, .runtime-panel select:focus')) render();
  }
}
async function runtimeTuningAction(action) {
  if (runtimeBusy) return;
  runtimeBusy = true; runtimeError = ''; render();
  try {
    const result = await api('/api/runtime/'+action, {method:'POST',body:JSON.stringify({...runtimeConfig,...runtimeTune})});
    if (result.value != null) runtimeTune.value = result.value;
    if (result.evs) {
      runtimeTune.evs_window_ms = result.evs.window_ms;
      runtimeTune.evs_stride_ms = result.evs.stride_ms;
    }
    runtimeTuneMessage = result.message + (result.camera ? '：'+result.camera.profile+(result.camera.enabled?'（ON）':'（OFF）') : result.evs ? `：${result.evs.window_ms} ms窓 / ${result.evs.stride_ms} msスライド（${result.evs.hz.toFixed(1)} Hz）` : '：'+result.value);
  } catch(error) { runtimeTuneMessage = error.message; }
  finally {runtimeBusy=false;if(state.tab==='runtime')render();}
}
setInterval(()=>{
  if (state.tab === 'runtime' && runtimeResult?.container_running && !runtimeBusy) runtimeRefreshBag();
},5000);
