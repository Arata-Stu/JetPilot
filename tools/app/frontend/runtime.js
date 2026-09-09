/* SSH operations are explicit. Opening this page never starts a remote process. */
const runtimeDefaults = {host: '', user: '', container: '', container_user: 'admin', host_workspace: '', screen: 'jetpilot-web', session: 'jetpilot-web', bringup: '/workspaces/scripts/bringup.sh', preset: 'record', sensor: 'realsense', vehicle: 'jpbb', rgb_fps: 30, infra_fps: 60, fixed: false, throttle: 0.2, model: '', map: '', bag: ''};
let runtimeConfig = {...runtimeDefaults};
try { Object.assign(runtimeConfig, JSON.parse(localStorage.getItem('jetpilot-runtime-v1') || '{}')); } catch (_) {}
let runtimeBusy = false;
let runtimeResult = null;
let runtimeError = '';
let runtimeLastUpdate = '';
let runtimeBag = {state:'unknown'};
let runtimeBagTime = '';
let runtimeBagBusy = false;
let runtimeTune = {node:'/e2e_control_decoder', parameter:'fixed_throttle', value:0.2, stream:'rgb', fps:30};
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
  const button = (action, label) => `<button ${runtimeBusy ? 'disabled' : ''} onclick="runtimeAction('${action}')">${label}</button>`;
  const offline = runtimeConfig.preset.startsWith('offline-');
  const states = {running: 'プロセス実行中（ROSの正常稼働はログで確認）', exited: '終了', not_started: '未起動'};
  return `<section class="panel runtime-panel"><h2>実機</h2><p>Jetsonの環境準備から起動・ログ確認まで。接続が切れてもscreenとtmuxに処理が残ります。</p>
  <fieldset ${runtimeBusy ? 'disabled' : ''}><legend>接続先</legend><div class="runtime-grid">
  ${field('host','Jetsonホスト','192.168.…')}${field('user','SSHユーザー')}${field('container','Dockerコンテナ名')}${field('container_user','コンテナ内ユーザー')}
  ${field('host_workspace','Jetsonホストの作業ディレクトリ','isaac-ros activateを実行する場所')}
  </div><details><summary>セッション・パス設定</summary><div class="runtime-grid">${field('screen','screen名')}${field('session','tmux名')}${field('bringup','コンテナ内bringupパス')}</div></details></fieldset>
  <p>${button('prepare','1. 環境を準備')} ${button('status','状態・ログ更新')}</p><p>環境準備はscreen内で <code>isaac-ros activate</code> を実行します。SSH鍵認証を使います。</p>
  <fieldset ${runtimeBusy ? 'disabled' : ''}><legend>起動設定</legend><div class="runtime-grid">
  ${select('preset','用途', [['record','データ収集'],['e2e','E2E走行'],['drive','通常走行：手動'],['runtime','通常走行：自己位置推定付き手動'],['competition','通常走行：ルールベース自動'],['tuning','通常走行：地図・ライン実車調整'],['offline-vslam','オフライン：地図なしVSLAM'],['offline-vslam-map','オフライン：保存地図VSLAM'],['offline-localization','オフライン：VGL・VSLAM']])}
  ${offline ? field('bag','bagのパス（コンテナ内）') : `${field('vehicle','車両プロファイル')}${select('sensor','センサー', [['realsense','RealSense'],['event-camera','EVS単独'],['realsense-silky','RealSense＋EVS'],['realsense-silky-flir','RealSense＋EVS＋FLIR']])}${runtimeConfig.sensor === 'event-camera' ? '' : ['rgb','infra'].map(stream => select(stream+'_fps',stream.toUpperCase()+' Hz',[[0,'OFF'],[30,'30 Hz'],[60,'60 Hz'],[90,'90 Hz']])).join('')}`}
  ${runtimeConfig.preset === 'record' ? `<label>操作方式<select onchange="runtimeChange('fixed',this.value==='fixed',true)"><option value="joy" ${!runtimeConfig.fixed?'selected':''}>Joy</option><option value="fixed" ${runtimeConfig.fixed?'selected':''}>固定スロットル（L2で停止）</option></select></label>`:''}
  ${runtimeConfig.preset === 'e2e' ? field('model','モデルディレクトリ（コンテナ内）','/workspaces/ros2_ws/models/e2e/camera_steering'):''}
  ${(runtimeConfig.preset === 'record' && runtimeConfig.fixed) || runtimeConfig.preset === 'e2e' ? field('throttle','固定モードで使うスロットル（0〜1）'):''}
  ${['runtime','competition','tuning','offline-vslam-map','offline-localization'].includes(runtimeConfig.preset) ? field('map','地図のパス（コンテナ内）'):''}
  </div></fieldset>
  <p>E2Eの制御方式はモデルのmetadataから自動設定します。起動後の走行操作は既存のJoyで行います。</p>
  <p>${button('preview','起動内容を確認')} ${button('start','2. bringupを起動')} ${button('stop','bringupを終了')}</p>
  <p>「終了」はプロセスへの終了要求です。走行中のブレーキ操作には使わず、Joyで停止してから終了してください。</p>
  ${offline ? '' : `<p>${button('record-start','記録開始')} ${button('record-stop','記録停止')}</p>`}
  <h3>記録状態</h3><p>${esc({unknown:'応答なし・未確認', recording:'● 記録中', idle:'停止中'}[runtimeBag.state] || '未確認')} ${esc(runtimeBagTime)}</p>
  <p>${esc(runtimeBag.current_uri || '')}</p><p>${esc(runtimeBag.message || '')}</p>
  <button onclick="runtimeRefreshBag()" ${runtimeBusy || runtimeBagBusy ? 'disabled' : ''}>記録状態を更新</button>
  <p>コンテナ接続確認後、この画面を開いている間は約5秒ごとに更新します。</p>
  <h3>実行中の調整</h3><fieldset ${runtimeBusy ? 'disabled' : ''}><div class="runtime-grid">
  <label>対象<select onchange="runtimeTuneTarget(this.value)"><option value="/e2e_control_decoder" ${runtimeTune.node==='/e2e_control_decoder'?'selected':''}>E2E出力</option><option value="/teleop_cmd_node" ${runtimeTune.node==='/teleop_cmd_node'?'selected':''}>Joy出力</option><option value="/path_tracking_controller_node" ${runtimeTune.node==='/path_tracking_controller_node'?'selected':''}>Controller</option></select></label>
  <label>項目<select onchange="runtimeTuneField(this.value)">${(runtimeTune.node==='/path_tracking_controller_node'?runtimeControllerFields:[['fixed_throttle','固定スロットル'],['steering_offset','ステア offset（−1〜1）'],['steering_scale','ステア scale（0〜3）'],...(runtimeTune.node==='/teleop_cmd_node'?[['throttle_scale','Joyスロットル倍率']]:[])]).map(([key,label])=>`<option value="${key}" ${runtimeTune.parameter===key?'selected':''}>${label}</option>`).join('')}</select></label>
  ${runtimeTune.parameter==='algorithm'?`<label>制御方式<select onchange="runtimeTune.value=this.value">${['pure_pursuit','map_pursuit','kinematic_mpc'].map(name=>`<option ${runtimeTune.value===name?'selected':''}>${name}</option>`).join('')}</select></label>`:`<label>値<input type="number" step="0.01" value="${esc(runtimeTune.value)}" onchange="runtimeTune.value=Number(this.value)"></label>`}
  </div><p><button onclick="runtimeTuningAction('param-get')">現在値を取得</button> <button onclick="runtimeTuningAction('param-set')">値を適用</button></p>
  <p>${runtimeTune.node==='/path_tracking_controller_node'?'Controllerの方式変更は制御器を入れ替えます。速度PID変更時は積分状態をリセットします。lookaheadはPure／Map Pursuitで使用します。':'ステア出力＝入力 × scale ＋ offset（最後に出力範囲に制限）。固定スロットル値は固定モード時に使用します。'}</p>
  <div class="runtime-grid"><label>RealSense<select onchange="runtimeTune.stream=this.value"><option value="rgb" ${runtimeTune.stream==='rgb'?'selected':''}>RGB</option><option value="infra" ${runtimeTune.stream==='infra'?'selected':''}>Infra</option></select></label>
  <label>Hz<select onchange="runtimeTune.fps=Number(this.value)">${[30,60,90].map(fps=>`<option ${runtimeTune.fps===fps?'selected':''}>${fps}</option>`).join('')}</select></label></div>
  <p><button onclick="runtimeTuningAction('camera-get')">カメラ設定を取得</button> <button onclick="runtimeTuningAction('camera-set')">Hzを適用</button></p>
  <p>Hz変更時は対象ストリームが一時停止します。解像度・元のON/OFF状態は維持します。</p>
  </fieldset><p role="status">${esc(runtimeTuneMessage)}</p><p>動的変更は現在のノードに適用されます。次回起動用の設定は変更しません。</p>
  <button onclick="setTab('maps')">地図・ライン・区間速度の実車調整を開く</button><p>centerline／raceline／custom lineの変更は、用途「地図・ライン実車調整」で起動してから地図画面で適用します。</p>
  <div role="status" aria-live="polite">${runtimeBusy?'操作中…':esc(runtimeResult?.message || '')}</div>
  ${runtimeError ? `<p role="alert">${esc(runtimeError)}</p>`:''}
  ${runtimeResult ? `<p>最終確認：${esc(runtimeLastUpdate)} ／ screen：${runtimeResult.screen_running ? '起動済み':'未確認・未起動'} ／ Docker：${runtimeResult.container_running ? '起動済み':'未確認・未起動'}</p><p>${esc(states[runtimeResult.state] || '')} ${runtimeResult.exit_code != null ? '終了コード：'+esc(runtimeResult.exit_code):''}</p>${runtimeResult.command ? `<pre>${esc(runtimeResult.command)}</pre>`:''}<h3>bringupログ</h3><pre class="runtime-log">${esc(runtimeResult.log || 'ログはまだありません')}</pre><details><summary>環境準備ログ</summary><pre class="runtime-log">${esc(runtimeResult.environment_log || 'ログはまだありません')}</pre></details>`:''}
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
    runtimeTuneMessage = result.message + (result.camera ? '：'+result.camera.profile+(result.camera.enabled?'（ON）':'（OFF）') : '：'+result.value);
  } catch(error) { runtimeTuneMessage = error.message; }
  finally {runtimeBusy=false;if(state.tab==='runtime')render();}
}
setInterval(()=>{
  if (state.tab === 'runtime' && runtimeResult?.container_running && !runtimeBusy) runtimeRefreshBag();
},5000);
