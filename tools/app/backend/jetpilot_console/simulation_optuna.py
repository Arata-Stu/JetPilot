"""Optional Optuna ask/tell service; simulation remains in the browser worker.

No Optuna import at Console startup. Sessions are bounded, expire, and never
write controller configuration or launch external commands.
"""
import importlib
import math
import threading
import time
import uuid


PURSUIT = {'minLookaheadM': (.15, 1.2), 'maxLookaheadM': (1.2, 3.5), 'lookaheadGainS': (0., .8)}
SPACES = {
    'pure_pursuit': PURSUIT,
    'map_pursuit': {**PURSUIT, 'mapLateralGain': (0., 1.5), 'mapScaleStart': (.5, 2.5),
                    'mapScaleEnd': (2.5, 5.), 'mapScaleFactor': (0., .5)},
    'kinematic_mpc': {'mpcSteps': (6, 24), 'mpcPathWeight': (.5, 12.),
                      'mpcHeadingWeight': (.05, 3.), 'mpcSteeringWeight': (.01, 1.),
                      'mpcTerminalWeight': (.1, 8.)},
}


def metric_score(metrics, duration, baseline=None):
    if not isinstance(metrics, dict):
        raise ValueError('評価結果が不正です。')
    numbers = {}
    for key in ('rmsError', 'maxError', 'steeringRate', 'distance', 'time'):
        value = metrics.get(key)
        # No samples (e.g. an immediate goal or failure) is not a perfect score.
        if value is None and key in ('rmsError', 'steeringRate'):
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('評価結果に不正な数値があります。')
        numbers[key] = value
    if metrics.get('status') not in ('時間終了', '終点到達') or numbers['distance'] < .05:
        return None
    if metrics['status'] == '時間終了' and numbers['time'] < duration - .001:
        return None
    if baseline and baseline['status'] == '終点到達' and metrics['status'] != '終点到達':
        return None
    # Reject stopped/shortened runs rather than rewarding their small error.
    if baseline and numbers['distance'] < .8 * baseline['distance']:
        return None
    return numbers['rmsError']


class SimulationOptimizer:
    def __init__(self, loader=None, clock=time.monotonic):
        self.loader = loader or (lambda: importlib.import_module('optuna'))
        self.clock = clock
        self.lock = threading.RLock()
        self.sessions = {}

    def _expire(self):
        for key in list(self.sessions):
            if self.clock() - self.sessions[key]['updated'] > 1800:
                del self.sessions[key]

    def start(self, body):
        controller = body.get('controller')
        if controller not in SPACES:
            raise ValueError('自動調整はcontrollerを1方式選択してください。')
        trials = body.get('trials', 20)
        if isinstance(trials, bool) or not isinstance(trials, int) or not 5 <= trials <= 50:
            raise ValueError('候補数は5〜50の整数です。')
        duration = body.get('duration')
        if isinstance(duration, bool) or not isinstance(duration, (float, int)) or not math.isfinite(duration) or not 1 <= duration <= 120:
            raise ValueError('実行時間は1〜120秒です。')
        settings = body.get('settings')
        if not isinstance(settings, dict) or len(settings) > 64:
            raise ValueError('Simulation設定が不正です。')
        if any(not isinstance(k, str) or isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for k, v in settings.items()):
            raise ValueError('Simulation設定は有限の数値が必要です。')
        if any(k not in settings for k in SPACES[controller]):
            raise ValueError('必要なcontroller設定がありません。ページを再読み込みしてください。')
        with self.lock:
            self._expire()
            if len(self.sessions) >= 8:
                raise ValueError('自動調整が多数開かれています。不要な調整を終了してください。')
            try:
                optuna = self.loader()
            except ImportError as exc:
                raise RuntimeError('Consoleを実行するPython環境にOptunaがありません。tools/app/requirements-simulation-optuna.txtの追加依存を導入してください。') from exc
            study = optuna.create_study(direction='minimize', sampler=optuna.samplers.TPESampler(seed=42, n_startup_trials=5))
            session_id = uuid.uuid4().hex
            self.sessions[session_id] = dict(study=study, controller=controller, settings=dict(settings),
                limit=trials, duration=duration, updated=self.clock(), pending=None, sequence=0,
                completed=0, baseline=None, best=None, done=False, baseline_received=False)
            return self._response(session_id)

    def _response(self, session_id):
        s = self.sessions[session_id]
        return dict(session_id=session_id, sequence=s['sequence'], settings=s['candidate'] if s.get('candidate') else s['settings'],
                    completed=s['completed'], trials=s['limit'], baseline=s['baseline'], best=s['best'],
                    done=s['done'], is_baseline=not s['baseline_received'])

    def tell(self, body):
        with self.lock:
            self._expire()
            key = body.get('session_id')
            if not isinstance(key, str) or key not in self.sessions:
                raise ValueError('自動調整が期限切れ、または終了しています。')
            s = self.sessions[key]
            if s['done'] or body.get('sequence') != s['sequence']:
                raise ValueError('評価する候補が一致しません。')
            s['updated'] = self.clock()
            metrics = body.get('metrics')
            score = metric_score(metrics, s['duration'], s['baseline']['metrics'] if s['baseline'] else None)
            entry = dict(score=score, metrics={k:metrics.get(k) for k in ('rmsError','maxError','steeringRate','distance','time','status')},
                         settings=dict(s.get('candidate') or s['settings']), sequence=s['sequence'])
            if not s['baseline_received']:
                s['baseline_received'] = True
                if score is not None:
                    s['baseline'] = entry
                    s['best'] = entry
            else:
                # Invalid candidates get a finite, dominating penalty; they cannot become a UI best.
                s['study'].tell(s['pending'], score if score is not None else 1e12)
                s['completed'] += 1
                if score is not None and (s['best'] is None or score < s['best']['score']):
                    s['best'] = entry
            if s['completed'] >= s['limit']:
                s['done'] = True
                return self._response(key)
            trial = s['study'].ask()
            candidate = dict(s['settings'])
            for name, (low, high) in SPACES[s['controller']].items():
                candidate[name] = trial.suggest_int(name, low, high) if name == 'mpcSteps' else trial.suggest_float(name, low, high)
            s['candidate'], s['pending'] = candidate, trial
            s['sequence'] += 1
            return self._response(key)

    def stop(self, body):
        with self.lock:
            key = body.get('session_id')
            if not isinstance(key, str):
                raise ValueError('session_idが必要です。')
            self.sessions.pop(key, None)
            return {'stopped': True}
