"""Pure state machine shared with the Jetson ROS adapter."""
from __future__ import annotations
import time
from .live_tuning import validate_snapshot


class TuningState:
    def __init__(self, map_id, clock=time.monotonic):
        self.map_id = map_id
        self.clock = clock
        self.active = None
        self.previous = None
        self.mode = None
        self.mode_at = float('-inf')
        self.speed = None
        self.speed_at = float('-inf')
        self.stopped_since = None
        self.localization = ''
        self.localization_at = float('-inf')
        self.pose = None
        self.pose_issue = "map → base_link のTFをまだ受信していません。"
        self.lease_at = float('-inf')
        self.conflict = False

    def update_speed(self, value):
        if self.clock() - self.speed_at > 0.5:
            self.stopped_since = None
        self.speed, self.speed_at = value, self.clock()
        if value is not None and abs(value) < 0.05:
            if self.stopped_since is None:
                self.stopped_since = self.clock()
        else:
            self.stopped_since = None

    def apply_issue(self):
        now = self.clock()
        if self.mode != 3 or now - self.mode_at > 1.5:
            return '操作モードをSTOPにしてください（新しい状態通知が必要です）。'
        if self.speed is None or now - self.speed_at > 0.5 or self.stopped_since is None or now - self.stopped_since < 1.0:
            return '新しい速度情報で1秒以上の停車を確認できません。'
        if self.conflict:
            return '別の自動制御ノードが起動しています。調整用controllerだけを起動してください。'
        return ''

    def apply(self, request, rollback=False):
        issue = self.apply_issue()
        if issue:
            raise ValueError(issue)
        revision = self.active['revision'] if self.active else ''
        if request.get('expected_revision') != revision:
            raise ValueError('実車側の版が変わりました。状態を再取得してください。')
        candidate = self.previous if rollback else request.get('snapshot')
        if candidate is None:
            raise ValueError('戻せる版がありません。')
        validate_snapshot(candidate, self.map_id)
        self.previous, self.active = self.active, candidate
        return self.status()

    def touch_lease(self):
        # An expired browser session can only be re-armed after a confirmed stop.
        if self.clock() - self.lease_at < 4.0 or not self.apply_issue():
            self.lease_at = self.clock()

    def ready(self):
        now = self.clock()
        return bool(self.active and not self.conflict and now - self.lease_at < 4.0
                    and self.localization == 'localized' and now - self.localization_at < 1.5
                    and self.pose is not None and now - self.speed_at < 0.5
                    and now - self.mode_at < 1.5)

    def status(self):
        now = self.clock()
        def age(stamp):
            return round(now - stamp, 3) if stamp != float('-inf') else None
        return {'map_id': self.map_id, 'revision': self.active['revision'] if self.active else '',
                'line': self.active['line'] if self.active else '',
                'can_rollback': self.previous is not None, 'mode': self.mode,
                'speed_mps': self.speed if now - self.speed_at < 0.5 else None,
                'localization': self.localization if now - self.localization_at < 1.5 else 'stale',
                'pose': self.pose, 'pose_issue': self.pose_issue if self.pose is None else '',
                'localization_age_s': age(self.localization_at), 'odometry_age_s': age(self.speed_at),
                'mode_age_s': age(self.mode_at), 'ready': self.ready(), 'lease_live': now - self.lease_at < 4.0,
                'apply_issue': self.apply_issue()}
