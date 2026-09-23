"""Dependency-free health and recovery policy for section routing."""
import math


class SectionGuard:
    def __init__(self, stable_sec=1.0, switch_sec=.15, max_speed=8., jump_margin=.5):
        self.stable_sec, self.switch_sec = stable_sec, switch_sec
        self.max_speed, self.jump_margin = max_speed, jump_margin
        self.last_pose = None
        self.healthy_since = None
        self.candidate = "unknown"
        self.candidate_since = 0.
        self.active = "unknown"

    def update(self, now, section, localized, pose, pose_stamp):
        healthy = localized and section != "unknown" and pose is not None
        if pose is not None:
            healthy = healthy and all(math.isfinite(v) for v in (*pose, pose_stamp))
            if self.last_pose is not None:
                previous, previous_stamp = self.last_pose
                dt = pose_stamp-previous_stamp
                distance = math.dist(pose, previous)
                healthy = healthy and dt >= 0 and distance <= self.jump_margin+self.max_speed*max(0., dt)
            self.last_pose = (pose, pose_stamp)
        if not healthy:
            self.healthy_since = None
            self.active = self.candidate = "unknown"
            return "unknown"
        if self.healthy_since is None:
            self.healthy_since = now
        if section != self.candidate:
            self.candidate, self.candidate_since = section, now
        if now-self.healthy_since >= self.stable_sec and now-self.candidate_since >= self.switch_sec:
            self.active = self.candidate
        return self.active
