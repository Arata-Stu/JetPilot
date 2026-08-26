from __future__ import annotations

import json
import os
import signal
import shlex
import subprocess
import threading
import time
import uuid
from collections import deque
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator, Sequence


ACTIVE_TASK_STATUSES = frozenset({"queued", "running", "stopping"})


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


@dataclass
class Task:
    task_id: str
    kind: str
    title: str
    command: list[str]
    cwd: str
    status: str
    pid: int | None = None
    pgid: int | None = None
    started_at: str | None = None
    ended_at: str | None = None
    exit_code: int | None = None
    log_path: str = ""
    artifacts: list[dict[str, Any]] = field(default_factory=list)
    error: str = ""
    resource_key: str = ""
    # Additional exclusive resources owned by this task.  ``resource_key`` is
    # intentionally retained as the primary resource so task state written by
    # older Console versions keeps loading unchanged.
    resource_keys: list[str] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        payload = asdict(self)
        # Keep the persisted/API shape of legacy single-resource tasks stable.
        # The extra field is only needed when a task actually owns more than
        # one resource.
        if not self.resource_keys:
            payload.pop("resource_keys", None)
        return payload

    def claimed_resource_keys(self) -> tuple[str, ...]:
        """Return all exclusive resources, including legacy task state."""

        keys: list[str] = []
        for value in (self.resource_key, *self.resource_keys):
            normalized = str(value or "").strip()
            if normalized and normalized not in keys:
                keys.append(normalized)
        return tuple(keys)


class TaskResourceConflict(RuntimeError):
    """Raised when an active task already owns an exclusive resource."""

    def __init__(self, resource_key: str, active_task: Task):
        self.resource_key = resource_key
        self.active_task = active_task.to_json()
        super().__init__(
            f"resource is already in use by task {active_task.task_id} ({active_task.title})"
        )


class TaskManager:
    def __init__(self, state_dir: Path, default_cwd: Path):
        self.state_dir = state_dir
        self.task_dir = state_dir / "tasks"
        self.default_cwd = default_cwd
        self.state_file = state_dir / "tasks.json"
        self.lock = threading.RLock()
        self.tasks: dict[str, Task] = {}
        self.processes: dict[str, subprocess.Popen[bytes]] = {}
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.task_dir.mkdir(parents=True, exist_ok=True)
        self._load()
        self._recover_running_tasks()

    def _load(self) -> None:
        if not self.state_file.exists():
            return
        try:
            raw = json.loads(self.state_file.read_text(encoding="utf-8"))
            for item in raw:
                task = Task(**item)
                self.tasks[task.task_id] = task
        except Exception:
            backup = self.state_file.with_suffix(".corrupt.json")
            self.state_file.replace(backup)

    def _save(self) -> None:
        tmp = self.state_file.with_suffix(".tmp")
        data = [task.to_json() for task in self.tasks.values()]
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=True), encoding="utf-8")
        tmp.replace(self.state_file)

    def _recover_running_tasks(self) -> None:
        changed = False
        with self.lock:
            for task in self.tasks.values():
                if task.status in ACTIVE_TASK_STATUSES:
                    task.status = "lost"
                    task.ended_at = _now()
                    task.error = "Console restarted while task state was active."
                    changed = True
            if changed:
                self._save()

    def list_tasks(self) -> list[dict[str, Any]]:
        with self.lock:
            return [
                task.to_json()
                for task in sorted(
                    self.tasks.values(),
                    key=lambda item: item.started_at or "",
                    reverse=True,
                )
            ]

    def get_task(self, task_id: str) -> Task | None:
        with self.lock:
            return self.tasks.get(task_id)

    def _resource_conflict_unlocked(
        self, requested_resource_keys: Sequence[str]
    ) -> tuple[str, Task] | None:
        for existing in self.tasks.values():
            if existing.status not in ACTIVE_TASK_STATUSES:
                continue
            claimed = set(existing.claimed_resource_keys())
            conflicting_resource = next(
                (key for key in requested_resource_keys if key in claimed),
                None,
            )
            if conflicting_resource is not None:
                return conflicting_resource, existing
        return None

    @contextmanager
    def guard_resources(self, resource_keys: Sequence[str]) -> Iterator[None]:
        """Prevent task start while a short synchronous mutation is in progress."""

        requested = tuple(
            dict.fromkeys(str(value or "").strip() for value in resource_keys)
        )
        requested = tuple(value for value in requested if value)
        with self.lock:
            conflict = self._resource_conflict_unlocked(requested)
            if conflict is not None:
                resource_key, active_task = conflict
                raise TaskResourceConflict(resource_key, active_task)
            yield

    def start(
        self,
        *,
        kind: str,
        title: str,
        command: list[str],
        cwd: str | None = None,
        artifacts: list[dict[str, Any]] | None = None,
        resource_key: str | None = None,
        resource_keys: list[str] | tuple[str, ...] | None = None,
    ) -> Task:
        requested_resource_keys: list[str] = []
        for value in (resource_key, *(resource_keys or ())):
            normalized = str(value or "").strip()
            if normalized and normalized not in requested_resource_keys:
                requested_resource_keys.append(normalized)
        normalized_resource_key = (
            requested_resource_keys[0] if requested_resource_keys else ""
        )
        additional_resource_keys = requested_resource_keys[1:]
        task_id = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{kind}-{uuid.uuid4().hex[:8]}"
        with self.lock:
            if requested_resource_keys:
                conflict = self._resource_conflict_unlocked(requested_resource_keys)
                if conflict is not None:
                    conflicting_resource, existing = conflict
                    raise TaskResourceConflict(conflicting_resource, existing)

            task_path = self.task_dir / task_id
            task_path.mkdir(parents=True, exist_ok=True)
            log_path = task_path / "output.log"
            task = Task(
                task_id=task_id,
                kind=kind,
                title=title,
                command=command,
                cwd=str(Path(cwd).expanduser() if cwd else self.default_cwd),
                status="queued",
                log_path=str(log_path),
                artifacts=artifacts or [],
                resource_key=normalized_resource_key,
                resource_keys=additional_resource_keys,
            )
            self.tasks[task_id] = task
            self._save()

        thread = threading.Thread(target=self._run_task, args=(task,), daemon=True)
        try:
            thread.start()
        except Exception as exc:
            with self.lock:
                task.status = "failed"
                task.error = f"Could not start task worker: {exc}"
                task.ended_at = _now()
                self._save()
            raise
        return task

    def _run_task(self, task: Task) -> None:
        with self.lock:
            # A task can be cancelled after it is queued but before this worker
            # acquires the lock.  Do not revive that task and spawn its process.
            if task.status != "queued":
                if task.status == "stopping":
                    task.status = "stopped"
                    task.ended_at = task.ended_at or _now()
                    self._save()
                return
            task.status = "running"
            task.started_at = _now()
            self._save()

        log_file = Path(task.log_path)
        with log_file.open("ab", buffering=0) as handle:
            handle.write(f"$ {self.format_command(task.command)}\n".encode())
            try:
                process = subprocess.Popen(
                    task.command,
                    cwd=task.cwd,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                with self.lock:
                    self.processes[task.task_id] = process
                    task.pid = process.pid
                    try:
                        task.pgid = os.getpgid(process.pid)
                    except OSError:
                        task.pgid = process.pid
                    stop_after_spawn = task.status == "stopping"
                    self._save()

                # stop() may run in the small window between Popen and process
                # registration.  Honour that cancellation as soon as the
                # process group becomes available.
                if stop_after_spawn:
                    try:
                        os.killpg(task.pgid or process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass

                exit_code = process.wait()
                with self.lock:
                    task.exit_code = exit_code
                    if task.status == "stopping":
                        task.status = "stopped"
                    else:
                        task.status = "success" if exit_code == 0 else "failed"
                    task.ended_at = _now()
                    self.processes.pop(task.task_id, None)
                    self._save()
            except Exception as exc:
                handle.write(f"\n[console-error] {exc}\n".encode())
                with self.lock:
                    task.status = "failed"
                    task.error = str(exc)
                    task.ended_at = _now()
                    self.processes.pop(task.task_id, None)
                    self._save()

    def stop(self, task_id: str) -> bool:
        with self.lock:
            task = self.tasks.get(task_id)
            if task is None or task.status not in ACTIVE_TASK_STATUSES:
                return False
            if task.status == "queued":
                task.status = "stopped"
                task.ended_at = _now()
                self._save()
                return True
            task.status = "stopping"
            self._save()
            process = self.processes.get(task_id)
            pgid = task.pgid or (os.getpgid(process.pid) if process else None)

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        def force_kill() -> None:
            time.sleep(5)
            with self.lock:
                current = self.tasks.get(task_id)
                current_pgid = current.pgid if current else None
                if current is None or current.status != "stopping" or current_pgid is None:
                    return
            try:
                os.killpg(current_pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        threading.Thread(target=force_kill, daemon=True).start()
        return True

    def read_log(self, task_id: str, tail: int | None = None) -> str:
        task = self.get_task(task_id)
        if task is None or not task.log_path:
            return ""
        path = Path(task.log_path)
        if not path.exists():
            return ""
        if tail is None:
            return path.read_text(encoding="utf-8", errors="replace")
        if tail <= 0:
            return ""
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            lines = deque(handle, maxlen=tail)
        return "".join(lines)

    @staticmethod
    def format_command(command: list[str]) -> str:
        return shlex.join(command)
