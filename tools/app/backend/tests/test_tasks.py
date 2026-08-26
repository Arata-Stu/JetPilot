from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from jetpilot_console.tasks import TaskManager


class TaskManagerCancellationTests(unittest.TestCase):
    def test_stopped_queued_task_is_not_started_by_worker(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            manager = TaskManager(root / "state", root)
            with patch("jetpilot_console.tasks.threading.Thread.start"):
                task = manager.start(
                    kind="test",
                    title="queued task",
                    command=["should-not-run"],
                )

            self.assertEqual(task.status, "queued")
            self.assertTrue(manager.stop(task.task_id))
            self.assertEqual(task.status, "stopped")

            with patch("jetpilot_console.tasks.subprocess.Popen") as popen:
                manager._run_task(task)
            popen.assert_not_called()
            self.assertEqual(task.status, "stopped")


if __name__ == "__main__":
    unittest.main()
