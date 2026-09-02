from __future__ import annotations

import io
import json
import os
import tempfile
import threading
import unittest
from dataclasses import replace
from email.message import Message
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from jetpilot_console.config import ConsoleConfig
from jetpilot_console.main import Handler
from jetpilot_console.security import (
    MAX_JSON_BODY_BYTES,
    RequestRejected,
    decode_json_object,
    env_flag,
    is_loopback_bind,
    resolve_under_root,
    same_origin_allowed,
    save_joy_profile_files,
    validate_json_request_headers,
    validate_remote_absolute_path,
    validate_request_host,
    validate_ssh_target,
)
from jetpilot_console.tasks import TaskManager, TaskResourceConflict


class HeaderValidationTests(unittest.TestCase):
    def test_same_origin_requires_matching_host_and_port(self) -> None:
        self.assertTrue(same_origin_allowed("127.0.0.1:8765", "http://127.0.0.1:8765"))
        self.assertTrue(same_origin_allowed("localhost", "http://localhost"))
        self.assertTrue(same_origin_allowed("localhost:8765", None))
        self.assertFalse(same_origin_allowed("127.0.0.1:8765", "http://127.0.0.1:9999"))
        self.assertFalse(same_origin_allowed("127.0.0.1:8765", "https://example.test"))
        self.assertFalse(same_origin_allowed("localhost:8765", "http://localhost:8765/path"))
        self.assertFalse(same_origin_allowed("127.0.0.1:8765", "null"))

    def test_json_headers_accept_parameters_and_enforce_limit(self) -> None:
        length = validate_json_request_headers(
            content_type="application/json; charset=utf-8",
            content_length="2",
            transfer_encoding=None,
            host="localhost:8765",
            origin="http://localhost:8765",
        )
        self.assertEqual(length, 2)

        with self.assertRaises(RequestRejected) as raised:
            validate_json_request_headers(
                content_type="application/json",
                content_length=str(MAX_JSON_BODY_BYTES + 1),
                transfer_encoding=None,
                host="localhost:8765",
                origin=None,
            )
        self.assertEqual(raised.exception.status, 413)

    def test_json_headers_reject_unsafe_request_shapes(self) -> None:
        cases = [
            ({"content_type": "text/plain", "content_length": "2"}, 415),
            ({"content_type": "application/json", "content_length": None}, 411),
            ({"content_type": "application/json", "content_length": "bad"}, 400),
            (
                {
                    "content_type": "application/json",
                    "content_length": "2",
                    "origin": "http://attacker.test",
                },
                403,
            ),
        ]
        for overrides, expected_status in cases:
            values = {
                "content_type": "application/json",
                "content_length": "2",
                "transfer_encoding": None,
                "host": "localhost:8765",
                "origin": None,
            }
            values.update(overrides)
            with self.subTest(values=values):
                with self.assertRaises(RequestRejected) as raised:
                    validate_json_request_headers(**values)
                self.assertEqual(raised.exception.status, expected_status)

    def test_json_body_must_be_utf8_object(self) -> None:
        self.assertEqual(decode_json_object(b'{"ok": true}'), {"ok": True})
        for invalid in (b"", b"[]", b"{", b"\xff"):
            with self.subTest(invalid=invalid), self.assertRaises(RequestRejected):
                decode_json_object(invalid)

    def test_loopback_and_environment_flags(self) -> None:
        self.assertTrue(is_loopback_bind("127.0.0.1"))
        self.assertTrue(is_loopback_bind("::1"))
        self.assertTrue(is_loopback_bind("localhost"))
        self.assertFalse(is_loopback_bind("0.0.0.0"))
        self.assertFalse(is_loopback_bind("192.168.1.2"))
        self.assertTrue(env_flag("YES"))
        self.assertFalse(env_flag("no", default=True))
        self.assertFalse(env_flag("unexpected"))

    def test_loopback_host_header_rejects_dns_rebinding_names(self) -> None:
        for host in ("localhost:8765", "127.0.0.1:8765", "[::1]:8765"):
            with self.subTest(host=host):
                validate_request_host(host, loopback_only=True)
        for host in (None, "attacker.test:8765", "127.0.0.1.attacker.test:8765"):
            with self.subTest(host=host), self.assertRaises(RequestRejected):
                validate_request_host(host, loopback_only=True)

    def test_path_resolver_contains_symlinks_and_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            parent = Path(temporary_directory)
            root = parent / "root"
            root.mkdir()
            child = root / "child"
            child.mkdir()
            outside = parent / "outside"
            outside.mkdir()
            (root / "linked").symlink_to(outside, target_is_directory=True)

            self.assertEqual(
                resolve_under_root("child", root, require_exists=True, require_directory=True),
                child.resolve(),
            )
            for candidate in ("../outside", outside, root / "linked"):
                with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                    resolve_under_root(candidate, root)

    def test_ssh_target_and_remote_paths_reject_option_injection(self) -> None:
        self.assertEqual(validate_ssh_target("tamiya", "192.168.55.1"), "tamiya@192.168.55.1")
        self.assertEqual(
            validate_remote_absolute_path("/home/tamiya/maps/course a", label="remote path"),
            "/home/tamiya/maps/course a",
        )
        for user, host in (
            ("-oProxyCommand=touch", "jetson.local"),
            ("tamiya", "-oProxyCommand=touch"),
            ("user@other", "jetson.local"),
        ):
            with self.subTest(user=user, host=host), self.assertRaises(ValueError):
                validate_ssh_target(user, host)
        for path in ("relative/path", "/maps/../escape", "/maps\ncommand"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                validate_remote_absolute_path(path, label="remote path")


class JoyProfileSaveTests(unittest.TestCase):
    def test_saves_allowlisted_files_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_root = Path(temporary_directory) / "joy_profiles"
            saved = save_joy_profile_files(
                output_root,
                {
                    "joy_profile.yaml": "profile: 1\n",
                    "teleop_cmd.param.yaml": "teleop: 1\n",
                },
            )
            self.assertEqual(
                saved,
                [
                    str(output_root.resolve() / "joy_profile.yaml"),
                    str(output_root.resolve() / "teleop_cmd.param.yaml"),
                ],
            )
            self.assertEqual((output_root / "joy_profile.yaml").read_text(), "profile: 1\n")
            self.assertEqual(
                [item for item in output_root.iterdir() if item.name.endswith(".tmp")],
                [],
            )

    def test_rejects_absolute_traversal_and_unknown_names(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_root = Path(temporary_directory) / "joy_profiles"
            for name in ("/tmp/joy_profile.yaml", "../joy_profile.yaml", "custom.yaml"):
                with self.subTest(name=name), self.assertRaises(ValueError):
                    save_joy_profile_files(output_root, {name: "unsafe\n"})

    def test_rejects_symlink_file_and_output_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            outside = temporary_root / "outside.yaml"
            outside.write_text("original\n")

            output_root = temporary_root / "joy_profiles"
            output_root.mkdir()
            (output_root / "joy_profile.yaml").symlink_to(outside)
            with self.assertRaises(ValueError):
                save_joy_profile_files(output_root, {"joy_profile.yaml": "changed\n"})
            self.assertEqual(outside.read_text(), "original\n")

            linked_root = temporary_root / "linked_profiles"
            linked_root.symlink_to(output_root, target_is_directory=True)
            with self.assertRaises(ValueError):
                save_joy_profile_files(linked_root, {"teleop_cmd.param.yaml": "unsafe\n"})


class TaskResourceLockTests(unittest.TestCase):
    def test_synchronous_guard_conflicts_with_active_task(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            manager = TaskManager(root / "state", root)
            resource_key = f"map-dir:{root / 'map-a'}"
            with patch.object(manager, "_run_task", return_value=None):
                active = manager.start(
                    kind="analyze-rosbag",
                    title="Analyze with map",
                    command=["true"],
                    resource_key=resource_key,
                )

                with self.assertRaises(TaskResourceConflict) as raised:
                    with manager.guard_resources([resource_key]):
                        self.fail("guard must not be entered while the map is active")

            self.assertEqual(raised.exception.active_task["task_id"], active.task_id)

    def test_active_resource_blocks_same_and_different_task_kinds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            manager = TaskManager(root / "state", root)
            resource_key = f"map-dir:{root / 'map-a'}"
            with patch.object(manager, "_run_task", return_value=None):
                active = manager.start(
                    kind="generate-raceline",
                    title="Generate raceline",
                    command=["true"],
                    resource_key=resource_key,
                )

                for status, attempted_kind in (
                    ("queued", "generate-raceline"),
                    ("running", "prepare-hd-raster"),
                    ("stopping", "generate-preview"),
                ):
                    with self.subTest(status=status, attempted_kind=attempted_kind):
                        with manager.lock:
                            active.status = status
                        task_count = len(manager.list_tasks())
                        with self.assertRaises(TaskResourceConflict) as raised:
                            manager.start(
                                kind=attempted_kind,
                                title="Conflicting map task",
                                command=["true"],
                                resource_key=resource_key,
                            )
                        self.assertEqual(len(manager.list_tasks()), task_count)
                        self.assertEqual(raised.exception.active_task["task_id"], active.task_id)
                        self.assertEqual(raised.exception.active_task["status"], status)

    def test_multi_resource_task_blocks_each_claimed_resource(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            manager = TaskManager(root / "state", root)
            bag_key = f"analysis-bag:{root / 'record/run-a'}"
            map_key = f"map-dir:{root / 'map/course-a'}"

            with patch.object(manager, "_run_task", return_value=None):
                active = manager.start(
                    kind="analyze-rosbag",
                    title="Analyze run-a",
                    command=["true"],
                    resource_key=bag_key,
                    resource_keys=[bag_key, map_key, map_key],
                )

                self.assertEqual(active.resource_key, bag_key)
                self.assertEqual(active.resource_keys, [map_key])
                self.assertEqual(active.claimed_resource_keys(), (bag_key, map_key))

                for attempted_key in (bag_key, map_key):
                    with self.subTest(attempted_key=attempted_key):
                        with self.assertRaises(TaskResourceConflict) as raised:
                            manager.start(
                                kind="map-build",
                                title="Conflicting task",
                                command=["true"],
                                resource_key=attempted_key,
                            )
                        self.assertEqual(raised.exception.resource_key, attempted_key)
                        self.assertEqual(
                            raised.exception.active_task["task_id"], active.task_id
                        )

                unrelated = manager.start(
                    kind="map-build",
                    title="Different map",
                    command=["true"],
                    resource_key=f"map-dir:{root / 'map/course-b'}",
                )
                self.assertNotEqual(unrelated.task_id, active.task_id)

    def test_legacy_single_resource_state_keeps_string_shape(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            state_dir = root / "state"
            manager = TaskManager(state_dir, root)
            resource_key = f"map-dir:{root / 'map-a'}"

            with patch.object(manager, "_run_task", return_value=None):
                task = manager.start(
                    kind="map-build",
                    title="Build map",
                    command=["true"],
                    resource_key=resource_key,
                )

            persisted = json.loads((state_dir / "tasks.json").read_text(encoding="utf-8"))
            self.assertEqual(persisted[0]["resource_key"], resource_key)
            self.assertNotIn("resource_keys", persisted[0])

            reloaded = TaskManager(state_dir, root).get_task(task.task_id)
            self.assertIsNotNone(reloaded)
            assert reloaded is not None
            self.assertEqual(reloaded.resource_key, resource_key)
            self.assertEqual(reloaded.claimed_resource_keys(), (resource_key,))
            self.assertNotIn("resource_keys", reloaded.to_json())

    def test_resource_check_and_registration_are_atomic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            manager = TaskManager(root / "state", root)
            resource_key = f"map-dir:{root / 'map-a'}"
            barrier = threading.Barrier(2)
            outcomes: list[str] = []
            outcomes_lock = threading.Lock()

            def attempt(kind: str) -> None:
                barrier.wait()
                try:
                    manager.start(
                        kind=kind,
                        title=kind,
                        command=["true"],
                        resource_key=resource_key,
                    )
                except TaskResourceConflict:
                    outcome = "conflict"
                else:
                    outcome = "started"
                with outcomes_lock:
                    outcomes.append(outcome)

            with patch.object(manager, "_run_task", return_value=None):
                threads = [
                    threading.Thread(target=attempt, args=("map-build",)),
                    threading.Thread(target=attempt, args=("generate-raceline",)),
                ]
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join(timeout=2.0)

            self.assertEqual(sorted(outcomes), ["conflict", "started"])
            self.assertEqual(len(manager.list_tasks()), 1)


class ConsoleEndpointTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.ros2_ws = root / "ros2_ws"
        app_root = Path(__file__).resolve().parents[2]
        self.environment = patch.dict(
            os.environ,
            {
                "JETPILOT_CONSOLE_APP_ROOT": str(app_root),
                "JETPILOT_WORKSPACE_ROOT": str(root / "repo"),
                "JETPILOT_CONSOLE_STATE_DIR": str(root / "state"),
                "ROS2_WS": str(self.ros2_ws),
                "RECORD_ROOT": str(root / "record"),
                "MAP_ROOT": str(root / "map"),
                "JETPILOT_CONSOLE_ENABLE_CUSTOM_COMMANDS": "false",
            },
        )
        self.environment.start()
        self.config = ConsoleConfig.from_env()

    def tearDown(self) -> None:
        self.environment.stop()
        self.temporary_directory.cleanup()

    def write_valid_centerline(self, map_dir: Path) -> Path:
        map_dir.mkdir(parents=True, exist_ok=True)
        centerline = map_dir / f"{map_dir.name}_hd_map_centerline.csv"
        centerline.write_text(
            "".join(f"{index * 0.1},0.0,0.5,0.5\n" for index in range(8)),
            encoding="utf-8",
        )
        return centerline

    def post(
        self,
        path: str,
        body: object,
        *,
        content_type: str = "application/json",
        origin: str | None = None,
        host: str = "127.0.0.1:8765",
        loopback_only: bool = True,
        config: ConsoleConfig | None = None,
        tasks: object | None = None,
    ) -> tuple[int, dict[str, object]]:
        encoded = json.dumps(body).encode()
        message = Message()
        message["Content-Type"] = content_type
        message["Content-Length"] = str(len(encoded))
        message["Host"] = host
        if origin is not None:
            message["Origin"] = origin

        class TasksMustNotStart:
            def start(self, **_: object) -> None:
                raise AssertionError("disabled command endpoint started a task")

        handler = Handler.__new__(Handler)
        handler.path = path
        handler.command = "POST"
        handler.request_version = "HTTP/1.1"
        handler.requestline = f"POST {path} HTTP/1.1"
        handler.client_address = ("127.0.0.1", 12345)
        handler.close_connection = True
        handler.headers = message
        handler.rfile = io.BytesIO(encoded)
        handler.wfile = io.BytesIO()
        handler.server = SimpleNamespace(
            state=SimpleNamespace(
                config=config or self.config,
                joy_only=False,
                loopback_only=loopback_only,
                tasks=tasks if tasks is not None else TasksMustNotStart(),
            )
        )
        handler.do_POST()
        raw_headers, raw_body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        self.last_response_headers = raw_headers.decode("iso-8859-1")
        status = int(raw_headers.splitlines()[0].split()[1])
        return status, json.loads(raw_body.decode())

    def get(self, path: str) -> tuple[int, dict[str, object]]:
        message = Message()
        message["Host"] = "127.0.0.1:8765"
        handler = Handler.__new__(Handler)
        handler.path = path
        handler.command = "GET"
        handler.request_version = "HTTP/1.1"
        handler.requestline = f"GET {path} HTTP/1.1"
        handler.client_address = ("127.0.0.1", 12345)
        handler.close_connection = True
        handler.headers = message
        handler.rfile = io.BytesIO()
        handler.wfile = io.BytesIO()
        handler.server = SimpleNamespace(
            state=SimpleNamespace(
                config=self.config,
                joy_only=False,
                loopback_only=True,
                tasks=SimpleNamespace(),
            )
        )
        handler.do_GET()
        raw_headers, raw_body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        status = int(raw_headers.splitlines()[0].split()[1])
        return status, json.loads(raw_body.decode())

    def test_arbitrary_command_endpoint_is_disabled_by_default(self) -> None:
        status, payload = self.post(
            "/api/tasks/run",
            {"command": "touch should-not-exist"},
            origin="http://127.0.0.1:8765",
        )
        self.assertEqual(status, 403)
        self.assertIn("disabled", str(payload.get("error")))

    def test_post_request_validation_runs_before_endpoint(self) -> None:
        status, _ = self.post("/api/tasks/run", {}, content_type="text/plain")
        self.assertEqual(status, 415)
        status, _ = self.post("/api/tasks/run", {}, origin="http://attacker.test")
        self.assertEqual(status, 403)
        status, _ = self.post(
            "/api/tasks/run",
            {},
            origin="http://attacker.test:8765",
            host="attacker.test:8765",
        )
        self.assertEqual(status, 403)

    def test_joy_save_uses_allowlist(self) -> None:
        status, _ = self.post(
            "/api/joy-profile/save",
            {"files": {"../escape.yaml": "unsafe\n"}},
        )
        self.assertEqual(status, 400)

        status, payload = self.post(
            "/api/joy-profile/save",
            {"files": {"joy_profile.yaml": "safe: true\n"}},
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload.get("ok"))
        output = self.ros2_ws / "joy_profiles/joy_profile.yaml"
        self.assertEqual(output.read_text(), "safe: true\n")

    def test_custom_commands_are_opt_in(self) -> None:
        self.assertFalse(self.config.enable_custom_commands)
        with patch.dict(os.environ, {"JETPILOT_CONSOLE_ENABLE_CUSTOM_COMMANDS": "true"}):
            self.assertTrue(ConsoleConfig.from_env().enable_custom_commands)

        remotely_bound_config = replace(self.config, enable_custom_commands=True)
        status, _ = self.post(
            "/api/tasks/run",
            {"command": "touch should-not-exist"},
            loopback_only=False,
            config=remotely_bound_config,
        )
        self.assertEqual(status, 403)

    def test_security_headers_prevent_cross_site_framing(self) -> None:
        self.post("/api/tasks/run", {})
        self.assertIn("X-Frame-Options: SAMEORIGIN", self.last_response_headers)
        self.assertIn("Content-Security-Policy: frame-ancestors 'self'", self.last_response_headers)
        self.assertIn("X-Content-Type-Options: nosniff", self.last_response_headers)

    def test_jetson_inspection_is_not_available_as_get(self) -> None:
        status, _ = self.get("/api/jetson/inspect")
        self.assertEqual(status, 404)

    def test_jetson_inspection_rejects_ssh_option_injection(self) -> None:
        status, payload = self.post(
            "/api/jetson/inspect",
            {
                "host": "jetson.local",
                "user": "-oProxyCommand=touch",
                "map_root": "/maps",
                "record_root": "/record",
            },
        )
        self.assertEqual(status, 200)
        self.assertFalse(payload.get("ok"))
        self.assertIn("SSH user", str(payload.get("error")))

    def test_preflight_endpoint_returns_blocked_report_and_rejects_invalid_actions(self) -> None:
        status, payload = self.post(
            "/api/preflight",
            {"action": "generate-preview"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload.get("action"), "generate-preview")
        self.assertFalse(payload.get("ready"))
        self.assertEqual(payload.get("status"), "blocked")
        self.assertTrue(payload.get("checks"))

        for action in ("unknown-action", 42, None):
            with self.subTest(action=action):
                status, payload = self.post("/api/preflight", {"action": action})
                self.assertEqual(status, 400)
                self.assertIn("action", str(payload.get("error")))

    def test_preflight_endpoint_reports_ready_raceline_inputs(self) -> None:
        map_dir = self.config.map_root / "course_a"
        self.write_valid_centerline(map_dir)

        status, payload = self.post(
            "/api/preflight",
            {
                "action": "generate-raceline",
                "map_dir": str(map_dir),
                "vehicle_width_m": 0.187,
                "safety_margin_m": 0.02,
                "direction": "reverse",
            },
        )

        self.assertEqual(status, 200)
        self.assertTrue(payload.get("ready"))
        self.assertIn(payload.get("status"), {"pass", "warning"})
        self.assertEqual(payload["summary"]["blocked"], 0)

    def test_blocked_preflight_does_not_start_map_task(self) -> None:
        map_dir = self.config.map_root / "missing-centerline"
        map_dir.mkdir(parents=True)

        status, payload = self.post(
            "/api/maps/generate-raceline",
            {"map_dir": str(map_dir)},
        )

        self.assertEqual(status, 409)
        self.assertIn("preflight", payload)
        self.assertFalse(payload["preflight"]["ready"])
        self.assertIn("preflight", str(payload.get("error")))

    def test_map_task_conflict_returns_active_task_without_adding_a_task(self) -> None:
        map_dir = self.config.map_root / "course_a"
        self.write_valid_centerline(map_dir)
        manager = TaskManager(self.config.state_dir, self.config.repo_root)

        with patch.object(manager, "_run_task", return_value=None):
            first_status, first_payload = self.post(
                "/api/maps/generate-raceline",
                {"map_dir": str(map_dir)},
                tasks=manager,
            )
            self.assertEqual(first_status, 201)
            active_task_id = first_payload["task"]["task_id"]
            task_count = len(manager.list_tasks())

            status, payload = self.post(
                "/api/maps/generate-raceline",
                {"map_dir": str(map_dir)},
                tasks=manager,
            )

        self.assertEqual(status, 409)
        self.assertEqual(len(manager.list_tasks()), task_count)
        self.assertEqual(payload["active_task"]["task_id"], active_task_id)
        self.assertEqual(payload["active_task"]["status"], "queued")
        self.assertIn("already writing", str(payload.get("error")))

    def test_task_local_paths_cannot_escape_configured_roots(self) -> None:
        outside = str(Path(self.temporary_directory.name).parent)
        requests = [
            (
                "/api/maps/build-vgl-vslam",
                {"rosbag": outside, "map_dir": str(self.config.map_root / "map")},
            ),
            ("/api/maps/generate-preview", {"map_dir": outside}),
            (
                "/api/transfers/jetson-to-local",
                {"remote_path": "/remote/bag", "local_path": outside},
            ),
            (
                "/api/transfers/local-to-jetson",
                {"local_path": outside, "remote_path": "/remote/map"},
            ),
        ]
        for endpoint, body in requests:
            with self.subTest(endpoint=endpoint):
                status, _ = self.post(endpoint, body)
                self.assertEqual(status, 400)

    def test_map_push_infers_remote_destination_from_map_directory_name(self) -> None:
        map_dir = self.config.map_root / "course_a"
        map_dir.mkdir(parents=True)
        manager = TaskManager(self.config.state_dir, self.config.repo_root)

        with patch.object(manager, "_run_task", return_value=None):
            status, payload = self.post(
                "/api/transfers/local-to-jetson",
                {
                    "host": "jetson.local",
                    "user": "tamiya",
                    "local_path": str(map_dir),
                    "remote_path": "",
                },
                tasks=manager,
            )

        self.assertEqual(status, 201)
        task = manager.get_task(str(payload["task"]["task_id"]))
        self.assertIsNotNone(task)
        assert task is not None
        command = " ".join(task.command)
        self.assertIn(f"{self.config.jetson_map_root}/course_a", command)

    def test_config_canonicalizes_symlinked_workspace_roots(self) -> None:
        root = Path(self.temporary_directory.name)
        real_map_root = root / "real-map"
        real_map_root.mkdir()
        map_alias = root / "map-alias"
        map_alias.symlink_to(real_map_root, target_is_directory=True)

        with patch.dict(os.environ, {"MAP_ROOT": str(map_alias)}):
            config = ConsoleConfig.from_env()

        self.assertEqual(config.map_root, real_map_root.resolve())

    def test_map_build_rejects_config_and_model_paths_outside_allowed_roots(self) -> None:
        rosbag = self.config.record_root / "run"
        rosbag.mkdir(parents=True)
        map_dir = self.config.map_root / "course_a"
        topic_dir = (
            self.config.ros2_ws
            / "src/launch/jetpilot_system_launch/config/localization"
        )
        topic_dir.mkdir(parents=True)
        topic_config = topic_dir / "vgl_camera_topics.yaml"
        topic_config.write_text("topics: {}\n", encoding="utf-8")
        model_dir = (
            self.config.ros2_ws
            / "isaac_ros_assets/models/visual_global_localization"
        )
        model_dir.mkdir(parents=True)
        outside = Path(self.temporary_directory.name).parent

        for override in (
            {"topic_config": str(outside / "topics.yaml")},
            {
                "topic_config": str(topic_config),
                "output_model_dir": str(outside / "models"),
            },
            {"topic_config": str(topic_config), "steps": "edex --unsafe-option"},
        ):
            with self.subTest(override=override):
                status, _ = self.post(
                    "/api/maps/build-vgl-vslam",
                    {
                        "rosbag": str(rosbag),
                        "map_dir": str(map_dir),
                        **override,
                    },
                )
                self.assertEqual(status, 400)

    def test_raceline_stage_passes_validated_vehicle_clearance(self) -> None:
        map_dir = self.config.map_root / "course_a"
        self.write_valid_centerline(map_dir)

        class RecordingTasks:
            def __init__(self) -> None:
                self.calls: list[dict[str, object]] = []

            def start(self, **kwargs: object) -> object:
                self.calls.append(kwargs)
                return SimpleNamespace(to_json=lambda: {"task_id": "raceline-test"})

        tasks = RecordingTasks()
        status, payload = self.post(
            "/api/maps/generate-raceline",
            {
                "map_dir": str(map_dir),
                "vehicle_width_m": 0.187,
                "safety_margin_m": 0.02,
                "direction": "reverse",
            },
            tasks=tasks,
        )

        self.assertEqual(status, 201)
        self.assertEqual(payload["task"]["task_id"], "raceline-test")
        self.assertTrue(payload["preflight"]["ready"])
        self.assertEqual(len(tasks.calls), 1)
        command = tasks.calls[0]["command"]
        self.assertIn("--vehicle-width-m 0.187", command[2])
        self.assertIn("--safety-margin-m 0.02", command[2])
        self.assertIn("--direction reverse", command[2])

        status, payload = self.post(
            "/api/maps/generate-raceline",
            {
                "map_dir": str(map_dir),
                "vehicle_width_m": -0.01,
                "safety_margin_m": 0.02,
            },
        )
        self.assertEqual(status, 400)
        self.assertIn("finite value", str(payload.get("error")))


class StaticFileTests(unittest.TestCase):
    def make_handler(self, frontend_root: Path) -> Handler:
        handler = Handler.__new__(Handler)
        handler.command = "GET"
        handler.request_version = "HTTP/1.1"
        handler.requestline = "GET / HTTP/1.1"
        handler.client_address = ("127.0.0.1", 12345)
        handler.close_connection = True
        handler.wfile = io.BytesIO()
        handler.server = SimpleNamespace(
            state=SimpleNamespace(config=SimpleNamespace(frontend_root=frontend_root))
        )
        return handler

    def response_status(self, handler: Handler) -> int:
        return int(handler.wfile.getvalue().splitlines()[0].split()[1])

    def test_encoded_absolute_path_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            frontend_root = Path(temporary_directory)
            (frontend_root / "index.html").write_text("safe")
            handler = self.make_handler(frontend_root)
            handler._static("/%2Fetc/passwd")
            self.assertEqual(self.response_status(handler), 400)

    def test_static_symlink_cannot_escape_frontend_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            parent = Path(temporary_directory)
            frontend_root = parent / "frontend"
            frontend_root.mkdir()
            (frontend_root / "index.html").write_text("safe")
            outside = parent / "outside.txt"
            outside.write_text("secret")
            (frontend_root / "leak.txt").symlink_to(outside)
            handler = self.make_handler(frontend_root)
            handler._static("/leak.txt")
            self.assertEqual(self.response_status(handler), 400)


if __name__ == "__main__":
    unittest.main()
