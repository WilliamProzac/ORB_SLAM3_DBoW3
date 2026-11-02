#!/usr/bin/env python3
"""Offline single-file portability, exact pairing and verified-transfer checks."""
import argparse
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import message_filters
import rospy
from sensor_msgs.msg import CameraInfo, Image

import test_rgbd_recorder as legacy

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "record_d435i_10fps_standalone.py"
spec = importlib.util.spec_from_file_location("d435i_standalone", SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class PortableBags(legacy.BagValidation):
    """Reuse the real LZ4 fixtures and failure-preservation tests for this file."""

    def setUp(self):
        self.original_recorder = legacy.recorder
        legacy.recorder = recorder
        recorder.STOP.clear()
        super().setUp()

    def tearDown(self):
        recorder.STOP.clear()
        legacy.recorder = self.original_recorder
        super().tearDown()

    def prepare_download(self, mode):
        local, remote, result = super().prepare_download(mode)
        with contextlib.redirect_stdout(io.StringIO()):
            recorder.check(argparse.Namespace(
                bag=remote / "rgbd.bag", report=local / "bag_check.json",
                rgb_topic=recorder.RGB, depth_topic=recorder.DEPTH, min_hz=3., sync_ms=5.))
        validation = json.loads((local / "bag_check.json").read_text())
        validation["sha256"] = result["sha256"]
        recorder.save(local / "bag_check.json", validation)
        return local, remote, result

    def test_report_must_match_exact_bag_before_deletion(self):
        for mode in ("missing", "hash", "size", "failed", "malformed"):
            with self.subTest(mode=mode):
                local, remote, result = self.prepare_download("report_" + mode)
                path = local / "bag_check.json"
                data = json.loads(path.read_text())
                if mode == "missing":
                    path.unlink()
                elif mode == "malformed":
                    path.write_text("{")
                else:
                    key, value = {"hash": ("sha256", "0" * 64), "size": ("bytes", 1),
                                  "failed": ("ok", False)}[mode]
                    data[key] = value
                    recorder.save(path, data)
                with mock.patch.object(recorder, "remove_verified_remote_bag") as remove, \
                        contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises((RuntimeError, OSError, ValueError)):
                        recorder.verify_download(argparse.Namespace(), str(remote), local, result)
                    remove.assert_not_called()
                self.assertTrue((remote / "rgbd.bag").exists())

    def test_host_verification_needs_no_ros_or_numpy(self):
        local, remote, result = self.prepare_download("no_host_ros")
        code = '''import argparse, importlib.util, json, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location("portable", sys.argv[1])
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
removed = []
m.remove_verified_remote_bag = lambda *args: removed.append(args)
assert m.verify_download(argparse.Namespace(), sys.argv[2], Path(sys.argv[3]), json.loads(sys.argv[4])) == 0
assert len(removed) == 1
assert "rosbag" not in sys.modules and "numpy" not in sys.modules
'''
        completed = subprocess.run([sys.executable, "-I", "-S", "-c", code, str(SCRIPT),
                                    str(remote), str(local), json.dumps(result)], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue((remote / "rgbd.bag").exists())  # Remote deletion was a capture.

    def test_board_session_generates_assets_and_validates_bag(self):
        run = self.root / "board_session"
        run.mkdir()
        template = self.make_bag("board_good")
        cfg = dict(busy_process=["nonexistent_camera_for_this_fixture"], reserve_gb=0,
                   init_script="", camera_start_script="", snapshot=[], rgb_topic=recorder.RGB,
                   depth_topic=recorder.DEPTH, min_hz=3., sync_ms=5., warmup=2., timeout=45.,
                   camera_info_topic=["/camera_10fps/color/camera_info"],
                   default_extra_topic=["/tf", "/tf_static"], extra_topic=[])
        recorder.save(run / "config.json", cfg)
        children = {}

        def start(command, log, env=None):
            child = mock.Mock(returncode=0)
            child.poll.return_value = 0 if "probe" in command else None
            if "probe" in command:
                recorder.save(run / "stream_check.json", dict(receive_hz={recorder.RGB: 10., recorder.DEPTH: 10.}, pair_ratio=1.))
            if command[0] == "rosbag":
                shutil.copy2(template, run / "rgbd.bag.active")
                children["recorder"] = child
            return child

        def stop(child, grace=30):
            if child is not None and child is children.get("recorder"):
                (run / "rgbd.bag.active").rename(run / "rgbd.bag")
            return True

        def say(message):
            if message.startswith("RECORDING:"):
                recorder.STOP.set()

        with mock.patch.object(recorder, "start", side_effect=start), \
                mock.patch.object(recorder, "stop_process", side_effect=stop), \
                mock.patch.object(recorder, "wait_for"), \
                mock.patch.object(recorder.threading, "Thread"), \
                mock.patch.object(recorder.socket, "socket") as socket, \
                mock.patch.object(recorder, "say", side_effect=say):
            socket.return_value.__enter__.return_value.getsockname.return_value = ("127.0.0.1", 12345)
            status = recorder.board(argparse.Namespace(config=str(run / "config.json")))
        result = json.loads((run / "board_result.json").read_text())
        self.assertEqual(status, 0, result)
        validation = json.loads((run / "bag_check.json").read_text())
        self.assertTrue(result["ok"])
        self.assertTrue(validation["ok"])
        self.assertEqual(validation["full_read_messages"], 60)
        self.assertEqual(validation["sha256"], result["sha256"])
        self.assertTrue((run / "start_rgbd10.sh.snapshot").is_file())
        self.assertTrue((run / "d435i_camera_30fps.launch").is_file())


class StandalonePortability(unittest.TestCase):
    def test_copied_file_runs_in_empty_directory_without_site_packages(self):
        with tempfile.TemporaryDirectory(prefix="d435i standalone ") as directory:
            script = Path(directory) / "record.py"
            shutil.copy2(SCRIPT, script)
            env = os.environ.copy()
            env["D435I_BOARD"] = "robot@10.0.0.20"
            result = subprocess.run([sys.executable, "-I", "-S", str(script), "--help"],
                                    cwd=directory, env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("robot@10.0.0.20", result.stdout)
            self.assertIn(str(Path(directory) / "bags"), result.stdout)
            self.assertEqual(list(Path(directory).iterdir()), [script])

    def test_check_env_parsing_and_command_line_priority(self):
        captured = []
        with mock.patch.dict(os.environ, {"D435I_BOARD": "old@10.0.0.1"}), \
                mock.patch.object(sys, "argv", [str(SCRIPT), "--board", "new@10.0.0.2", "--check-env"]), \
                mock.patch.object(recorder, "prepare_host", side_effect=lambda args: captured.append(args)), \
                mock.patch.object(recorder, "host") as host, mock.patch.object(signal, "signal"):
            self.assertEqual(recorder.main(), 0)
            host.assert_not_called()
        self.assertEqual(captured[0].board, "new@10.0.0.2")

    def test_fetch_environment_check_works_without_ros_setup(self):
        args = argparse.Namespace(board="robot@10.0.0.2", remote_root="", fetch="d435i10_test",
                                  ros_setup="/does/not/exist/setup.bash", camera_setup="")
        with mock.patch.object(recorder, "ssh_command", side_effect=lambda args, cmd: ["bash", "-c", cmd]), \
                contextlib.redirect_stdout(io.StringIO()):
            recorder.prepare_host(args)
        self.assertEqual(args.remote_root, str(Path.home() / "d435i_10fps_bags"))

    def test_missing_local_dependency_is_reported_before_connecting(self):
        with mock.patch.object(recorder.shutil, "which", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "缺少 ssh"):
                recorder.local_environment()

    def test_generated_launch_and_start_script_are_self_contained(self):
        with tempfile.TemporaryDirectory(prefix="d435i launch ") as directory:
            path = recorder.write_camera_files(Path(directory))
            result = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            launch = ET.fromstring((Path(directory) / "d435i_camera_30fps.launch").read_text())
            args = {arg.attrib["name"]: arg.attrib["value"] for arg in launch.findall("include/arg")}
            self.assertEqual(args["color_fps"], "30")
            self.assertEqual(args["depth_fps"], "30")
            self.assertEqual(args["align_depth"], "true")
            self.assertNotIn("rgbd_rate_adapter", recorder.CAMERA_LAUNCH)
            self.assertIn("decimate", path.read_text())

    def test_both_transfer_tools_use_selected_target(self):
        for rsync in (True, False):
            with self.subTest(rsync=rsync), mock.patch.object(recorder.subprocess, "run") as run:
                recorder.transfer(argparse.Namespace(board="robot@10.0.0.2", use_rsync=rsync),
                                  "/home/robot/recordings/run", Path("/tmp/local bags"))
                command = run.call_args.args[0]
                self.assertEqual(command[0], "rsync" if rsync else "scp")
                self.assertIn("robot@10.0.0.2:/home/robot/recordings/run/" + ("" if rsync else "."), command)

    @unittest.skipUnless(shutil.which("rsync"), "rsync is optional on the host")
    def test_rsync_fetch_repairs_same_size_same_mtime_corruption(self):
        with tempfile.TemporaryDirectory(prefix="d435i transfer ") as directory:
            remote, local = Path(directory) / "remote", Path(directory) / "local"
            remote.mkdir()
            local.mkdir()
            (remote / "rgbd.bag").write_bytes(b"correct bag payload")
            shutil.copy2(remote / "rgbd.bag", local / "rgbd.bag")
            original = (local / "rgbd.bag").stat()
            (local / "rgbd.bag").write_bytes(b"x" * original.st_size)
            os.utime(local / "rgbd.bag", ns=(original.st_atime_ns, original.st_mtime_ns))
            execute = subprocess.run

            def local_rsync(command, **kwargs):
                # Use the real rsync, substituting a local source for SSH transport.
                transport = command.index("-e")
                command = command[:transport] + command[transport + 2:]
                command[-2] = str(remote) + "/"
                return execute(command, capture_output=True, **kwargs)

            with mock.patch.object(recorder.subprocess, "run", side_effect=local_rsync):
                recorder.transfer(argparse.Namespace(board="fixture", use_rsync=True), str(remote), local)
            self.assertEqual((local / "rgbd.bag").read_bytes(), (remote / "rgbd.bag").read_bytes())

    def test_exact_pairing_then_every_third_pair_and_camera_info(self):
        filters = []
        publishers = {}

        def subscriber(*args, **kwargs):
            source = message_filters.SimpleFilter()
            filters.append(source)
            return source

        def publisher(topic, *args, **kwargs):
            publishers[topic] = mock.Mock()
            return publishers[topic]

        with mock.patch.object(message_filters, "Subscriber", side_effect=subscriber), \
                mock.patch.object(rospy, "Publisher", side_effect=publisher), \
                mock.patch.object(rospy, "Subscriber"), \
                mock.patch.object(rospy.Time, "now", return_value=rospy.Time(100)):
            node = recorder.RgbdDecimator()
            info = CameraInfo()
            info.width = 640
            info.header.stamp = rospy.Time(5)
            node.info_callback(info, "color")
            node.info_callback(info, "depth")
            for i in range(7):
                color, depth = Image(), Image()
                color.header.stamp = depth.header.stamp = rospy.Time(100 + i)
                color.header.frame_id = depth.header.frame_id = "camera_color_optical_frame"
                filters[0].signalMessage(color)
                filters[1].signalMessage(depth)
            mismatch = Image()
            mismatch.header.stamp = rospy.Time(200)
            filters[0].signalMessage(mismatch)
        selected = [call.args[0].header.stamp.to_sec() for call in publishers[recorder.RGB].publish.call_args_list]
        self.assertEqual(selected, [100., 103., 106.])
        self.assertEqual(node.pairs, 7)  # Unmatched color was excluded before decimation.
        self.assertEqual(publishers[recorder.DEPTH].publish.call_count, 3)
        self.assertEqual(publishers["/camera_10fps/color/camera_info"].publish.call_count, 3)
        self.assertEqual(publishers["/camera_10fps/aligned_depth_to_color/camera_info"].publish.call_count, 3)
        self.assertEqual(info.header.stamp, rospy.Time(5))  # Cached calibration was not modified.


if __name__ == "__main__":
    unittest.main()
