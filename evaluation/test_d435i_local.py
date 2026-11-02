#!/usr/bin/env python3
"""Offline tests for a USB D435i recorded on the same Linux host."""
import argparse
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
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
SCRIPT = ROOT / "record_d435i_10fps_local.py"
spec = importlib.util.spec_from_file_location("d435i_local", SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class LocalBags(unittest.TestCase):
    make_bag = legacy.BagValidation.make_bag
    check_bag = legacy.BagValidation.check_bag
    test_healthy_bag = legacy.BagValidation.test_healthy_bag
    test_reject_bad_streams = legacy.BagValidation.test_reject_bad_streams
    test_reject_truncated_and_active_bags = legacy.BagValidation.test_reject_truncated_and_active_bags

    def setUp(self):
        self.original = legacy.recorder
        legacy.recorder = recorder
        recorder.STOP.clear()
        legacy.BagValidation.setUp(self)

    def tearDown(self):
        recorder.STOP.clear()
        legacy.recorder = self.original
        legacy.BagValidation.tearDown(self)

    def simulate_session(self, mode="good", control="signal", preflight_code=0,
                         keep_active=False, stop_error=False, no_space=False):
        recorder.STOP.clear()
        template = self.make_bag(mode)
        args = argparse.Namespace(output=str(self.root / "sessions"), duration=10 if control == "timer" else 0,
                                  reserve_gb=1 if no_space else 0, extra_topic=["/extra/test"],
                                  min_hz=3., sync_ms=5., warmup=2., timeout=45.,
                                  busy_process=["nonexistent_d435i_fixture_process"])
        children, stopped, messages = {}, [], []
        locations = []

        def start(command, log, env=None):
            run = Path(log).parent
            if not locations:
                locations.append(run)
            child = mock.Mock(returncode=preflight_code if "probe" in command else 0)
            child.kind = Path(log).name
            child.poll.return_value = preflight_code if "probe" in command else None
            children[child.kind] = child
            if "probe" in command:
                recorder.save(run / "stream_check.json", {"ok": not preflight_code})
            if command[0] == "rosbag":
                children["record_command"] = command
                shutil.copy2(template, run / "rgbd.bag.active")
            if Path(log).name == "camera.log":
                self.assertEqual(command[0], "bash")  # Data disks may be mounted noexec.
            return child

        def stop(child, grace=30):
            if child is None:
                return True
            stopped.append(child.kind)
            if child.kind == "record.log" and not keep_active:
                (locations[0] / "rgbd.bag.active").rename(locations[0] / "rgbd.bag")
            if stop_error and child.kind == "camera.log":
                raise OSError("fixture shutdown error")
            return True

        def say(message):
            messages.append(message)
            if message.startswith("RECORDING:") and control == "signal":
                recorder.on_stop(signal.SIGINT, None)

        stdin = mock.Mock()
        stdin.isatty.return_value = control == "enter"
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(recorder, "start", side_effect=start))
            stack.enter_context(mock.patch.object(recorder, "stop_process", side_effect=stop))
            stack.enter_context(mock.patch.object(recorder, "wait_for"))
            stack.enter_context(mock.patch.object(recorder, "say", side_effect=say))
            socket = stack.enter_context(mock.patch.object(recorder.socket, "socket"))
            socket.return_value.__enter__.return_value.getsockname.return_value = ("127.0.0.1", 12345)
            stack.enter_context(mock.patch.object(sys, "stdin", stdin))
            if control == "enter":
                stack.enter_context(mock.patch.object(recorder.select, "select", return_value=([stdin], [], [])))
            if control == "timer":
                times = iter([100., 110.])
                stack.enter_context(mock.patch.object(recorder.time, "monotonic", side_effect=lambda: next(times, 110.)))
            if no_space:
                stack.enter_context(mock.patch.object(recorder.shutil, "disk_usage", return_value=argparse.Namespace(free=0)))
            status = recorder.record_local(args)
        run, = list(Path(args.output).glob("d435i10_local_*"))[-1:]
        result = json.loads((run / "session_result.json").read_text())
        if control == "enter":
            stdin.readline.assert_called_once()
        return status, run, result, children, stopped

    def test_ctrl_c_seals_valid_bag_and_keeps_it(self):
        status, run, result, children, stopped = self.simulate_session()
        self.assertEqual(status, 0, result)
        self.assertTrue(result["ok"])
        self.assertEqual(stopped, ["record.log", "stream_check.log", "camera.log", "master.log"])
        self.assertTrue((run / "rgbd.bag").is_file())
        self.assertEqual((run / "SHA256SUMS").read_text().split()[0], recorder.digest(run / "rgbd.bag"))
        validation = json.loads((run / "bag_check.json").read_text())
        self.assertTrue(validation["ok"])
        self.assertEqual(validation["full_read_messages"], 60)
        self.assertEqual(validation["sha256"], result["sha256"])
        self.assertIn("/extra/test", children["record_command"])
        self.assertNotIn("/camera/color/image_raw", children["record_command"])

    def test_duration_stops_after_recording_begins(self):
        status, run, result, _, _ = self.simulate_session(control="timer")
        self.assertEqual(status, 0, result)
        self.assertTrue(recorder.STOP.is_set())
        self.assertTrue((run / "rgbd.bag").exists())

    def test_enter_stops_and_validates(self):
        status, _, result, _, _ = self.simulate_session(control="enter")
        self.assertEqual(status, 0, result)

    def test_bad_bag_is_kept_with_failed_report(self):
        status, run, result, _, _ = self.simulate_session(mode="missing_depth")
        self.assertEqual(status, 1)
        self.assertFalse(result["ok"])
        self.assertTrue((run / "rgbd.bag").exists())
        self.assertFalse(json.loads((run / "bag_check.json").read_text())["ok"])

    def test_unfinished_active_bag_is_kept(self):
        status, run, result, _, _ = self.simulate_session(keep_active=True)
        self.assertEqual(status, 1)
        self.assertFalse(result["ok"])
        self.assertTrue((run / "rgbd.bag.active").exists())

    def test_preflight_failure_does_not_start_recording(self):
        status, run, result, children, _ = self.simulate_session(preflight_code=1)
        self.assertEqual(status, 1)
        self.assertFalse(result["ok"])
        self.assertNotIn("record.log", children)
        self.assertTrue((run / "config.json").exists())
        self.assertTrue((run / "d435i_camera_30fps.launch").exists())

    def test_shutdown_error_does_not_skip_other_cleanup_or_delete_bag(self):
        status, run, result, _, stopped = self.simulate_session(stop_error=True)
        self.assertEqual(status, 1)
        self.assertIn("master.log", stopped)
        self.assertTrue((run / "rgbd.bag").exists())
        self.assertTrue(any("fixture shutdown error" in error for error in result["errors"]))

    def test_insufficient_space_preserves_session_and_starts_no_process(self):
        status, run, result, children, _ = self.simulate_session(no_space=True)
        self.assertEqual(status, 1)
        self.assertFalse(children)
        self.assertTrue(any("空间不足" in error for error in result["errors"]))
        self.assertTrue((run / "config.json").exists())


class LocalPortability(unittest.TestCase):
    def test_single_file_help_without_ros_or_repository(self):
        with tempfile.TemporaryDirectory(prefix="d435i local ") as directory:
            script = Path(directory) / "record.py"
            shutil.copy2(SCRIPT, script)
            result = subprocess.run([sys.executable, "-I", "-S", str(script), "--help"],
                                    cwd=directory, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("--duration", result.stdout)
            self.assertIn("--check-env", result.stdout)
            self.assertNotIn("--board", result.stdout)
            self.assertNotIn("--fetch", result.stdout)
            self.assertEqual(list(Path(directory).iterdir()), [script])

    def test_ros_bootstrap_preserves_arguments_and_check_env_opens_no_camera(self):
        with tempfile.TemporaryDirectory(prefix="d435i env ") as directory:
            root = Path(directory)
            package = root / "camera package"
            (package / "launch").mkdir(parents=True)
            (package / "launch/rs_camera.launch").write_text("<launch/>")
            binary = root / "bin"
            binary.mkdir()
            rospack = binary / "rospack"
            rospack.write_text('#!/bin/bash\nset -e\ntest "$D435I_LOCAL_TEST" = ready\nprintf "%s\\n" ' + shlex.quote(str(package)) + '\n')
            rospack.chmod(0o755)
            ros_setup = root / "ros setup.bash"
            ros_setup.write_text('test "$#" = 0\nsource /opt/ros/noetic/setup.bash\nexport PATH=' + shlex.quote(str(binary)) + ':"$PATH"\n')
            camera_setup = root / "camera setup.bash"
            camera_setup.write_text('test "$#" = 0\nexport D435I_LOCAL_TEST=ready\n')
            output = root / "output bags"
            argv = [str(SCRIPT), "--ros-setup", str(ros_setup), "--camera-setup", str(camera_setup),
                    "--output", str(output), "--duration", "10", "--check-env"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(recorder.os, "execv") as execute, \
                    mock.patch.object(signal, "signal"):
                recorder.main()
            executable, command = execute.call_args.args
            result = subprocess.run(command, executable=executable, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(str(package), result.stdout)
            self.assertFalse(output.exists())

    def test_check_env_does_not_call_record_local(self):
        with mock.patch.object(sys, "argv", [str(SCRIPT), "--ros-ready", "--check-env"]), \
                mock.patch.object(recorder, "validate_environment") as validate, \
                mock.patch.object(recorder, "record_local") as record, mock.patch.object(signal, "signal"):
            self.assertEqual(recorder.main(), 0)
            validate.assert_called_once()
            record.assert_not_called()

    def test_missing_ros_setup_is_reported_before_exec(self):
        with mock.patch.object(recorder.os, "execv") as execute:
            with self.assertRaisesRegex(RuntimeError, "--ros-setup"):
                recorder.bootstrap_ros(argparse.Namespace(ros_setup="/no/such/ros/setup.bash", camera_setup=""))
            execute.assert_not_called()

    def test_exact_time_pairing_then_one_in_three_with_calibration(self):
        filters, publishers = [], {}

        def subscriber(*args, **kwargs):
            source = message_filters.SimpleFilter()
            filters.append(source)
            return source

        def publisher(topic, *args, **kwargs):
            publishers[topic] = mock.Mock()
            return publishers[topic]

        with mock.patch.object(message_filters, "Subscriber", side_effect=subscriber), \
                mock.patch.object(rospy, "Publisher", side_effect=publisher), mock.patch.object(rospy, "Subscriber"), \
                mock.patch.object(rospy.Time, "now", return_value=rospy.Time(100)):
            node = recorder.RgbdDecimator()
            info = CameraInfo()
            info.header.stamp = rospy.Time(5)
            node.info_callback(info, "color")
            node.info_callback(info, "depth")
            for i in range(7):
                color, depth = Image(), Image()
                color.header.stamp = depth.header.stamp = rospy.Time(100 + i)
                filters[0].signalMessage(color)
                filters[1].signalMessage(depth)
            mismatch = Image()
            mismatch.header.stamp = rospy.Time(200)
            filters[0].signalMessage(mismatch)
        selected = [call.args[0].header.stamp.to_sec() for call in publishers[recorder.RGB].publish.call_args_list]
        self.assertEqual(selected, [100., 103., 106.])
        self.assertEqual(node.pairs, 7)
        self.assertEqual(publishers[recorder.DEPTH].publish.call_count, 3)
        self.assertEqual(publishers["/camera_10fps/color/camera_info"].publish.call_count, 3)
        self.assertEqual(publishers["/camera_10fps/aligned_depth_to_color/camera_info"].publish.call_count, 3)
        self.assertEqual(info.header.stamp, rospy.Time(5))

    def test_launch_is_30fps_aligned_and_start_script_parses(self):
        with tempfile.TemporaryDirectory(prefix="d435i launch ") as directory:
            script = recorder.write_camera_files(Path(directory))
            result = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            launch = ET.fromstring(recorder.CAMERA_LAUNCH)
            args = {arg.attrib["name"]: arg.attrib["value"] for arg in launch.findall("include/arg")}
            self.assertEqual(args["color_fps"], "30")
            self.assertEqual(args["depth_fps"], "30")
            self.assertEqual(args["align_depth"], "true")
            self.assertEqual(args["enable_gyro"], "false")
            self.assertEqual(args["enable_accel"], "false")


if __name__ == "__main__":
    unittest.main()
