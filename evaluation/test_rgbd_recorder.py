#!/usr/bin/env python3
"""Offline regression: actual ROS bags, including invalid payloads/truncation."""
import contextlib
import io
import json
from argparse import Namespace
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import rosbag
import rospy
from sensor_msgs.msg import Image

import rgbd_recorder as recorder


class BagValidation(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="rgbd_recorder_test_")
        self.root = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def make_bag(self, mode):
        path = self.root / (mode + ".bag")
        with rosbag.Bag(str(path), "w", compression="lz4") as bag:
            for i in range(30):
                for topic, encoding, pixel in ((recorder.RGB, "rgb8", 3),
                                                (recorder.DEPTH, "16UC1", 2)):
                    if mode == "missing_depth" and pixel == 2:
                        continue
                    msg = Image()
                    msg.width = msg.height = 16
                    msg.step = 16 * pixel
                    msg.encoding = encoding
                    msg.header.frame_id = "camera_infra1_frame"
                    stamp = rospy.Time.from_sec(100 + i * .1)
                    msg.header.stamp = stamp
                    msg.data = bytes([5]) * 768 if pixel == 3 else bytes([232, 3]) * 256
                    if mode == "zero_depth" and pixel == 2:
                        msg.data = bytes(512)
                    if mode == "bad_stamp" and i == 29:
                        msg.header.stamp = rospy.Time(1)
                    if mode == "bad_payload" and i == 29:
                        msg.data = b""
                    if mode == "unsynchronized" and pixel == 2:
                        msg.header.stamp = rospy.Time.from_sec(100 + i * .1 + .04)
                    bag.write(topic, msg, stamp)
        return path

    def check_bag(self, bag):
        report = self.root / "report.json"
        with contextlib.redirect_stdout(io.StringIO()):
            code = recorder.check(Namespace(bag=bag, report=report, rgb_topic=recorder.RGB,
                                            depth_topic=recorder.DEPTH, min_hz=3., sync_ms=5.))
        return code, json.loads(report.read_text())

    def test_healthy_bag(self):
        code, result = self.check_bag(self.make_bag("good"))
        self.assertEqual(code, 0)
        self.assertEqual(result["full_read_messages"], 60)
        self.assertEqual(result["paired_frames"], 30)

    def test_reject_bad_streams(self):
        for mode in ("missing_depth", "zero_depth", "bad_stamp", "bad_payload", "unsynchronized"):
            with self.subTest(mode=mode):
                code, result = self.check_bag(self.make_bag(mode))
                self.assertEqual(code, 1)
                self.assertTrue(result["errors"])

    def test_reject_truncated_and_active_bags(self):
        good = self.make_bag("good").read_bytes()
        for suffix, data in ((".bag", good[:len(good) // 2]), (".bag.active", good)):
            with self.subTest(suffix=suffix):
                path = self.root / ("incomplete" + suffix)
                path.write_bytes(data)
                code, result = self.check_bag(path)
                self.assertEqual(code, 1)
                self.assertTrue(result["errors"])


    def prepare_download(self, mode):
        local, remote = self.root / (mode + "_local"), self.root / (mode + "_remote")
        local.mkdir()
        remote.mkdir()
        data = self.make_bag(mode).read_bytes()
        for directory in (local, remote):
            (directory / "rgbd.bag").write_bytes(data)
            (directory / "camera.log").write_text("keep this log")
        recorder.save(local / "config.json", dict(rgb_topic=recorder.RGB, depth_topic=recorder.DEPTH,
                                                   min_hz=3., sync_ms=5.))
        result = dict(ok=True, bag="rgbd.bag", sha256=recorder.digest(local / "rgbd.bag"))
        return local, remote, result

    def test_delete_only_verified_board_bag_and_allow_retry(self):
        local, remote, result = self.prepare_download("good")
        # Execute the actual remote deletion command against a local fixture.
        with mock.patch.object(recorder, "ssh_command", side_effect=lambda args, cmd: ["bash", "-c", cmd]), \
                contextlib.redirect_stdout(io.StringIO()):
            for attempt in range(2):
                self.assertEqual(recorder.verify_download(Namespace(), str(remote), local, result), 0)
                self.assertFalse((remote / "rgbd.bag").exists())
                self.assertTrue((remote / "camera.log").is_file())
                self.assertEqual(recorder.digest(local / "rgbd.bag"), result["sha256"])

    def test_failed_validation_never_deletes_board_bag(self):
        for mode in ("hash_mismatch", "missing_depth", "recording_error", "ssh_error"):
            with self.subTest(mode=mode):
                local, remote, result = self.prepare_download(mode)
                if mode == "hash_mismatch":
                    result["sha256"] = "0" * 64
                if mode == "recording_error":
                    result["ok"] = False
                with mock.patch.object(recorder, "remove_verified_remote_bag") as remove, \
                        contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(RuntimeError):
                        recorder.verify_download(Namespace(), str(remote), local, result,
                                                 status=255 if mode == "ssh_error" else 0)
                    remove.assert_not_called()
                self.assertTrue((remote / "rgbd.bag").is_file())

    def test_remote_change_or_active_bag_prevents_deletion(self):
        for mode in ("changed", "active"):
            with self.subTest(mode=mode):
                local, remote, result = self.prepare_download(mode)
                if mode == "changed":
                    (remote / "rgbd.bag").write_bytes(b"different recording")
                else:
                    (remote / "rgbd.bag.active").write_bytes(b"recording")
                with mock.patch.object(recorder, "ssh_command", side_effect=lambda args, cmd: ["bash", "-c", cmd]), \
                        contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(RuntimeError):
                        recorder.verify_download(Namespace(), str(remote), local, result)
                self.assertTrue((remote / "rgbd.bag").is_file())
                self.assertTrue((local / "rgbd.bag").is_file())


if __name__ == "__main__":
    unittest.main()
