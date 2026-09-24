#!/usr/bin/env python3
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from rdk_rgbd_live import STOP, control, relay


class LiveTests(unittest.TestCase):
    def setUp(self):
        STOP.clear()

    def tearDown(self):
        STOP.clear()

    def test_relay_is_live_and_logs_original(self):
        read_fd, write_fd = os.pipe()
        with tempfile.TemporaryDirectory() as d, os.fdopen(read_fd) as source, os.fdopen(write_fd, 'w') as sender:
            seen = threading.Event()
            output = []
            def capture(line):
                output.append(line)
                seen.set()
            log = Path(d)/'node.log'
            with patch('rdk_rgbd_live.say', capture):
                thread = threading.Thread(target=relay, args=(source, log, '[SLAM] '))
                thread.start()
                sender.write('hello live\n')
                sender.flush()
                self.assertTrue(seen.wait(2))  # no EOF required for forwarding
                self.assertTrue(thread.is_alive())
                sender.close()
                thread.join(2)
            self.assertEqual(output, ['[SLAM] hello live'])
            self.assertEqual(log.read_text(), 'hello live\n')

    def test_control_eof_stops(self):
        read_fd, write_fd = os.pipe()
        os.close(write_fd)
        with os.fdopen(read_fd) as source:
            control(source, timeout=.1)
        self.assertTrue(STOP.is_set())

    def test_control_timeout_stops(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd) as source, os.fdopen(write_fd, 'w'):
            control(source, timeout=.05)
        self.assertTrue(STOP.is_set())

    def test_control_stop_command(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd) as source, os.fdopen(write_fd, 'w') as sender:
            sender.write('stop\n')
            sender.flush()
            control(source, timeout=.1)
        self.assertTrue(STOP.is_set())


if __name__ == '__main__':
    unittest.main()
