#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path

from d435i_slam import make_settings, benchmark, percentiles, finalize


class AdaptationTests(unittest.TestCase):
    def info(self):
        return dict(width=640, height=480, distortion_model='plumb_bob',
                    K=[600., 0., 320., 0., 601., 240., 0., 0., 1.],
                    D=[.1, .2, .3, .4, .5])

    def test_calibration_and_units(self):
        result = make_settings(self.info())
        for line in ('Camera1.fx: 600.0', 'Camera1.fy: 601.0', 'Camera1.k3: 0.5',
                     'Camera.height: 480', 'Camera.fps: 10', 'Camera.RGB: 1',
                     'RGBD.DepthMapFactor: 1000.0', 'Camera.type: "PinHole"'):
            self.assertIn(line, result)
        self.assertNotIn('SaveAtlasToFile:', result)

    def test_reject_bad_calibration(self):
        for change in ({'height': 544}, {'distortion_model': 'equidistant'},
                       {'D': []}, {'K': [0.]*9}, {'K': [float('nan')]*9}):
            with self.assertRaises(ValueError):
                make_settings(dict(self.info(), **change))

    def test_four_distortion_terms(self):
        info = self.info()
        info['D'] = info['D'][:4]
        self.assertIn('Camera1.k3: 0.0', make_settings(info))

    def test_sequence_gap_and_histogram(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'benchmark.txt'
            p.write_text('seq.0.base=10\nseq.0.word.0=7\nseq.1.base=10\nseq.1.word.0=7\n'
                         'seq.2.base=10\nseq.2.word.0=5\ncallback_ms_bin.10=2\ncallback_ms_bin.20=1\n')
            raw, seqs, hist = benchmark(p)
            self.assertEqual(seqs[0]-seqs[2], {11})
            self.assertEqual(hist['count'], 3)
            self.assertEqual(hist['p50'], 10.5)
            self.assertEqual(hist['max'], 20.5)

    def test_percentiles(self):
        self.assertEqual(percentiles([]), {'count': 0})
        self.assertEqual(percentiles([1, 2, 3])['p50'], 2)

    def test_finalize_after_logging(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            log = path/'node.log'
            log.write_text('before shutdown\n')
            finalize(path)
            before = (path/'SHA256SUMS').read_text()
            log.write_text('before shutdown\nafter atexit\n')
            finalize(path)
            self.assertNotEqual(before, (path/'SHA256SUMS').read_text())
            self.assertEqual(len((path/'SHA256SUMS').read_text().splitlines()), 1)


if __name__ == '__main__':
    unittest.main()
