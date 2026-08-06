#!/usr/bin/env python3
"""Regression tests for XFeat* reliability and grid-quota selection."""

import sys
import unittest
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "xfeat_rdk_x5"))
sys.path.insert(0, str(ROOT / "Thirdparty" / "accelerated_features"))

from evaluate_fine_matcher import grid_quota_order, guided_stereo_matches
from modules.xfeat import XFeat


class GridQuotaTest(unittest.TestCase):
    def test_dense_api_returns_sorted_reliability(self):
        model = XFeat(top_k=32)
        image = torch.linspace(0.0, 1.0, 64 * 64).reshape(1, 1, 64, 64)
        output = model.detectAndComputeDense(image, top_k=32, multiscale=False)
        self.assertIn("scores", output)
        self.assertEqual(tuple(output["scores"].shape), (1, 32))
        self.assertTrue(torch.isfinite(output["scores"]).all())
        self.assertTrue(torch.all(output["scores"][:, :-1] >= output["scores"][:, 1:]))

    def test_grid_quota_is_hard_and_deterministic(self):
        points = torch.tensor(
            [[5.0, 5.0], [10.0, 10.0], [20.0, 20.0],
             [60.0, 10.0], [70.0, 20.0], [70.0, 70.0]]
        )
        rankings = torch.tensor([0.9, 0.8, 0.7, 0.95, 0.6, 0.5])
        selected = grid_quota_order(
            points, rankings, 6, 100, 100, columns=2, rows=2,
            maximum_per_cell=1,
        )
        self.assertEqual(selected.tolist(), [3, 0, 5])
        repeated = grid_quota_order(
            points, rankings, 6, 100, 100, columns=2, rows=2,
            maximum_per_cell=1,
        )
        self.assertTrue(torch.equal(selected, repeated))

    def test_guided_matching_uses_reliability_and_grid_quota(self):
        points = np.asarray(
            [[8.0, 8.0], [16.0, 16.0], [24.0, 24.0],
             [72.0, 8.0], [72.0, 72.0]], dtype=np.float32
        )
        descriptors = torch.eye(64, dtype=torch.float32)[: len(points)]
        features = {
            "points": points,
            "descriptors": descriptors,
            "scores": np.asarray([0.9, 0.8, 0.7, 0.95, 0.6], dtype=np.float32),
            "scales": np.ones(len(points), dtype=np.float32),
            "image_size": (100, 100),
        }
        query, train, cosine = guided_stereo_matches(
            features, features, 0.82, 0.9, 2.0, 160.0, 5,
            grid_columns=2, grid_rows=2, grid_maximum_per_cell=1,
        )
        self.assertEqual(query.tolist(), [3, 0, 4])
        self.assertEqual(train.tolist(), query.tolist())
        self.assertTrue(torch.allclose(cosine, torch.ones_like(cosine)))


if __name__ == "__main__":
    unittest.main()
