/**
 * This file is part of ORB-SLAM3.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 */

#ifndef ORB_SLAM3_IMU_PARAMETER_VALIDATION_H
#define ORB_SLAM3_IMU_PARAMETER_VALIDATION_H

#include <opencv2/core.hpp>

#include <cmath>
#include <string>

namespace ORB_SLAM3 {
namespace imu_parameter_validation {

struct ScaledNoise {
  float gyro;
  float acc;
  float gyroWalk;
  float accWalk;
};

// Both loaders convert directly from float matrix storage. Validate that
// storage and the original transform before any conversion to Eigen/Sophus.
inline bool ValidateTransform(const cv::Mat &transform, std::string &error) {
  if (transform.rows != 4 || transform.cols != 4) {
    error = "IMU transform must be a 4x4 matrix";
    return false;
  }
  if (transform.type() != CV_32FC1 || !transform.isContinuous()) {
    error = "IMU transform must have continuous single-channel CV_32F storage";
    return false;
  }

  double values[4][4];
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      values[row][col] = transform.at<float>(row, col);
      if (!std::isfinite(values[row][col])) {
        error = "IMU transform elements must be finite";
        return false;
      }
    }
  }

  double orthogonalitySquared = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double product = 0.0;
      for (int k = 0; k < 3; ++k)
        product += values[k][row] * values[k][col];
      const double difference = product - (row == col ? 1.0 : 0.0);
      orthogonalitySquared += difference * difference;
    }
  }
  if (!(std::sqrt(orthogonalitySquared) < 1e-5)) {
    error = "IMU rotation orthogonality error must be below 1e-5";
    return false;
  }

  const double determinant =
      values[0][0] * (values[1][1] * values[2][2] -
                      values[1][2] * values[2][1]) -
      values[0][1] * (values[1][0] * values[2][2] -
                      values[1][2] * values[2][0]) +
      values[0][2] * (values[1][0] * values[2][1] -
                      values[1][1] * values[2][0]);
  if (!(std::abs(determinant - 1.0) < 1e-5)) {
    error = "IMU rotation determinant error must be below 1e-5";
    return false;
  }
  for (int col = 0; col < 4; ++col) {
    const double expected = col == 3 ? 1.0 : 0.0;
    if (std::abs(values[3][col] - expected) > 1e-6) {
      error = "IMU transform last-row error must not exceed 1e-6";
      return false;
    }
  }
  return true;
}

// Match the float arithmetic used by the existing loaders and Calib::Set;
// checking the input alone would miss overflow in the covariance squares.
inline bool ScaleNoise(float frequency, float ng, float na, float ngw,
                       float naw, ScaledNoise &scaled, std::string &error) {
  if (!std::isfinite(frequency) || frequency <= 0.0f) {
    error = "IMU.Frequency must be finite and greater than zero";
    return false;
  }
  const float inputs[] = {ng, na, ngw, naw};
  const char *names[] = {"IMU.NoiseGyro", "IMU.NoiseAcc", "IMU.GyroWalk",
                         "IMU.AccWalk"};
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(inputs[i]) || inputs[i] < 0.0f) {
      error = std::string(names[i]) + " must be finite and non-negative";
      return false;
    }
  }

  const float sf = std::sqrt(frequency);
  if (!std::isfinite(sf) || sf <= 0.0f) {
    error = "IMU frequency square root must be finite and greater than zero";
    return false;
  }
  const ScaledNoise candidate = {ng * sf, na * sf, ngw / sf, naw / sf};
  const float results[] = {candidate.gyro, candidate.acc, candidate.gyroWalk,
                           candidate.accWalk};
  for (int i = 0; i < 4; ++i) {
    const float variance = results[i] * results[i];
    if (!std::isfinite(results[i]) || !std::isfinite(variance)) {
      error = std::string(names[i]) +
              " scaled noise and covariance square must be finite";
      return false;
    }
  }
  scaled = candidate;
  return true;
}

} // namespace imu_parameter_validation
} // namespace ORB_SLAM3

#endif
