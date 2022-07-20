#pragma once

#include <eigen3/Eigen/Dense>

inline bool ShouldUse321RotationSequence(const Eigen::Matrix3d &R) {
  return abs(R(2, 0)) < abs(R(2, 1));
}

inline double Euler321Yaw(const Eigen::Matrix3d &R) {
  return atan2(R(1, 0), R(0, 0));
}

inline double Euler321Yaw(const Eigen::Quaterniond &q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

inline double Euler312Yaw(const Eigen::Matrix3d &R) {
  return atan2(-R(0, 1), R(1, 1));
}

inline double Euler312Yaw(const Eigen::Quaterniond &q) {
  return atan2(2.0 * (q.w() * q.z() - q.x() * q.y()),
               q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z());
}

Eigen::Matrix3d UpdateYawInRotationMatrix(double yaw, const Eigen::Matrix3d &R);
