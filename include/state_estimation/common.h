#pragma once

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <stdint.h>

#include <Eigen/Dense>

struct ImuSample {
  uint64_t time_ns{0};
  Eigen::Vector3d delta_angle;
  Eigen::Vector3d delta_velocity;
  double delta_angle_dt;
  double delta_velocity_dt;
};

struct BaroSample {
  uint64_t time_ns{0};
  double height;
};

struct VisionSample {
  uint64_t time_ns{0};
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position_variance;
  Eigen::Vector3d velocity_variance;
  double angular_variance;
};

constexpr uint64_t kVisionMaxInterval = (uint64_t)200e6;
constexpr uint64_t kBaroMaxInterval = (uint64_t)200e6;

struct settings {
  int min_sensor_interval_us{(int)20e6};
  double vision_delay_us{(int)100e6};

  double gyro_noise{1.5e-2};
  double accel_noise{3.5e-1};

  double gyro_bias_noise{1.0e-3};
  double accel_bias_noise{1.0e-2};

  double baro_noise{0.02};

  double velocity_time_constant{0.25};
  double position_time_constant{0.25};
}
