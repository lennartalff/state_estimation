#pragma once

#include <stdint.h>

#include <eigen3/Eigen/Dense>

constexpr uint64_t kVisionMaxIntervalUs = (uint64_t)200e3;
constexpr uint64_t kBaroMaxIntervalUs = (uint64_t)200e3;
constexpr uint64_t kFilterUpdatePeriodUs = (uint64_t)10e3;
constexpr double kGravity = 9.81;
constexpr double kPi = 3.1415926;

struct StateSample {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d delta_angle_bias;
  Eigen::Vector3d delta_velocity_bias;
};

struct ImuSample {
  uint64_t time_us{0};
  Eigen::Vector3d delta_angle;
  Eigen::Vector3d delta_velocity;
  double delta_angle_dt;
  double delta_velocity_dt;
};

struct BaroSample {
  uint64_t time_us{0};
  double height;
};

struct VisionSample {
  uint64_t time_us{0};
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position_variance;
  Eigen::Vector3d velocity_variance;
  double angular_variance;
};

struct OutputSample {
  uint64_t time_us{0};
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d position;
};

struct ImuBiasEstimation {
  double time_constant{0.5};
  double accel_bias_magnitude_limit{0.4};  ///< max acceleration bias magnitude
  double accel_magnitude_limit{25.0};  ///< max acceleration magnitude for which
                                       ///< bias estimation will be applied.
  double gyro_magnitude_limit{
      3.0};  /// < max gyroscope rate vector magnitude for which bias estimation
             /// will be applied.
};

struct Settings {
  uint64_t min_observation_interval_us{(uint64_t)20e3};
  uint64_t vision_delay_us{(uint64_t)100e3};
  uint64_t baro_delay_us{(uint64_t)0};

  double gyro_noise{1.5e-2};
  double accel_noise{3.5e-1};

  double gyro_bias_noise{1.0e-3};
  double accel_bias_noise{1.0e-2};

  double baro_noise{0.02};

  double velocity_time_constant{0.25};
  double position_time_constant{0.25};

  ImuBiasEstimation imu_bias_estimation;

  double initial_tilt_error{0.1};
  double initial_gyro_bias{0.1};
  double initial_accel_bias{0.2};

  Eigen::Vector3d position_upper_limit = {2.0, 4.0, 0.5};
  Eigen::Vector3d position_lower_limit = {0.0, 0.0, -2.0};
  double velocity_limit{15.0};

  Eigen::Vector3d velocity_noise = {0.1, 0.1, 0.1};
  Eigen::Vector3d position_noise = {0.1, 0.1, 0.1};
};

template <typename T>
T clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}
