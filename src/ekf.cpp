#include <state_estimation/ekf.h>

#include <cmath>

bool Ekf::Init(uint64_t timestamp_us) {
  bool ret = InitInterface(timestamp_us);
  Reset();
  return ret;
}

void Ekf::Reset() {
  state_.velocity.setZero();
  state_.position.setZero();
  state_.delta_angle_bias.setZero();
  state_.delta_velocity_bias.setZero();
  state_.orientation.setIdentity();

  output_new_.orientation.setIdentity();
  output_new_.position.setZero();
  output_new_.velocity.setZero();

  imu_updated_ = false;
  filter_initialized_ = false;
}

bool Ekf::InitFilter() {
  const ImuSample &imu_init = imu_buffer_.Newest();
  if (imu_init.delta_velocity_dt < 1e-4 || imu_init.delta_angle_dt < 1e-4) {
    return false;
  }

  if (is_first_imu_sample_) {
    acceleration_filter_.Reset(imu_init.delta_velocity /
                               imu_init.delta_velocity_dt);
    gyro_filter_.Reset(imu_init.delta_angle / imu_init.delta_angle_dt);
    is_first_imu_sample_ = false;
  } else {
    acceleration_filter_.Update(imu_init.delta_velocity /
                                imu_init.delta_velocity_dt);
    gyro_filter_.Update(imu_init.delta_angle / imu_init.delta_angle_dt);
  }

  if (baro_buffer_.PopFirstOlderThan(imu_sample_delayed_.time_us,
                                     &baro_sample_delayed_)) {
    if (baro_sample_delayed_.time_us != 0) {
      // TODO: change this automatic height offset to some more reasonable
      // method. For example setting it manually.
      if (baro_counter_ == 0) {
        baro_height_offset_ = baro_sample_delayed_.height;
      } else {
        baro_height_offset_ =
            0.9 * baro_height_offset_ + 0.1 * baro_sample_delayed_.height;
      }

      baro_counter_++;
    }
  }

  if (baro_counter_ < observation_buffer_length_) {
    return false;
  }

  if (!InitTilt()) {
    return false;
  }
  // TODO: reset fusion times to latest imu

  InitCovariance();
  AlignOutputFilter();
  return true;
}

bool Ekf::InitTilt() {
  const double acceleration_norm = acceleration_filter_.State().norm();
  const double gyro_norm = gyro_filter_.State().norm();

  if (acceleration_norm < 0.8 * kGravity ||
      acceleration_norm > 1.2 * kGravity || gyro_norm > 15.0 * kPi / 180.0) {
    return false;
  }

  const Eigen::Vector3d gravity_body =
      acceleration_filter_.State().normalized();
  // TODO: check for correctness. also see:
  // https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
  const double pitch = asin(gravity_body(0));
  const double roll = atan2(-gravity_body(1), -gravity_body(2));
  const auto unit_x = Eigen::Vector3d::UnitX();
  const auto unit_y = Eigen::Vector3d::UnitY();
  const auto unit_z = Eigen::Vector3d::UnitZ();
  state_.orientation = Eigen::AngleAxisd(roll, unit_x) *
                       Eigen::AngleAxisd(pitch, unit_y) *
                       Eigen::AngleAxisd(0.0, unit_z);
}

bool Ekf::Update() {
  bool updated = false;
  if (!filter_initialized_) {
    filter_initialized_ = InitFilter();
    if (!filter_initialized_) {
      return false;
    }
  }

  if (imu_updated_) {
    updated = true;
    PredictState();
    PredictCovariance();
  }
  CalculateOutputState(latest_imu_sample_);
  return updated;
}
