#include <state_estimation/ekf.h>

void Ekf::SetControlBaroHeight() {
  control_status_.flags.baro_height = true;
  control_status_.flags.vision_height = false;
}

void Ekf::SetControlVisionHeight() {
  control_status_.flags.vision_height = true;
  control_status_.flags.baro_height = false;
}

Eigen::Vector3d Ekf::CalcRotationVectorVariances() {
  Eigen::Vector3d rot_vec_var;
  double q0, q1, q2, q3;
  if (state_.orientation.w() >= 0.0) {
    q0 = state_.orientation.w();
    q1 = state_.orientation.x();
    q2 = state_.orientation.y();
    q3 = state_.orientation.z();
  } else {
    q0 = -state_.orientation.w();
    q1 = -state_.orientation.x();
    q2 = -state_.orientation.y();
    q3 = -state_.orientation.z();
  }
  float t2 = q0 * q0;
  float t3 = acos(q0);
  float t4 = -t2 + 1.0;
  float t5 = t2 - 1.0;
  if ((t4 > 1e-9) && (t5 < -1e-9)) {
    float t6 = 1.0 / t5;
    float t7 = q1 * t6 * 2.0f;
    float t8 = 1.0 / pow(t4, 1.5);
    float t9 = q0 * q1 * t3 * t8 * 2.0;
    float t10 = t7 + t9;
    float t11 = 1.0 / sqrt(t4);
    float t12 = q2 * t6 * 2.0;
    float t13 = q0 * q2 * t3 * t8 * 2.0;
    float t14 = t12 + t13;
    float t15 = q3 * t6 * 2.0;
    float t16 = q0 * q3 * t3 * t8 * 2.0;
    float t17 = t15 + t16;
    rot_vec_var(0) =
        t10 * (P_(0, 0) * t10 + P_(1, 0) * t3 * t11 * 2.0) +
        t3 * t11 * (P_(0, 1) * t10 + P_(1, 1) * t3 * t11 * 2.0) * 2.0;
    rot_vec_var(1) =
        t14 * (P_(0, 0) * t14 + P_(2, 0) * t3 * t11 * 2.0) +
        t3 * t11 * (P_(0, 2) * t14 + P_(2, 2) * t3 * t11 * 2.0) * 2.0;
    rot_vec_var(2) =
        t17 * (P_(0, 0) * t17 + P_(3, 0) * t3 * t11 * 2.0) +
        t3 * t11 * (P_(0, 3) * t17 + P_(3, 3) * t3 * t11 * 2.0) * 2.0;
  } else {
    rot_vec_var = 4.0f * P_.block<3, 3>(1, 1).diagonal();
  }

  return rot_vec_var;
}

void Ekf::ResetHeight() {
  if (control_status_.flags.baro_height) {
    const BaroSample &baro_newest = baro_buffer_.Newest();
    if (!baro_height_faulty_) {
      ResetVerticalPositionTo(baro_newest.height + baro_height_offset_);
      P_.row(StateIndex::position_z).setZero();
      P_.col(StateIndex::position_z).setZero();
      P_(StateIndex::position_z, StateIndex::position_z) =
          square(settings_.baro_noise);
    } else {
      // TODO: reset to some old estimate?
    }
  } else if (control_status_.flags.vision_height) {
    const VisionSample &vision_newest = vision_buffer_.Newest();
    if (vision_newest.time_us >= vision_sample_delayed_.time_us) {
      ResetVerticalPositionTo(vision_newest.position(2));
    } else {
      ResetVerticalPositionTo(vision_sample_delayed_.position(2));
    }
  }

  ResetVerticalVelocityTo(0.0);
  P_.row(StateIndex::velocity_z).setZero();
  P_.col(StateIndex::velocity_z).setZero();
  P_(StateIndex::velocity_z, StateIndex::velocity_z) = 10.0;
}

void Ekf::ResetVerticalVelocityTo(double velocity) {
  const double delta_velocity = velocity - state_.velocity(2);

  state_.velocity(2) = velocity;

  output_new_.velocity(2) += delta_velocity;
  for (int i = 0; i < output_buffer_.Length(); ++i) {
    output_buffer_[i].velocity(2) += delta_velocity;
  }
}

void Ekf::ResetVerticalPositionTo(double position) {
  const double position_prev = state_.position(2);
  const double delta_position = position - position_prev;

  state_.position(2) = position;

  output_new_.position(2) += delta_position;
  for (int i = 0; i < output_buffer_.Length(); ++i) {
    output_buffer_[i].position(2) += delta_position;
  }
}

void Ekf::UpdateBaroHeightOffset() {
  if (!control_status_.flags.baro_height && baro_data_ready_) {
    const double local_time_step =
        clip<double>(1e-6 * delta_time_baro_us_, 0.0, 1.0);
    const double offset_rate_correction =
        0.1 * (state_.position(2) -
               (baro_sample_delayed_.height + baro_height_offset_));
    baro_height_offset_ +=
        local_time_step * clip<double>(offset_rate_correction, -0.1, 0.1);
  }
}

void Ekf::StartBaroHeightFusion() {
  SetControlBaroHeight();
  height_sensor_offset_ = 0.0;
}

void Ekf::CheckVerticalAccelHealth() {
  bool is_inertial_nav_falling = false;
  bool is_pos_vel_independent = false;
  if (IsRecent(vertical_position_fuse_attempt_time_us_, (uint64_t)1e6)) {
    is_inertial_nav_falling = vertical_position_innovation_ratio_ >
                              settings_.vertical_innovation_test_limit;
  }

  const unsigned int clip_count_limit = 1000000 / kFilterUpdatePeriodUs;
  const bool is_clipping = imu_sample_delayed_.delta_velocity_clipping[0] ||
                           imu_sample_delayed_.delta_velocity_clipping[1] ||
                           imu_sample_delayed_.delta_velocity_clipping[2];
  if (is_clipping && accel_clip_counter_ < clip_count_limit) {
    ++accel_clip_counter_;
  } else if (accel_clip_counter_ > 0) {
    --accel_clip_counter_;
  }

  const bool is_clipping_frequently = accel_clip_counter_ > 0;
  const bool bad_vertical_accel =
      (is_pos_vel_independent || is_clipping_frequently) &&
      is_inertial_nav_falling;

  if (bad_vertical_accel) {
    time_bad_vertical_accel_us_ = time_last_imu_;
  } else {
    time_good_vertical_accel_us_ = time_last_imu_;
  }

  if (fault_status_.flags.bad_acceleration_vertical) {
    fault_status_.flags.bad_acceleration_vertical =
        IsRecent(time_bad_vertical_accel_us_, kBadAccelProbation);
  } else {
    fault_status_.flags.bad_acceleration_vertical = bad_vertical_accel;
  }
}

bool Ekf::ResetYawToVision() {
  const double yaw_new =
      vision_sample_delayed_.orientation.toRotationMatrix().eulerAngles(0, 1,
                                                                        2)(2);
  const double yaw_new_variance =
      std::max<double>(vision_sample_delayed_.angular_variance, square(1e-2));
  ResetQuaternionStateYaw(yaw_new, yaw_new_variance, true);
  return true;
}

void Ekf::ResetQuaternionStateYaw(double yaw, double yaw_var,
                                  bool update_buffer) {
  const Eigen::Quaterniond q_prev = state_.orientation;
  R_to_earth_ = state_.orientation.toRotationMatrix();
  R_to_earth = UpdateYawInRotationMatrix(yaw, R_to_earth_);
  const Eigen::Quaterniond q_after_reset(R_to_earth_);
  const Eigen::Quaterniond q_error(
      (q_after_reset * q_prev.inverse()).normalized());

  state_.orientation = q_after_reset.normalized();
  UncorrelateQuaternion();
  if (yaw_var > __DBL_EPSILON__) {
    IncreateQuaternionYawErrorVariance(yaw_var);
  }

  if (update_buffer) {
    for (int i = 0; i < output_buffer_.Length(); ++i) {
      output_buffer_[i].orientation = q_error * output_new_.orientation;
    }
    output_new_.orientation = q_error * output_new_.orientation;
  }
}
