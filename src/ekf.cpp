#include <state_estimation/ekf.h>
#include <state_estimation/util.h>

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

  dt_average_ = kFilterUpdatePeriodUs * 1e-6;

  output_new_.orientation.setIdentity();
  output_new_.position.setZero();
  output_new_.velocity.setZero();
  delta_angle_correction_.setZero();

  accel_magnitude_filtered_ = 0.0;
  gyro_magnitude_filtered_ = 0.0;
  prev_delta_velocity_bias_var_.setZero();

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
  state_.orientation.normalize();
  R_to_earth_ = state_.orientation.matrix();
  return true;
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
    UpdateSensorFusion();
  }
  CalculateOutputState(latest_imu_sample_);
  return updated;
}

void Ekf::InitCovariance() {
  P_.setZero();
  delta_angle_bias_var_accumulated_.setZero();
  delta_velocity_bias_var_accumulated_.setZero();

  const double dt = kFilterUpdatePeriodUs * 1e-6;
  ResetQuaternionCovariance();

  // velocity
  P_(4, 4) = pow(std::max(settings_.velocity_noise(0), 0.01), 2);
  P_(5, 5) = pow(std::max(settings_.velocity_noise(1), 0.01), 2);
  P_(6, 6) = pow(std::max(settings_.velocity_noise(2), 0.01), 2);

  // position
  P_(7, 7) = pow(std::max(settings_.position_noise(0), 0.01), 2);
  P_(8, 8) = pow(std::max(settings_.position_noise(1), 0.01), 2);
  P_(9, 9) = pow(std::max(settings_.position_noise(2), 0.01), 2);

  // gyro bias
  P_(10, 10) = pow(settings_.initial_gyro_bias * dt, 2);
  P_(12, 12) = P_(11, 11) = P_(10, 10);

  // accel bias
  P_(13, 13) = pow(settings_.initial_accel_bias * dt, 2);
  P_(15, 15) = P_(14, 14) = P_(13, 13);
}

void Ekf::InitQuaternionCovariances() {
  // TODO: check if more sophisticated initialization is necessary.
  SetZeroQuaternionCovariance();
  P_.topLeftCorner<4, 4>().setIdentity();
}

void Ekf::ResetQuaternionCovariance() {
  SetZeroQuaternionCovariance();
  InitQuaternionCovariances();
}

void Ekf::SetZeroQuaternionCovariance() {
  P_.topRows<4>().setZero();
  P_.leftCols<4>().setZero();
}

void Ekf::PredictState() {
  Eigen::Vector3d delta_angle_corrected =
      imu_sample_delayed_.delta_angle - state_.delta_angle_bias;
  const Eigen::Quaterniond delta_quat(Eigen::AngleAxisd{delta_angle_corrected});
  state_.orientation = (state_.orientation * delta_quat).normalized();

  const Eigen::Vector3d delta_velocity_corrected =
      imu_sample_delayed_.delta_velocity - state_.delta_velocity_bias;

  const Eigen::Vector3d velocity_last = state_.velocity;
  state_.velocity += delta_velocity_corrected;
  // TODO: check if sign is correct
  state_.velocity(2) += kGravity * imu_sample_delayed_.delta_velocity_dt;

  // trapezoidal integration
  state_.position += (velocity_last + state_.velocity) *
                     imu_sample_delayed_.delta_velocity_dt * 0.5;

  ConstrainStates();

  double dt = 0.5 * (imu_sample_delayed_.delta_velocity_dt +
                     imu_sample_delayed_.delta_angle_dt);
  dt = clip<double>(dt, 0.5 * kFilterUpdatePeriodUs * 1e-6,
                    2.0 * kFilterUpdatePeriodUs * 1e-6);

  dt_average_ = 0.99 * dt_average_ + 0.01 * dt;

  if (imu_sample_delayed_.delta_angle_dt >
      0.25 * kFilterUpdatePeriodUs * 1e-6) {
    angular_rate_delayed_raw_ =
        imu_sample_delayed_.delta_angle / imu_sample_delayed_.delta_angle_dt;
  }
}

void Ekf::PredictCovariance() {
  const double &qw = state_.orientation.w();
  const double &qx = state_.orientation.x();
  const double &qy = state_.orientation.y();
  const double &qz = state_.orientation.z();

  const double &delta_angle_x = imu_sample_delayed_.delta_angle(0);
  const double &delta_angle_y = imu_sample_delayed_.delta_angle(1);
  const double &delta_angle_z = imu_sample_delayed_.delta_angle(2);

  const double &delta_velocity_x = imu_sample_delayed_.delta_velocity(0);
  const double &delta_velocity_y = imu_sample_delayed_.delta_velocity(1);
  const double &delta_velocity_z = imu_sample_delayed_.delta_velocity(2);

  const double &delta_angle_bias_x = state_.delta_angle_bias(0);
  const double &delta_angle_bias_y = state_.delta_angle_bias(1);
  const double &delta_angle_bias_z = state_.delta_angle_bias(2);

  const double &delta_velocity_bias_x = state_.delta_velocity_bias(0);
  const double &delta_velocity_bias_y = state_.delta_velocity_bias(1);
  const double &delta_velocity_bias_z = state_.delta_velocity_bias(2);

  const double dt = kFilterUpdatePeriodUs * 1e-6;
  const double dt_inv = 1.0 / dt;

  const double delta_angle_bias_noise =
      dt * dt * clip<double>(settings_.gyro_bias_noise, 0.0, 1.0);
  const double delta_velocity_bias_noise =
      dt * dt * clip<double>(settings_.accel_bias_noise, 0.0, 1.0);

  const double alpha =
      clip<double>(dt / settings_.imu_bias_estimation.time_constant, 0.0, 1.0);
  const double beta = 1.0 - alpha;

  gyro_magnitude_filtered_ =
      std::max<double>(dt_inv * imu_sample_delayed_.delta_angle.norm(),
                       beta * gyro_magnitude_filtered_);
  accel_magnitude_filtered_ =
      std::max<double>(dt_inv * imu_sample_delayed_.delta_velocity.norm(),
                       beta * accel_magnitude_filtered_);
  accel_vec_filtered_ = alpha * dt_inv * imu_sample_delayed_.delta_velocity +
                        beta * accel_vec_filtered_;

  const bool high_manouvre_level =
      ((gyro_magnitude_filtered_ >
        settings_.imu_bias_estimation.gyro_magnitude_limit) ||
       (accel_magnitude_filtered_ >
        settings_.imu_bias_estimation.accel_magnitude_limit));

  for (int state_index = 13; state_index < 16; ++state_index) {
    const int index = state_index - 13;
    // TODO: introduce in air state and make bias always observable in air.
    // see:
    // https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/covariance.cpp#L154
    const bool bias_observable = std::abs(R_to_earth_(2, index)) > 0.8;
    // TODO: maybe also check for clipped imu data
    const bool inhibit_axis = !bias_observable || high_manouvre_level;

    if (inhibit_axis) {
      if (!accel_bias_inhibited_[index]) {
        prev_delta_velocity_bias_var_(index) = P_(state_index, state_index);
        accel_bias_inhibited_[index] = true;
      }

    } else {
      if (accel_bias_inhibited_[index]) {
        P_(state_index, state_index) = prev_delta_velocity_bias_var_(index);
        accel_bias_inhibited_[index] = false;
      }
    }
  }

  StateMatrixd process_noise_var;
  process_noise_var.diagonal().block<3, 1>(10, 0).setConstant(
      delta_angle_bias_noise * delta_angle_bias_noise);
  process_noise_var.diagonal().block<3, 1>(13, 0).setConstant(
      delta_velocity_bias_noise * delta_velocity_bias_noise);

  double gyro_noise = clip<double>(settings_.gyro_noise, 0.0, 1.0);
  const double delta_angle_x_var = dt * dt * gyro_noise * gyro_noise;
  const double delta_angle_y_var = delta_angle_x_var;
  const double delta_angle_z_var = delta_angle_x_var;

  double accel_noise = clip<double>(settings_.accel_noise, 0.0, 1.0);
  const double delta_velocity_x_var = dt * dt * accel_noise * accel_noise;
  const double delta_velocity_y_var = delta_velocity_x_var;
  const double delta_velocity_z_var = delta_velocity_x_var;

  // TODO: maybe also check for clipped imu data
  // see:
  // https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/covariance.cpp#L243
  //
  const StateMatrixd &P = P_;
  StateMatrixd P_new;

  // TODO: insert generated code here!
  const double tmp0 = pow(qx, 2);
  const double tmp1 = 0.25 * delta_angle_x_var;
  const double tmp2 = pow(qy, 2);
  const double tmp3 = 0.25 * delta_angle_y_var;
  const double tmp4 = pow(qz, 2);
  const double tmp5 = 0.25 * delta_angle_z_var;
  const double tmp6 = 0.5 * qx;
  const double tmp7 = 0.5 * qy;
  const double tmp8 = 0.5 * qz;
  const double tmp9 = P(10, 12) * tmp8;
  const double tmp10 = delta_angle_bias_x - delta_angle_x;
  const double tmp11 = 0.5 * tmp10;
  const double tmp12 = delta_angle_bias_y - delta_angle_y;
  const double tmp13 = 0.5 * tmp12;
  const double tmp14 = delta_angle_bias_z - delta_angle_z;
  const double tmp15 = 0.5 * tmp14;
  const double tmp16 = P(0, 10) + P(1, 10) * tmp11 + P(10, 10) * tmp6 +
                       P(10, 11) * tmp7 + P(2, 10) * tmp13 + P(3, 10) * tmp15 +
                       tmp9;
  const double tmp17 = P(10, 11) * tmp6;
  const double tmp18 = P(0, 11) + P(1, 11) * tmp11 + P(11, 11) * tmp7 +
                       P(11, 12) * tmp8 + P(2, 11) * tmp13 + P(3, 11) * tmp15 +
                       tmp17;
  const double tmp19 = P(11, 12) * tmp7;
  const double tmp20 = P(0, 12) + P(1, 12) * tmp11 + P(10, 12) * tmp6 +
                       P(12, 12) * tmp8 + P(2, 12) * tmp13 + P(3, 12) * tmp15 +
                       tmp19;
  const double tmp21 = P(1, 2) * tmp13;
  const double tmp22 = P(0, 1) + P(1, 1) * tmp11 + P(1, 10) * tmp6 +
                       P(1, 11) * tmp7 + P(1, 12) * tmp8 + P(1, 3) * tmp15 +
                       tmp21;
  const double tmp23 = P(2, 3) * tmp15;
  const double tmp24 = P(0, 2) + P(1, 2) * tmp11 + P(2, 10) * tmp6 +
                       P(2, 11) * tmp7 + P(2, 12) * tmp8 + P(2, 2) * tmp13 +
                       tmp23;
  const double tmp25 = P(1, 3) * tmp11;
  const double tmp26 = P(0, 3) + P(2, 3) * tmp13 + P(3, 10) * tmp6 +
                       P(3, 11) * tmp7 + P(3, 12) * tmp8 + P(3, 3) * tmp15 +
                       tmp25;
  const double tmp27 = P(0, 1) * tmp11;
  const double tmp28 = P(0, 2) * tmp13;
  const double tmp29 = P(0, 3) * tmp15;
  const double tmp30 = P(0, 0) + P(0, 10) * tmp6 + P(0, 11) * tmp7 +
                       P(0, 12) * tmp8 + tmp27 + tmp28 + tmp29;
  const double tmp31 = 0.5 * qw;
  const double tmp32 = qy * qz;
  const double tmp33 = qw * qx;
  const double tmp34 = qx * qz;
  const double tmp35 = qw * qy;
  const double tmp36 = qx * qy;
  const double tmp37 = qw * qz;
  const double tmp38 = 2 * tmp2;
  const double tmp39 = 2 * tmp4 - 1;
  const double tmp40 = tmp38 + tmp39;
  const double tmp41 = P(0, 13) + P(1, 13) * tmp11 + P(10, 13) * tmp6 +
                       P(11, 13) * tmp7 + P(12, 13) * tmp8 + P(2, 13) * tmp13 +
                       P(3, 13) * tmp15;
  const double tmp42 = tmp34 + tmp35;
  const double tmp43 = P(0, 15) + P(1, 15) * tmp11 + P(10, 15) * tmp6 +
                       P(11, 15) * tmp7 + P(12, 15) * tmp8 + P(2, 15) * tmp13 +
                       P(3, 15) * tmp15;
  const double tmp44 = 2 * tmp43;
  const double tmp45 = delta_velocity_bias_y - delta_velocity_y;
  const double tmp46 = qy * tmp45;
  const double tmp47 = delta_velocity_bias_z - delta_velocity_z;
  const double tmp48 = qz * tmp47;
  const double tmp49 = tmp46 + tmp48;
  const double tmp50 = 2 * tmp22;
  const double tmp51 = qy * tmp47;
  const double tmp52 = qz * tmp45;
  const double tmp53 = -tmp52;
  const double tmp54 = tmp51 + tmp53;
  const double tmp55 = 2 * tmp30;
  const double tmp56 = qw * tmp47;
  const double tmp57 = qx * tmp45;
  const double tmp58 = delta_velocity_bias_x - delta_velocity_x;
  const double tmp59 = qy * tmp58;
  const double tmp60 = tmp56 + tmp57 - 2 * tmp59;
  const double tmp61 = 2 * tmp24;
  const double tmp62 = qw * tmp45;
  const double tmp63 = qx * tmp47;
  const double tmp64 = qz * tmp58;
  const double tmp65 = -tmp62 + tmp63 - 2 * tmp64;
  const double tmp66 = 2 * tmp26;
  const double tmp67 = -tmp36 + tmp37;
  const double tmp68 = P(0, 14) + P(1, 14) * tmp11 + P(10, 14) * tmp6 +
                       P(11, 14) * tmp7 + P(12, 14) * tmp8 + P(2, 14) * tmp13 +
                       P(3, 14) * tmp15;
  const double tmp69 = 2 * tmp68;
  const double tmp70 = P(0, 4) + P(1, 4) * tmp11 + P(2, 4) * tmp13 +
                       P(3, 4) * tmp15 + P(4, 10) * tmp6 + P(4, 11) * tmp7 +
                       P(4, 12) * tmp8;
  const double tmp71 = 2 * tmp0;
  const double tmp72 = tmp39 + tmp71;
  const double tmp73 = tmp36 + tmp37;
  const double tmp74 = 2 * tmp41;
  const double tmp75 = qx * tmp58;
  const double tmp76 = tmp48 + tmp75;
  const double tmp77 = qz * tmp58 - tmp63;
  const double tmp78 = qw * tmp58;
  const double tmp79 = tmp51 - 2 * tmp52 + tmp78;
  const double tmp80 = -tmp59;
  const double tmp81 = -tmp56 - 2 * tmp57 - tmp80;
  const double tmp82 = -tmp32 + tmp33;
  const double tmp83 = P(0, 5) + P(1, 5) * tmp11 + P(2, 5) * tmp13 +
                       P(3, 5) * tmp15 + P(5, 10) * tmp6 + P(5, 11) * tmp7 +
                       P(5, 12) * tmp8;
  const double tmp84 = tmp38 + tmp71 - 1;
  const double tmp85 = tmp32 + tmp33;
  const double tmp86 = tmp46 + tmp75;
  const double tmp87 = tmp57 + tmp80;
  const double tmp88 = tmp62 - 2 * tmp63 + tmp64;
  const double tmp89 = -2 * tmp51 - tmp53 - tmp78;
  const double tmp90 = -tmp34 + tmp35;
  const double tmp91 = P(0, 6) + P(1, 6) * tmp11 + P(2, 6) * tmp13 +
                       P(3, 6) * tmp15 + P(6, 10) * tmp6 + P(6, 11) * tmp7 +
                       P(6, 12) * tmp8;
  const double tmp92 = P(10, 11) * tmp8;
  const double tmp93 = P(10, 12) * tmp7;
  const double tmp94 = -P(0, 10) * tmp11 + P(1, 10) - P(10, 10) * tmp31 -
                       P(2, 10) * tmp15 + 0.5 * P(3, 10) * tmp12 + tmp92 -
                       tmp93;
  const double tmp95 = P(10, 12) * tmp31;
  const double tmp96 = -P(0, 12) * tmp11 + P(1, 12) + 0.5 * P(11, 12) * qz -
                       P(12, 12) * tmp7 - P(2, 12) * tmp15 +
                       0.5 * P(3, 12) * tmp12 - tmp95;
  const double tmp97 = P(0, 2) * tmp15;
  const double tmp98 = P(0, 3) * tmp13;
  const double tmp99 = P(0, 0) * tmp11 - P(0, 1) + P(0, 10) * tmp31 -
                       0.5 * P(0, 11) * qz + P(0, 12) * tmp7 + tmp97 - tmp98;
  const double tmp100 = -tmp99;
  const double tmp101 = -P(1, 2);
  const double tmp102 = P(0, 2) * tmp11;
  const double tmp103 = P(2, 10) * tmp31 - 0.5 * P(2, 11) * qz +
                        P(2, 12) * tmp7 + P(2, 2) * tmp15 -
                        0.5 * P(2, 3) * tmp12 + tmp101 + tmp102;
  const double tmp104 = -tmp103;
  const double tmp105 = pow(qw, 2);
  const double tmp106 = P(10, 11) * tmp31;
  const double tmp107 = -P(0, 11) * tmp11 + P(1, 11) + 0.5 * P(11, 11) * qz -
                        P(2, 11) * tmp15 + 0.5 * P(3, 11) * tmp12 - tmp106 -
                        tmp19;
  const double tmp108 = -P(1, 3);
  const double tmp109 = P(0, 3) * tmp11;
  const double tmp110 = P(3, 10) * tmp31 - 0.5 * P(3, 11) * qz +
                        P(3, 12) * tmp7 - 0.5 * P(3, 3) * tmp12 + tmp108 +
                        tmp109 + tmp23;
  const double tmp111 = -tmp110;
  const double tmp112 = P(1, 2) * tmp15;
  const double tmp113 = P(1, 3) * tmp13;
  const double tmp114 = -P(1, 1) + P(1, 10) * tmp31 - 0.5 * P(1, 11) * qz +
                        P(1, 12) * tmp7 + tmp112 - tmp113 + tmp27;
  const double tmp115 = -tmp114;
  const double tmp116 = P(4, 10) * tmp31;
  const double tmp117 = P(4, 12) * tmp7;
  const double tmp118 = P(0, 4) * tmp11;
  const double tmp119 = P(2, 4) * tmp15;
  const double tmp120 = P(0, 15) * tmp11 - P(1, 15) + P(10, 15) * tmp31 -
                        P(11, 15) * tmp8 + P(12, 15) * tmp7 + P(2, 15) * tmp15 -
                        P(3, 15) * tmp13;
  const double tmp121 = 2 * tmp42;
  const double tmp122 = P(0, 14) * tmp11 - P(1, 14) + P(10, 14) * tmp31 -
                        P(11, 14) * tmp8 + P(12, 14) * tmp7 + P(2, 14) * tmp15 -
                        P(3, 14) * tmp13;
  const double tmp123 = 2 * tmp122;
  const double tmp124 = P(0, 13) * tmp11 - P(1, 13) + P(10, 13) * tmp31 -
                        P(11, 13) * tmp8 + P(12, 13) * tmp7 + P(2, 13) * tmp15 -
                        P(3, 13) * tmp13;
  const double tmp125 = 2 * tmp114;
  const double tmp126 = 2 * tmp99;
  const double tmp127 = 2 * tmp103;
  const double tmp128 = 2 * tmp110;
  const double tmp129 = -tmp122;
  const double tmp130 = -tmp120;
  const double tmp131 = -tmp79;
  const double tmp132 = -tmp124;
  const double tmp133 = 2 * tmp73;
  const double tmp134 = 2 * tmp76;
  const double tmp135 = -tmp47;
  const double tmp136 = -tmp58;
  const double tmp137 = 2 * qx * tmp135 - 2 * qz * tmp136;
  const double tmp138 = 2 * qw * tmp135 - 4 * qx * tmp45 - 2 * qy * tmp136;
  const double tmp139 = -P(1, 5);
  const double tmp140 = P(0, 5) * tmp11 + P(2, 5) * tmp15 -
                        0.5 * P(3, 5) * tmp12 + P(5, 10) * tmp31 -
                        0.5 * P(5, 11) * qz + P(5, 12) * tmp7 + tmp139;
  const double tmp141 = P(6, 10) * tmp31;
  const double tmp142 = P(6, 12) * tmp7;
  const double tmp143 = P(0, 6) * tmp11;
  const double tmp144 = P(2, 6) * tmp15;
  const double tmp145 = 2 * tmp90;
  const double tmp146 = -P(1, 4);
  const double tmp147 = -P(1, 6);
  const double tmp148 = P(11, 12) * tmp6;
  const double tmp149 = -P(0, 11) * tmp13 + 0.5 * P(1, 11) * tmp14 -
                        P(11, 11) * tmp31 + P(2, 11) - P(3, 11) * tmp11 +
                        tmp148 - tmp92;
  const double tmp150 = -P(0, 10) * tmp13 + 0.5 * P(1, 10) * tmp14 -
                        P(10, 10) * tmp8 + 0.5 * P(10, 12) * qx + P(2, 10) -
                        P(3, 10) * tmp11 - tmp106;
  const double tmp151 = -P(2, 3);
  const double tmp152 = -0.5 * P(1, 3) * tmp14 + P(3, 10) * tmp8 +
                        P(3, 11) * tmp31 - 0.5 * P(3, 12) * qx +
                        P(3, 3) * tmp11 + tmp151 + tmp98;
  const double tmp153 = -tmp152;
  const double tmp154 = P(0, 1) * tmp15;
  const double tmp155 = P(0, 0) * tmp13 + P(0, 10) * tmp8 + P(0, 11) * tmp31 -
                        0.5 * P(0, 12) * qx - P(0, 2) + tmp109 - tmp154;
  const double tmp156 = -tmp155;
  const double tmp157 = P(11, 12) * tmp31;
  const double tmp158 = -P(0, 12) * tmp13 + 0.5 * P(1, 12) * tmp14 +
                        0.5 * P(12, 12) * qx + P(2, 12) - P(3, 12) * tmp11 -
                        tmp157 - tmp9;
  const double tmp159 = P(0, 1) * tmp13;
  const double tmp160 = -0.5 * P(1, 1) * tmp14 + P(1, 10) * tmp8 +
                        P(1, 11) * tmp31 - 0.5 * P(1, 12) * qx + tmp101 +
                        tmp159 + tmp25;
  const double tmp161 = -tmp160;
  const double tmp162 = P(2, 3) * tmp11;
  const double tmp163 = P(2, 10) * tmp8 + P(2, 11) * tmp31 -
                        0.5 * P(2, 12) * qx - P(2, 2) - tmp112 + tmp162 + tmp28;
  const double tmp164 = -tmp163;
  const double tmp165 = P(4, 10) * tmp8;
  const double tmp166 = P(4, 11) * tmp31;
  const double tmp167 = P(0, 4) * tmp13;
  const double tmp168 = P(3, 4) * tmp11;
  const double tmp169 = P(0, 15) * tmp13 - P(1, 15) * tmp15 + P(10, 15) * tmp8 +
                        P(11, 15) * tmp31 - P(12, 15) * tmp6 - P(2, 15) +
                        P(3, 15) * tmp11;
  const double tmp170 = P(0, 14) * tmp13 - P(1, 14) * tmp15 + P(10, 14) * tmp8 +
                        P(11, 14) * tmp31 - P(12, 14) * tmp6 - P(2, 14) +
                        P(3, 14) * tmp11;
  const double tmp171 = 2 * tmp170;
  const double tmp172 = P(0, 13) * tmp13 - P(1, 13) * tmp15 + P(10, 13) * tmp8 +
                        P(11, 13) * tmp31 - P(12, 13) * tmp6 - P(2, 13) +
                        P(3, 13) * tmp11;
  const double tmp173 = 2 * tmp160;
  const double tmp174 = 2 * tmp155;
  const double tmp175 = 2 * tmp163;
  const double tmp176 = 2 * tmp152;
  const double tmp177 = -tmp170;
  const double tmp178 = -tmp169;
  const double tmp179 = -tmp172;
  const double tmp180 = -P(2, 5);
  const double tmp181 = P(0, 5) * tmp13 - 0.5 * P(1, 5) * tmp14 +
                        P(3, 5) * tmp11 + P(5, 10) * tmp8 + P(5, 11) * tmp31 -
                        0.5 * P(5, 12) * qx + tmp180;
  const double tmp182 = P(6, 10) * tmp8;
  const double tmp183 = P(6, 11) * tmp31;
  const double tmp184 = P(0, 6) * tmp13;
  const double tmp185 = P(3, 6) * tmp11;
  const double tmp186 = -P(2, 4);
  const double tmp187 = -P(2, 6);
  const double tmp188 = -P(0, 12) * tmp15 - P(1, 12) * tmp13 -
                        P(12, 12) * tmp31 + 0.5 * P(2, 12) * tmp10 + P(3, 12) -
                        tmp148 + tmp93;
  const double tmp189 = -P(0, 11) * tmp15 - P(1, 11) * tmp13 +
                        0.5 * P(10, 11) * qy - P(11, 11) * tmp6 +
                        0.5 * P(2, 11) * tmp10 + P(3, 11) - tmp157;
  const double tmp190 = P(1, 1) * tmp13 - 0.5 * P(1, 10) * qy +
                        P(1, 11) * tmp6 + P(1, 12) * tmp31 -
                        0.5 * P(1, 2) * tmp10 + tmp108 + tmp154;
  const double tmp191 = -tmp190;
  const double tmp192 = P(0, 0) * tmp15 - 0.5 * P(0, 10) * qy +
                        P(0, 11) * tmp6 + P(0, 12) * tmp31 - P(0, 3) - tmp102 +
                        tmp159;
  const double tmp193 = -tmp192;
  const double tmp194 = -P(0, 10) * tmp15 - P(1, 10) * tmp13 +
                        0.5 * P(10, 10) * qy + 0.5 * P(2, 10) * tmp10 +
                        P(3, 10) - tmp17 - tmp95;
  const double tmp195 = -0.5 * P(2, 10) * qy + P(2, 11) * tmp6 +
                        P(2, 12) * tmp31 - 0.5 * P(2, 2) * tmp10 + tmp151 +
                        tmp21 + tmp97;
  const double tmp196 = -tmp195;
  const double tmp197 = -P(3, 10) * tmp7 + P(3, 11) * tmp6 + P(3, 12) * tmp31 -
                        P(3, 3) + tmp113 - tmp162 + tmp29;
  const double tmp198 = P(4, 11) * tmp6;
  const double tmp199 = P(4, 12) * tmp31;
  const double tmp200 = P(0, 4) * tmp15;
  const double tmp201 = P(1, 4) * tmp13;
  const double tmp202 = P(0, 15) * tmp15 + P(1, 15) * tmp13 - P(10, 15) * tmp7 +
                        P(11, 15) * tmp6 + P(12, 15) * tmp31 -
                        P(2, 15) * tmp11 - P(3, 15);
  const double tmp203 = P(0, 14) * tmp15 + P(1, 14) * tmp13 - P(10, 14) * tmp7 +
                        P(11, 14) * tmp6 + P(12, 14) * tmp31 -
                        P(2, 14) * tmp11 - P(3, 14);
  const double tmp204 = 2 * tmp203;
  const double tmp205 = P(0, 13) * tmp15 + P(1, 13) * tmp13 - P(10, 13) * tmp7 +
                        P(11, 13) * tmp6 + P(12, 13) * tmp31 -
                        P(2, 13) * tmp11 - P(3, 13);
  const double tmp206 = 2 * tmp190;
  const double tmp207 = 2 * tmp192;
  const double tmp208 = 2 * tmp195;
  const double tmp209 = 2 * tmp197;
  const double tmp210 = -tmp203;
  const double tmp211 = -tmp202;
  const double tmp212 = -tmp205;
  const double tmp213 = -P(3, 5);
  const double tmp214 = P(0, 5) * tmp15 + P(1, 5) * tmp13 -
                        0.5 * P(2, 5) * tmp10 - 0.5 * P(5, 10) * qy +
                        P(5, 11) * tmp6 + P(5, 12) * tmp31 + tmp213;
  const double tmp215 = P(6, 11) * tmp6;
  const double tmp216 = P(6, 12) * tmp31;
  const double tmp217 = P(0, 6) * tmp15;
  const double tmp218 = P(1, 6) * tmp13;
  const double tmp219 = -P(3, 4);
  const double tmp220 = -P(3, 6);
  const double tmp221 = 2 * tmp49;
  const double tmp222 = 2 * tmp54;
  const double tmp223 = 2 * tmp60;
  const double tmp224 = 2 * tmp65;
  const double tmp225 = -P(0, 13) * tmp222 - P(1, 13) * tmp221 +
                        P(13, 13) * tmp40 + 2 * P(13, 14) * tmp67 -
                        P(13, 15) * tmp121 - P(2, 13) * tmp223 -
                        P(3, 13) * tmp224 + P(4, 13);
  const double tmp226 = -P(0, 14) * tmp222 - P(1, 14) * tmp221 +
                        P(13, 14) * tmp40 + 2 * P(14, 14) * tmp67 -
                        P(14, 15) * tmp121 - P(2, 14) * tmp223 -
                        P(3, 14) * tmp224 + P(4, 14);
  const double tmp227 = -P(0, 15) * tmp222 - P(1, 15) * tmp221 +
                        P(13, 15) * tmp40 + 2 * P(14, 15) * tmp67 -
                        P(15, 15) * tmp121 - P(2, 15) * tmp223 -
                        P(3, 15) * tmp224 + P(4, 15);
  const double tmp228 = -P(0, 1) * tmp222 - P(1, 1) * tmp221 +
                        P(1, 13) * tmp40 + 2 * P(1, 14) * tmp67 -
                        P(1, 15) * tmp121 - P(1, 2) * tmp223 -
                        P(1, 3) * tmp224 - tmp146;
  const double tmp229 = -P(0, 0) * tmp222 - P(0, 1) * tmp221 +
                        P(0, 13) * tmp40 + 2 * P(0, 14) * tmp67 -
                        P(0, 15) * tmp121 - P(0, 2) * tmp223 -
                        P(0, 3) * tmp224 + P(0, 4);
  const double tmp230 = -P(0, 2) * tmp222 - P(1, 2) * tmp221 +
                        P(2, 13) * tmp40 + 2 * P(2, 14) * tmp67 -
                        P(2, 15) * tmp121 - P(2, 2) * tmp223 -
                        P(2, 3) * tmp224 - tmp186;
  const double tmp231 = -P(0, 3) * tmp222 - P(1, 3) * tmp221 -
                        P(2, 3) * tmp223 + P(3, 13) * tmp40 +
                        2 * P(3, 14) * tmp67 - P(3, 15) * tmp121 -
                        P(3, 3) * tmp224 - tmp219;
  const double tmp232 = P(0, 4) * tmp222 + P(1, 4) * tmp221 + P(2, 4) * tmp223 +
                        P(3, 4) * tmp224 - P(4, 13) * tmp40 -
                        2 * P(4, 14) * tmp67 + P(4, 15) * tmp121 - P(4, 4);
  const double tmp233 = 2 * tmp229;
  const double tmp234 = 2 * tmp231;
  const double tmp235 = 2 * tmp228;
  const double tmp236 = -P(4, 5);
  const double tmp237 = P(0, 5) * tmp222 + P(1, 5) * tmp221 + P(2, 5) * tmp223 +
                        P(3, 5) * tmp224 - P(5, 13) * tmp40 -
                        2 * P(5, 14) * tmp67 + P(5, 15) * tmp121 + tmp236;
  const double tmp238 = 2 * tmp85;
  const double tmp239 = 2 * tmp89;
  const double tmp240 = -P(4, 6);
  const double tmp241 = P(0, 6) * tmp222 + P(1, 6) * tmp221 + P(2, 6) * tmp223 +
                        P(3, 6) * tmp224 - P(6, 13) * tmp40 -
                        2 * P(6, 14) * tmp67 + P(6, 15) * tmp121 + tmp240;
  const double tmp242 = 2 * tmp77;
  const double tmp243 = 2 * tmp79;
  const double tmp244 = 2 * tmp81;
  const double tmp245 = -P(0, 14) * tmp242 - P(1, 14) * tmp244 -
                        P(13, 14) * tmp133 + P(14, 14) * tmp72 +
                        2 * P(14, 15) * tmp82 - P(2, 14) * tmp134 -
                        P(3, 14) * tmp243 + P(5, 14);
  const double tmp246 = -P(0, 15) * tmp242 - P(1, 15) * tmp244 -
                        P(13, 15) * tmp133 + P(14, 15) * tmp72 +
                        2 * P(15, 15) * tmp82 - P(2, 15) * tmp134 -
                        P(3, 15) * tmp243 + P(5, 15);
  const double tmp247 = -P(0, 13) * tmp242 - P(1, 13) * tmp244 -
                        P(13, 13) * tmp133 + P(13, 14) * tmp72 +
                        2 * P(13, 15) * tmp82 - P(2, 13) * tmp134 -
                        P(3, 13) * tmp243 + P(5, 13);
  const double tmp248 = -P(0, 2) * tmp242 - P(1, 2) * tmp244 -
                        P(2, 13) * tmp133 + P(2, 14) * tmp72 +
                        2 * P(2, 15) * tmp82 - P(2, 2) * tmp134 -
                        P(2, 3) * tmp243 - tmp180;
  const double tmp249 = -P(0, 0) * tmp242 - P(0, 1) * tmp244 -
                        P(0, 13) * tmp133 + P(0, 14) * tmp72 +
                        2 * P(0, 15) * tmp82 - P(0, 2) * tmp134 -
                        P(0, 3) * tmp243 + P(0, 5);
  const double tmp250 = -P(0, 3) * tmp242 - P(1, 3) * tmp244 -
                        P(2, 3) * tmp134 - P(3, 13) * tmp133 +
                        P(3, 14) * tmp72 + 2 * P(3, 15) * tmp82 -
                        P(3, 3) * tmp243 - tmp213;
  const double tmp251 = -P(0, 1) * tmp242 - P(1, 1) * tmp244 -
                        P(1, 13) * tmp133 + P(1, 14) * tmp72 +
                        2 * P(1, 15) * tmp82 - P(1, 2) * tmp134 -
                        P(1, 3) * tmp243 - tmp139;
  const double tmp252 = P(0, 5) * tmp242 + P(1, 5) * tmp244 + P(2, 5) * tmp134 +
                        P(3, 5) * tmp243 + P(5, 13) * tmp133 -
                        P(5, 14) * tmp72 - 2 * P(5, 15) * tmp82 - P(5, 5);
  const double tmp253 = 2 * tmp86;
  const double tmp254 = 2 * tmp87;
  const double tmp255 = 2 * tmp88;
  const double tmp256 = -P(5, 6);
  const double tmp257 = P(0, 6) * tmp242 + P(1, 6) * tmp244 + P(2, 6) * tmp134 +
                        P(3, 6) * tmp243 + P(6, 13) * tmp133 -
                        P(6, 14) * tmp72 - 2 * P(6, 15) * tmp82 + tmp256;
  const double tmp258 = -P(0, 15) * tmp254 - P(1, 15) * tmp255 +
                        2 * P(13, 15) * tmp90 - P(14, 15) * tmp238 +
                        P(15, 15) * tmp84 - P(2, 15) * tmp239 -
                        P(3, 15) * tmp253 + P(6, 15);
  const double tmp259 = -P(0, 13) * tmp254 - P(1, 13) * tmp255 +
                        2 * P(13, 13) * tmp90 - P(13, 14) * tmp238 +
                        P(13, 15) * tmp84 - P(2, 13) * tmp239 -
                        P(3, 13) * tmp253 + P(6, 13);
  const double tmp260 = -P(0, 14) * tmp254 - P(1, 14) * tmp255 +
                        2 * P(13, 14) * tmp90 - P(14, 14) * tmp238 +
                        P(14, 15) * tmp84 - P(2, 14) * tmp239 -
                        P(3, 14) * tmp253 + P(6, 14);
  const double tmp261 = P(0, 6) * tmp254 + P(1, 6) * tmp255 + P(2, 6) * tmp239 +
                        P(3, 6) * tmp253 - 2 * P(6, 13) * tmp90 +
                        P(6, 14) * tmp238 - P(6, 15) * tmp84 - P(6, 6);
  P_new(0, 0) = tmp0 * tmp1 + tmp11 * tmp22 + tmp13 * tmp24 + tmp15 * tmp26 +
                tmp16 * tmp6 + tmp18 * tmp7 + tmp2 * tmp3 + tmp20 * tmp8 +
                tmp30 + tmp4 * tmp5;
  P_new(0, 1) = -tmp1 * tmp33 - tmp11 * tmp30 + tmp13 * tmp26 - tmp15 * tmp24 -
                tmp16 * tmp31 + tmp18 * tmp8 - tmp20 * tmp7 + tmp22 +
                tmp3 * tmp32 - tmp32 * tmp5;
  P_new(1, 1) = 0.25 * delta_angle_x_var * tmp105 +
                0.25 * delta_angle_y_var * tmp4 +
                0.25 * delta_angle_z_var * tmp2 + 0.5 * qz * tmp107 -
                tmp100 * tmp11 - tmp104 * tmp15 + 0.5 * tmp111 * tmp12 -
                tmp114 - tmp31 * tmp94 - tmp7 * tmp96;
  P_new(0, 2) = -tmp1 * tmp34 - tmp11 * tmp26 - tmp13 * tmp30 + tmp15 * tmp22 -
                tmp16 * tmp8 - tmp18 * tmp31 + tmp20 * tmp6 + tmp24 -
                tmp3 * tmp35 + tmp34 * tmp5;
  P_new(1, 2) = 0.25 * delta_angle_x_var * qw * qz + 0.5 * qx * tmp96 -
                tmp100 * tmp13 - tmp103 - tmp107 * tmp31 - tmp11 * tmp111 +
                0.5 * tmp115 * tmp14 - tmp3 * tmp37 - tmp36 * tmp5 -
                tmp8 * tmp94;
  P_new(2, 2) = 0.25 * delta_angle_x_var * tmp4 +
                0.25 * delta_angle_y_var * tmp105 +
                0.25 * delta_angle_z_var * tmp0 + 0.5 * qx * tmp158 -
                tmp11 * tmp153 - tmp13 * tmp156 + 0.5 * tmp14 * tmp161 -
                tmp149 * tmp31 - tmp150 * tmp8 - tmp163;
  P_new(0, 3) = tmp1 * tmp36 + tmp11 * tmp24 - tmp13 * tmp22 - tmp15 * tmp30 +
                tmp16 * tmp7 - tmp18 * tmp6 - tmp20 * tmp31 + tmp26 -
                tmp3 * tmp36 - tmp37 * tmp5;
  P_new(1, 3) = 0.25 * delta_angle_z_var * qw * qy + 0.5 * qy * tmp94 -
                tmp1 * tmp35 + 0.5 * tmp10 * tmp104 - tmp100 * tmp15 -
                tmp107 * tmp6 - tmp110 - tmp115 * tmp13 - tmp3 * tmp34 -
                tmp31 * tmp96;
  P_new(2, 3) = 0.25 * delta_angle_y_var * qw * qx + 0.5 * qy * tmp150 -
                tmp1 * tmp32 + 0.5 * tmp10 * tmp164 - tmp13 * tmp161 -
                tmp149 * tmp6 - tmp15 * tmp156 - tmp152 - tmp158 * tmp31 -
                tmp33 * tmp5;
  P_new(3, 3) = 0.25 * delta_angle_x_var * tmp2 +
                0.25 * delta_angle_y_var * tmp0 +
                0.25 * delta_angle_z_var * tmp105 + 0.5 * qy * tmp194 +
                0.5 * tmp10 * tmp196 - tmp13 * tmp191 - tmp15 * tmp193 -
                tmp188 * tmp31 - tmp189 * tmp6 - tmp197;
  P_new(0, 4) = tmp40 * tmp41 - tmp42 * tmp44 - tmp49 * tmp50 - tmp54 * tmp55 -
                tmp60 * tmp61 - tmp65 * tmp66 + tmp67 * tmp69 + tmp70;
  P_new(1, 4) = P(1, 4) + P(3, 4) * tmp13 + P(4, 11) * tmp8 - tmp116 - tmp117 -
                tmp118 - tmp119 + tmp120 * tmp121 - tmp123 * tmp67 -
                tmp124 * tmp40 + tmp125 * tmp49 + tmp126 * tmp54 +
                tmp127 * tmp60 + tmp128 * tmp65;
  P_new(2, 4) = P(1, 4) * tmp15 + P(2, 4) + P(4, 12) * tmp6 + tmp121 * tmp169 -
                tmp165 - tmp166 - tmp167 - tmp168 - tmp171 * tmp67 -
                tmp172 * tmp40 + tmp173 * tmp49 + tmp174 * tmp54 +
                tmp175 * tmp60 + tmp176 * tmp65;
  P_new(3, 4) = P(2, 4) * tmp11 + P(3, 4) + P(4, 10) * tmp7 + tmp121 * tmp202 -
                tmp198 - tmp199 - tmp200 - tmp201 - tmp204 * tmp67 -
                tmp205 * tmp40 + tmp206 * tmp49 + tmp207 * tmp54 +
                tmp208 * tmp60 + tmp209 * tmp65;
  P_new(4, 4) = delta_velocity_x_var * pow(tmp40, 2) +
                4 * delta_velocity_y_var * pow(tmp67, 2) +
                4 * delta_velocity_z_var * pow(tmp42, 2) - tmp121 * tmp227 -
                tmp221 * tmp228 - tmp222 * tmp229 - tmp223 * tmp230 -
                tmp224 * tmp231 + tmp225 * tmp40 + 2 * tmp226 * tmp67 - tmp232;
  P_new(0, 5) = tmp44 * tmp82 - tmp50 * tmp81 - tmp55 * tmp77 - tmp61 * tmp76 -
                tmp66 * tmp79 + tmp68 * tmp72 - tmp73 * tmp74 + tmp83;
  P_new(1, 5) = -tmp100 * tmp137 - tmp104 * tmp134 + 2 * tmp111 * tmp131 -
                tmp115 * tmp138 + tmp129 * tmp72 + 2 * tmp130 * tmp82 -
                tmp132 * tmp133 - tmp140;
  P_new(2, 5) = 2 * tmp131 * tmp153 - tmp133 * tmp179 - tmp134 * tmp164 -
                tmp137 * tmp156 - tmp138 * tmp161 + tmp177 * tmp72 +
                2 * tmp178 * tmp82 - tmp181;
  P_new(3, 5) = -2 * tmp131 * tmp197 - tmp133 * tmp212 - tmp134 * tmp196 -
                tmp137 * tmp193 - tmp138 * tmp191 + tmp210 * tmp72 +
                2 * tmp211 * tmp82 - tmp214;
  P_new(4, 5) = -delta_velocity_x_var * tmp133 * tmp40 +
                2 * delta_velocity_y_var * tmp67 * tmp72 -
                4 * delta_velocity_z_var * tmp42 * tmp82 - tmp133 * tmp225 -
                tmp134 * tmp230 + tmp226 * tmp72 + 2 * tmp227 * tmp82 -
                tmp233 * tmp77 - tmp234 * tmp79 - tmp235 * tmp81 - tmp237;
  P_new(5, 5) = 4 * delta_velocity_x_var * pow(tmp73, 2) +
                delta_velocity_y_var * pow(tmp72, 2) +
                4 * delta_velocity_z_var * pow(tmp82, 2) - tmp133 * tmp247 -
                tmp134 * tmp248 - tmp242 * tmp249 - tmp243 * tmp250 -
                tmp244 * tmp251 + tmp245 * tmp72 + 2 * tmp246 * tmp82 - tmp252;
  P_new(0, 6) = tmp43 * tmp84 - tmp50 * tmp88 - tmp55 * tmp87 - tmp61 * tmp89 -
                tmp66 * tmp86 - tmp69 * tmp85 + tmp74 * tmp90 + tmp91;
  P_new(1, 6) = P(1, 6) + P(3, 6) * tmp13 + P(6, 11) * tmp8 - tmp120 * tmp84 +
                tmp123 * tmp85 - tmp124 * tmp145 + tmp125 * tmp88 +
                tmp126 * tmp87 + tmp127 * tmp89 + tmp128 * tmp86 - tmp141 -
                tmp142 - tmp143 - tmp144;
  P_new(2, 6) = P(1, 6) * tmp15 + P(2, 6) + P(6, 12) * tmp6 - tmp145 * tmp172 -
                tmp169 * tmp84 + tmp171 * tmp85 + tmp173 * tmp88 +
                tmp174 * tmp87 + tmp175 * tmp89 + tmp176 * tmp86 - tmp182 -
                tmp183 - tmp184 - tmp185;
  P_new(3, 6) = P(2, 6) * tmp11 + P(3, 6) + P(6, 10) * tmp7 - tmp145 * tmp205 -
                tmp202 * tmp84 + tmp204 * tmp85 + tmp206 * tmp88 +
                tmp207 * tmp87 + tmp208 * tmp89 + tmp209 * tmp86 - tmp215 -
                tmp216 - tmp217 - tmp218;
  P_new(4, 6) = 2 * delta_velocity_x_var * tmp40 * tmp90 -
                4 * delta_velocity_y_var * tmp67 * tmp85 -
                delta_velocity_z_var * tmp121 * tmp84 + 2 * tmp225 * tmp90 -
                tmp226 * tmp238 + tmp227 * tmp84 - tmp230 * tmp239 -
                tmp233 * tmp87 - tmp234 * tmp86 - tmp235 * tmp88 - tmp241;
  P_new(5, 6) = -4 * delta_velocity_x_var * tmp73 * tmp90 -
                delta_velocity_y_var * tmp238 * tmp72 +
                2 * delta_velocity_z_var * tmp82 * tmp84 - tmp238 * tmp245 -
                tmp239 * tmp248 + tmp246 * tmp84 + 2 * tmp247 * tmp90 -
                tmp249 * tmp254 - tmp250 * tmp253 - tmp251 * tmp255 - tmp257;
  P_new(6, 6) =
      4 * delta_velocity_x_var * pow(tmp90, 2) +
      4 * delta_velocity_y_var * pow(tmp85, 2) +
      delta_velocity_z_var * pow(tmp84, 2) - tmp238 * tmp260 -
      tmp239 * (-P(0, 2) * tmp254 - P(1, 2) * tmp255 + 2 * P(2, 13) * tmp90 -
                P(2, 14) * tmp238 + P(2, 15) * tmp84 - P(2, 2) * tmp239 -
                P(2, 3) * tmp253 - tmp187) -
      tmp253 * (-P(0, 3) * tmp254 - P(1, 3) * tmp255 - P(2, 3) * tmp239 +
                2 * P(3, 13) * tmp90 - P(3, 14) * tmp238 + P(3, 15) * tmp84 -
                P(3, 3) * tmp253 - tmp220) -
      tmp254 * (-P(0, 0) * tmp254 - P(0, 1) * tmp255 + 2 * P(0, 13) * tmp90 -
                P(0, 14) * tmp238 + P(0, 15) * tmp84 - P(0, 2) * tmp239 -
                P(0, 3) * tmp253 + P(0, 6)) -
      tmp255 * (-P(0, 1) * tmp254 - P(1, 1) * tmp255 + 2 * P(1, 13) * tmp90 -
                P(1, 14) * tmp238 + P(1, 15) * tmp84 - P(1, 2) * tmp239 -
                P(1, 3) * tmp253 - tmp147) +
      tmp258 * tmp84 + 2 * tmp259 * tmp90 - tmp261;
  P_new(0, 7) = P(0, 7) + P(1, 7) * tmp11 + P(2, 7) * tmp13 + P(3, 7) * tmp15 +
                P(7, 10) * tmp6 + P(7, 11) * tmp7 + P(7, 12) * tmp8 +
                dt * tmp70;
  P_new(1, 7) = -P(0, 7) * tmp11 + P(1, 7) - P(2, 7) * tmp15 +
                0.5 * P(3, 7) * tmp12 - P(7, 10) * tmp31 + 0.5 * P(7, 11) * qz -
                P(7, 12) * tmp7 +
                dt * (0.5 * P(3, 4) * tmp12 + 0.5 * P(4, 11) * qz - tmp116 -
                      tmp117 - tmp118 - tmp119 - tmp146);
  P_new(2, 7) = -P(0, 7) * tmp13 + 0.5 * P(1, 7) * tmp14 + P(2, 7) -
                P(3, 7) * tmp11 - P(7, 10) * tmp8 - P(7, 11) * tmp31 +
                0.5 * P(7, 12) * qx +
                dt * (0.5 * P(1, 4) * tmp14 + 0.5 * P(4, 12) * qx - tmp165 -
                      tmp166 - tmp167 - tmp168 - tmp186);
  P_new(3, 7) = -P(0, 7) * tmp15 - P(1, 7) * tmp13 + 0.5 * P(2, 7) * tmp10 +
                P(3, 7) + 0.5 * P(7, 10) * qy - P(7, 11) * tmp6 -
                P(7, 12) * tmp31 +
                dt * (0.5 * P(2, 4) * tmp10 + 0.5 * P(4, 10) * qy - tmp198 -
                      tmp199 - tmp200 - tmp201 - tmp219);
  P_new(4, 7) = -P(0, 7) * tmp222 - P(1, 7) * tmp221 - P(2, 7) * tmp223 -
                P(3, 7) * tmp224 + P(4, 7) + P(7, 13) * tmp40 +
                2 * P(7, 14) * tmp67 - P(7, 15) * tmp121 - dt * tmp232;
  P_new(5, 7) = -P(0, 7) * tmp242 - P(1, 7) * tmp244 - P(2, 7) * tmp134 -
                P(3, 7) * tmp243 + P(5, 7) - P(7, 13) * tmp133 +
                P(7, 14) * tmp72 + 2 * P(7, 15) * tmp82 +
                dt * (-P(0, 4) * tmp242 - P(1, 4) * tmp244 - P(2, 4) * tmp134 -
                      P(3, 4) * tmp243 - P(4, 13) * tmp133 + P(4, 14) * tmp72 +
                      2 * P(4, 15) * tmp82 - tmp236);
  P_new(6, 7) = -P(0, 7) * tmp254 - P(1, 7) * tmp255 - P(2, 7) * tmp239 -
                P(3, 7) * tmp253 + P(6, 7) + 2 * P(7, 13) * tmp90 -
                P(7, 14) * tmp238 + P(7, 15) * tmp84 +
                dt * (-P(0, 4) * tmp254 - P(1, 4) * tmp255 - P(2, 4) * tmp239 -
                      P(3, 4) * tmp253 + 2 * P(4, 13) * tmp90 -
                      P(4, 14) * tmp238 + P(4, 15) * tmp84 - tmp240);
  P_new(7, 7) = P(4, 7) * dt + P(7, 7) + dt * (P(4, 4) * dt + P(4, 7));
  P_new(0, 8) = P(0, 8) + P(1, 8) * tmp11 + P(2, 8) * tmp13 + P(3, 8) * tmp15 +
                P(8, 10) * tmp6 + P(8, 11) * tmp7 + P(8, 12) * tmp8 +
                dt * tmp83;
  P_new(1, 8) = -P(0, 8) * tmp11 + P(1, 8) - P(2, 8) * tmp15 +
                0.5 * P(3, 8) * tmp12 - P(8, 10) * tmp31 + 0.5 * P(8, 11) * qz -
                P(8, 12) * tmp7 - dt * tmp140;
  P_new(2, 8) = -P(0, 8) * tmp13 + 0.5 * P(1, 8) * tmp14 + P(2, 8) -
                P(3, 8) * tmp11 - P(8, 10) * tmp8 - P(8, 11) * tmp31 +
                0.5 * P(8, 12) * qx - dt * tmp181;
  P_new(3, 8) = -P(0, 8) * tmp15 - P(1, 8) * tmp13 + 0.5 * P(2, 8) * tmp10 +
                P(3, 8) + 0.5 * P(8, 10) * qy - P(8, 11) * tmp6 -
                P(8, 12) * tmp31 - dt * tmp214;
  P_new(4, 8) = -P(0, 8) * tmp222 - P(1, 8) * tmp221 - P(2, 8) * tmp223 -
                P(3, 8) * tmp224 + P(4, 8) + P(8, 13) * tmp40 +
                2 * P(8, 14) * tmp67 - P(8, 15) * tmp121 - dt * tmp237;
  P_new(5, 8) = -P(0, 8) * tmp242 - P(1, 8) * tmp244 - P(2, 8) * tmp134 -
                P(3, 8) * tmp243 + P(5, 8) - P(8, 13) * tmp133 +
                P(8, 14) * tmp72 + 2 * P(8, 15) * tmp82 - dt * tmp252;
  P_new(6, 8) = -P(0, 8) * tmp254 - P(1, 8) * tmp255 - P(2, 8) * tmp239 -
                P(3, 8) * tmp253 + P(6, 8) + 2 * P(8, 13) * tmp90 -
                P(8, 14) * tmp238 + P(8, 15) * tmp84 +
                dt * (-P(0, 5) * tmp254 - P(1, 5) * tmp255 - P(2, 5) * tmp239 -
                      P(3, 5) * tmp253 + 2 * P(5, 13) * tmp90 -
                      P(5, 14) * tmp238 + P(5, 15) * tmp84 - tmp256);
  P_new(7, 8) = P(4, 8) * dt + P(7, 8) + dt * (P(4, 5) * dt + P(5, 7));
  P_new(8, 8) = P(5, 8) * dt + P(8, 8) + dt * (P(5, 5) * dt + P(5, 8));
  P_new(0, 9) = P(0, 9) + P(1, 9) * tmp11 + P(2, 9) * tmp13 + P(3, 9) * tmp15 +
                P(9, 10) * tmp6 + P(9, 11) * tmp7 + P(9, 12) * tmp8 +
                dt * tmp91;
  P_new(1, 9) = -P(0, 9) * tmp11 + P(1, 9) - P(2, 9) * tmp15 +
                0.5 * P(3, 9) * tmp12 - P(9, 10) * tmp31 + 0.5 * P(9, 11) * qz -
                P(9, 12) * tmp7 +
                dt * (0.5 * P(3, 6) * tmp12 + 0.5 * P(6, 11) * qz - tmp141 -
                      tmp142 - tmp143 - tmp144 - tmp147);
  P_new(2, 9) = -P(0, 9) * tmp13 + 0.5 * P(1, 9) * tmp14 + P(2, 9) -
                P(3, 9) * tmp11 - P(9, 10) * tmp8 - P(9, 11) * tmp31 +
                0.5 * P(9, 12) * qx +
                dt * (0.5 * P(1, 6) * tmp14 + 0.5 * P(6, 12) * qx - tmp182 -
                      tmp183 - tmp184 - tmp185 - tmp187);
  P_new(3, 9) = -P(0, 9) * tmp15 - P(1, 9) * tmp13 + 0.5 * P(2, 9) * tmp10 +
                P(3, 9) + 0.5 * P(9, 10) * qy - P(9, 11) * tmp6 -
                P(9, 12) * tmp31 +
                dt * (0.5 * P(2, 6) * tmp10 + 0.5 * P(6, 10) * qy - tmp215 -
                      tmp216 - tmp217 - tmp218 - tmp220);
  P_new(4, 9) = -P(0, 9) * tmp222 - P(1, 9) * tmp221 - P(2, 9) * tmp223 -
                P(3, 9) * tmp224 + P(4, 9) + P(9, 13) * tmp40 +
                2 * P(9, 14) * tmp67 - P(9, 15) * tmp121 - dt * tmp241;
  P_new(5, 9) = -P(0, 9) * tmp242 - P(1, 9) * tmp244 - P(2, 9) * tmp134 -
                P(3, 9) * tmp243 + P(5, 9) - P(9, 13) * tmp133 +
                P(9, 14) * tmp72 + 2 * P(9, 15) * tmp82 - dt * tmp257;
  P_new(6, 9) = -P(0, 9) * tmp254 - P(1, 9) * tmp255 - P(2, 9) * tmp239 -
                P(3, 9) * tmp253 + P(6, 9) + 2 * P(9, 13) * tmp90 -
                P(9, 14) * tmp238 + P(9, 15) * tmp84 - dt * tmp261;
  P_new(7, 9) = P(4, 9) * dt + P(7, 9) + dt * (P(4, 6) * dt + P(6, 7));
  P_new(8, 9) = P(5, 9) * dt + P(8, 9) + dt * (P(5, 6) * dt + P(6, 8));
  P_new(9, 9) = P(6, 9) * dt + P(9, 9) + dt * (P(6, 6) * dt + P(6, 9));
  P_new(0, 10) = tmp16;
  P_new(1, 10) = tmp94;
  P_new(2, 10) = tmp150;
  P_new(3, 10) = tmp194;
  P_new(4, 10) = -P(0, 10) * tmp222 - P(1, 10) * tmp221 + P(10, 13) * tmp40 +
                 2 * P(10, 14) * tmp67 - P(10, 15) * tmp121 -
                 P(2, 10) * tmp223 - P(3, 10) * tmp224 + P(4, 10);
  P_new(5, 10) = -P(0, 10) * tmp242 - P(1, 10) * tmp244 - P(10, 13) * tmp133 +
                 P(10, 14) * tmp72 + 2 * P(10, 15) * tmp82 - P(2, 10) * tmp134 -
                 P(3, 10) * tmp243 + P(5, 10);
  P_new(6, 10) = -P(0, 10) * tmp254 - P(1, 10) * tmp255 +
                 2 * P(10, 13) * tmp90 - P(10, 14) * tmp238 +
                 P(10, 15) * tmp84 - P(2, 10) * tmp239 - P(3, 10) * tmp253 +
                 P(6, 10);
  P_new(7, 10) = P(4, 10) * dt + P(7, 10);
  P_new(8, 10) = P(5, 10) * dt + P(8, 10);
  P_new(9, 10) = P(6, 10) * dt + P(9, 10);
  P_new(10, 10) = P(10, 10);
  P_new(0, 11) = tmp18;
  P_new(1, 11) = tmp107;
  P_new(2, 11) = tmp149;
  P_new(3, 11) = tmp189;
  P_new(4, 11) = -P(0, 11) * tmp222 - P(1, 11) * tmp221 + P(11, 13) * tmp40 +
                 2 * P(11, 14) * tmp67 - P(11, 15) * tmp121 -
                 P(2, 11) * tmp223 - P(3, 11) * tmp224 + P(4, 11);
  P_new(5, 11) = -P(0, 11) * tmp242 - P(1, 11) * tmp244 - P(11, 13) * tmp133 +
                 P(11, 14) * tmp72 + 2 * P(11, 15) * tmp82 - P(2, 11) * tmp134 -
                 P(3, 11) * tmp243 + P(5, 11);
  P_new(6, 11) = -P(0, 11) * tmp254 - P(1, 11) * tmp255 +
                 2 * P(11, 13) * tmp90 - P(11, 14) * tmp238 +
                 P(11, 15) * tmp84 - P(2, 11) * tmp239 - P(3, 11) * tmp253 +
                 P(6, 11);
  P_new(7, 11) = P(4, 11) * dt + P(7, 11);
  P_new(8, 11) = P(5, 11) * dt + P(8, 11);
  P_new(9, 11) = P(6, 11) * dt + P(9, 11);
  P_new(10, 11) = P(10, 11);
  P_new(11, 11) = P(11, 11);
  P_new(0, 12) = tmp20;
  P_new(1, 12) = tmp96;
  P_new(2, 12) = tmp158;
  P_new(3, 12) = tmp188;
  P_new(4, 12) = -P(0, 12) * tmp222 - P(1, 12) * tmp221 + P(12, 13) * tmp40 +
                 2 * P(12, 14) * tmp67 - P(12, 15) * tmp121 -
                 P(2, 12) * tmp223 - P(3, 12) * tmp224 + P(4, 12);
  P_new(5, 12) = -P(0, 12) * tmp242 - P(1, 12) * tmp244 - P(12, 13) * tmp133 +
                 P(12, 14) * tmp72 + 2 * P(12, 15) * tmp82 - P(2, 12) * tmp134 -
                 P(3, 12) * tmp243 + P(5, 12);
  P_new(6, 12) = -P(0, 12) * tmp254 - P(1, 12) * tmp255 +
                 2 * P(12, 13) * tmp90 - P(12, 14) * tmp238 +
                 P(12, 15) * tmp84 - P(2, 12) * tmp239 - P(3, 12) * tmp253 +
                 P(6, 12);
  P_new(7, 12) = P(4, 12) * dt + P(7, 12);
  P_new(8, 12) = P(5, 12) * dt + P(8, 12);
  P_new(9, 12) = P(6, 12) * dt + P(9, 12);
  P_new(10, 12) = P(10, 12);
  P_new(11, 12) = P(11, 12);
  P_new(12, 12) = P(12, 12);

  for (int i = 10; i < 13; ++i) {
    const int index = i - 10;
    P_new(i, i) = kahan_summation(P_new(i, i), process_noise_var(i),
                                  delta_angle_bias_var_accumulated_(index));
  }

  if (!accel_bias_inhibited_[0]) {
    P_new(0, 13) = tmp41;
    P_new(1, 13) = tmp132;
    P_new(2, 13) = tmp179;
    P_new(3, 13) = tmp212;
    P_new(4, 13) = tmp225;
    P_new(5, 13) = tmp247;
    P_new(6, 13) = tmp259;
    P_new(7, 13) = P(4, 13) * dt + P(7, 13);
    P_new(8, 13) = P(5, 13) * dt + P(8, 13);
    P_new(9, 13) = P(6, 13) * dt + P(9, 13);
    P_new(10, 13) = P(10, 13);
    P_new(11, 13) = P(11, 13);
    P_new(12, 13) = P(12, 13);
    P_new(13, 13) = P(13, 13);

    P_new(13, 13) = kahan_summation(P_new(13, 13), process_noise_var(13),
                                    delta_velocity_bias_var_accumulated_(0));
  } else {
    P_new.row(13).setConstant(prev_delta_velocity_bias_var_(0));
    P_new.col(13).setConstant(prev_delta_velocity_bias_var_(0));
    delta_velocity_bias_var_accumulated_(0) = 0.0;
  }
  if (!accel_bias_inhibited_[1]) {
    P_new(0, 14) = tmp68;
    P_new(1, 14) = tmp129;
    P_new(2, 14) = tmp177;
    P_new(3, 14) = tmp210;
    P_new(4, 14) = tmp226;
    P_new(5, 14) = tmp245;
    P_new(6, 14) = tmp260;
    P_new(7, 14) = P(4, 14) * dt + P(7, 14);
    P_new(8, 14) = P(5, 14) * dt + P(8, 14);
    P_new(9, 14) = P(6, 14) * dt + P(9, 14);
    P_new(10, 14) = P(10, 14);
    P_new(11, 14) = P(11, 14);
    P_new(12, 14) = P(12, 14);
    P_new(13, 14) = P(13, 14);
    P_new(14, 14) = P(14, 14);

    P_new(14, 14) = kahan_summation(P_new(14, 14), process_noise_var(14),
                                    delta_velocity_bias_var_accumulated_(1));
  } else {
    P_new.row(14).setConstant(prev_delta_velocity_bias_var_(1));
    P_new.col(14).setConstant(prev_delta_velocity_bias_var_(1));
    delta_velocity_bias_var_accumulated_(1) = 0.0;
  }
  if (!accel_bias_inhibited_[2]) {
    P_new(0, 15) = tmp43;
    P_new(1, 15) = tmp130;
    P_new(2, 15) = tmp178;
    P_new(3, 15) = tmp211;
    P_new(4, 15) = tmp227;
    P_new(5, 15) = tmp246;
    P_new(6, 15) = tmp258;
    P_new(7, 15) = P(4, 15) * dt + P(7, 15);
    P_new(8, 15) = P(5, 15) * dt + P(8, 15);
    P_new(9, 15) = P(6, 15) * dt + P(9, 15);
    P_new(10, 15) = P(10, 15);
    P_new(11, 15) = P(11, 15);
    P_new(12, 15) = P(12, 15);
    P_new(13, 15) = P(13, 15);
    P_new(14, 15) = P(14, 15);
    P_new(15, 15) = P(15, 15);

    P_new(15, 15) = kahan_summation(P_new(15, 15), process_noise_var(15),
                                    delta_velocity_bias_var_accumulated_(2));
  } else {
    P_new.row(15).setConstant(prev_delta_velocity_bias_var_(2));
    P_new.col(15).setConstant(prev_delta_velocity_bias_var_(2));
    delta_velocity_bias_var_accumulated_(2) = 0.0;
  }

  if ((P(7, 7) + P(8, 8)) > 1e4) {
    for (int i = 7; i < 9; ++i) {
      for (int j = 0; j < kNumStates; ++j) {
        P_new(i, j) = P_(i, j);
        P_new(j, i) = P_(j, i);
      }
    }
  }

  for (int i = 0; i < kNumStates; ++i) {
    P_(i, i) = P_new(i, i);
  }
  // TODO: check if fixCovarianceErrors() is necessary
}

void Ekf::CalculateOutputState(const ImuSample &imu_sample) {
  const double dt_scale_correction = imu_dt_average_ / dt_average_;
  const Eigen::Vector3d delta_angle(
      imu_sample.delta_angle - state_.delta_angle_bias * dt_scale_correction +
      delta_angle_correction_);
  const double spin_delta_angle_D =
      delta_angle.dot(Eigen::Vector3d(R_to_earth_now_.row(2)));
  yaw_delta_ef_ += spin_delta_angle_D;

  yaw_rate_lpf_ef_ = 0.95 * yaw_rate_lpf_ef_ +
                     0.05 * spin_delta_angle_D / imu_sample.delta_angle_dt;
  const Eigen::Quaterniond delta_quat(Eigen::AngleAxisd{delta_angle});
  output_new_.time_us = imu_sample.time_us;
  output_new_.orientation = output_new_.orientation * delta_quat;
  output_new_.orientation.normalize();
  R_to_earth_now_ = output_new_.orientation.toRotationMatrix();

  const Eigen::Vector3d delta_velocity_body{imu_sample.delta_velocity -
                                            state_.delta_velocity_bias *
                                                dt_scale_correction};
  Eigen::Vector3d delta_velocity_earth{R_to_earth_now_ * delta_velocity_body};

  // TODO: check sign;
  delta_velocity_earth(2) -= kGravity * imu_sample.delta_velocity_dt;

  if (imu_sample.delta_velocity_dt > 1e-4) {
    velocity_derivative_ =
        delta_velocity_earth * (1.0 / imu_sample.delta_velocity_dt);
  }
  const Eigen::Vector3d velocity_last(output_new_.velocity);
  output_new_.velocity += delta_velocity_earth;
  // trapezoidal integration
  const Eigen::Vector3d delta_position =
      (output_new_.velocity + velocity_last) *
      (0.5 * imu_sample.delta_velocity_dt);
  output_new_.position += delta_position;

  if (imu_sample.delta_angle_dt > 1e-4) {
    const Eigen::Vector3d angular_rate =
        imu_sample.delta_angle * (1.0 / imu_sample.delta_angle_dt);
    const Eigen::Vector3d velocity_imu_rel_body =
        angular_rate.cross(settings_.imu_position_body);
    velocity_rel_imu_body_enu_ = R_to_earth_now_ * velocity_imu_rel_body;
  }

  if (imu_updated_) {
    output_buffer_.Push(output_new_);
    const OutputSample &output_delayed = output_buffer_.Oldest();
    const Eigen::Quaterniond q_error(
        (state_.orientation.inverse() * output_delayed.orientation)
            .normalized());
    const double scalar = (q_error.w() >= 0.0) ? -2.0 : 2.0;
    const Eigen::Vector3d delta_angle_error{
        scalar * q_error.x(), scalar * q_error.y(), scalar * q_error.z()};
    const double time_delay =
        std::max((imu_sample.time_us - imu_sample_delayed_.time_us) * 1e-6,
                 imu_dt_average_);
    const double attitude_gain = 0.5 * imu_dt_average_ / time_delay;

    delta_angle_correction_ = delta_angle_error * attitude_gain;
    output_tracking_error_(0) = delta_angle_error.norm();
    const double velocity_gain =
        dt_average_ /
        clip<double>(settings_.velocity_time_constant, dt_average_, 10.0);
    const double position_gain =
        dt_average_ /
        clip<double>(settings_.position_time_constant, dt_average_, 10.0);

    const Eigen::Vector3d velocity_error(state_.velocity -
                                         output_delayed.velocity);
    const Eigen::Vector3d position_error(state_.position -
                                         output_delayed.position);
    output_tracking_error_(1) = velocity_error.norm();
    output_tracking_error_(2) = position_error.norm();

    velocity_error_integral_ += velocity_error;
    const Eigen::Vector3d velocity_correction =
        velocity_error * velocity_gain +
        velocity_error_integral_ * square(velocity_gain) * 0.1;

    position_error_integral_ += position_error;
    const Eigen::Vector3d position_correction =
        position_error * position_gain +
        position_error_integral_ * square(position_gain) * 0.1;
    CorrectOutputBuffer(velocity_correction, position_correction);
  }
}

void Ekf::CorrectOutputBuffer(const Eigen::Vector3d &velocity_correction,
                              const Eigen::Vector3d &position_correction) {
  for (int index = 0; index < output_buffer_.Length(); ++index) {
    output_buffer_[index].velocity += velocity_correction;
    output_buffer_[index].position += position_correction;
  }
  output_new_ = output_buffer_.Newest();
}

void Ekf::Fuse(const StateVectord &K, double innovation) {
  Eigen::Vector4d tmp = K.block<4, 1>(0, 0) * innovation;
  state_.orientation.w() = tmp(0);
  state_.orientation.x() = tmp(1);
  state_.orientation.y() = tmp(2);
  state_.orientation.z() = tmp(3);
  state_.orientation.normalize();
  state_.velocity -= K.block<3, 1>(4, 0) * innovation;
  state_.position -= K.block<3, 1>(7, 0) * innovation;
  state_.delta_angle_bias -= K.block<3, 1>(10, 0) * innovation;
  state_.delta_velocity_bias -= K.block<3, 1>(13, 0) * innovation;
}

void Ekf::FuseHeading() {
  double yaw_var;
  double measured_heading;

  if (control_status_.flags.vision_yaw) {
    yaw_var = vision_sample_delayed_.angular_variance;
  } else {
    yaw_var = 0.01;
  }

  R_to_earth_ = state_.orientation.matrix();
  if (ShouldUse321RotationSequence(R_to_earth_)) {
    const double predicted_heading = Euler321Yaw(R_to_earth_);
    if (control_status_.flags.vision_yaw) {
      measured_heading = Euler321Yaw(vision_sample_delayed_.orientation);
    } else {
      measured_heading = predicted_heading;
    }

    bool fuse_zero_innovation = false;
    last_static_yaw_ = predicted_heading;
    FuseYaw321(measured_heading, yaw_var, fuse_zero_innovation);
  } else {
    const double predicted_heading = Euler312Yaw(R_to_earth_);
    if (control_status_.flags.vision_yaw) {
      measured_heading = Euler312Yaw(vision_sample_delayed_.orientation);
    } else {
      measured_heading = predicted_heading;
    }

    bool fuse_zero_innovation = false;
    last_static_yaw_ = predicted_heading;
    FuseYaw312(measured_heading, yaw_var, fuse_zero_innovation);
  }
}

// copy of
// https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/mag_fusion.cpp#L420
void Ekf::FuseYaw321(double yaw, double yaw_var, bool zero_innovation) {
  // assign intermediate state variables
  const double q0 = state_.orientation.w();
  const double q1 = state_.orientation.x();
  const double q2 = state_.orientation.y();
  const double q3 = state_.orientation.z();

  const double R_YAW = std::max(yaw_var, 1.0e-4);
  const double measurement = wrap_pi(yaw);

  // calculate 321 yaw observation matrix
  // choose A or B computational paths to avoid singularity in derivation at
  // +-90 degrees yaw
  bool canUseA = false;
  const double SA0 = 2 * q3;
  const double SA1 = 2 * q2;
  const double SA2 = SA0 * q0 + SA1 * q1;
  const double SA3 = square(q0) + square(q1) - square(q2) - square(q3);
  double SA4, SA5_inv;
  if (square(SA3) > 1e-6) {
    SA4 = 1.0 / square(SA3);
    SA5_inv = square(SA2) * SA4 + 1;
    canUseA = abs(SA5_inv) > 1e-6;
  }

  bool canUseB = false;
  const double SB0 = 2 * q0;
  const double SB1 = 2 * q1;
  const double SB2 = SB0 * q3 + SB1 * q2;
  const double SB4 = square(q0) + square(q1) - square(q2) - square(q3);
  double SB3, SB5_inv;
  if (square(SB2) > 1e-6) {
    SB3 = 1.0 / square(SB2);
    SB5_inv = SB3 * square(SB4) + 1;
    canUseB = abs(SB5_inv) > 1e-6;
  }

  Eigen::Vector4d H_YAW;

  if (canUseA && (!canUseB || abs(SA5_inv) >= abs(SB5_inv))) {
    const double SA5 = 1.0 / SA5_inv;
    const double SA6 = 1.0 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_YAW(0) = SA5 * (SA0 * SA6 - SA8 * q0);
    H_YAW(1) = SA5 * (SA1 * SA6 - SA8 * q1);
    H_YAW(2) = SA5 * (SA1 * SA7 + SA9 * q1);
    H_YAW(3) = SA5 * (SA0 * SA7 + SA9 * q0);
  } else if (canUseB && (!canUseA || abs(SB5_inv) > abs(SA5_inv))) {
    const double SB5 = 1.0 / SB5_inv;
    const double SB6 = 1.0 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_YAW(0) = -SB5 * (SB0 * SB6 - SB8 * q3);
    H_YAW(1) = -SB5 * (SB1 * SB6 - SB8 * q2);
    H_YAW(2) = -SB5 * (-SB1 * SB7 - SB9 * q2);
    H_YAW(3) = -SB5 * (-SB0 * SB7 - SB9 * q3);
  } else {
    return;
  }

  // calculate the yaw innovation and wrap to the interval between +-pi
  double innovation;
  if (zero_innovation) {
    innovation = 0.0;
  } else {
    innovation =
        wrap_pi(atan2(R_to_earth_(1, 0), R_to_earth_(0, 0)) - measurement);
  }

  // define the innovation gate size
  double innov_gate = std::max(settings_.heading_innovation_gate, 1.0);

  // Update the quaternion states and covariance matrix
  UpdateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

// copy of
// https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/mag_fusion.cpp#L500
void Ekf::FuseYaw312(double yaw, double yaw_var, bool zero_innovation) {
  const double q0 = state_.orientation.w();
  const double q1 = state_.orientation.x();
  const double q2 = state_.orientation.y();
  const double q3 = state_.orientation.z();

  const double R_YAW = std::max<double>(yaw_var, 1.0e-4);
  const double measurement = wrap_pi(yaw);

  // calculate 312 yaw observation matrix
  // choose A or B computational paths to avoid singularity in derivation at
  // +-90 degrees yaw
  bool canUseA = false;
  const double SA0 = 2 * q3;
  const double SA1 = 2 * q2;
  const double SA2 = SA0 * q0 - SA1 * q1;
  const double SA3 = square(q0) - square(q1) + square(q2) - square(q3);
  double SA4, SA5_inv;
  if (square(SA3) > 1e-6) {
    SA4 = 1.0 / square(SA3);
    SA5_inv = square(SA2) * SA4 + 1;
    canUseA = abs(SA5_inv) > 1e-6f;
  }

  bool canUseB = false;
  const double SB0 = 2 * q0;
  const double SB1 = 2 * q1;
  const double SB2 = -SB0 * q3 + SB1 * q2;
  const double SB4 = -square(q0) + square(q1) - square(q2) + square(q3);
  double SB3, SB5_inv;
  if (square(SB2) > 1e-6) {
    SB3 = 1.0 / square(SB2);
    SB5_inv = SB3 * square(SB4) + 1;
    canUseB = abs(SB5_inv) > 1e-6f;
  }

  Eigen::Vector4d H_YAW;

  if (canUseA && (!canUseB || abs(SA5_inv) >= abs(SB5_inv))) {
    const double SA5 = 1.0 / SA5_inv;
    const double SA6 = 1.0 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_YAW(0) = SA5 * (SA0 * SA6 - SA8 * q0);
    H_YAW(1) = SA5 * (-SA1 * SA6 + SA8 * q1);
    H_YAW(2) = SA5 * (-SA1 * SA7 - SA9 * q1);
    H_YAW(3) = SA5 * (SA0 * SA7 + SA9 * q0);
  } else if (canUseB && (!canUseA || abs(SB5_inv) > abs(SA5_inv))) {
    const double SB5 = 1.0 / SB5_inv;
    const double SB6 = 1.0 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_YAW(0) = -SB5 * (-SB0 * SB6 + SB8 * q3);
    H_YAW(1) = -SB5 * (SB1 * SB6 - SB8 * q2);
    H_YAW(2) = -SB5 * (-SB1 * SB7 - SB9 * q2);
    H_YAW(3) = -SB5 * (SB0 * SB7 + SB9 * q3);
  } else {
    return;
  }

  double innovation;
  if (zero_innovation) {
    innovation = 0.0f;
  } else {
    // calculate the the innovation and wrap to the interval between +-pi
    innovation =
        wrap_pi(atan2(-R_to_earth_(0, 1), R_to_earth_(1, 1)) - measurement);
  }

  // define the innovation gate size
  double innov_gate = std::max<double>(settings_.heading_innovation_gate, 1.0);

  // Update the quaternion states and covariance matrix
  UpdateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

void Ekf::UpdateQuaternion(const double innovation, const double variance,
                           const double gate_sigma,
                           const Eigen::Vector4d &yaw_jacobian) {
  heading_innovation_ = variance;
  for (int row = 0; row < 4; ++row) {
    double tmp = 0.0;
    for (int col = 0; col < 4; ++col) {
      tmp += P_(row, col) * yaw_jacobian(col);
    }
    heading_innovation_var_ += yaw_jacobian(row) * tmp;
  }
  double heading_innovation_var_inv;

  if (heading_innovation_var_ >= variance) {
    fault_status_.flags.bad_heading = false;
    heading_innovation_var_inv = 1.0 / heading_innovation_var_;
  } else {
    fault_status_.flags.bad_heading = true;
    InitCovariance();
    // TODO: log this!
    return;
  }
  StateVectord Kfusion;
  for (int row = 0; row < 16; ++row) {
    for (int col = 0; col < 4; ++col) {
      Kfusion(row) += P_(row, col) * yaw_jacobian(col);
    }
    Kfusion(row) *= heading_innovation_var_inv;
  }
  yaw_test_ratio_ =
      square(innovation) / (square(gate_sigma) * heading_innovation_var_);
  if (yaw_test_ratio_ > 1.0) {
    innovation_check_status_.flags.reject_yaw = true;
    if (!control_status_.flags.in_air &&
        IsTimedOut(time_last_in_air_us_, (uint64_t)5e6)) {
      double gate_limit = sqrt((square(gate_sigma) * heading_innovation_var_));
      heading_innovation_ = clip(innovation, -gate_limit, gate_limit);
      ResetYawGyroBiasCov();
    } else {
      return;
    }
  } else {
    innovation_check_status_.flags.reject_yaw = false;
    heading_innovation_ = innovation;
  }

  StateMatrixd KHP;
  double KH[4];
  for (int row = 0; row < StateIndex::NumStates; row++) {
    KH[0] = Kfusion(row) * yaw_jacobian(0);
    KH[1] = Kfusion(row) * yaw_jacobian(1);
    KH[2] = Kfusion(row) * yaw_jacobian(2);
    KH[3] = Kfusion(row) * yaw_jacobian(3);

    for (int column = 0; column < StateIndex::NumStates; column++) {
      double tmp = KH[0] * P_(0, column);
      tmp += KH[1] * P_(1, column);
      tmp += KH[2] * P_(2, column);
      tmp += KH[3] * P_(3, column);
      KHP(row, column) = tmp;
    }
  }
  const bool healthy = CheckAndFixCovarianceUpdate(KHP);
  fault_status_.flags.bad_heading = !healthy;
  if (healthy) {
    P_ -= KHP;
    FixCovarianceErrors(true);
    Fuse(Kfusion, heading_innovation_);
  }
}

bool Ekf::CheckAndFixCovarianceUpdate(const StateMatrixd &KHP) {
  bool healthy = true;
  for (int i = 0; i < kNumStates; ++i) {
    if (P_(i, i) < KHP(i, i)) {
      healthy = false;
      P_.row(i).setConstant(0.0);
      P_.col(i).setConstant(0.0);
    }
  }
  return healthy;
}

void Ekf::FixCovarianceErrors(bool force_symmetry) {
  double P_lim[5] = {};
  P_lim[0] = 1.0;  // quaternion max var
  P_lim[1] = 1e6;  // velocity max var
  P_lim[2] = 1e6;  // position max var
  P_lim[3] = 1.0;  // gyro max var
  P_lim[4] = 1.0;  // delta velocity z bias max var
  for (int i = 0; i < 4; ++i) {
    P_(i, i) = clip(P_(i, i), 0.0, P_lim[0]);
  }
  for (int i = 4; i < 7; ++i) {
    P_(i, i) = clip(P_(i, i), 1e-6, P_lim[1]);
  }
  for (int i = 7; i < 10; ++i) {
    P_(i, i) = clip(P_(i, i), 1e-6, P_lim[2]);
  }
  for (int i = 10; i < 13; ++i) {
    P_(i, i) = clip(P_(i, i), 0.0, P_lim[3]);
  }
  if (force_symmetry) {
    MakeCovarianceSymmetric<13>(0);
  }

  if (!accel_bias_inhibited_[0] || !accel_bias_inhibited_[1] ||
      !accel_bias_inhibited_[2]) {
    const double min_safe_state_var = 1e-9;
    double max_state_var = min_safe_state_var;
    bool reset_required = false;

    for (int state_index = 13; state_index < 16; ++state_index) {
      if (accel_bias_inhibited_[state_index - 13]) {
        continue;
      }
      if (P_(state_index, state_index) > max_state_var) {
        max_state_var = P_(state_index, state_index);
      } else if (P_(state_index, state_index) < min_safe_state_var) {
        reset_required = true;
      }
    }
    const double min_state_var_target = 5e-8;
    double min_allowed_state_var =
        std::max(0.01 * max_state_var, min_state_var_target);
    for (int state_index = 13; state_index < 16; ++state_index) {
      if (accel_bias_inhibited_[state_index - 13]) {
        continue;
      }
      P_(state_index, state_index) =
          clip(P_(state_index, state_index), min_allowed_state_var,
               square(0.1 * kGravity * dt_average_));
    }
    if (reset_required) {
      for (int i = 13; i < 16; ++i) {
        P_.row(i).setConstant(0.0);
        P_.col(i).setConstant(0.0);
      }
    }
    const double delta_velocity_bias_limit =
        0.9 * settings_.imu_bias_estimation.accel_bias_magnitude_limit *
        dt_average_;
    const double down_delta_velocity_bias =
        state_.delta_velocity_bias.dot(Eigen::Vector3d(R_to_earth_.row(2)));

    bool bad_acceleration_bias =
        (abs(down_delta_velocity_bias) > delta_velocity_bias_limit &&
         ((down_delta_velocity_bias * vision_position_innovation_(2) < 0.0) ||
          (down_delta_velocity_bias * baro_height_innovation_(2) < 0.0 &&
           control_status_.flags.baro_height)));
    if (!bad_acceleration_bias) {
      fault_status_.flags.bad_acceleration_bias = false;
      time_acceleration_bias_check_us_ = time_last_imu_;
    } else {
      fault_status_.flags.bad_acceleration_bias = true;
    }
    if (IsTimedOut(time_acceleration_bias_check_us_, (uint64_t)7e6)) {
      for (int i = 0; i < 3; ++i) {
        P_.row(13 + i).setConstant(0.0);
        P_.col(13 + i).setConstant(0.0);
        time_acceleration_bias_check_us_ = time_last_imu_;
        fault_status_.flags.bad_acceleration_bias = false;
      }
    } else if (force_symmetry) {
      MakeCovarianceSymmetric<3>(13);
    }
  }
}

bool Ekf::FuseHorizontalPosition(const Eigen::Vector3d &innovation,
                                 const Eigen::Vector2d &innovation_gate,
                                 const Eigen::Vector3d &observation_var,
                                 Eigen::Vector3d &innovation_var,
                                 Eigen::Vector2d &test_ratio,
                                 bool inhibit_gate = false) {
  innovation_var(0) = P_(7, 7) + observation_var(0);
  innovation_var(1) = P_(8, 8) + observation_var(1);
  test_ratio(0) = std::max(
      square(innovation(0)) / (square(innovation_gate(0)) * innovation_var(0)),
      square(innovation(1)) / (square(innovation_gate(0)) * innovation_var(1)));

  const bool innovation_check_pass = test_ratio(0) <= 1.0;
  if (innovation_check_pass &&
      test_ratio(0) > square(100.0 / innovation_gate(0))) {
    if (inhibit_gate && test_ratio(0) > square(100.0 / innovation_gate(0))) {
      return false;
    }
    time_last_vision_ = time_last_imu_;
    FuseVelocityPositionHeight(innovation(0), innovation_var(0), 3);
    FuseVelocityPositionHeight(innovation(1), innovation_var(1), 4);
    return true;
  } else {
    innovation_check_status_.flags.reject_horizontal_position = true;
    return false;
  }
}

bool Ekf::FuseVerticalPosition(const Eigen::Vector3d &innovation,
                               const Eigen::Vector2d &innovation_gate,
                               const Eigen::Vector3d &observation_var,
                               Eigen::Vector3d &innovation_var,
                               Eigen::Vector2d &test_ratio) {
  innovation_var(2) = P_(9, 9) + observation_var(2);
  test_ratio(1) =
      square(innovation(2)) / (square(innovation_gate(1)) * innovation_var(2));
  vertical_position_innovation_ratio_ = innovation(2) / sqrt(innovation_var(2));
  vertical_position_fuse_attempt_time_us_ = time_last_imu_;
  bool innovation_check_pass = test_ratio(1) <= 1.0;

  double innovation_tmp;
  // TODO: check for bad vertical acceleration
  // see:
  // https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/vel_pos_fusion.cpp#L160
  innovation_tmp = innovation(2);
  if (innovation_check_pass) {
    time_last_height_fuse_ = time_last_imu_;
    innovation_check_status_.flags.reject_vertical_position = false;
    FuseVelocityPositionHeight(innovation_tmp, innovation_var(2), 5);
    return true;
  } else {
    innovation_check_status_.flags.reject_vertical_position = true;
    return false;
  }
}

void Ekf::FuseVelocityPositionHeight(const double innovation,
                                     const double innovation_var,
                                     const int observation_index) {
  StateVectord K_fusion;
  const int state_index = observation_index + 4;
  for (int row = 0; row < kNumStates; ++row) {
    K_fusion(row) = P_(row, state_index) / innovation_var;
  }
  StateMatrixd KHP;
  for (int row = 0; row < kNumStates; ++row) {
    for (int col = 0; col < kNumStates; ++col) {
      KHP(row, col) = K_fusion(row) * P_(state_index, col);
    }
  }
  bool healthy = true;
  for (int i = 0; i < kNumStates; ++i) {
    if (P_(i, i) < KHP(i, i)) {
      P_.row(i).setConstant(0.0);
      P_.col(i).setConstant(0.0);
      healthy = false;
    }
  }
  SetVelocityPositionFaultStatus(observation_index, !healthy);
  if (healthy) {
    P_ -= KHP;
    FixCovarianceErrors(true);
    Fuse(K_fusion, innovation);
  }
}

void Ekf::SetVelocityPositionFaultStatus(const int index, const bool is_bad) {
  switch (index) {
    case 0:
      fault_status_.flags.bad_velocity_x = is_bad;
      break;
    case 1:
      fault_status_.flags.bad_velocity_y = is_bad;
      break;
    case 2:
      fault_status_.flags.bad_velocity_z = is_bad;
      break;
    case 3:
      fault_status_.flags.bad_position_x = is_bad;
      break;
    case 4:
      fault_status_.flags.bad_position_y = is_bad;
      break;
    case 5:
      fault_status_.flags.bad_position_z = is_bad;
      break;
    default:
      break;
  }
}

void Ekf::UpdateDeadreckoningStatus() {}

// see
// https://github.com/PX4/PX4-Autopilot/blob/8cc6d02af324ba713beb80b374236f70ac7f0a9a/src/modules/ekf2/EKF/ekf_helper.cpp#L1073

void Ekf::UpdateSensorFusion() {
  control_status_prev_.value = control_status_.value;
  if (!control_status_.flags.tilt_align) {
    const Eigen::Vector3d angle_err_vec_var = CalcRotationVectorVariances();

    if ((angle_err_vec_var(0) + angle_err_vec_var(1)) <
        square(3.0 / 180 * kPi)) {
      control_status_.flags.tilt_align = true;
    }
  }

  const BaroSample &baro_init = baro_buffer_.Newest();
  baro_height_faulty_ = !IsRecent(baro_init.time_us, 2 * kBaroMaxIntervalUs);
  delta_time_baro_us_ = baro_sample_delayed_.time_us;
  baro_data_ready_ = baro_buffer_.PopFirstOlderThan(imu_sample_delayed_.time_us,
                                                    &baro_sample_delayed_);

  if (baro_data_ready_) {
    delta_time_baro_us_ = baro_sample_delayed_.time_us - delta_time_baro_us_;
  }
  vision_data_ready_ = vision_buffer_.PopFirstOlderThan(
      imu_sample_delayed_.time_us, &vision_sample_delayed_);

  UpdateHeightSensorTimeout();
  UpdateHeightFusion();
  UpdateVisionFusion();
  UpdateDeadreckoningStatus();
}

void Ekf::UpdateHeightSensorTimeout() {
  CheckVerticalAccelHealth();
  const bool continuous_bad_accel = IsTimedOut(
      time_good_vertical_accel_us_, settings_.bad_accel_reset_delay_us);
  const bool timed_out = IsTimedOut(time_last_height_fuse_, (uint64_t)5e6);
  if (timed_out || continuous_bad_accel) {
    bool request_height_reset = false;

    if (control_status_.flags.baro_height) {
      const bool prev_bad_vertical_accel =
          IsRecent(time_bad_vertical_accel_us_, kBadAccelProbation);
      if (!baro_height_faulty_) {
        request_height_reset = true;
      }
    } else if (control_status_.flags.vision_height) {
      const VisionSample &vision_init = vision_buffer_.Newest();
      const bool vision_data_available =
          IsRecent(vision_init.time_us, 2 * kVisionMaxIntervalUs);
      if (vision_data_available) {
        request_height_reset = true;
      } else if (!baro_height_faulty_) {
        StartBaroHeightFusion();
        request_height_reset = true;
      }
    }

    if (request_height_reset) {
      ResetHeight();
      time_last_height_fuse_ = time_last_imu_;
    }
  }
}

void Ekf::UpdateHeightFusion() {
  bool fuse_height = false;
  switch (settings_.height_mode) {
    case HeightMode::kBaro:
      if (baro_data_ready_ && !baro_height_faulty_) {
        StartBaroHeightFusion();
        fuse_height = true;
      }
      break;
    case HeightMode::kVision:
      if (!control_status_.flags.vision_height &&
          IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        fuse_height = true;
        SetControlVisionHeight();
        // vision height flag is not yet set. Reset the height until the flag is
        // set.
        ResetHeight();
      }
      if (control_status_.flags.vision_height && vision_data_ready_) {
        fuse_height = true;
      } else if (control_status_.flags.baro_height && baro_data_ready_ &&
                 !baro_height_faulty_) {
        fuse_height = true;
      }
      break;
  }
  UpdateBaroHeightOffset();

  if (fuse_height) {
    if (control_status_.flags.baro_height) {
      Eigen::Vector2d baro_innovation_gate;
      Eigen::Vector3d baro_observation_var;
      // TODO: check the signs of the following equation
      baro_height_innovation_(2) =
          state_.position(2) -
          (baro_sample_delayed_.height + baro_height_offset_);
      baro_observation_var(2) =
          square(std::max<double>(settings_.baro_noise, 0.01));
      baro_innovation_gate(1) =
          std::max<double>(settings_.baro_innovation_gate, 1.0);

      FuseVerticalPosition(baro_height_innovation_, baro_innovation_gate,
                           baro_observation_var, baro_height_innovation_var_,
                           baro_height_test_ratio_);
    } else if (control_status_.flags.vision_height) {
      Eigen::Vector2d vision_innovation_gate;
      Eigen::Vector3d vision_observation_var;

      vision_position_innovation_(2) =
          state_.position(2) - vision_sample_delayed_.position(2);
      vision_observation_var(2) = std::max<double>(
          vision_sample_delayed_.position_variance(2), square(0.01));
      vision_innovation_gate(1) =
          std::max<double>(settings_.vision_innovation_gate, 1.0);
      FuseVerticalPosition(vision_position_innovation_, vision_innovation_gate,
                           vision_observation_var,
                           vision_position_innovation_var_,
                           vision_position_test_ratio_);
    }
  }
}

void Ekf::UpdateVisionFusion() {
  if (vision_data_ready_) {
    if (control_status_.flags.tilt_align && control_status_.flags.yaw_align) {
      if (IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        StartVisionPositionFusion();
      }
    }

    if (!control_status_.flags.vision_yaw && control_status_.flags.tilt_align) {
      if (IsRecent(time_last_vision_, 2 * kVisionMaxIntervalUs)) {
        if (ResetYawToVision()) {
          control_status_.flags.yaw_align = true;
          StartVisionYawFusion();
        }
      }
    }

    if (control_status_.flags.vision_position) {
      Eigen::Vector3d vision_observation_var;
      Eigen::Vector2d vision_innovation_gate;

      Eigen::Vector3d position_measurement = vision_sample_delayed_.position;
      Eigen::Matrix3d position_var = Eigen::DiagonalMatrix<double, 3>(
          vision_sample_delayed_.position_variance);
      vision_position_innovation_(0) =
          state_.position(0) - position_measurement(0);
      vision_position_innovation_(1) =
          state_.position(1) - position_measurement(1);
      vision_observation_var(0) =
          std::max<double>(position_var(0, 0), square(0.01));
      vision_observation_var(1) =
          std::max<double>(position_var(1, 1), square(0.01));

      if (IsTimedOut(time_last_horizontal_position_fuse_,
                     settings_.reset_timeout_max_us)) {
        ResetVelocity();
        ResetHorizontalPosition();
      }
      vision_innovation_gate(0) =
          std::max<double>(settings_.vision_innovation_gate, 1.0);
      FuseHorizontalPosition(vision_position_innovation_,
                             vision_innovation_gate, vision_observation_var,
                             vision_position_innovation_var_,
                             vision_position_test_ratio_);
    }
    if (control_status_.flags.vision_yaw) {
      FuseHeading();
    }
  } else if ((control_status_.flags.vision_position ||
              control_status_.flags.vision_yaw) &&
             IsTimedOut(time_last_vision_, settings_.reset_timeout_max_us)) {
    StopVisionFusion();
  }
}
