#pragma once
#include <state_estimation/common.h>
#include <state_estimation/interface.h>
#include <state_estimation/alpha_filter.h>

#include <eigen3/Eigen/Dense>

class Ekf final : public Interface {
 public:
  static constexpr int kNumStates{16};
  typedef Eigen::Matrix<double, kNumStates, 1> StateVectord;
  typedef Eigen::Matrix<double, kNumStates, kNumStates> StateMatrixd;
  Ekf() = default;
  virtual ~Ekf() = default;

  bool Init(uint64_t timestamp_us) override;

  bool Update();
  void PredictState();
  void PredictCovariance();
  void CalculateOutputState(const ImuSample &imu_sample);
  bool InitTilt();
  void InitCovariance();
  void AlignOutputFilter();

 private:
  void Reset();
  bool InitFilter();

  StateSample state_{};
  bool filter_initialized_{false};

  bool baro_data_ready_{false};
  bool vision_data_ready_{false};

  bool is_first_imu_sample_{true};
  int baro_counter_{0};
  AlphaFilter<Eigen::Vector3d> acceleration_filter_{0.1};
  AlphaFilter<Eigen::Vector3d> gyro_filter_{0.1};

  double baro_height_offset_{0.0};

  uint64_t time_deadreckoning_{0};

  StateMatrixd P_;
  Eigen::Vector3d delta_velocity_bias_var_accumulated_;
  Eigen::Vector3d delta_angle_bias_var_accumulated_;

  Eigen::Vector3d vision_position_innovation_;
  Eigen::Vector3d vision_position_innovation_var_;

  Eigen::Vector3d baro_height_innovation_;
  Eigen::Vector3d baro_height_innovation_var_;
};
