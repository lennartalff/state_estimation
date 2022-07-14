#pragma once
#include <state_estimation/alpha_filter.h>
#include <state_estimation/common.h>
#include <state_estimation/interface.h>

#include <eigen3/Eigen/Dense>

class Ekf final : public Interface {
 public:
  static constexpr int kNumStates{16};
  typedef Eigen::Matrix<double, kNumStates, 1> StateVectord;
  typedef Eigen::Matrix<double, kNumStates, kNumStates> StateMatrixd;
  Ekf() = default;
  virtual ~Ekf() = default;

  bool Init(uint64_t timestamp_us) override;

  void UpdateDeadreckoningStatus();

  bool Update();
  void PredictState();
  void PredictCovariance();
  void CalculateOutputState(const ImuSample &imu_sample);
  void CorrectOutputBuffer(const Eigen::Vector3d &velocity_correction,
                           const Eigen::Vector3d &position_correction);
  void Fuse(const StateVectord &K, double innovation);
  bool FuseHorizontalPosition(const Eigen::Vector3d &innovation,
                              const Eigen::Vector2d &innovation_gate,
                              const Eigen::Vector3d &observation_var,
                              Eigen::Vector3d &innovation_var,
                              Eigen::Vector2d &test_ratio, bool inhibit_gate);
  bool FuseVerticalPosition(const Eigen::Vector3d &innovation,
                            const Eigen::Vector2d &innovation_gate,
                            const Eigen::Vector3d &observation_var,
                            Eigen::Vector3d &innovation_var,
                            Eigen::Vector2d &test_ratio);
  void FuseVelocityPositionHeight(const double innovation,
                                  const double innovation_var,
                                  const int observation_index);
  void FixCovarianceErrors(bool force_symmetry);
  void SetVelocityPositionFaultStatus(const int index, const bool is_bad);
  bool InitTilt();
  void InitCovariance();
  void InitQuaternionCovariances();
  void ResetQuaternionCovariance();
  void SetZeroQuaternionCovariance();
  void AlignOutputFilter();
  void ConstrainStates();
  bool CheckAndFixCovarianceUpdate(const StateMatrixd &KHP);
  void UpdateSensorFusion();
  void UpdateVisionFusion();
  void UpdateHeightFusion();
  bool IsTimedOut(uint64_t timestamp_us, uint64_t timeout_period_us) {
    return (timestamp_us + timeout_period_us) < time_last_imu_;
  }
  template <int Width>
  void MakeCovarianceBlockSymmetric(int first) {
    if (Width > 1) {
      for (int row = first + 1; row < first + Width; ++row) {
        for (int col = first; col < row; ++col) {
          double tmp = (P_(row, col) + P_(col, row)) / 2.0;
          P_(row, col) = P_(col, row) = tmp;
        }
      }
    }
  }

  template <int Width>
  void MakeCovarianceSymmetric(int first) {
    MakeCovarianceBlockSymmetric<Width>(first);
    for (int row = first; row < first + Width; ++row) {
      for (int col = 0; col < first; ++col) {
        double tmp = (P_(row, col) + P_(col, row)) / 2.0;
        P_(row, col) = P_(col, row) = tmp;
      }
      for (int col = first + Width; col < kNumStates; ++col) {
        double tmp = (P_(row, col) + P_(col, row)) / 2.0;
        P_(row, col) = P_(col, row) = tmp;
      }
    }
  }

 private:
  void Reset();
  bool InitFilter();

  StateSample state_{};
  bool filter_initialized_{false};

  bool baro_data_ready_{false};
  bool vision_data_ready_{false};

  Eigen::Matrix3d R_to_earth_;

  uint64_t time_acceleration_bias_check_us_{0};

  double yaw_delta_ef_{0.0};
  double yaw_rate_lpf_ef_{0.0};

  bool is_first_imu_sample_{true};
  int baro_counter_{0};

  AlphaFilter<Eigen::Vector3d> acceleration_filter_{0.1};
  AlphaFilter<Eigen::Vector3d> acceleration_magnitude_filter{0.1};
  AlphaFilter<Eigen::Vector3d> gyro_filter_{0.1};

  bool accel_bias_inhibited_[3]{};
  double gyro_magnitude_filtered_{0.0};
  double accel_magnitude_filtered_{0.0};
  Eigen::Vector3d accel_vec_filtered_;
  Eigen::Vector3d prev_delta_velocity_bias_var_;

  double baro_height_offset_{0.0};
  double dt_average_{kFilterUpdatePeriodUs * 1e-6};

  uint64_t time_deadreckoning_{0};

  Eigen::Vector3d delta_angle_correction_;
  Eigen::Vector3d velocity_error_integral_;
  Eigen::Vector3d position_error_integral_;
  Eigen::Vector3d output_tracking_error_;

  double vertical_position_innovation_ratio_{0.0};
  uint64_t vertical_position_fuse_attempt_time_us_{0};

  StateMatrixd P_;
  Eigen::Vector3d delta_velocity_bias_var_accumulated_;
  Eigen::Vector3d delta_angle_bias_var_accumulated_;

  Eigen::Vector3d vision_position_innovation_;
  Eigen::Vector3d vision_position_innovation_var_;

  Eigen::Vector3d baro_height_innovation_;
  Eigen::Vector3d baro_height_innovation_var_;

  Eigen::Vector3d angular_rate_delayed_raw_;
};
