#pragma once
#include <state_estimation/common.h>
#include <state_estimation/ring_buffer.h>

#include <eigen3/Eigen/Dense>

class Interface {
 public:
  void SetImuData(const ImuSample &imu_sample);
  void SetBaroData(const BaroSample &baro_sample);
  void SetVisionData(const VisionSample &vision_sample);
  Eigen::Vector3d Position();

 protected:
  Interface() = default;
  virtual ~Interface() = default;
  virtual bool Init(uint64_t timestamp_us) = 0;
  Settings settings_;
  int imu_buffer_length_{0};
  int observation_buffer_length_{0};
  uint64_t imu_dt_average_{0};

  ImuSample imu_sample_delayed_{};

  BaroSample baro_sample_delayed_{};
  VisionSample vision_sample_delayed_{};

  OutputSample output_new_{};
  ImuSample latest_imu_sample_{};
  Eigen::Vector3d velocity_rel_imu_to_body_enu_;
  Eigen::Vector3d dvelocity_;

  bool imu_updated_{false};
  bool initialized_{false};

  RingBuffer<ImuSample> imu_buffer_{100};
  RingBuffer<OutputSample> output_buffer_{100};
  RingBuffer<BaroSample> baro_buffer_{100};
  RingBuffer<VisionSample> vision_buffer_{100};

  uint64_t time_last_imu_{0};
  uint64_t time_last_baro_{0};
  uint64_t time_last_vision_{0};

  bool InitInterface(uint64_t timestamp_us);

 private:
  uint64_t min_observation_interval_us{0};
  Eigen::Vector3d delta_angle_previous_;
  Eigen::Vector3d delta_velocity_previous_;
  uint64_t baro_timestamp_sum{0};
};
