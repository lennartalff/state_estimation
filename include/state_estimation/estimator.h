#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>

class Estimator final : public rclcpp::Node {
 public:
  Estimator();
  //////////////////////////////////////////////////////////////////////////////
  // message callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void OnBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  void ResetImuWatchdog() {
    imu_watchdog_.reset();
    imu_timed_out = false;
  }
  bool IsImuTimedOut() { return imu_timed_out; }
  void OnImuWatchdog();
  void Run();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::TimerBase::SharedPtr imu_watchdog_;
  bool imu_timed_out{false};
  uint64_t imu_time_last_us;
  Ekf ekf_;
  struct EstimatorParams {
    double baro_atmo_pressure = 101325.0;
    double baro_sealevel_offset = 0.0;
  }params_;
};
