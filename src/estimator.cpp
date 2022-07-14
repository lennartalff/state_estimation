#include <state_estimation/common.h>
#include <state_estimation/ekf.h>
#include <state_estimation/estimator.h>
#include <state_estimation/interface.h>
#include <geometry_msgs/msg/quaternion.hpp>
using std::placeholders::_1;

Estimator::Estimator() : Node("estimator_node") {
  imu_time_last_us = (uint64_t)now().nanoseconds() / 1000;
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&Estimator::OnImu, this, _1));
  baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      "pressure", 10, std::bind(&Estimator::OnBaro, this, _1));
  vision_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "vision_pose", 10, std::bind(&Estimator::OnVision, this, _1));
  imu_watchdog_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100),
                           std::bind(&Estimator::OnImuWatchdog, this));
}

void Estimator::OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Reset IMU Watchdog since we obviously got IMU data
  ResetImuWatchdog();
  ImuSample imu_sample_new;
  uint64_t dt_us;
  double dt;

  imu_sample_new.time_us =
      (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() / 1e-3);
  dt_us = clip<uint64_t>(imu_sample_new.time_us, 1000, 100000);
  dt = dt_us * 1e-6;
  imu_time_last_us = imu_sample_new.time_us;
  imu_sample_new.delta_angle_dt = dt;
  imu_sample_new.delta_velocity_dt = dt;
  imu_sample_new.delta_angle =
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z) *
      dt;
  imu_sample_new.delta_velocity =
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                      msg->linear_acceleration.z) *
      dt;

  ekf_.SetImuData(imu_sample_new);
  PublishAttitude();
  BaroUpdate();
  VisionUpdate();
  if (ekf_.Update()) {
    PublishPose();
    PublishSensorBias();
  }
}

void Estimator::OnBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  baro_updated_ = true;
  // TODO: apply transformation from baro frame to body frame
  double pressure_at_bodyframe = msg->fluid_pressure;

  baro_sample_.height =
      -(pressure_at_bodyframe - params_.baro_atmo_pressure) / 1e4 +
      params_.baro_sealevel_offset;
  baro_sample_.time_us =
      (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() * 1e-3);
  // ekf_.SetBaroData(sample);
}

void Estimator::OnVision(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  vision_updated_ = true;
  geometry_msgs::msg::Quaternion &q = msg->pose.pose.orientation;
  geometry_msgs::msg::Point &p = msg->pose.pose.position;
  vision_sample_.time_us = (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() * 1e-3);
  vision_sample_.orientation.w() = q.w;
  vision_sample_.orientation.x() = q.x;
  vision_sample_.orientation.y() = q.y;
  vision_sample_.orientation.z() = q.z;
  vision_sample_.position(0) = p.x;
  vision_sample_.position(1) = p.y;
  vision_sample_.position(2) = p.z;
}

void Estimator::BaroUpdate() {
  if (baro_updated_) {
    baro_updated_ = false;
    ekf_.SetBaroData(baro_sample_);
  }
}

void Estimator::VisionUpdate() {
  if (vision_updated_) {
    vision_updated_ = false;
    ekf_.SetVisionData(vision_sample_);
  }
}

void Estimator::OnImuWatchdog() {
  // TODO: reset the estimator. IMU should never time out.
  imu_timed_out = true;
  RCLCPP_ERROR(
      get_logger(),
      "Imu data timed out! This should never happen during operation.");
}

void Estimator::Run() {}

int main() { return 0; }
