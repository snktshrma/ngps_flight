#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>

#include <ap_ukf/node/fusionCore.h>

using MeasurementQueue =
    std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
                        Measurement>;

class FusionRos : public rclcpp::Node {
 public:
  explicit FusionRos(const rclcpp::NodeOptions& options);
  ~FusionRos() = default;
  void reset();

 protected:
  void update();
  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  nav_msgs::msg::Odometry getFilteredOdomMessage();

  void loadParams();
  Eigen::MatrixXd loadSquareMatrixFromParams(const std::vector<double>& data,
                                             int dim);
  void clearMeasurementQueue();

  rclcpp::Time last_expected;

  struct Params {
    double frequency;
    int vps_soft_frames;
    double vps_chi2_threshold;
    Eigen::MatrixXd initialCov;
    Eigen::MatrixXd processNoiseCov;

    Params()
        : initialCov(STATE_SIZE, STATE_SIZE),
          processNoiseCov(STATE_SIZE, STATE_SIZE) {}
  } params_;

  Fusion filter_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vps_sub_;

  std::string base_link_frame_;
  std::string odom_frame_;

  MeasurementQueue measurement_queue_;

  double last_vio_vx_{0.0};
  double last_vio_vy_{0.0};
};
