#include "ap_ukf/node/fusionRos.h"

#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

FusionRos::FusionRos(const rclcpp::NodeOptions& options)
    : Node("fusion_ros", options),
      filter_(1.0, 2, 0),
      base_link_frame_("base_link"),
      odom_frame_("odom") {
  loadParams();
  filter_.setInitialCovariance(params_.initialCov);
  filter_.setProcessNoise(params_.processNoiseCov);
  filter_.reset();

  try {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/topic/sensor/odom_state", 60);
    vio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/vio", 50,
        std::bind(&FusionRos::vioCallback, this, _1));
    vps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/vps", 50,
        std::bind(&FusionRos::vpsCallback, this, _1));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Unable to create publishers/subscribers: %s",
                 e.what());
  }
  last_expected = this->get_clock()->now();
  const double period_ms = 1000.0 / params_.frequency;
  update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&FusionRos::update, this));
}

void FusionRos::reset() { clearMeasurementQueue(); }

void FusionRos::update() {
  auto last_cycle_duration =
      (this->now().seconds() - last_expected.seconds());
  last_expected = this->now();

  if (last_cycle_duration > 2. / params_.frequency) {
    RCLCPP_WARN(get_logger(),
                "Failed to meet update rate! Last cycle took %.2f seconds",
                last_cycle_duration);
  }

  const rclcpp::Time currentTime = this->now();
  while (!measurement_queue_.empty() && rclcpp::ok()) {
    MeasurementPtr z = measurement_queue_.top();
    if (z->time > currentTime.seconds()) {
      break;
    }
    measurement_queue_.pop();
    filter_.processMeasurement(*(z.get()));
  }

  if (filter_.isInitialized()) {
    nav_msgs::msg::Odometry filtered_state = getFilteredOdomMessage();
    odom_pub_->publish(filtered_state);
  }
}

nav_msgs::msg::Odometry FusionRos::getFilteredOdomMessage() {
  assert(filter_.isInitialized());

  const Eigen::Matrix<double, STATE_SIZE, 1>& state = filter_.getState();
  const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& cov =
      filter_.getCovariance();

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = state(StateX);
  odom.pose.pose.position.y = state(StateY);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;

  odom.twist.twist.linear.x = state(StateVx);
  odom.twist.twist.linear.y = state(StateVy);
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  for (size_t i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = 0.0;
  }
  odom.pose.covariance[0] = cov(0, 0);
  odom.pose.covariance[1] = cov(0, 1);
  odom.pose.covariance[6] = cov(1, 0);
  odom.pose.covariance[7] = cov(1, 1);
  odom.pose.covariance[14] = 1e6;
  odom.pose.covariance[21] = 1e6;
  odom.pose.covariance[28] = 1e6;
  odom.pose.covariance[35] = 1e6;

  for (size_t i = 0; i < 36; ++i) {
    odom.twist.covariance[i] = 0.0;
  }
  odom.twist.covariance[0] = cov(2, 2);
  odom.twist.covariance[1] = cov(2, 3);
  odom.twist.covariance[6] = cov(3, 2);
  odom.twist.covariance[7] = cov(3, 3);

  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_link_frame_;
  return odom;
}

void FusionRos::vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_vio_vx_ = msg->twist.twist.linear.x;
  last_vio_vy_ = msg->twist.twist.linear.y;

  if (!filter_.isInitialized()) {
    return;
  }

  MeasurementPtr z = std::make_shared<Measurement>(VIO_SIZE);
  z->type = MeasurementTypeVio;
  auto& stamp = msg->header.stamp;
  rclcpp::Time time(stamp.sec, stamp.nanosec);
  z->time = time.seconds();

  z->measurement.resize(VIO_SIZE);
  z->measurement(VioX) = msg->pose.pose.position.x;
  z->measurement(VioY) = msg->pose.pose.position.y;
  z->measurement(VioVx) = msg->twist.twist.linear.x;
  z->measurement(VioVy) = msg->twist.twist.linear.y;

  z->covariance.setZero();
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      z->covariance(i, j) = msg->pose.covariance[6 * i + j];
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      z->covariance(i + 2, j + 2) = msg->twist.covariance[6 * i + j];
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (z->covariance(i, i) < 1e-9) {
      z->covariance(i, i) = 1e-2;
    }
  }

  measurement_queue_.push(z);
}

void FusionRos::vpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto& stamp = msg->header.stamp;
  rclcpp::Time t(stamp.sec, stamp.nanosec);
  const double base_time = t.seconds();

  const double tx = msg->pose.pose.position.x;
  const double ty = msg->pose.pose.position.y;

  Eigen::Matrix2d R;
  R(0, 0) = msg->pose.covariance[0];
  R(0, 1) = msg->pose.covariance[1];
  R(1, 0) = msg->pose.covariance[6];
  R(1, 1) = msg->pose.covariance[7];
  if (R(0, 0) < 1e-12) {
    R(0, 0) = 4.0;
  }
  if (R(1, 1) < 1e-12) {
    R(1, 1) = 4.0;
  }

  if (!filter_.isInitialized()) {
    MeasurementPtr b = std::make_shared<Measurement>(STATE_SIZE);
    b->type = MeasurementTypeBootstrap;
    b->time = base_time;
    b->measurement.resize(STATE_SIZE);
    b->measurement << tx, ty, last_vio_vx_, last_vio_vy_;
    b->covariance = params_.initialCov;
    measurement_queue_.push(b);
    return;
  }

  const int N = std::max(1, params_.vps_soft_frames);
  Eigen::Vector2d target(tx, ty);
  Eigen::Vector2d p0(filter_.getState()(StateX), filter_.getState()(StateY));

  Eigen::Matrix2d Rstep = R * static_cast<double>(N);

  if (params_.vps_chi2_threshold > 0.0) {
    Eigen::Matrix2d Pxy = filter_.getCovariance().block<2, 2>(0, 0);
    Eigen::Vector2d innov = target - p0;
    Eigen::Matrix2d S = Pxy + R;
    Eigen::Vector2d y = S.ldlt().solve(innov);
    double d2 = innov.dot(y);
    if (d2 > params_.vps_chi2_threshold) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "VPS chi2 reject: d2=%.3f (target %.3f, %.3f)", d2,
                           tx, ty);
      return;
    }
  }

  for (int k = 1; k <= N; ++k) {
    const double a = static_cast<double>(k) / static_cast<double>(N);
    Eigen::Vector2d zk = p0 + a * (target - p0);

    MeasurementPtr z = std::make_shared<Measurement>(VPS_SIZE);
    z->type = MeasurementTypeVps;
    z->time = base_time + static_cast<double>(k) * 1e-5;
    z->measurement.resize(VPS_SIZE);
    z->measurement(0) = zk(0);
    z->measurement(1) = zk(1);
    z->covariance.resize(2, 2);
    z->covariance = Rstep;
    measurement_queue_.push(z);
  }
}

void FusionRos::loadParams() {
  params_.frequency = get_parameter("frequency").as_double();
  params_.vps_soft_frames = static_cast<int>(get_parameter("vps_soft_frames").as_int());
  params_.vps_chi2_threshold = get_parameter("vps_chi2_threshold").as_double();
  base_link_frame_ = get_parameter("base_link_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();

  std::vector<double> a =
      get_parameter("initial_estimate_covariance").as_double_array();
  std::vector<double> b =
      get_parameter("process_noise_covariance").as_double_array();
  params_.initialCov = loadSquareMatrixFromParams(a, STATE_SIZE);
  params_.processNoiseCov = loadSquareMatrixFromParams(b, STATE_SIZE);
}

Eigen::MatrixXd FusionRos::loadSquareMatrixFromParams(
    const std::vector<double>& data, int dim) {
  Eigen::MatrixXd m(dim, dim);
  size_t count = 0;
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      if (count < data.size()) {
        m(i, j) = data[count];
        ++count;
      } else {
        m(i, j) = 0.0;
      }
    }
  }
  return m;
}

void FusionRos::clearMeasurementQueue() {
  while (!measurement_queue_.empty() && rclcpp::ok()) {
    measurement_queue_.pop();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto nh = std::make_shared<FusionRos>(options);
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
