#include "GnssEkf.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gnss_ekf {

  GnssEkf::GnssEkf(const rclcpp::NodeOptions& options)
  : rclcpp::Node("gnss_ekf", options)
  {
    // Load the reference coordinates and abort if they are not set
    double ref_lat_deg = this->declare_parameter<double>("ref_lat", INFINITY);
    double ref_lon_deg = this->declare_parameter<double>("ref_lon", INFINITY);
    double ref_alt = this->declare_parameter<double>("ref_alt", INFINITY);
    if (!std::isfinite(ref_lat_deg) || !std::isfinite(ref_lon_deg) || !std::isfinite(ref_alt)) {
      RCLCPP_ERROR(this->get_logger(), "Unspecified reference coordinates parameters!");
      exit(1);
    }

    // Load Kalman filter parameters
    double q_pos = this->declare_parameter<double>("q_pos", 0.1);
    double q_heading = this->declare_parameter<double>("q_heading", 0.1);
    double q_speed = this->declare_parameter<double>("q_speed", 0.1);
    double q_yaw_rate = this->declare_parameter<double>("q_yaw_rate", 0.1);
    double r_gps = this->declare_parameter<double>("r_gps", 1.0);
    double r_speed = this->declare_parameter<double>("r_speed", 0.01);
    double r_yaw_rate = this->declare_parameter<double>("r_yaw_rate", 0.01);

    Q_.setZero();
    Q_(POS_X, POS_X) = q_pos * q_pos;
    Q_(POS_Y, POS_Y) = q_pos * q_pos;
    Q_(HEADING, HEADING) = q_heading * q_heading;
    Q_(SPEED, SPEED) = q_speed * q_speed;
    Q_(YAW_RATE, YAW_RATE) = q_yaw_rate * q_yaw_rate;

    R_gps_.setZero();
    R_gps_(0, 0) = r_gps * r_gps;
    R_gps_(1, 1) = r_gps * r_gps;

    R_twist_.setZero();
    R_twist_(0, 0) = r_speed * r_speed;
    R_twist_(1, 1) = r_yaw_rate * r_yaw_rate;

    // Convert reference coordinates to ECEF and store in 'ref_ecef' variable
    ref_ecef_ = computeEcef(ref_lat_deg * M_PI / 180.0, ref_lon_deg * M_PI / 180.0, ref_alt);
    enu_rot_mat_ = computeEnuRotMat(ref_lat_deg * M_PI / 180.0, ref_lon_deg * M_PI / 180.0, ref_alt);

    // Subscribe to input data
    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_fix", 1, std::bind(&GnssEkf::recvFix, this, std::placeholders::_1));
    sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("twist", 1, std::bind(&GnssEkf::recvTwist, this, std::placeholders::_1));

    // Initialize Kalman filter state
    X_.setZero();
    P_.setIdentity();

    // Set up TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  void GnssEkf::recvTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    updateFilterTwist(msg->header.stamp, msg->twist);

    // Identity transform from map to odom to satisfy nav stack
    geometry_msgs::msg::TransformStamped identity_transform;
    identity_transform.header.stamp = rclcpp::Time(estimate_stamp_);
    identity_transform.header.frame_id = "map";
    identity_transform.child_frame_id = "odom";
    identity_transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(identity_transform);

    // TF transform with current EKF estimate
    geometry_msgs::msg::TransformStamped ekf_transform;
    ekf_transform.header.stamp = rclcpp::Time(estimate_stamp_);
    ekf_transform.header.frame_id = "odom";
    ekf_transform.child_frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, X_(HEADING));
    tf2::convert(q, ekf_transform.transform.rotation);
    ekf_transform.transform.translation.x = X_(POS_X);
    ekf_transform.transform.translation.y = X_(POS_Y);
    ekf_transform.transform.translation.z = 0;
    tf_broadcaster_->sendTransform(ekf_transform);
  }

  void GnssEkf::recvFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    Eigen::Vector3d current_position = computeEnu(ref_ecef_, enu_rot_mat_, msg->latitude, msg->longitude, msg->altitude);
    updateFilterGPS(msg->header.stamp, current_position);
  }

  StateVector GnssEkf::statePrediction(double dt, const StateVector& old_state) {
    // Implement state prediction step
    StateVector new_state;
    new_state(POS_X) = old_state(POS_X) + dt * old_state(SPEED) * cos(old_state(HEADING));
    new_state(POS_Y) = old_state(POS_Y) + dt * old_state(SPEED) * sin(old_state(HEADING));
    new_state(HEADING) = old_state(HEADING) + dt * old_state(YAW_RATE);
    new_state(SPEED) = old_state(SPEED);
    new_state(YAW_RATE) = old_state(YAW_RATE);
    return new_state;
  }

  StateMatrix GnssEkf::stateJacobian(double dt, const StateVector& state) {
    double sin_heading = sin(state(HEADING));
    double cos_heading = cos(state(HEADING));

    // Populate state Jacobian with current state values
    StateMatrix A;
    A.row(POS_X) << 1, 0, -dt * state(SPEED) * sin_heading, dt * cos_heading, 0;
    A.row(POS_Y) << 0, 1, dt * state(SPEED) * cos_heading, dt * sin_heading, 0;
    A.row(HEADING) << 0, 0, 1, 0, dt;
    A.row(SPEED) << 0, 0, 0, 1, 0;
    A.row(YAW_RATE) << 0, 0, 0, 0, 1;
    return A;
  }

  StateMatrix GnssEkf::covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov) {
    // Propagate covariance matrix one step
    StateMatrix new_cov;
    new_cov = A * old_cov * A.transpose() + Q;
    return new_cov;
  }

  void GnssEkf::updateFilterGPS(const rclcpp::Time& current_time, const Eigen::Vector3d& position)
  {
    // Initialize state estimate directly if this is the first GPS measurement
    if (!estimate_stamp_) {
      X_ << position.x(), position.y(), 0.0, 0.0, 0.0;
      P_.setIdentity();
      estimate_stamp_ = current_time.nanoseconds();
      return;
    }

    // Compute amount of time to advance the state prediction
    double dt = 1e-9 * (current_time.nanoseconds() - estimate_stamp_);

    // Propagate estimate prediction and store in predicted variables
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Construct C matrix for a GPS update (X and Y position measurements)
    Eigen::Matrix<double, 2, 5> C;
    C.row(0) << 1, 0, 0, 0, 0;
    C.row(1) << 0, 1, 0, 0, 0;

    // Compute expected measurement
    Eigen::Matrix<double, 2, 1> expected_meas;
    expected_meas << predicted_state(POS_X), predicted_state(POS_Y);

    // Put GPS measurements in an Eigen object
    Eigen::Matrix<double, 2, 1> real_meas;
    real_meas << position.x(), position.y();

    // Compute Kalman gain
    Eigen::Matrix<double, 2, 2> S;
    S = C * predicted_cov * C.transpose() + R_gps_;
    Eigen::Matrix<double, 5, 2> K;
    K = predicted_cov * C.transpose() * S.inverse();

    // Update filter estimate based on difference between actual and expected measurements
    X_ = predicted_state + K * (real_meas - expected_meas);

    // Update estimate error covariance using Kalman gain matrix
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

    // Wrap heading estimate into the range -pi to pi
    if (X_(HEADING) > M_PI) {
      X_(HEADING) -= 2 * M_PI;
    } else if (X_(HEADING) < -M_PI) {
      X_(HEADING) += 2 * M_PI;
    }

    // Set estimate time stamp to the measurement's time
    estimate_stamp_ = current_time.nanoseconds();
  }

  void GnssEkf::updateFilterTwist(const rclcpp::Time& current_time, const geometry_msgs::msg::Twist& twist)
  {
    if (!estimate_stamp_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for first GPS fix, ignoring this update");
      return;
    }

    // Compute amount of time to advance the state prediction
    double dt = 1e-9 * (current_time.nanoseconds() - estimate_stamp_);

    // Propagate estimate prediction and store in predicted variables
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Construct C matrix for a twist update (speed and yaw rate measurement)
    Eigen::Matrix<double, 2, 5> C;
    C.row(0) << 0, 0, 0, 1, 0;
    C.row(1) << 0, 0, 0, 0, 1;

    // Compute expected measurement
    Eigen::Matrix<double, 2, 1> expected_meas;
    expected_meas << predicted_state(SPEED), predicted_state(YAW_RATE);

    // Put twist measurements in an Eigen object
    Eigen::Matrix<double, 2, 1> real_meas;
    real_meas << twist.linear.x, twist.angular.z;

    // Compute Kalman gain
    Eigen::Matrix<double, 2, 2> S;
    S = C * predicted_cov * C.transpose() + R_twist_;
    Eigen::Matrix<double, 5, 2> K;
    K = predicted_cov * C.transpose() * S.inverse();

    // Update filter estimate based on difference between actual and expected measurements
    X_ = predicted_state + K * (real_meas - expected_meas);

    // Update estimate error covariance using Kalman gain matrix
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

    // Wrap heading estimate into the range -pi to pi
    if (X_(HEADING) > M_PI) {
      X_(HEADING) -= 2 * M_PI;
    } else if (X_(HEADING) < -M_PI) {
      X_(HEADING) += 2 * M_PI;
    }

    // Set estimate time stamp to the measurement's time
    estimate_stamp_ = current_time.nanoseconds();
  }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_ekf::GnssEkf)
