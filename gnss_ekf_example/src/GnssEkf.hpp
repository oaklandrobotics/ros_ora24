#pragma once

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_tools/conversions.h>

#include "ecef.hpp"

namespace gnss_ekf {

  typedef Eigen::Matrix<double, 5, 1> StateVector;
  typedef Eigen::Matrix<double, 5, 5> StateMatrix;
  enum { POS_X=0, POS_Y, HEADING, SPEED, YAW_RATE };

  class GnssEkf : public rclcpp::Node {
    public:
      GnssEkf(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
      void recvTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
      void recvFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

      // Methods to iterate the Kalman filter
      void updateFilterGPS(const rclcpp::Time& current_time, const Eigen::Vector3d& position);
      void updateFilterTwist(const rclcpp::Time& current_time, const geometry_msgs::msg::Twist& twist);

      // Methods to predict states and propagate uncertainty
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;

      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      Eigen::Vector3d ref_ecef_;
      Eigen::Matrix3d enu_rot_mat_;

      // Estimate state, covariance, and current time stamp
      StateVector X_;
      StateMatrix P_;
      uint64_t estimate_stamp_;

      // Process noise covariance
      StateMatrix Q_;

      // Measurement noise covariances
      Eigen::Matrix2d R_twist_;
      Eigen::Matrix2d R_gps_;
  };

}
