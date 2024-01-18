#pragma once 
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class Microinterpolator
{
protected:

  trajectory_msgs::msg::JointTrajectory trj_;
  rclcpp::Node node = rclcpp::Node("microinterpolator");

  unsigned int order_;
  bool trj_set_;
public:
  Microinterpolator();
  bool setTrajectory(const trajectory_msgs::msg::JointTrajectory& trj);
  trajectory_msgs::msg::JointTrajectory getTrajectory(){return trj_;}
  /* setSplineOrder:
   * order = 0 positions are continuous
   * order = 1 velocities are continuous
   * order = 2 accelerations are continuous
   * order = 3 jerks are continuous, supposed zero at the waypoints
   * order = 4 snaps are continuous, supposed zero at the waypoints
   */
  void setSplineOrder(const unsigned int& order);
  bool interpolate(const rclcpp::Duration& time, trajectory_msgs::msg::JointTrajectoryPoint& pnt, const double& scaling=1.0);
  rclcpp::Duration trjTime();
};
  