
#ifndef SCALED_FJT_CONTROLLER
#define SCALED_FJT_CONTROLLER

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "scaled_fjt_controller/microinterpolator.h"
#include <std_msgs/msg/int16.hpp>

namespace scaled_fjt_controller
{
class ScaledFjtController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledFjtController() = default;
  ~ScaledFjtController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  
  rclcpp_action::GoalResponse   goal_received_callback (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void                          goal_accepted_callback (std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  trajectory_msgs::msg::JointTrajectory trj_;
  std::shared_ptr<Microinterpolator> microinterpolator_;
  trajectory_msgs::msg::JointTrajectoryPoint current_point_;  

  double speed_ovr_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speed_ovr_sub_;

  void SpeedOvrCb(const std_msgs::msg::Int16 ovr);

protected:
  struct TimeData
  {
    TimeData() : time(rclcpp::Duration::from_seconds(0.0)),scaled_time(rclcpp::Duration::from_seconds(0.0)){}
    rclcpp::Duration time;
    rclcpp::Duration scaled_time;
  };

  ScaledFjtController::TimeData td_;
private:
  bool init_microint_;

};
}  

#endif