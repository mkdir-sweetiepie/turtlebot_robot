/**
 * @file /include/robot_master/lift_controller.hpp
 *
 * @brief Header for the lift controller.
 *
 * @date May 2025
 **/

#ifndef ROBOT_MASTER_LIFT_CONTROLLER_HPP
#define ROBOT_MASTER_LIFT_CONTROLLER_HPP

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/lift_action.hpp"
#include "std_msgs/msg/float32.hpp"

namespace robot_master {

enum class LiftStatus { UP, DOWN, MOVING };

class LiftController {
 public:
  explicit LiftController(std::shared_ptr<rclcpp::Node> node);
  ~LiftController();

  // Control methods
  void moveUp();
  void moveDown();
  void stop();

  // Status methods
  double getCurrentHeight() const;
  LiftStatus getStatus() const;

 private:
  // Constants
  static constexpr double MAX_HEIGHT = 0.25;  // meters

  // Node pointer
  std::shared_ptr<rclcpp::Node> node_;

  // Action client for lift control
  using LiftAction = robot_msgs::action::LiftAction;
  using LiftGoalHandle = rclcpp_action::ClientGoalHandle<LiftAction>;
  rclcpp_action::Client<LiftAction>::SharedPtr action_client_;

  // Publisher for lift position
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lift_position_pub_;

  // Timer for publishing lift position
  rclcpp::TimerBase::SharedPtr position_timer_;

  // Current status and position
  LiftStatus lift_status_;
  double current_height_;

  // Private methods
  void sendLiftGoal(const LiftAction::Goal& goal);
  void publishLiftPosition();

  // Action client callbacks
  void goalResponseCallback(const LiftGoalHandle::SharedPtr& goal_handle);
  void feedbackCallback(const LiftGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const LiftAction::Feedback> feedback);
  void resultCallback(const LiftGoalHandle::WrappedResult& result);
};

}  // namespace robot_master

#endif  // ROBOT_MASTER_LIFT_CONTROLLER_HPP