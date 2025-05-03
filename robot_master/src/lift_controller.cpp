/**
 * @file /src/lift_controller.cpp
 *
 * @brief Implementation for the lift controller.
 *
 * @date May 2025
 **/

#include "../include/robot_master/lift_controller.hpp"

namespace robot_master {

LiftController::LiftController(std::shared_ptr<rclcpp::Node> node) : node_(node), lift_status_(LiftStatus::DOWN), current_height_(0.0) {
  // Initialize the client for LiftAction
  action_client_ = rclcpp_action::create_client<robot_msgs::action::LiftAction>(node_, "lift_control");

  // Initialize the lift position publisher
  lift_position_pub_ = node_->create_publisher<std_msgs::msg::Float32>("lift_position", 10);

  // Start the position update timer (10 Hz)
  position_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LiftController::publishLiftPosition, this));

  RCLCPP_INFO(node_->get_logger(), "Lift controller initialized");
}

LiftController::~LiftController() {
  // Make sure the lift is down when destroying the controller
  if (lift_status_ != LiftStatus::DOWN) {
    RCLCPP_WARN(node_->get_logger(), "Lift controller being destroyed while lift is not down");
  }
}

void LiftController::moveUp() {
  if (lift_status_ == LiftStatus::MOVING) {
    RCLCPP_WARN(node_->get_logger(), "Lift is already moving");
    return;
  }

  if (lift_status_ == LiftStatus::UP) {
    RCLCPP_INFO(node_->get_logger(), "Lift is already up");
    return;
  }

  auto goal = robot_msgs::action::LiftAction::Goal();
  goal.command = "up";
  goal.target_height = MAX_HEIGHT;

  RCLCPP_INFO(node_->get_logger(), "Sending lift up command");

  lift_status_ = LiftStatus::MOVING;
  sendLiftGoal(goal);
}

void LiftController::moveDown() {
  if (lift_status_ == LiftStatus::MOVING) {
    RCLCPP_WARN(node_->get_logger(), "Lift is already moving");
    return;
  }

  if (lift_status_ == LiftStatus::DOWN) {
    RCLCPP_INFO(node_->get_logger(), "Lift is already down");
    return;
  }

  auto goal = robot_msgs::action::LiftAction::Goal();
  goal.command = "down";
  goal.target_height = 0.0;

  RCLCPP_INFO(node_->get_logger(), "Sending lift down command");

  lift_status_ = LiftStatus::MOVING;
  sendLiftGoal(goal);
}

void LiftController::stop() {
  if (lift_status_ != LiftStatus::MOVING) {
    RCLCPP_INFO(node_->get_logger(), "Lift is not moving");
    return;
  }

  auto goal = robot_msgs::action::LiftAction::Goal();
  goal.command = "stop";
  goal.target_height = current_height_;

  RCLCPP_INFO(node_->get_logger(), "Sending lift stop command");

  sendLiftGoal(goal);
}

void LiftController::sendLiftGoal(const robot_msgs::action::LiftAction::Goal& goal) {
  // Wait for action server to be available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Lift action server not available");
    lift_status_ = (current_height_ < 0.1) ? LiftStatus::DOWN : LiftStatus::UP;
    return;
  }

  // Send the goal
  auto send_goal_options = rclcpp_action::Client<robot_msgs::action::LiftAction>::SendGoalOptions();

  // Set callbacks for goal response, feedback, and result
  send_goal_options.goal_response_callback = std::bind(&LiftController::goalResponseCallback, this, std::placeholders::_1);

  send_goal_options.feedback_callback = std::bind(&LiftController::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(&LiftController::resultCallback, this, std::placeholders::_1);

  // Send the goal
  action_client_->async_send_goal(goal, send_goal_options);
}

void LiftController::goalResponseCallback(const LiftGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    lift_status_ = (current_height_ < 0.1) ? LiftStatus::DOWN : LiftStatus::UP;
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the lift action server");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Lift goal accepted by the server");
}

void LiftController::feedbackCallback(const LiftGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const robot_msgs::action::LiftAction::Feedback> feedback) {
  current_height_ = feedback->current_height;

  // Update status based on height
  if (current_height_ < 0.1) {
    lift_status_ = LiftStatus::DOWN;
  } else if (std::abs(current_height_ - MAX_HEIGHT) < 0.1) {
    lift_status_ = LiftStatus::UP;
  } else {
    lift_status_ = LiftStatus::MOVING;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Lift height: %.2f", current_height_);
}

void LiftController::resultCallback(const LiftGoalHandle::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Lift action succeeded");
      if (current_height_ < 0.1) {
        lift_status_ = LiftStatus::DOWN;
      } else if (std::abs(current_height_ - MAX_HEIGHT) < 0.1) {
        lift_status_ = LiftStatus::UP;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Lift action was aborted");
      lift_status_ = (current_height_ < 0.1) ? LiftStatus::DOWN : LiftStatus::UP;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Lift action was canceled");
      lift_status_ = (current_height_ < 0.1) ? LiftStatus::DOWN : LiftStatus::UP;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code from lift action");
      lift_status_ = (current_height_ < 0.1) ? LiftStatus::DOWN : LiftStatus::UP;
      break;
  }
}

void LiftController::publishLiftPosition() {
  auto msg = std_msgs::msg::Float32();
  msg.data = current_height_;
  lift_position_pub_->publish(msg);
}

double LiftController::getCurrentHeight() const { return current_height_; }

LiftStatus LiftController::getStatus() const { return lift_status_; }

}  // namespace robot_master