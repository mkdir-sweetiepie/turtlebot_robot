/**
 * @file /src/navigation_node.cpp
 *
 * @brief Implementation for the navigation node.
 *
 * @date May 2025
 **/

#include "../include/robot_navigation/robot_navigation.hpp"

namespace turtlebot_navigation {

NavigationNode::NavigationNode() : Node("navigation_node") {
  // Initialize the nav2 action client
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");

  // Initialize the navigation service server
  navigate_service_ = this->create_service<robot_msgs::srv::NavigateToParcel>(
      "navigate_to_parcel", std::bind(&NavigationNode::handleNavigationRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
}

void NavigationNode::handleNavigationRequest(const std::shared_ptr<rmw_request_id_t> /*request_header*/, const std::shared_ptr<robot_msgs::srv::NavigateToParcel::Request> request,
                                             std::shared_ptr<robot_msgs::srv::NavigateToParcel::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received navigation request: x=%.2f, y=%.2f, yaw=%.2f", request->x, request->y, request->yaw);

  // Set up callback to handle the navigation result
  auto callback = [this, response](bool success, const std::string& message) {
    response->success = success;
    response->message = message;

    RCLCPP_INFO(this->get_logger(), "Navigation result: success=%d, message=%s", success, message.c_str());
  };

  // Send the navigation goal
  sendNavigationGoal(request->x, request->y, request->yaw, callback);
}

void NavigationNode::sendNavigationGoal(double x, double y, double yaw, std::function<void(bool, const std::string&)> callback) {
  // Store the callback
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    current_callback_ = callback;
  }

  // Wait for the action server to be available
  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
    callback(false, "Navigation action server not available");
    return;
  }

  // Create the goal pose
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();

  // Set position
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;

  // Convert yaw to quaternion
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw * M_PI / 180.0);  // Convert degrees to radians
  goal_msg.pose.pose.orientation = tf2::toMsg(quat);

  RCLCPP_INFO(this->get_logger(), "Sending navigation goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

  // Setup send goal options
  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&NavigationNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&NavigationNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&NavigationNode::resultCallback, this, std::placeholders::_1);

  // Send the goal
  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationNode::goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (current_callback_) {
      current_callback_(false, "Goal was rejected by the action server");
      current_callback_ = nullptr;
    }

    return;
  }

  RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
}

void NavigationNode::feedbackCallback(const NavigateGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const NavigateAction::Feedback> feedback) {
  // 수정된 부분: goal_handle->get_goal() 호출 제거
  // 대신 피드백 데이터만 사용

  // feedback에 있는 데이터 직접 사용
  double distance_remaining = feedback->distance_remaining;

  RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f meters", distance_remaining);
}

void NavigationNode::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  std::string message;
  bool success = false;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      message = "Navigation succeeded";
      success = true;
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      message = "Navigation aborted";
      RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      message = "Navigation canceled";
      RCLCPP_ERROR(this->get_logger(), "Navigation canceled");
      break;
    default:
      message = "Unknown result code";
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }

  // Call the stored callback
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (current_callback_) {
    current_callback_(success, message);
    current_callback_ = nullptr;
  }
}

}  // namespace turtlebot_navigation

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtlebot_navigation::NavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}