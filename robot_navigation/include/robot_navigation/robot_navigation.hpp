/**
 * @file /include/turtlebot_navigation/navigation_node.hpp
 *
 * @brief Header for the navigation node.
 *
 * @date May 2025
 **/

#ifndef TURTLEBOT_NAVIGATION_NODE_HPP
#define TURTLEBOT_NAVIGATION_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/srv/navigate_to_parcel.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace turtlebot_navigation {

class NavigationNode : public rclcpp::Node {
 public:
  NavigationNode();

 private:
  // Service callbacks
  void handleNavigationRequest(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<robot_msgs::srv::NavigateToParcel::Request> request,
                               std::shared_ptr<robot_msgs::srv::NavigateToParcel::Response> response);

  // Action client for Nav2
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;

  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;

  // Service server
  rclcpp::Service<robot_msgs::srv::NavigateToParcel>::SharedPtr navigate_service_;

  // Navigation functions
  void sendNavigationGoal(double x, double y, double yaw, std::function<void(bool, const std::string&)> callback);

  // Navigation action client callbacks
  void goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle);
  void feedbackCallback(const NavigateGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const NavigateAction::Feedback> feedback);
  void resultCallback(const NavigateGoalHandle::WrappedResult& result);

  // Callback tracking
  std::mutex callback_mutex_;
  std::function<void(bool, const std::string&)> current_callback_;
};

}  // namespace turtlebot_navigation

#endif  // TURTLEBOT_NAVIGATION_NODE_HPP