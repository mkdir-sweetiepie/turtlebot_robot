#ifndef ROBOT_NAVIGATION_NAVIGATION_MANAGER_HPP
#define ROBOT_NAVIGATION_NAVIGATION_MANAGER_HPP

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_msgs/srv/navigate_to_parcel.hpp>
#include <vector>

namespace robot_navigation {

class NavigationManager : public rclcpp::Node {
 public:
  NavigationManager();
  ~NavigationManager() = default;

  bool isNavigating() const { return navigation_active_; }

 private:

  // Nav2 action client
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;

  // Initial pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  // Services
  rclcpp::Service<robot_msgs::srv::NavigateToParcel>::SharedPtr navigate_service_;

  // State management
  bool navigation_active_{false};

  // Methods
  void setInitialPose();
  bool waitForNav2();
  void handleNavigationRequest(const std::shared_ptr<robot_msgs::srv::NavigateToParcel::Request> request, std::shared_ptr<robot_msgs::srv::NavigateToParcel::Response> response);
  void handleNavigationResult(const GoalHandle::WrappedResult& result);
};

}  // namespace robot_navigation

#endif  // ROBOT_NAVIGATION_NAVIGATION_MANAGER_HPP