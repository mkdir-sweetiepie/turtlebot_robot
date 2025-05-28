#include "robot_navigation/navigation_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace robot_navigation {

NavigationManager::NavigationManager() : Node("navigation_manager") {
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  navigate_service_ =
      create_service<robot_msgs::srv::NavigateToParcel>("navigate_to_parcel", std::bind(&NavigationManager::handleNavigationRequest, this, std::placeholders::_1, std::placeholders::_2));

  setInitialPose();  // 초기 위치 설정

  RCLCPP_INFO(this->get_logger(), "내비게이션 매니저 시작");
}

// 초기 위치 설정
void NavigationManager::setInitialPose() {
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = this->now();
  initial_pose.pose.pose.position.x = 0.01;
  initial_pose.pose.pose.position.y = 0.0;
  initial_pose.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 5; ++i) {
    initial_pose_pub_->publish(initial_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

// Nav2 활성화 대기
bool NavigationManager::waitForNav2() {
  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 액션 서버를 사용할 수 없습니다!");
    return false;
  }
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->get_logger(), "Nav2 활성화됨.");
  return true;
}

// 내비게이션 요청 처리
void NavigationManager::handleNavigationRequest(const std::shared_ptr<robot_msgs::srv::NavigateToParcel::Request> request, std::shared_ptr<robot_msgs::srv::NavigateToParcel::Response> response) {
  if (navigation_active_) {
    response->success = false;
    response->message = "내비게이션 진행 중";
    RCLCPP_ERROR(this->get_logger(), "내비게이션 요청 거부: 이미 활성화됨");
    return;
  }

  navigation_active_ = true;

  if (!waitForNav2()) {
    response->success = false;
    response->message = "Nav2 초기화 실패";
    RCLCPP_ERROR(this->get_logger(), "Nav2 초기화 실패");
    navigation_active_ = false;
    return;
  }

  // 요청에서 목표 위치 설정
  auto goal = NavigateAction::Goal();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = request->x;
  pose.pose.position.y = request->y;
  pose.pose.position.z = 0.0;
  double yaw = request->yaw;
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  pose.pose.orientation.z = sy;
  pose.pose.orientation.w = cy;
  goal.pose = pose;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&NavigationManager::handleNavigationResult, this, std::placeholders::_1);

  auto future = navigate_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, yaw=%.2f로 내비게이션 중", request->x, request->y, request->yaw);

  response->success = true;
  response->message = "내비게이션 시작";
}

// 내비게이션 결과 처리
void NavigationManager::handleNavigationResult(const GoalHandle::WrappedResult& result) {
  navigation_active_ = false;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "내비게이션 성공");
  } else {
    RCLCPP_ERROR(this->get_logger(), "내비게이션 실패");
  }
}

}  // namespace robot_navigation