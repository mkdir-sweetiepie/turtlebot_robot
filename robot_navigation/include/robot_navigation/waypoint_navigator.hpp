/**
 * @file waypoint_navigator.hpp
 * @brief TurtleBot3 웨이포인트 네비게이션 + 정밀 제어 액션 통합
 * @date May 2025
 */

#ifndef ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP
#define ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/precise_control.hpp"
#include "robot_msgs/msg/log_message.hpp"
#include "robot_msgs/msg/ocr_request.hpp"
#include "robot_msgs/msg/ocr_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_navigation {

struct Waypoint {
  std::string name;
  double x;
  double y;
  double yaw;  // 라디안 단위
};

class WaypointNavigator : public rclcpp::Node {
 public:
  WaypointNavigator();
  virtual ~WaypointNavigator() = default;

  // 로그 시스템
  void publishSystemLog(const std::string& level, const std::string& message);

 private:
  // 액션 타입 정의
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  using PreciseControlAction = robot_msgs::action::PreciseControl;
  using PreciseControlGoalHandle = rclcpp_action::ClientGoalHandle<PreciseControlAction>;

  // 웨이포인트 관련
  std::vector<Waypoint> search_waypoints_;

  // 액션 클라이언트
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;
  rclcpp_action::Client<PreciseControlAction>::SharedPtr precise_control_client_;

  // OCR 제어 및 통신
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ocr_control_pub_;
  rclcpp::Publisher<robot_msgs::msg::OCRRequest>::SharedPtr ocr_request_pub_;
  rclcpp::Subscription<robot_msgs::msg::OCRResult>::SharedPtr ocr_result_sub_;

  // 마스터와 통신용
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_result_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_request_sub_;

  // 통합 로그 시스템
  rclcpp::Publisher<robot_msgs::msg::LogMessage>::SharedPtr system_log_pub_;

  // 상태 관리
  std::string target_item_;
  bool mission_active_;
  size_t current_waypoint_index_;
  bool navigation_active_;
  bool waiting_for_result_;
  bool waiting_for_ocr_;
  bool returning_home_;
  bool ocr_active_;
  bool precise_control_active_;

  // OCR 상태 관리
  int64_t current_request_id_;
  std::chrono::steady_clock::time_point ocr_start_time_;

  // 비동기 처리용
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_future<NavigateGoalHandle::SharedPtr> current_goal_future_;

  // OCR 제어
  void enableOCR();
  void disableOCR();

  // 초기화 관련
  void initializeSearchWaypoints();
  void setInitialPose();
  bool waitForServices();

  // 네비게이션 관련
  void navigateToNextWaypoint();
  geometry_msgs::msg::PoseStamped createPoseFromWaypoint(const Waypoint& waypoint);

  // 상태 처리
  void checkStatus();
  void handleNavigationSuccess();
  void handleNavigationFailure();

  // OCR 관련
  void performOCRScan();
  void ocrResultCallback(const robot_msgs::msg::OCRResult::SharedPtr msg);

  // 정밀 제어 액션 관련
  void callPreciseControlAction();
  void preciseControlGoalResponseCallback(const PreciseControlGoalHandle::SharedPtr& goal_handle);
  void preciseControlFeedbackCallback(const PreciseControlGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const PreciseControlAction::Feedback> feedback);
  void preciseControlResultCallback(const PreciseControlGoalHandle::WrappedResult& result);

  // 통신 관련
  void searchRequestCallback(const std_msgs::msg::String::SharedPtr msg);
  void sendSearchResult(const std::string& result);

  // 네비게이션 콜백 함수
  void goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle);
  void feedbackCallback(const NavigateGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const NavigateAction::Feedback> feedback);
  void resultCallback(const NavigateGoalHandle::WrappedResult& result);
};

}  // namespace robot_navigation

#endif  // ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP