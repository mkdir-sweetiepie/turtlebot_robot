/**
 * @file /include/robot_navigation/waypoint_navigator.hpp
 *
 * @brief TurtleBot3 웨이포인트 네비게이션 + OCR 서비스 통합
 *
 * @date May 2025
 **/

#ifndef ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP
#define ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/srv/ocr_scan.hpp"
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

  // 초기 위치 설정
  void setInitialPose(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0);

  // 시스템 활성화 대기
  bool waitUntilNav2Active();
  bool waitForOCRService();

  // 물품 검색 시작
  bool startItemSearch(const std::string& item_id);

  // 검색용 웨이포인트 설정
  void setSearchWaypoints();

 private:
  // 웨이포인트 관련
  std::vector<Waypoint> waypoints_;
  std::vector<std::string> location_names_;

  // Nav2 액션 클라이언트
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;

  // OCR 서비스 클라이언트
  rclcpp::Client<robot_msgs::srv::OCRScan>::SharedPtr ocr_scan_client_;

  // 검색 상태 관리
  bool navigation_active_;
  bool search_active_;
  bool item_found_;
  size_t current_waypoint_index_;
  std::string target_item_;

  // 네비게이션 관련 메서드
  void navigateToNextWaypoint();
  void navigateToHome();
  geometry_msgs::msg::PoseStamped createPoseFromWaypoint(const Waypoint& waypoint);

  // OCR 스캔 관련 메서드
  void performOCRScan();
  void handleOCRResult(std::shared_ptr<robot_msgs::srv::OCRScan::Response> response);

  // 네비게이션 콜백 함수
  void goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle);
  void feedbackCallback(const NavigateGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const NavigateAction::Feedback> feedback);
  void resultCallback(const NavigateGoalHandle::WrappedResult& result);
};

}  // namespace robot_navigation

#endif  // ROBOT_NAVIGATION_WAYPOINT_NAVIGATOR_HPP