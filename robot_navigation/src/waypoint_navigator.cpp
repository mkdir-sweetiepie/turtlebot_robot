/**
 * @file /src/waypoint_navigator.cpp
 *
 * @brief TurtleBot3 웨이포인트 네비게이션 + OCR 서비스 통합
 *
 * @date May 2025
 **/

#include "robot_navigation/waypoint_navigator.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_navigation {

WaypointNavigator::WaypointNavigator() : Node("waypoint_navigator"), navigation_active_(false), current_waypoint_index_(0), search_active_(false), item_found_(false), target_item_("") {
  // 네비게이션 액션 클라이언트 초기화
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");

  // OCR 스캔 서비스 클라이언트 초기화
  ocr_scan_client_ = this->create_client<robot_msgs::srv::OCRScan>("ocr_scan_request");

  RCLCPP_INFO(this->get_logger(), "웨이포인트 네비게이터 노드가 시작되었습니다.");
}

void WaypointNavigator::setInitialPose(double x, double y, double z, double w) {
  auto initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = this->now();

  initial_pose.pose.pose.position.x = x;
  initial_pose.pose.pose.position.y = y;
  initial_pose.pose.pose.position.z = 0.0;

  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = z;
  initial_pose.pose.pose.orientation.w = w;

  for (int i = 0; i < 5; ++i) {
    initial_pose_pub->publish(initial_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "초기 위치가 설정되었습니다: (%.2f, %.2f)", x, y);
}

bool WaypointNavigator::waitUntilNav2Active() {
  RCLCPP_INFO(this->get_logger(), "Nav2 시스템이 활성화될 때까지 대기 중...");

  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 액션 서버가 활성화되지 않았습니다!");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->get_logger(), "Nav2 시스템 활성화 완료!");
  return true;
}

bool WaypointNavigator::waitForOCRService() {
  RCLCPP_INFO(this->get_logger(), "OCR 서비스 대기 중...");

  if (!ocr_scan_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "OCR 서비스가 활성화되지 않았습니다!");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "OCR 서비스 활성화 완료!");
  return true;
}

void WaypointNavigator::setSearchWaypoints() {
  waypoints_.clear();
  location_names_.clear();

  // 검색용 4개 거점 설정
  location_names_ = {"거점1", "거점2", "거점3", "거점4"};

  std::vector<std::tuple<double, double, double>> coords = {
      std::make_tuple(0.5, 0.5, M_PI / 2),    // 거점1
      std::make_tuple(0.8, 0.5, M_PI / 2),    // 거점2
      std::make_tuple(0.8, -0.5, -M_PI / 2),  // 거점3
      std::make_tuple(0.5, -0.5, -M_PI / 2),  // 거점4
  };

  for (size_t i = 0; i < coords.size(); i++) {
    Waypoint wp;
    wp.name = location_names_[i];
    wp.x = std::get<0>(coords[i]);
    wp.y = std::get<1>(coords[i]);
    wp.yaw = std::get<2>(coords[i]);
    waypoints_.push_back(wp);

    RCLCPP_INFO(this->get_logger(), "검색 거점 %zu: %s (x=%.2f, y=%.2f, 방향=%.1f°)", i + 1, wp.name.c_str(), wp.x, wp.y, wp.yaw * 180.0 / M_PI);
  }
}

bool WaypointNavigator::startItemSearch(const std::string& item_id) {
  if (search_active_) {
    RCLCPP_WARN(this->get_logger(), "이미 검색이 진행 중입니다.");
    return false;
  }

  target_item_ = item_id;
  search_active_ = true;
  item_found_ = false;
  current_waypoint_index_ = 0;

  RCLCPP_INFO(this->get_logger(), "물품 '%s' 검색을 시작합니다.", item_id.c_str());

  // 초기 위치 설정 및 시스템 준비
  setInitialPose(0.01, 0.0, 0.01, 1.0);
  setSearchWaypoints();

  if (!waitUntilNav2Active() || !waitForOCRService()) {
    search_active_ = false;
    return false;
  }

  // 첫 번째 거점으로 이동 시작
  navigateToNextWaypoint();
  return true;
}

void WaypointNavigator::navigateToNextWaypoint() {
  if (!search_active_ || item_found_) return;

  // 모든 거점 검색 완료 확인
  if (current_waypoint_index_ >= waypoints_.size()) {
    RCLCPP_INFO(this->get_logger(), "모든 거점에서 물품 '%s'을(를) 찾지 못했습니다.", target_item_.c_str());
    search_active_ = false;
    return;
  }

  const auto& waypoint = waypoints_[current_waypoint_index_];
  RCLCPP_INFO(this->get_logger(), "%s로 이동 중... (%zu/%zu)", waypoint.name.c_str(), current_waypoint_index_ + 1, waypoints_.size());

  auto goal_pose = createPoseFromWaypoint(waypoint);
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = goal_pose;

  navigation_active_ = true;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&WaypointNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);

  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointNavigator::performOCRScan() {
  if (!search_active_ || current_waypoint_index_ >= waypoints_.size()) return;

  const auto& current_waypoint = waypoints_[current_waypoint_index_];

  RCLCPP_INFO(this->get_logger(), "%s에서 OCR 스캔 시작", current_waypoint.name.c_str());

  auto request = std::make_shared<robot_msgs::srv::OCRScan::Request>();
  request->target_item_id = target_item_;
  request->current_location = current_waypoint.name;
  request->location_index = static_cast<int32_t>(current_waypoint_index_);

  auto result_future = ocr_scan_client_->async_send_request(request);

  // 블로킹 방식으로 응답 대기 (15초 타임아웃)
  auto status = result_future.wait_for(std::chrono::seconds(15));

  if (status == std::future_status::ready) {
    auto response = result_future.get();
    handleOCRResult(response);
  } else {
    RCLCPP_ERROR(this->get_logger(), "OCR 스캔 타임아웃");
    // 다음 거점으로 이동
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

void WaypointNavigator::handleOCRResult(std::shared_ptr<robot_msgs::srv::OCRScan::Response> response) {
  RCLCPP_INFO(this->get_logger(), "OCR 결과: %s", response->message.c_str());

  if (response->item_found) {
    RCLCPP_INFO(this->get_logger(), "목표 물품 '%s' 발견! Master 노드에서 리프트 동작을 처리합니다", target_item_.c_str());
    item_found_ = true;
    search_active_ = false;

    // Master 노드가 자동으로 리프트 동작을 처리하므로 대기 후 홈으로 복귀
    std::this_thread::sleep_for(std::chrono::seconds(8));  // 리프트 동작 완료 대기
    navigateToHome();

  } else {
    RCLCPP_INFO(this->get_logger(), "물품을 찾지 못했습니다. 다음 거점으로 이동");
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

void WaypointNavigator::navigateToHome() {
  RCLCPP_INFO(this->get_logger(), "홈으로 복귀합니다");

  // 홈 위치로 이동
  Waypoint home_waypoint;
  home_waypoint.name = "홈";
  home_waypoint.x = 0.01;
  home_waypoint.y = 0.0;
  home_waypoint.yaw = 0.0;

  auto goal_pose = createPoseFromWaypoint(home_waypoint);
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = goal_pose;

  navigation_active_ = true;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.result_callback = [this](const NavigateGoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "홈 복귀 완료! 물품 검색 미션 종료");
    } else {
      RCLCPP_WARN(this->get_logger(), "홈 복귀 실패");
    }
    navigation_active_ = false;
  };

  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

geometry_msgs::msg::PoseStamped WaypointNavigator::createPoseFromWaypoint(const Waypoint& waypoint) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;

  double cy = std::cos(waypoint.yaw * 0.5);
  double sy = std::sin(waypoint.yaw * 0.5);
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = sy;
  pose.pose.orientation.w = cy;

  return pose;
}

void WaypointNavigator::goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "목표 거부됨");
    navigation_active_ = false;
    // 다음 거점으로 이동 시도
    current_waypoint_index_++;
    navigateToNextWaypoint();
  } else {
    RCLCPP_INFO(this->get_logger(), "목표 수락됨");
  }
}

void WaypointNavigator::feedbackCallback(const NavigateGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const NavigateAction::Feedback> feedback) {
  if (current_waypoint_index_ < waypoints_.size()) {
    auto current_pose = feedback->current_pose.pose;
    double target_x = waypoints_[current_waypoint_index_].x;
    double target_y = waypoints_[current_waypoint_index_].y;

    double dx = target_x - current_pose.position.x;
    double dy = target_y - current_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    RCLCPP_DEBUG(this->get_logger(), "현재 위치: x=%.2f, y=%.2f, 목표까지 거리: %.2fm", current_pose.position.x, current_pose.position.y, distance);
  }
}

void WaypointNavigator::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  navigation_active_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (current_waypoint_index_ < waypoints_.size()) {
      const auto& waypoint = waypoints_[current_waypoint_index_];
      RCLCPP_INFO(this->get_logger(), "%s 도착 성공!", waypoint.name.c_str());

      // 거점 도착 후 OCR 스캔 시작
      performOCRScan();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "네비게이션 실패, 다음 거점으로 이동");
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

}  // namespace robot_navigation

// 메인 함수
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto waypoint_navigator = std::make_shared<robot_navigation::WaypointNavigator>();

  // 사용자 입력을 통한 물품 검색 시작 (실제로는 서비스 호출 또는 토픽을 통해 받을 수 있음)
  std::string target_item = "정우경";  // 예시 아이템

  if (argc > 1) {
    target_item = argv[1];
  }

  RCLCPP_INFO(waypoint_navigator->get_logger(), "물품 '%s' 검색을 시작합니다.", target_item.c_str());

  waypoint_navigator->startItemSearch(target_item);

  rclcpp::spin(waypoint_navigator);

  rclcpp::shutdown();
  return 0;
}