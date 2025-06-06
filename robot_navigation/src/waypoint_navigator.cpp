/**
 * @file waypoint_navigator.cpp
 * @brief 물품 검색 통합 웨이포인트 네비게이터 (최적화된 버전)
 * @date May 2025
 */

#include "robot_navigation/waypoint_navigator.hpp"

#include <cmath>
#include <thread>

namespace robot_navigation {

WaypointNavigator::WaypointNavigator() : Node("waypoint_navigator"), target_item_(""), mission_active_(false), current_waypoint_index_(0), navigation_active_(false) {
  // 액션 클라이언트 초기화
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");

  // OCR 서비스 클라이언트 초기화
  ocr_scan_client_ = this->create_client<robot_msgs::srv::OCRScan>("ocr_scan_request");

  // 마스터와 통신용 퍼블리셔/서브스크라이버
  search_result_pub_ = this->create_publisher<std_msgs::msg::String>("item_search_result", 10);
  search_request_sub_ = this->create_subscription<std_msgs::msg::String>("item_search_request", 10, std::bind(&WaypointNavigator::searchRequestCallback, this, std::placeholders::_1));

  // 검색용 웨이포인트 초기화
  initializeSearchWaypoints();

  RCLCPP_INFO(this->get_logger(), "🤖 물품 검색 웨이포인트 네비게이터가 시작되었습니다.");
  RCLCPP_INFO(this->get_logger(), "📱 마스터 윈도우에서 물품 검색 요청을 기다리고 있습니다...");
}

void WaypointNavigator::initializeSearchWaypoints() {
  search_waypoints_.clear();

  // 최적화된 8개 웨이포인트 (더 안정적인 경로)
  search_waypoints_ = {
      {"시작 위치", 0.01, 0.0, 0.0},        // 시작 위치
      {"경유 위치 A", 0.25, 0.0, 0.0},      // 경유 위치 A (더 가까이)
      {"위치 1", 0.45, 0.45, M_PI / 2},     // 위치 1 (더 보수적)
      {"위치 2", 0.75, 0.45, M_PI / 2},     // 위치 2
      {"위치 3", 0.75, -0.45, -M_PI / 2},   // 위치 3
      {"위치 4", 0.45, -0.45, -M_PI / 2},   // 위치 4
      {"경유 위치 A", 0.25, 0.0, 0.0},      // 경유 위치 A (복귀)
      {"시작 위치 (귀환)", 0.01, 0.0, 0.0}  // 시작 위치 (귀환)
  };

  RCLCPP_INFO(this->get_logger(), "📍 웨이포인트 %zu개가 설정되었습니다:", search_waypoints_.size());
  for (size_t i = 0; i < search_waypoints_.size(); i++) {
    const auto& wp = search_waypoints_[i];
    RCLCPP_INFO(this->get_logger(), "   %zu. %s: (%.2f, %.2f, %.1f°)", i + 1, wp.name.c_str(), wp.x, wp.y, wp.yaw * 180.0 / M_PI);
  }
}

void WaypointNavigator::searchRequestCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string request = msg->data;

  if (request.find("START:") == 0) {
    // 물품 검색 시작 요청
    target_item_ = request.substr(6);  // "START:" 제거

    if (mission_active_) {
      RCLCPP_WARN(this->get_logger(), "⚠️ 이미 미션이 진행 중입니다.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "\n🚀 물품 '%s' 검색 미션을 시작합니다!", target_item_.c_str());

    mission_active_ = true;
    current_waypoint_index_ = 0;

    // 초기 설정
    setInitialPose();

    if (!waitForServices()) {
      mission_active_ = false;
      sendSearchResult("ERROR:서비스 연결 실패");
      return;
    }

    // 3초 대기 후 시작 (Nav2 초기화 대기)
    RCLCPP_INFO(this->get_logger(), "⏱️ Nav2 초기화 대기 중... (3초)");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 첫 번째 웨이포인트로 이동 시작
    navigateToNextWaypoint();

  } else if (request == "CANCEL") {
    // 미션 취소 요청
    RCLCPP_INFO(this->get_logger(), "🛑 미션 취소 요청을 받았습니다.");
    cancelCurrentNavigation();
    mission_active_ = false;
    navigation_active_ = false;
  }
}

void WaypointNavigator::setInitialPose() {
  auto initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = this->now();
  initial_pose.pose.pose.position.x = 0.01;
  initial_pose.pose.pose.position.y = 0.0;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = 0.0;
  initial_pose.pose.pose.orientation.w = 1.0;

  // 여러 번 발행하여 확실히 설정
  for (int i = 0; i < 5; ++i) {
    initial_pose_pub->publish(initial_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_INFO(this->get_logger(), "📍 초기 위치가 설정되었습니다.");
}

bool WaypointNavigator::waitForServices() {
  RCLCPP_INFO(this->get_logger(), "🔗 필요한 서비스들을 기다리는 중...");

  // Nav2 액션 서버 대기
  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(this->get_logger(), "❌ Nav2 액션 서버를 찾을 수 없습니다!");
    return false;
  }

  // OCR 스캔 서비스 대기
  if (!ocr_scan_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "❌ OCR 스캔 서비스를 찾을 수 없습니다!");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "✅ 모든 서비스가 준비되었습니다.");
  return true;
}

void WaypointNavigator::navigateToNextWaypoint() {
  if (!mission_active_) return;

  // 모든 거점 검색 완료 확인
  if (current_waypoint_index_ >= search_waypoints_.size()) {
    RCLCPP_INFO(this->get_logger(), "❌ 모든 거점에서 물품 '%s'을(를) 찾지 못했습니다.", target_item_.c_str());
    sendSearchResult("NOT_FOUND:" + target_item_);
    sendSearchResult("MISSION_COMPLETE");
    mission_active_ = false;
    return;
  }

  const auto& waypoint = search_waypoints_[current_waypoint_index_];
  RCLCPP_INFO(this->get_logger(), "\n🎯 %s로 이동 중... (%zu/%zu)", waypoint.name.c_str(), current_waypoint_index_ + 1, search_waypoints_.size());

  auto goal_pose = createPoseFromWaypoint(waypoint);
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = goal_pose;

  navigation_active_ = true;
  current_goal_handle_.reset();  // 이전 핸들 초기화

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&WaypointNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);

  // 비동기 목표 전송
  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointNavigator::sendSearchResult(const std::string& result) {
  auto msg = std_msgs::msg::String();
  msg.data = result;
  search_result_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "📡 마스터에 결과 전송: %s", result.c_str());
}

void WaypointNavigator::cancelCurrentNavigation() {
  if (navigation_active_ && current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "🛑 현재 네비게이션을 취소합니다.");
    navigate_client_->async_cancel_goal(current_goal_handle_);
  }
}

void WaypointNavigator::performOCRScan() {
  if (!mission_active_ || current_waypoint_index_ >= search_waypoints_.size()) return;

  const auto& current_waypoint = search_waypoints_[current_waypoint_index_];

  RCLCPP_INFO(this->get_logger(), "🔍 %s에서 '%s' 검색 중...", current_waypoint.name.c_str(), target_item_.c_str());

  // OCR 스캔 서비스 요청 생성
  auto request = std::make_shared<robot_msgs::srv::OCRScan::Request>();
  request->target_item_id = target_item_;
  request->current_location = current_waypoint.name;
  request->location_index = static_cast<int32_t>(current_waypoint_index_);

  // 비동기 서비스 호출
  auto result_future = ocr_scan_client_->async_send_request(request);

  // 결과 대기 (최대 15초)
  auto status = result_future.wait_for(std::chrono::seconds(15));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "❌ OCR 스캔 서비스 타임아웃!");
    current_waypoint_index_++;
    navigateToNextWaypoint();
    return;
  }

  auto response = result_future.get();
  handleOCRResult(response);
}

void WaypointNavigator::handleOCRResult(std::shared_ptr<robot_msgs::srv::OCRScan::Response> response) {
  RCLCPP_INFO(this->get_logger(), "📋 OCR 결과: %s", response->message.c_str());

  if (response->item_found) {
    RCLCPP_INFO(this->get_logger(), "\n🎉 목표 물품 '%s'을(를) 발견했습니다!", target_item_.c_str());

    // 발견 결과를 마스터에 전송
    sendSearchResult("FOUND:" + target_item_);

    // 8초 대기 (마스터에서 리프트 동작 수행)
    RCLCPP_INFO(this->get_logger(), "⏱️ 리프트 동작 완료 대기 중... (8초)");
    rclcpp::sleep_for(std::chrono::seconds(8));

    RCLCPP_INFO(this->get_logger(), "🏠 홈으로 복귀합니다...");

    // 남은 경유지들을 거쳐서 홈으로 복귀
    for (size_t j = current_waypoint_index_ + 1; j < search_waypoints_.size(); j++) {
      if (!mission_active_) break;

      const auto& return_waypoint = search_waypoints_[j];
      RCLCPP_INFO(this->get_logger(), "🎯 %s로 이동 중... (복귀 경로)", return_waypoint.name.c_str());

      // 재시도 로직 추가
      bool success = false;
      for (int retry = 0; retry < 2 && !success; retry++) {
        if (retry > 0) {
          RCLCPP_WARN(this->get_logger(), "🔄 재시도 %d/1...", retry);
        }
        success = navigateToWaypointBlocking(return_waypoint);
      }

      if (success) {
        RCLCPP_INFO(this->get_logger(), "✅ %s 도착!", return_waypoint.name.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ %s 이동 실패, 계속 진행", return_waypoint.name.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "🏁 미션 완료! 홈 복귀 성공!");
    sendSearchResult("MISSION_COMPLETE");
    mission_active_ = false;

  } else {
    RCLCPP_INFO(this->get_logger(), "❌ %s에서 물품을 찾지 못했습니다. 다음 위치로 이동합니다.", search_waypoints_[current_waypoint_index_].name.c_str());
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

bool WaypointNavigator::navigateToWaypointBlocking(const Waypoint& waypoint) {
  auto goal_pose = createPoseFromWaypoint(waypoint);
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();

  // 비동기 목표 전송
  auto goal_handle_future = navigate_client_->async_send_goal(goal_msg, send_goal_options);

  // 목표 수락 대기 (블로킹)
  auto wait_result = goal_handle_future.wait_for(std::chrono::seconds(5));
  if (wait_result != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "❌ 목표 전송 타임아웃!");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "❌ 목표가 거부되었습니다!");
    return false;
  }

  // 결과 대기 (블로킹) - 30초로 단축
  auto result_future = navigate_client_->async_get_result(goal_handle);
  auto result_wait = result_future.wait_for(std::chrono::seconds(30));  // 60 → 30초

  if (result_wait != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "⚠️ 네비게이션 타임아웃 (30초). 목표 취소.");
    navigate_client_->async_cancel_goal(goal_handle);
    return false;
  }

  auto result = result_future.get();
  return (result.code == rclcpp_action::ResultCode::SUCCEEDED);
}

geometry_msgs::msg::PoseStamped WaypointNavigator::createPoseFromWaypoint(const Waypoint& waypoint) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;

  // 쿼터니언 계산
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
    RCLCPP_ERROR(this->get_logger(), "❌ 목표 거부됨");
    handleNavigationFailure();
  } else {
    current_goal_handle_ = goal_handle;  // 핸들 저장
    RCLCPP_INFO(this->get_logger(), "✅ 목표 수락됨");
  }
}

void WaypointNavigator::feedbackCallback(const NavigateGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const NavigateAction::Feedback> feedback) {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    auto current_pose = feedback->current_pose.pose;
    double target_x = search_waypoints_[current_waypoint_index_].x;
    double target_y = search_waypoints_[current_waypoint_index_].y;

    double dx = target_x - current_pose.position.x;
    double dy = target_y - current_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 5초마다 한 번씩만 로그 출력 (스팸 방지)
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(this->get_logger(), "🚶 이동 중... 목표까지 거리: %.2fm", distance);
      last_log_time = now;
    }
  }
}

void WaypointNavigator::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  navigation_active_ = false;
  current_goal_handle_.reset();  // 핸들 초기화

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    handleNavigationSuccess();
  } else {
    RCLCPP_WARN(this->get_logger(), "⚠️ 네비게이션 실패 (코드: %d)", static_cast<int>(result.code));
    handleNavigationFailure();
  }
}

void WaypointNavigator::handleNavigationSuccess() {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    const auto& waypoint = search_waypoints_[current_waypoint_index_];
    RCLCPP_INFO(this->get_logger(), "✅ %s 도착 성공!", waypoint.name.c_str());

    // 물품 검색은 "위치 1~4"에서만 수행
    if (waypoint.name.find("위치") != std::string::npos && waypoint.name != "시작 위치" && waypoint.name != "시작 위치 (귀환)") {
      // OCR 스캔 수행
      performOCRScan();
    } else {
      // 경유지나 시작/귀환 위치에서는 3초 대기 후 다음으로
      RCLCPP_INFO(this->get_logger(), "⏱️ 경유지에서 3초 대기...");
      rclcpp::sleep_for(std::chrono::seconds(3));
      current_waypoint_index_++;
      navigateToNextWaypoint();
    }
  }
}

void WaypointNavigator::handleNavigationFailure() {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    const auto& waypoint = search_waypoints_[current_waypoint_index_];
    RCLCPP_WARN(this->get_logger(), "⚠️ %s 이동 실패, 다음 거점으로 이동", waypoint.name.c_str());
  }
  current_waypoint_index_++;
  navigateToNextWaypoint();
}

}  // namespace robot_navigation

// 메인 함수
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto navigator = std::make_shared<robot_navigation::WaypointNavigator>();

  RCLCPP_INFO(navigator->get_logger(), "🤖 웨이포인트 네비게이터가 준비되었습니다.");
  RCLCPP_INFO(navigator->get_logger(), "📱 마스터 윈도우에서 물품 검색을 시작하세요!");

  rclcpp::spin(navigator);

  rclcpp::shutdown();
  return 0;
}