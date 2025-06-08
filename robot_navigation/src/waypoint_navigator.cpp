/**
 * @file waypoint_navigator.cpp
 * @brief 물품 검색 통합 웨이포인트 네비게이터 (OCR 토픽 방식, 서비스 완전 제거)
 * @date May 2025
 */

#include "robot_navigation/waypoint_navigator.hpp"

#include <cmath>
#include <thread>

namespace robot_navigation {

WaypointNavigator::WaypointNavigator()
    : Node("waypoint_navigator"),
      target_item_(""),
      mission_active_(false),
      current_waypoint_index_(0),
      navigation_active_(false),
      waiting_for_result_(false),
      waiting_for_ocr_(false),
      current_request_id_(0),
      returning_home_(false) {
  // 액션 클라이언트 초기화
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");

  // OCR 토픽 초기화 (서비스 완전 제거)
  ocr_request_pub_ = this->create_publisher<robot_msgs::msg::OCRRequest>("ocr_scan_request", 10);
  ocr_result_sub_ = this->create_subscription<robot_msgs::msg::OCRResult>("ocr_scan_result", 10, std::bind(&WaypointNavigator::ocrResultCallback, this, std::placeholders::_1));

  // 마스터와 통신용 퍼블리셔/서브스크라이버
  search_result_pub_ = this->create_publisher<std_msgs::msg::String>("item_search_result", 10);
  search_request_sub_ = this->create_subscription<std_msgs::msg::String>("item_search_request", 10, std::bind(&WaypointNavigator::searchRequestCallback, this, std::placeholders::_1));

  // 검색용 웨이포인트 초기화
  initializeSearchWaypoints();

  // 상태 체크 타이머 (비동기 처리용)
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointNavigator::checkStatus, this));

  RCLCPP_INFO(this->get_logger(), "물품 검색 웨이포인트 네비게이터가 시작되었습니다 (OCR 토픽 방식).");
  RCLCPP_INFO(this->get_logger(), "마스터 윈도우에서 물품 검색 요청을 기다리고 있습니다...");
}

void WaypointNavigator::initializeSearchWaypoints() {
  search_waypoints_.clear();

  search_waypoints_ = {{"시작 위치", 0.01, 0.0, 0.0},    {"경유 위치 A", 0.5, 0.0, 0.0},   {"위치 1", 0.5, 0.4, M_PI / 2},  {"위치 2", 0.9, 0.4, M_PI / 2},
                       {"위치 3", 0.9, -0.4, -M_PI / 2}, {"위치 4", 0.5, -0.4, -M_PI / 2}, {"경유 위치 A", 0.3, 0.0, M_PI}, {"시작 위치 (귀환)", 0.01, 0.0, M_PI}};

  RCLCPP_INFO(this->get_logger(), "웨이포인트 %zu개가 설정되었습니다:", search_waypoints_.size());
  for (size_t i = 0; i < search_waypoints_.size(); i++) {
    const auto& wp = search_waypoints_[i];
    RCLCPP_INFO(this->get_logger(), "%zu. %s: (%.2f, %.2f, %.1f도)", i + 1, wp.name.c_str(), wp.x, wp.y, wp.yaw * 180.0 / M_PI);
  }
}

// 상태 체크 타이머 콜백
void WaypointNavigator::checkStatus() {
  if (!mission_active_) return;

  // 네비게이션 목표 응답 체크
  if (waiting_for_result_ && current_goal_future_.valid()) {
    auto status = current_goal_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      auto goal_handle = current_goal_future_.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "목표가 거부되었습니다!");
        handleNavigationFailure();
      }
    }
  }

  // OCR 타임아웃 체크 (10초)
  if (waiting_for_ocr_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - ocr_start_time_).count();

    if (elapsed >= 30) {
      RCLCPP_WARN(this->get_logger(), "OCR 타임아웃 (10초), 다음 위치로 이동");
      waiting_for_ocr_ = false;
      current_waypoint_index_++;
      navigateToNextWaypoint();
    }
  }
}

// 마스터에서 물품 입력 받으면 내비게이션 시작
void WaypointNavigator::searchRequestCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string request = msg->data;

  if (request.find("START:") == 0) {
    target_item_ = request.substr(6);

    if (mission_active_) {
      RCLCPP_WARN(this->get_logger(), "이미 미션이 진행 중입니다.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "물품 '%s' 검색 미션을 시작합니다!", target_item_.c_str());

    mission_active_ = true;
    current_waypoint_index_ = 0;
    waiting_for_result_ = false;
    waiting_for_ocr_ = false;
    returning_home_ = false; // 복귀 모드 초기화

    setInitialPose();

    if (!waitForServices()) {
      mission_active_ = false;
      sendSearchResult("ERROR:서비스 연결 실패");
      return;
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
    navigateToNextWaypoint();

  } else if (request == "CANCEL") {
    RCLCPP_INFO(this->get_logger(), "미션 취소 요청을 받았습니다.");
    mission_active_ = false;
    navigation_active_ = false;
    waiting_for_result_ = false;
    waiting_for_ocr_ = false;
    returning_home_ = false; // 취소 시에도 초기화
  }
}

bool WaypointNavigator::waitForServices() {
  RCLCPP_INFO(this->get_logger(), "Nav2 액션 서버를 기다리는 중...");

  // Nav2 액션 서버 대기만 확인 (OCR은 토픽이므로 별도 확인 불필요)
  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 액션 서버를 찾을 수 없습니다!");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Nav2 액션 서버가 준비되었습니다.");
  return true;
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

  for (int i = 0; i < 5; ++i) {
    initial_pose_pub->publish(initial_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "초기 위치가 설정되었습니다.");
}

// 마스터에 현재 어떤 상태인지 결과 전송
void WaypointNavigator::sendSearchResult(const std::string& result) {
  auto msg = std_msgs::msg::String();
  msg.data = result;
  search_result_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "마스터에 결과 전송: %s", result.c_str());
}

// 다음 웨이포인트로 이동
void WaypointNavigator::navigateToNextWaypoint() {
  if (!mission_active_) return;

  if (current_waypoint_index_ >= search_waypoints_.size()) {
    RCLCPP_INFO(this->get_logger(), "모든 거점에서 물품 '%s'을(를) 찾지 못했습니다.", target_item_.c_str());
    sendSearchResult("NOT_FOUND:" + target_item_);
    sendSearchResult("MISSION_COMPLETE");
    mission_active_ = false;
    return;
  }

  const auto& waypoint = search_waypoints_[current_waypoint_index_];
  RCLCPP_INFO(this->get_logger(), "%s로 이동 중... (%zu/%zu)", waypoint.name.c_str(), current_waypoint_index_ + 1, search_waypoints_.size());

  auto goal_pose = createPoseFromWaypoint(waypoint);
  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = goal_pose;

  navigation_active_ = true;
  waiting_for_result_ = true;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&WaypointNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);

  current_goal_future_ = navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

// OCR 결과 콜백
void WaypointNavigator::ocrResultCallback(const robot_msgs::msg::OCRResult::SharedPtr msg) {
  // 현재 요청에 대한 응답인지 확인
  if (!waiting_for_ocr_ || msg->request_id != current_request_id_) {
    return;
  }

  waiting_for_ocr_ = false;

  RCLCPP_INFO(this->get_logger(), "OCR 결과 수신: %s", msg->message.c_str());

  if (msg->item_found) {
    RCLCPP_INFO(this->get_logger(), "목표 물품 '%s'을(를) 발견했습니다!", target_item_.c_str());
    sendSearchResult("FOUND:" + target_item_);

    // 복귀 모드 활성화
    returning_home_ = true;
    current_waypoint_index_ = 6;  // 경유지 A (복귀용)

    RCLCPP_INFO(this->get_logger(), "복귀 모드 시작: 경유지 A를 거쳐 홈으로 복귀합니다...");
    navigateToNextWaypoint();
  } else {
    RCLCPP_INFO(this->get_logger(), "%s에서 물품을 찾지 못했습니다. 다음 위치로 이동합니다.", search_waypoints_[current_waypoint_index_].name.c_str());
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

// 웨이포인트에서 PoseStamped 메시지 생성
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

// 액션 목표 응답 콜백
void WaypointNavigator::goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "목표 거부됨");
    handleNavigationFailure();
  } else {
    RCLCPP_INFO(this->get_logger(), "목표 수락됨");
  }
}

// 액션 피드백 콜백
void WaypointNavigator::feedbackCallback(const NavigateGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const NavigateAction::Feedback> feedback) {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    auto current_pose = feedback->current_pose.pose;
    double target_x = search_waypoints_[current_waypoint_index_].x;
    double target_y = search_waypoints_[current_waypoint_index_].y;

    double dx = target_x - current_pose.position.x;
    double dy = target_y - current_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(this->get_logger(), "이동 중... 목표까지 거리: %.2fm", distance);
      last_log_time = now;
    }
  }
}

// 액션 결과 콜백
void WaypointNavigator::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  navigation_active_ = false;
  waiting_for_result_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    handleNavigationSuccess();
  } else {
    handleNavigationFailure();
  }
}

// 네비게이션 성공 시 호출되는 핸들러
void WaypointNavigator::handleNavigationSuccess() {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    const auto& waypoint = search_waypoints_[current_waypoint_index_];
    RCLCPP_INFO(this->get_logger(), "%s 도착 성공!", waypoint.name.c_str());

    // 복귀 모드가 아니고 물품 검색 위치인 경우에만 OCR 스캔
    if (!returning_home_ && waypoint.name.find("위치") != std::string::npos && waypoint.name != "시작 위치" && waypoint.name != "시작 위치 (귀환)" && waypoint.name != "경유 위치 A") {
      RCLCPP_INFO(this->get_logger(), "3초 대기 후 OCR 스캔 시작...");
      rclcpp::sleep_for(std::chrono::seconds(3));
      performOCRScan();
    } else {
      // 경유지이거나 복귀 모드인 경우
      RCLCPP_INFO(this->get_logger(), "경유지에서 3초 대기...");
      rclcpp::sleep_for(std::chrono::seconds(3));

      current_waypoint_index_++;

      // 마지막 웨이포인트(홈)에 도착했고 복귀 모드인 경우 미션 완료
      if (returning_home_ && current_waypoint_index_ >= search_waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "홈 복귀 완료! 미션 성공!");
        sendSearchResult("MISSION_COMPLETE");
        mission_active_ = false;
        returning_home_ = false;
        return;
      }

      navigateToNextWaypoint();
    }
  }
}

// 네비게이션 실패 시 호출되는 핸들러
void WaypointNavigator::handleNavigationFailure() {
  RCLCPP_WARN(this->get_logger(), "네비게이션 실패, 다음 거점으로 이동");
  current_waypoint_index_++;
  navigateToNextWaypoint();
}

// OCR 스캔 수행
void WaypointNavigator::performOCRScan() {
  if (!mission_active_ || current_waypoint_index_ >= search_waypoints_.size()) return;

  const auto& current_waypoint = search_waypoints_[current_waypoint_index_];

  RCLCPP_INFO(this->get_logger(), "%s에서 '%s' 검색 중...", current_waypoint.name.c_str(), target_item_.c_str());

  // OCR 요청 메시지 생성
  auto request_msg = robot_msgs::msg::OCRRequest();
  request_msg.target_item_id = target_item_;
  request_msg.current_location = current_waypoint.name;
  request_msg.location_index = static_cast<int32_t>(current_waypoint_index_);

  // 고유한 요청 ID 생성 (타임스탬프 사용)
  current_request_id_ = this->now().nanoseconds();
  request_msg.request_id = current_request_id_;

  // OCR 상태 설정
  waiting_for_ocr_ = true;
  ocr_start_time_ = std::chrono::steady_clock::now();

  // OCR 요청 발행
  ocr_request_pub_->publish(request_msg);

  RCLCPP_INFO(this->get_logger(), "OCR 요청 전송 (ID: %ld)", current_request_id_);
}

}  // namespace robot_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto navigator = std::make_shared<robot_navigation::WaypointNavigator>();

  RCLCPP_INFO(navigator->get_logger(), "웨이포인트 네비게이터가 준비되었습니다 (OCR 토픽 방식).");
  RCLCPP_INFO(navigator->get_logger(), "마스터 윈도우에서 물품 검색을 시작하세요!");

  rclcpp::spin(navigator);

  rclcpp::shutdown();
  return 0;
}