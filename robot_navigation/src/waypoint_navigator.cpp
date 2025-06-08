/**
 * @file waypoint_navigator.cpp
 * @brief 물품 검색 통합 웨이포인트 네비게이터
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
      returning_home_(false),
      ocr_active_(false),
      precise_control_active_(false) {
  // === 액션 클라이언트 초기화 ===
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(this, "navigate_to_pose");
  precise_control_client_ = rclcpp_action::create_client<PreciseControlAction>(this, "precise_control");

  // === OCR 제어 토픽 추가 ===
  ocr_control_pub_ = this->create_publisher<std_msgs::msg::Bool>("ocr_control", 10);
  ocr_request_pub_ = this->create_publisher<robot_msgs::msg::OCRRequest>("ocr_scan_request", 10);
  ocr_result_sub_ = this->create_subscription<robot_msgs::msg::OCRResult>("ocr_scan_result", 10, std::bind(&WaypointNavigator::ocrResultCallback, this, std::placeholders::_1));

  // === 마스터와 통신용 퍼블리셔/서브스크라이버 ===
  search_result_pub_ = this->create_publisher<std_msgs::msg::String>("item_search_result", 10);
  search_request_sub_ = this->create_subscription<std_msgs::msg::String>("item_search_request", 10, std::bind(&WaypointNavigator::searchRequestCallback, this, std::placeholders::_1));

  // === 통합 로그 시스템 ===
  system_log_pub_ = this->create_publisher<robot_msgs::msg::LogMessage>("system_log", 100);

  // 검색용 웨이포인트 초기화
  initializeSearchWaypoints();

  // 상태 체크 타이머 (비동기 처리용)
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointNavigator::checkStatus, this));

  publishSystemLog("INFO", "물품 검색 웨이포인트 네비게이터가 시작되었습니다 (OCR 자동 제어).");
  publishSystemLog("INFO", "마스터 윈도우에서 물품 검색 요청을 기다리고 있습니다...");
}

// ================ OCR 제어 함수들 ================
void WaypointNavigator::enableOCR() {
  if (!ocr_active_) {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    ocr_control_pub_->publish(msg);
    ocr_active_ = true;
    publishSystemLog("INFO", "OCR 시스템 활성화");
  }
}

void WaypointNavigator::disableOCR() {
  if (ocr_active_) {
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    ocr_control_pub_->publish(msg);
    ocr_active_ = false;
    publishSystemLog("INFO", "OCR 시스템 비활성화");
  }
}

// ================ 통합 로그 시스템 ================
void WaypointNavigator::publishSystemLog(const std::string& level, const std::string& message) {
  auto log_msg = robot_msgs::msg::LogMessage();
  log_msg.node_name = "navigation";
  log_msg.level = level;
  log_msg.message = message;
  log_msg.timestamp = this->now();
  system_log_pub_->publish(log_msg);
}

// ================ (N->O) 목표지점 도착 후 OCR 활성화 및 스캔 시작 ================
void WaypointNavigator::performOCRScan() {
  if (!mission_active_ || current_waypoint_index_ >= search_waypoints_.size()) return;

  const auto& current_waypoint = search_waypoints_[current_waypoint_index_];

  publishSystemLog("INFO", current_waypoint.name + "에서 '" + target_item_ + "' 검색 시작...");

  // 2초 대기 후 OCR 요청 전송
  rclcpp::sleep_for(std::chrono::seconds(2));

  // OCR 스캔 요청
  auto request_msg = robot_msgs::msg::OCRRequest();
  request_msg.target_item_id = target_item_;
  request_msg.current_location = current_waypoint.name;
  request_msg.location_index = static_cast<int32_t>(current_waypoint_index_);

  current_request_id_ = this->now().nanoseconds();
  request_msg.request_id = current_request_id_;

  waiting_for_ocr_ = true;
  ocr_start_time_ = std::chrono::steady_clock::now();

  ocr_request_pub_->publish(request_msg);
  publishSystemLog("INFO", "OCR 스캔 요청 전송 (ID: " + std::to_string(current_request_id_) + ")");
  // 2초 대기 후 OCR 요청 전송
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // OCR 시스템 활성화
  enableOCR();
}

// ================ (N->M) 검색 결과 전송 ================
void WaypointNavigator::sendSearchResult(const std::string& result) {
  auto msg = std_msgs::msg::String();
  msg.data = result;
  search_result_pub_->publish(msg);
  publishSystemLog("INFO", "마스터에 결과 전송: " + result);
}

// ================ (O->N) OCR 결과 수신 및 자동 비활성화 ================
void WaypointNavigator::ocrResultCallback(const robot_msgs::msg::OCRResult::SharedPtr msg) {
  if (!waiting_for_ocr_ || msg->request_id != current_request_id_) {
    return;
  }

  waiting_for_ocr_ = false;
  publishSystemLog("INFO", "OCR 결과 수신: " + msg->message);

  // OCR 완료 시 즉시 비활성화
  disableOCR();

  if (msg->item_found) {
    publishSystemLog("INFO", "목표 물품 '" + target_item_ + "'을(를) 발견했습니다!");

    // OCR 완료 시 즉시 비활성화
    disableOCR();

    publishSystemLog("INFO", "정밀 제어 액션 시작...");
    callPreciseControlAction();

  } else {
    publishSystemLog("INFO", search_waypoints_[current_waypoint_index_].name + "에서 물품을 찾지 못했습니다.");

    // 3초 대기 후 다음 위치로 이동
    rclcpp::sleep_for(std::chrono::seconds(3));
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

// ================ 정밀 제어 액션 호출 ================
void WaypointNavigator::callPreciseControlAction() {
  if (precise_control_active_) {
    publishSystemLog("WARN", "정밀 제어 액션이 이미 진행 중입니다.");
    return;
  }

  if (!precise_control_client_->wait_for_action_server(std::chrono::seconds(5))) {
    publishSystemLog("ERROR", "정밀 제어 액션 서버를 찾을 수 없습니다!");
    // 실패 시 다음 위치로 이동
    current_waypoint_index_++;
    navigateToNextWaypoint();
    return;
  }

  auto goal_msg = PreciseControlAction::Goal();
  goal_msg.action_type = "pickup_sequence";

  precise_control_active_ = true;

  auto send_goal_options = rclcpp_action::Client<PreciseControlAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::preciseControlGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&WaypointNavigator::preciseControlFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&WaypointNavigator::preciseControlResultCallback, this, std::placeholders::_1);

  publishSystemLog("INFO", "정밀 제어 액션 요청 전송");
  precise_control_client_->async_send_goal(goal_msg, send_goal_options);
}

// ================ 정밀 제어 액션 콜백 함수들 ================
void WaypointNavigator::preciseControlGoalResponseCallback(const PreciseControlGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    publishSystemLog("ERROR", "정밀 제어 액션 목표가 거부되었습니다!");
    precise_control_active_ = false;
    current_waypoint_index_++;
    navigateToNextWaypoint();
  } else {
    publishSystemLog("INFO", "정밀 제어 액션 목표가 수락되었습니다.");
  }
}

void WaypointNavigator::preciseControlFeedbackCallback(const PreciseControlGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const PreciseControlAction::Feedback> feedback) {
  publishSystemLog("INFO", "정밀 제어 진행: " + feedback->current_step + " (" + std::to_string((int)(feedback->progress * 100)) + "%)");
}

void WaypointNavigator::preciseControlResultCallback(const PreciseControlGoalHandle::WrappedResult& result) {
  precise_control_active_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    publishSystemLog("INFO", "정밀 제어 액션 완료! 소요시간: " + std::to_string(result.result->total_duration) + "초");
    publishSystemLog("INFO", "리프트 최종 높이: " + std::to_string(result.result->final_height) + "m");

    sendSearchResult("FOUND:" + target_item_);

    returning_home_ = true;
    current_waypoint_index_ = 6;  // 경유지 A (복귀용)

    publishSystemLog("INFO", "복귀 모드 시작: 경유지 A를 거쳐 홈으로 복귀합니다...");

    // 3초 대기 후 복귀 시작
    rclcpp::sleep_for(std::chrono::seconds(3));
    navigateToNextWaypoint();

  } else {
    publishSystemLog("ERROR", "정밀 제어 액션 실패: " + result.result->message);
    // 실패 시 다음 위치로 이동
    current_waypoint_index_++;
    navigateToNextWaypoint();
  }
}

// ============================ 내비게이션 처리 ===========================

// ================ (M->N) 물품 입력 받음 master sub 콜백 ================
void WaypointNavigator::searchRequestCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string request = msg->data;

  if (request.find("START:") == 0) {
    target_item_ = request.substr(6);

    if (mission_active_) {
      publishSystemLog("WARN", "이미 미션이 진행 중입니다.");
      return;
    }

    publishSystemLog("INFO", "물품 '" + target_item_ + "' 검색 미션을 시작합니다!");

    mission_active_ = true;
    current_waypoint_index_ = 0;
    waiting_for_result_ = false;
    waiting_for_ocr_ = false;
    returning_home_ = false;

    // OCR 초기 비활성화
    disableOCR();

    setInitialPose();

    if (!waitForServices()) {
      mission_active_ = false;
      sendSearchResult("ERROR:서비스 연결 실패");
      return;
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
    navigateToNextWaypoint();

  } else if (request == "CANCEL") {
    publishSystemLog("INFO", "미션 취소 요청을 받았습니다.");
    disableOCR();                     // OCR 비활성화
    precise_control_active_ = false;  // 정밀 제어 액션 중단
    mission_active_ = false;
    navigation_active_ = false;
    waiting_for_result_ = false;
    waiting_for_ocr_ = false;
    returning_home_ = false;
  }
}

// ================ 초기 위치 설정 ================
void WaypointNavigator::initializeSearchWaypoints() {
  search_waypoints_.clear();

  search_waypoints_ = {{"시작 위치", 0.01, 0.0, 0.0},    {"경유 위치 A", 0.5, 0.0, 0.0},   {"위치 1", 0.5, 0.4, M_PI / 2},  {"위치 2", 0.9, 0.4, M_PI / 2},
                       {"위치 3", 0.9, -0.4, -M_PI / 2}, {"위치 4", 0.5, -0.4, -M_PI / 2}, {"경유 위치 A", 0.3, 0.0, M_PI}, {"시작 위치 (귀환)", 0.01, 0.0, M_PI}};

  publishSystemLog("INFO", "웨이포인트 " + std::to_string(search_waypoints_.size()) + "개가 설정되었습니다:");
  for (size_t i = 0; i < search_waypoints_.size(); i++) {
    const auto& wp = search_waypoints_[i];
    publishSystemLog("INFO", std::to_string(i + 1) + ". " + wp.name + ": (" + std::to_string(wp.x) + ", " + std::to_string(wp.y) + ", " + std::to_string(wp.yaw * 180.0 / M_PI) + "도)");
  }
}

// ================ 네비게이션 액션 서버 대기 ================
bool WaypointNavigator::waitForServices() {
  publishSystemLog("INFO", "Nav2 액션 서버를 기다리는 중...");

  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(10))) {
    publishSystemLog("ERROR", "Nav2 액션 서버를 찾을 수 없습니다!");
    return false;
  }

  publishSystemLog("INFO", "Nav2 액션 서버가 준비되었습니다.");

  publishSystemLog("INFO", "정밀 제어 액션 서버를 기다리는 중...");

  if (!precise_control_client_->wait_for_action_server(std::chrono::seconds(10))) {
    publishSystemLog("ERROR", "정밀 제어 액션 서버를 찾을 수 없습니다!");
    return false;
  }

  publishSystemLog("INFO", "정밀 제어 액션 서버가 준비되었습니다.");
  return true;
}

// ================ 네비게이션 상태 체크 ================
void WaypointNavigator::checkStatus() {
  if (!mission_active_) return;

  // 네비게이션 목표 응답 체크
  if (waiting_for_result_ && current_goal_future_.valid()) {
    auto status = current_goal_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      auto goal_handle = current_goal_future_.get();
      if (!goal_handle) {
        publishSystemLog("ERROR", "목표가 거부되었습니다!");
        handleNavigationFailure();
      }
    }
  }

  // OCR 타임아웃 체크 (30초)
  if (waiting_for_ocr_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - ocr_start_time_).count();

    if (elapsed >= 30) {
      publishSystemLog("WARN", "OCR 타임아웃 (30초), OCR 비활성화 후 다음 위치로 이동");
      waiting_for_ocr_ = false;
      disableOCR();  // 타임아웃 시에도 OCR 비활성화
      current_waypoint_index_++;
      navigateToNextWaypoint();
    }
  }
}

// ================ 초기 위치 세팅 ================
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

  publishSystemLog("INFO", "초기 위치가 설정되었습니다.");
}

// ================ 다음 웨이포인트로 이동 ================
void WaypointNavigator::navigateToNextWaypoint() {
  if (!mission_active_) return;

  if (current_waypoint_index_ >= search_waypoints_.size()) {
    publishSystemLog("INFO", "모든 거점에서 물품 '" + target_item_ + "'을(를) 찾지 못했습니다.");
    sendSearchResult("NOT_FOUND:" + target_item_);
    sendSearchResult("MISSION_COMPLETE");
    mission_active_ = false;
    return;
  }

  const auto& waypoint = search_waypoints_[current_waypoint_index_];
  publishSystemLog("INFO", waypoint.name + "로 이동 중... (" + std::to_string(current_waypoint_index_ + 1) + "/" + std::to_string(search_waypoints_.size()) + ")");

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

// ================ 웨이포인트에서 Pose 생성 ================
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

// ================ 네비게이션 콜백 함수 ================
void WaypointNavigator::goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    publishSystemLog("ERROR", "목표 거부됨");
    handleNavigationFailure();
  } else {
    publishSystemLog("INFO", "목표 수락됨");
  }
}

// ================ 피드백 콜백 함수 ================
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
      publishSystemLog("INFO", "이동 중... 목표까지 거리: " + std::to_string(distance) + "m");
      last_log_time = now;
    }
  }
}

// ================ 결과 콜백 함수 ================
void WaypointNavigator::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  navigation_active_ = false;
  waiting_for_result_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    handleNavigationSuccess();
  } else {
    handleNavigationFailure();
  }
}

// ================ 네비게이션 성공 처리 (OCR 자동 제어 포함) ================
void WaypointNavigator::handleNavigationSuccess() {
  if (current_waypoint_index_ < search_waypoints_.size()) {
    const auto& waypoint = search_waypoints_[current_waypoint_index_];
    publishSystemLog("INFO", waypoint.name + " 도착 성공!");

    // 복귀 모드가 아니고 물품 검색 위치인 경우에만 OCR 스캔
    if (!returning_home_ && waypoint.name.find("위치") != std::string::npos && waypoint.name != "시작 위치" && waypoint.name != "시작 위치 (귀환)" && waypoint.name != "경유 위치 A") {
      publishSystemLog("INFO", "3초 대기 후 OCR 시스템 활성화 및 스캔 시작...");
      rclcpp::sleep_for(std::chrono::seconds(3));
      performOCRScan();

    } else {
      publishSystemLog("INFO", "경유지에서 3초 대기...");
      rclcpp::sleep_for(std::chrono::seconds(3));

      current_waypoint_index_++;

      // 마지막 웨이포인트(홈)에 도착했고 복귀 모드인 경우 미션 완료
      if (returning_home_ && current_waypoint_index_ >= search_waypoints_.size()) {
        publishSystemLog("INFO", "홈 복귀 완료! 미션 성공!");
        sendSearchResult("MISSION_COMPLETE");
        mission_active_ = false;
        returning_home_ = false;
        disableOCR();  // 미션 완료 시 OCR 확실히 비활성화
        return;
      }

      navigateToNextWaypoint();
    }
  }
}

// ================ 네비게이션 실패 처리 ================
void WaypointNavigator::handleNavigationFailure() {
  publishSystemLog("WARN", "네비게이션 실패, OCR 비활성화 후 다음 거점으로 이동");
  disableOCR();  // 실패 시에도 OCR 비활성화
  current_waypoint_index_++;
  navigateToNextWaypoint();
}

}  // namespace robot_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto navigator = std::make_shared<robot_navigation::WaypointNavigator>();

  navigator->publishSystemLog("INFO", "웨이포인트 네비게이터가 준비되었습니다 (통합 로그 시스템).");
  navigator->publishSystemLog("INFO", "마스터 윈도우에서 물품 검색을 시작하세요!");

  rclcpp::spin(navigator);

  rclcpp::shutdown();
  return 0;
}