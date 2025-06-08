#include "../include/robot_master/qnode.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_master {

QNode::QNode()
    : current_work_state_(WorkState::IDLE),
      target_item_(""),
      lift_performing_action_(false),
      ocr_scan_active_(false),
      precise_step_(0),
      navigation_mode_(false),
      arrive(false),
      action_in_progress_(false) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_master");

  initPubSub();
  lift_controller_ = std::make_shared<LiftController>(node);

  start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    turtleRun();
    Q_EMIT dataReceived();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::initPubSub() {
  // === 로봇 주행 제어 퍼블리셔/ 비전 서브스크라이버 통신 ===
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));

  // === 네비게이션 시스템과 통신 ===
  search_request_pub = node->create_publisher<std_msgs::msg::String>("item_search_request", 10);
  search_result_sub = node->create_subscription<std_msgs::msg::String>("item_search_result", 10, std::bind(&QNode::searchResultCallback, this, std::placeholders::_1));

  // === OCR 매칭 토픽 통신 ===
  ocr_request_sub_ = node->create_subscription<robot_msgs::msg::OCRRequest>("ocr_scan_request", 10, std::bind(&QNode::ocrRequestCallback, this, std::placeholders::_1));
  ocr_result_pub_ = node->create_publisher<robot_msgs::msg::OCRResult>("ocr_scan_result", 10);

  // === OCR 제어 추가 ===
  ocr_control_pub_ = node->create_publisher<std_msgs::msg::Bool>("ocr_control", 10);

  // === 통합 로그 시스템 ===
  system_log_pub_ = node->create_publisher<robot_msgs::msg::LogMessage>("system_log", 100);
  system_log_sub_ = node->create_subscription<robot_msgs::msg::LogMessage>("system_log", 100, std::bind(&QNode::systemLogCallback, this, std::placeholders::_1));

  // === Service 초기화 ===
  item_info_client_ = node->create_client<robot_msgs::srv::ItemInfo>("item_info_service");
  item_info_service_ = node->create_service<robot_msgs::srv::ItemInfo>("item_info_service", std::bind(&QNode::handleItemInfoServiceRequest, this, std::placeholders::_1, std::placeholders::_2));

  // === Action Server 초기화 ===
  precise_control_action_server_ =
      rclcpp_action::create_server<PreciseControlAction>(node, "precise_control", std::bind(&QNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                                                         std::bind(&QNode::handleCancel, this, std::placeholders::_1), std::bind(&QNode::handleAccepted, this, std::placeholders::_1));

  publishSystemLog("INFO", "QNode 초기화 완료 (액션 서버 포함)");
}

// ================ 통합 로그 시스템 ================
void QNode::publishSystemLog(const std::string& level, const std::string& message) {
  auto log_msg = robot_msgs::msg::LogMessage();
  log_msg.node_name = "master";
  log_msg.level = level;
  log_msg.message = message;
  log_msg.timestamp = node->now();

  system_log_pub_->publish(log_msg);

  // 마스터 자신의 로그는 바로 GUI에 표시
  Q_EMIT logMessage(QString("[MASTER] %1").arg(QString::fromStdString(message)));
}

void QNode::systemLogCallback(const robot_msgs::msg::LogMessage::SharedPtr msg) {
  // 다른 노드에서 온 로그만 GUI에 표시 (자신의 로그는 제외)
  if (msg->node_name != "master") {
    QString formatted_message = QString("[%1] %2").arg(QString::fromStdString(msg->node_name).toUpper()).arg(QString::fromStdString(msg->message));
    Q_EMIT logMessage(formatted_message);
  }
}

// ================ 물품 Service 구현 ================
void QNode::queryItemInfo(const std::string& item_id) {
  if (!item_info_client_->wait_for_service(std::chrono::seconds(5))) {
    publishSystemLog("WARN", "물품 정보 서비스를 찾을 수 없습니다.");
    return;
  }

  auto request = std::make_shared<robot_msgs::srv::ItemInfo::Request>();
  request->item_id = item_id;

  publishSystemLog("INFO", "물품 정보 조회 중: " + item_id);

  auto result_future = item_info_client_->async_send_request(request);

  auto future_result = result_future.wait_for(std::chrono::seconds(5));
  if (future_result == std::future_status::ready) {
    auto response = result_future.get();
    if (response->item_exists) {
      publishSystemLog("INFO", "물품 정보: " + response->description + " (" + response->category + ", " + response->storage_info + ")");
    } else {
      publishSystemLog("WARN", "물품 정보를 찾을 수 없습니다.");
    }
  } else {
    publishSystemLog("ERROR", "물품 정보 조회 타임아웃");
  }
}

void QNode::handleItemInfoServiceRequest(const std::shared_ptr<robot_msgs::srv::ItemInfo::Request> request, std::shared_ptr<robot_msgs::srv::ItemInfo::Response> response) {
  publishSystemLog("INFO", "물품 정보 요청 받음: " + request->item_id);

  std::map<std::string, std::tuple<std::string, std::string, float, std::string>> item_database = {{"초코프렌즈우유", std::make_tuple("초콜릿 맛 우유", "음료", 0.2f, "냉장보관 필요")},
                                                                                                   {"계란과자", std::make_tuple("달걀 모양 과자", "스낵", 0.15f, "상온보관 가능")},
                                                                                                   {"정우경", std::make_tuple("팀 리더", "사람", 163.1f, "홍지현한테 혼나는 사람")},
                                                                                                   {"홍지현", std::make_tuple("팀 멤버", "사람", 163.7f, "정우경한테 혼내는 사람")},
                                                                                                   {"음료수", std::make_tuple("일반 음료", "음료", 0.25f, "냉장보관 권장")},
                                                                                                   {"과자", std::make_tuple("일반 과자", "스낵", 0.1f, "상온보관 가능")}};

  auto it = item_database.find(request->item_id);
  if (it != item_database.end()) {
    response->item_exists = true;
    response->description = std::get<0>(it->second);
    response->category = std::get<1>(it->second);
    response->expected_size = std::get<2>(it->second);
    response->storage_info = std::get<3>(it->second);
  } else {
    response->item_exists = false;
    response->description = "";
    response->category = "";
    response->expected_size = 0.0f;
    response->storage_info = "";
  }

  publishSystemLog("INFO", "물품 정보 응답: " + request->item_id + " (존재: " + (response->item_exists ? "예" : "아니오") + ")");
}

// ================ 정밀 제어 Action 구현 ================
rclcpp_action::GoalResponse QNode::handleGoal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PreciseControlAction::Goal> goal) {
  publishSystemLog("INFO", "정밀 제어 액션 요청: " + goal->action_type);

  if (action_in_progress_) {
    publishSystemLog("WARN", "다른 액션이 진행 중입니다.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->action_type != "pickup_sequence" && goal->action_type != "dropoff_sequence") {
    publishSystemLog("WARN", "지원하지 않는 액션 타입: " + goal->action_type);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse QNode::handleCancel(const std::shared_ptr<GoalHandlePreciseControl> /*goal_handle*/) {
  publishSystemLog("INFO", "정밀 제어 액션 취소 요청");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void QNode::handleAccepted(const std::shared_ptr<GoalHandlePreciseControl> goal_handle) {
  current_goal_handle_ = goal_handle;
  action_in_progress_ = true;
  precise_control_start_time_ = std::chrono::steady_clock::now();

  publishSystemLog("INFO", "정밀 제어 액션 시작됨");

  // 액션을 별도 스레드에서 실행
  std::thread([this, goal_handle]() { executePreciseControlAction(goal_handle); }).detach();
}

void QNode::executePreciseControlAction(const std::shared_ptr<GoalHandlePreciseControl> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PreciseControlAction::Result>();

  publishSystemLog("INFO", "정밀 제어 액션 실행 시작: " + goal->action_type);

  auto start_time = std::chrono::steady_clock::now();

  try {
    // **중요: 네비게이션 모드 비활성화 및 드라이빙 활성화**
    navigation_mode_ = false;
    RobotDriving::start = true;

    // 1단계: 180도 회전
    publishSystemLog("INFO", "1단계: 180도 회전 시작");
    publishActionFeedback("180도 회전", 1, 0.0f);

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 1.0;  // 회전 속도

    // 3.37초 동안 180도 회전
    auto rotation_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - rotation_start).count();

      if (elapsed >= 3.37) break;  // 3.37초 후 종료

      float progress = std::min(elapsed / 3.37, 1.0);
      publishActionFeedback("180도 회전", 1, progress);

      pub_motor->publish(twist_msg);  // **직접 모터에 명령 전송**
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (goal_handle->is_canceling()) {
      twist_msg.angular.z = 0.0;
      pub_motor->publish(twist_msg);
      result->success = false;
      result->message = "액션이 취소되었습니다.";
      goal_handle->canceled(result);
      action_in_progress_ = false;
      RobotDriving::start = false;
      return;
    }

    // 회전 정지
    twist_msg.angular.z = 0.0;
    pub_motor->publish(twist_msg);
    publishSystemLog("INFO", "1단계 완료: 180도 회전 완료");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5초 대기

    // 2단계: 20cm 후진
    publishSystemLog("INFO", "2단계: 20cm 후진 시작");
    publishActionFeedback("20cm 후진", 2, 0.0f);

    twist_msg.linear.x = -0.1;  // 후진 속도
    twist_msg.angular.z = 0.0;

    // 2.3초 동안 후진
    auto backward_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - backward_start).count();

      if (elapsed >= 2.3) break;  // 2.3초 후 종료

      float progress = std::min(elapsed / 2.3, 1.0);
      publishActionFeedback("20cm 후진", 2, progress);

      pub_motor->publish(twist_msg);  // **직접 모터에 명령 전송**
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (goal_handle->is_canceling()) {
      twist_msg.linear.x = 0.0;
      pub_motor->publish(twist_msg);
      result->success = false;
      result->message = "액션이 취소되었습니다.";
      goal_handle->canceled(result);
      action_in_progress_ = false;
      RobotDriving::start = false;
      return;
    }

    // 후진 정지
    twist_msg.linear.x = 0.0;
    pub_motor->publish(twist_msg);
    RobotDriving::start = false;  // 드라이빙 비활성화
    publishSystemLog("INFO", "2단계 완료: 20cm 후진 완료");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5초 대기

    // 3단계: 리프트 동작
    publishSystemLog("INFO", "3단계: 리프트 동작 시작");
    publishActionFeedback("리프트 동작", 3, 0.0f);

    if (goal->action_type == "pickup_sequence") {
      lift_controller_->moveUp();
      publishSystemLog("INFO", "리프트 올리기 시작");
    } else {
      lift_controller_->moveDown();
      publishSystemLog("INFO", "리프트 내리기 시작");
    }

    // 1.0초 동안 리프트 동작
    auto lift_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - lift_start).count();

      if (elapsed >= 1.0) break;  // 1.0초 후 종료

      float progress = std::min(elapsed / 1.0, 1.0);
      publishActionFeedback("리프트 동작", 3, progress);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    lift_controller_->stop();
    publishSystemLog("INFO", "3단계 완료: 리프트 동작 완료");

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(end_time - start_time).count();

    result->success = true;
    result->message = "정밀 제어 시퀀스 완료";
    result->total_duration = static_cast<float>(elapsed);
    result->final_height = static_cast<float>(lift_controller_->getCurrentHeight());

    publishSystemLog("INFO", "정밀 제어 액션 완료 (" + std::to_string(elapsed) + "초)");

    goal_handle->succeed(result);

    // 정밀 제어 완료 후 상태 초기화
    lift_performing_action_ = false;

  } catch (const std::exception& e) {
    publishSystemLog("ERROR", "정밀 제어 액션 오류: " + std::string(e.what()));

    // 에러 시 모든 동작 정지
    auto stop_twist = geometry_msgs::msg::Twist();
    pub_motor->publish(stop_twist);
    RobotDriving::start = false;
    lift_controller_->stop();

    result->success = false;
    result->message = std::string("액션 실행 오류: ") + e.what();
    goal_handle->abort(result);

    lift_performing_action_ = false;
  }

  action_in_progress_ = false;
  current_goal_handle_ = nullptr;
  publishSystemLog("INFO", "정밀 제어 액션 스레드 종료");
}

void QNode::publishActionFeedback(const std::string& step, int step_num, float progress) {
  if (current_goal_handle_) {
    auto feedback = std::make_shared<PreciseControlAction::Feedback>();
    feedback->current_step = step;
    feedback->step_number = step_num;
    feedback->progress = progress;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - precise_control_start_time_).count();
    feedback->elapsed_time = static_cast<float>(elapsed);

    current_goal_handle_->publish_feedback(feedback);

    // 로그는 중요한 단계에서만 출력 (스팸 방지)
    if (step_num != 1 || (int)(progress * 100) % 25 == 0) {
      publishSystemLog("INFO", "액션 진행: " + step + " (" + std::to_string((int)(progress * 100)) + "%)");
    }
  }
}

// ================ 물품 입력 후 내비게이션 pub 코드 ================
void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    publishSystemLog("WARN", "다른 작업이 진행 중입니다.");
    return;
  }

  target_item_ = item;
  setState(WorkState::WORKING);
  navigation_mode_ = true;

  queryItemInfo(item);

  publishSystemLog("INFO", "'" + item + "' 물품 검색 작업을 시작합니다.");

  auto msg = std_msgs::msg::String();
  msg.data = "START:" + item;
  search_request_pub->publish(msg);

  publishSystemLog("INFO", "네비게이션 시스템에 검색 요청을 전송했습니다.");
}

// ================ 물품 발견 후 OCR 결과 전송 (순환 호출 방지) ================
void QNode::performItemFoundActions() {
  if (lift_performing_action_ || action_in_progress_) {
    publishSystemLog("WARN", "정밀 제어가 이미 진행 중입니다. 중복 호출 방지됨.");
    return;
  }

  publishSystemLog("INFO", "물품 발견! OCR 비활성화");

  // **중요: OCR 즉시 비활성화**
  auto ocr_control_msg = std_msgs::msg::Bool();
  ocr_control_msg.data = false;
  ocr_control_pub_->publish(ocr_control_msg);

  // **순환 호출 방지**: OCR 결과만 전송하고 액션은 네비게이션에서 직접 호출
  lift_performing_action_ = true;
  navigation_mode_ = false;

  // 네비게이션에게 물품 발견 알림
  auto result_msg = robot_msgs::msg::OCRResult();
  result_msg.request_id = current_request_id_;
  result_msg.item_found = true;
  result_msg.detected_text = target_item_;
  result_msg.confidence = 1.0f;
  result_msg.target_item_id = target_item_;
  result_msg.current_location = current_location_;
  result_msg.message = "목표 물품 발견";

  ocr_result_pub_->publish(result_msg);
  publishSystemLog("INFO", "네비게이션에게 물품 발견 알림 전송");
}

// ================ OCR 요청 내비게이션 콜백 ================
void QNode::ocrRequestCallback(const robot_msgs::msg::OCRRequest::SharedPtr msg) {
  publishSystemLog("INFO", "OCR 요청 수신: " + msg->current_location + "에서 '" + msg->target_item_id + "' 검색 (ID: " + std::to_string(msg->request_id) + ")");

  if (lift_performing_action_ || action_in_progress_) {
    publishSystemLog("WARN", "액션 진행 중이므로 OCR 요청 거부 (ID: " + std::to_string(msg->request_id) + ")");
    return;
  }

  target_item_ = msg->target_item_id;
  current_request_id_ = msg->request_id;
  current_location_ = msg->current_location;
  setState(WorkState::WORKING);

  publishSystemLog("INFO", msg->current_location + "에서 '" + msg->target_item_id + "' OCR 스캔 시작");

  scan_start_time_ = std::chrono::steady_clock::now();
  arrive = true;
  ocr_scan_active_ = true;

  // 10초 타임아웃 설정
  auto timeout_timer = node->create_wall_timer(std::chrono::seconds(10), [this]() {
    if (ocr_scan_active_) {
      publishSystemLog("WARN", "OCR 스캔 타임아웃 (10초)");
      sendOCRResult(false, "", 0.0f, "OCR 스캔 타임아웃");
    }
  });

  publishSystemLog("INFO", "비전 시스템에서 OCR 결과 대기 중...");
}

// ================ 검색 결과 내비게이션 sub 콜백 ================
void QNode::searchResultCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string result = msg->data;

  if (result.find("FOUND:") == 0) {
    std::string found_item = result.substr(6);
    publishSystemLog("INFO", "물품 '" + found_item + "'을(를) 발견했습니다!");

  } else if (result.find("NOT_FOUND:") == 0) {
    std::string item = result.substr(10);
    publishSystemLog("WARN", "모든 거점을 검색했지만 물품 '" + item + "'을(를) 찾지 못했습니다.");
    navigation_mode_ = false;
    setState(WorkState::IDLE);

  } else if (result.find("MISSION_COMPLETE") == 0) {
    publishSystemLog("INFO", "물품 검색 미션이 완료되었습니다!");
    navigation_mode_ = false;
    setState(WorkState::COMPLETED);
  }
}

// ================ OCR 결과 매칭 및 콜백 ================
void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!vision_msg || lift_performing_action_ || action_in_progress_) {
    return;
  }

  float confidence = vision_msg->confidence;

  if (vision_msg->ocr_detected) {
    publishSystemLog("INFO", "OCR 감지: '" + vision_msg->ocr_text + "' (신뢰도: " + std::to_string((int)(confidence * 100)) + "%%)");

    if (isTextMatch(vision_msg->ocr_text, target_item_, confidence)) {
      publishSystemLog("INFO", "목표 물품 매칭 성공!");
      publishSystemLog("INFO", "OCR 스캔 종료, 네비게이션에 알림 전송");

      if (arrive) performItemFoundActions();
      arrive = false;

    } else {
      publishSystemLog("INFO", "물품 불일치, 계속 스캔 중...");
    }
  }
}

// ================ OCR 결과 전송 ================
void QNode::sendOCRResult(bool found, const std::string& detected_text, float confidence, const std::string& message) {
  if (!found && !ocr_scan_active_) {
    publishSystemLog("DEBUG", "OCR 스캔이 이미 비활성화됨, 결과 전송 취소");
    return;
  }

  ocr_scan_active_ = false;

  auto result_msg = robot_msgs::msg::OCRResult();
  result_msg.request_id = current_request_id_;
  result_msg.item_found = found;
  result_msg.detected_text = detected_text;
  result_msg.confidence = confidence;
  result_msg.target_item_id = target_item_;
  result_msg.current_location = current_location_;
  result_msg.message = message;

  ocr_result_pub_->publish(result_msg);

  if (!found) {
    navigation_mode_ = true;  // 실패 시에만 네비게이션 모드 계속
    setState(WorkState::IDLE);
  }

  publishSystemLog("INFO", "OCR 결과 전송: " + message);
  publishSystemLog("INFO", "OCR 결과 전송 완료 (ID: " + std::to_string(current_request_id_) + ", 발견: " + (found ? "예" : "아니오") + ")");
}

// ================ 작업 취소 기능 ================
void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;

  publishSystemLog("INFO", "작업 취소 중...");

  // OCR 비활성화
  auto ocr_control_msg = std_msgs::msg::Bool();
  ocr_control_msg.data = false;
  ocr_control_pub_->publish(ocr_control_msg);

  // 액션 취소
  if (current_goal_handle_) {
    auto cancel_result = std::make_shared<PreciseControlAction::Result>();
    cancel_result->success = false;
    cancel_result->message = "사용자에 의해 취소됨";
    current_goal_handle_->canceled(cancel_result);
    current_goal_handle_ = nullptr;
    action_in_progress_ = false;
  }

  // 네비게이션 취소
  auto msg = std_msgs::msg::String();
  msg.data = "CANCEL";
  search_request_pub->publish(msg);

  // 상태 초기화
  navigation_mode_ = false;
  ocr_scan_active_ = false;
  setState(WorkState::IDLE);
  target_item_ = "";
  lift_performing_action_ = false;
  precise_step_ = 0;
  RobotDriving::start = false;

  // 로봇 정지
  geometry_msgs::msg::Twist stop_twist;
  pub_motor->publish(stop_twist);

  liftStop();
  publishSystemLog("INFO", "작업이 취소되었습니다");
}

// ================ 작업 상태 관리 ================
void QNode::setState(WorkState new_state) {
  if (current_work_state_ != new_state) {
    current_work_state_ = new_state;
    Q_EMIT workStateChanged(static_cast<int>(new_state));
  }
}

// ================ 로봇 주행 및 리프트 제어 ================
void QNode::turtleRun() {
  // **중요: 액션 진행 중일 때는 turtleRun에서 제어하지 않음**
  if (!navigation_mode_ && !action_in_progress_) {
    driving_.go();
    pub_motor->publish(driving_.motor_value_);
  }
}

void QNode::liftUp() {
  if (lift_controller_) {
    lift_controller_->moveUp();
  }
}

void QNode::liftDown() {
  if (lift_controller_) {
    lift_controller_->moveDown();
  }
}

void QNode::liftStop() {
  if (lift_controller_) {
    lift_controller_->stop();
  }
}

double QNode::getLiftHeight() {
  if (lift_controller_) {
    return lift_controller_->getCurrentHeight();
  }
  return 0.0;
}

// ================ 문자열 정규화 및 유사도 계산 ================
std::string QNode::normalizeString(const std::string& str) {
  std::string result;
  for (char c : str) {
    if (std::isalnum(c)) {
      result += std::tolower(c);
    }
  }
  return result;
}

double QNode::calculateSimilarity(const std::string& str1, const std::string& str2) {
  size_t len1 = str1.length();
  size_t len2 = str2.length();

  if (len1 == 0) return len2 == 0 ? 1.0 : 0.0;
  if (len2 == 0) return 0.0;

  std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1));

  for (size_t i = 0; i <= len1; i++) dp[i][0] = i;
  for (size_t j = 0; j <= len2; j++) dp[0][j] = j;

  for (size_t i = 1; i <= len1; i++) {
    for (size_t j = 1; j <= len2; j++) {
      if (str1[i - 1] == str2[j - 1]) {
        dp[i][j] = dp[i - 1][j - 1];
      } else {
        dp[i][j] = std::min({dp[i - 1][j], dp[i][j - 1], dp[i - 1][j - 1]}) + 1;
      }
    }
  }

  int max_len = std::max(len1, len2);
  return 1.0 - (double)dp[len1][len2] / max_len;
}

bool QNode::isTextMatch(const std::string& detected_text, const std::string& target_text, float confidence) {
  std::string detected_lower = detected_text;
  std::string target_lower = target_text;
  std::transform(detected_lower.begin(), detected_lower.end(), detected_lower.begin(), ::tolower);
  std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);

  if (detected_lower == target_lower) {
    publishSystemLog("INFO", "정확한 매칭: '" + detected_text + "' == '" + target_text + "'");
    return true;
  }

  if (detected_lower.find(target_lower) != std::string::npos || target_lower.find(detected_lower) != std::string::npos) {
    publishSystemLog("INFO", "부분 매칭: '" + detected_text + "' <-> '" + target_text + "'");
    return true;
  }

  std::string normalized_detected = normalizeString(detected_text);
  std::string normalized_target = normalizeString(target_text);

  if (!normalized_detected.empty() && !normalized_target.empty()) {
    if (normalized_detected == normalized_target) {
      publishSystemLog("INFO", "정규화 매칭: '" + detected_text + "' -> '" + normalized_detected + "'");
      return true;
    }

    double similarity = calculateSimilarity(normalized_detected, normalized_target);

    publishSystemLog(
        "INFO", "유사도: '" + normalized_detected + "' vs '" + normalized_target + "' = " + std::to_string((int)(similarity * 100)) + "%% (신뢰도: " + std::to_string((int)(confidence * 100)) + "%%)");

    if (similarity >= 0.85 && confidence >= 0.7) {
      publishSystemLog("INFO", "유사도 매칭 성공: " + std::to_string((int)(similarity * 100)) + "%%");
      return true;
    }
  }

  return false;
}

}  // namespace robot_master