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
      precise_control_mode_(false) {
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
  // 기존 퍼블리셔/서브스크라이버
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));

  // 네비게이션 시스템과 통신용
  search_request_pub = node->create_publisher<std_msgs::msg::String>("item_search_request", 10);
  search_result_sub = node->create_subscription<std_msgs::msg::String>("item_search_result", 10, std::bind(&QNode::searchResultCallback, this, std::placeholders::_1));

  // OCR 토픽 통신
  ocr_request_sub_ = node->create_subscription<robot_msgs::msg::OCRRequest>("ocr_scan_request", 10, std::bind(&QNode::ocrRequestCallback, this, std::placeholders::_1));
  ocr_result_pub_ = node->create_publisher<robot_msgs::msg::OCRResult>("ocr_scan_result", 10);

  RCLCPP_INFO(node->get_logger(), "QNode 초기화 완료 (ROS2 타이머 방식)");
}

// 로봇 내비게이션 시작 전송
void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다.");
    return;
  }

  target_item_ = item;
  setState(WorkState::WORKING);
  navigation_mode_ = true;
  Q_EMIT logMessage(QString("'%1' 물품 검색 작업을 시작합니다.").arg(QString::fromStdString(item)));

  // 네비게이션 시스템에 검색 요청 전송
  auto msg = std_msgs::msg::String();
  msg.data = "START:" + item;
  search_request_pub->publish(msg);

  Q_EMIT logMessage("네비게이션 시스템에 검색 요청을 전송했습니다.");
}

// 네비게이션 시스템에서 검색 결과 수신
void QNode::ocrRequestCallback(const robot_msgs::msg::OCRRequest::SharedPtr msg) {
  RCLCPP_INFO(node->get_logger(), "OCR 요청 수신: %s에서 '%s' 검색 (ID: %ld)", msg->current_location.c_str(), msg->target_item_id.c_str(), msg->request_id);

  // 리프트 동작 중이면 요청 거부
  if (lift_performing_action_) {
    RCLCPP_WARN(node->get_logger(), "리프트 동작 중이므로 OCR 요청 거부 (ID: %ld)", msg->request_id);
    return;
  }

  // 새 OCR 요청 처리
  target_item_ = msg->target_item_id;
  current_request_id_ = msg->request_id;
  current_location_ = msg->current_location;
  setState(WorkState::WORKING);

  Q_EMIT logMessage(QString("%1에서 '%2' OCR 스캔 시작").arg(QString::fromStdString(msg->current_location)).arg(QString::fromStdString(msg->target_item_id)));

  // OCR 스캔 상태 설정
  scan_start_time_ = std::chrono::steady_clock::now();
  arrive = true;

  // ROS2 타이머로 타임아웃 설정 (10초)
  auto timeout_timer = node->create_wall_timer(std::chrono::seconds(10), [this]() {
    Q_EMIT logMessage("OCR 스캔 타임아웃 (10초)");
    sendOCRResult(false, "", 0.0f, "OCR 스캔 타임아웃");
  });

  Q_EMIT logMessage("비전 시스템에서 OCR 결과 대기 중...");
}

void QNode::searchResultCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string result = msg->data;

  if (result.find("FOUND:") == 0) {
    // 물품 발견됨
    std::string found_item = result.substr(6);
    Q_EMIT logMessage(QString("물품 '%1'을(를) 발견했습니다!").arg(QString::fromStdString(found_item)));

  } else if (result.find("NOT_FOUND:") == 0) {
    // 물품 찾지 못함
    std::string item = result.substr(10);
    Q_EMIT logMessage(QString("모든 거점을 검색했지만 물품 '%1'을(를) 찾지 못했습니다.").arg(QString::fromStdString(item)));
    // 네비게이션 모드 비활성화
    navigation_mode_ = false;
    setState(WorkState::IDLE);

  } else if (result.find("MISSION_COMPLETE") == 0) {
    // 미션 완료
    Q_EMIT logMessage("물품 검색 미션이 완료되었습니다!");
    navigation_mode_ = false;
    setState(WorkState::COMPLETED);
  }
}

// 비전 시스템에서 OCR 결과 수신
void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  // 중복 호출 방지를 위한 강화된 체크
  if (!vision_msg || lift_performing_action_) {
    return;  // 이미 정밀 제어가 진행 중이거나 OCR이 비활성화된 경우 무시
  }

  QString detected_text = QString::fromStdString(vision_msg->ocr_text);
  float confidence = vision_msg->confidence;

  if (vision_msg->ocr_detected) {
    Q_EMIT logMessage(QString("OCR 감지: '%1' (신뢰도: %2%%)").arg(detected_text).arg(QString::number(confidence * 100, 'f', 1)));

    // 텍스트 매칭 확인
    if (isTextMatch(vision_msg->ocr_text, target_item_, confidence)) {
      Q_EMIT logMessage(QString("목표 물품 매칭 성공!"));

      Q_EMIT logMessage("OCR 스캔 종료, 정밀 제어 시작");

      // 정밀 제어 시작
      if (arrive) performItemFoundActions();
      arrive = false;

    } else {
      Q_EMIT logMessage("물품 불일치, 계속 스캔 중...");
    }
  }
}

void QNode::performItemFoundActions() {
  // 이중 체크로 중복 호출 완전 방지
  if (lift_performing_action_) {
    Q_EMIT logMessage("정밀 제어가 이미 진행 중입니다. 중복 호출 방지됨.");
    return;
  }

  // 직접 정밀 제어 시작
  startPreciseControl();
}

// 직접 정밀 제어 시작 (ROS2 타이머 사용)
void QNode::startPreciseControl() {
  if (lift_performing_action_) {
    Q_EMIT logMessage("정밀 제어가 이미 진행 중입니다. 중복 호출 방지됨.");
    return;
  }

  lift_performing_action_ = true;
  precise_control_mode_ = true;  // ← 정밀 제어 모드 활성화
  precise_step_ = 0;
  precise_control_start_time_ = std::chrono::steady_clock::now();

  Q_EMIT logMessage("물품 발견! 정밀 제어 시퀀스 시작");
  Q_EMIT logMessage("1단계: 180도 회전 시작");

  RCLCPP_INFO(node->get_logger(), "정밀 제어 시퀀스 시작 - 1단계: 180도 회전");

  // 1단계: 180도 회전 시작
  navigation_mode_ = false;
  RobotDriving::start = true;   // ← 핵심: RobotDriving 활성화
  driving_.setSpeed(0.0, 1.0);  // 제자리 회전

  // 3.2초 후 다음 단계로 이동 (ROS2 타이머 사용)
  precise_control_timer_ = node->create_wall_timer(std::chrono::milliseconds(3370), std::bind(&QNode::executePreciseControlStep, this));
}

// 정밀 제어 단계별 실행 (ROS2 타이머 콜백)
void QNode::executePreciseControlStep() {
  // 타이머 정지
  if (precise_control_timer_) {
    precise_control_timer_->cancel();
    precise_control_timer_ = nullptr;
  }

  auto current_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - precise_control_start_time_).count();

  switch (precise_step_) {
    case 0: {
      // 1단계 완료: 180도 회전 완료, 2단계 시작
      driving_.setSpeed(0.0, 0.0);  // 회전 정지
      Q_EMIT logMessage("1단계 완료: 180도 회전 완료");
      Q_EMIT logMessage("2단계: 20cm 후진 시작");

      RCLCPP_INFO(node->get_logger(), "1단계 완료 - 2단계: 20cm 후진 시작");

      precise_step_ = 1;
      RobotDriving::start = true;    // ← 계속 활성화 상태 유지
      driving_.setSpeed(-0.1, 0.0);  // 후진

      // 2초 후 다음 단계로 이동 (ROS2 타이머)
      precise_control_timer_ = node->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&QNode::executePreciseControlStep, this));
      break;
    }

    case 1: {
      // 2단계 완료: 20cm 후진 완료, 3단계 시작
      driving_.setSpeed(0.0, 0.0);  // 후진 정지
      RobotDriving::start = false;  // ← 로봇 이동 정지

      Q_EMIT logMessage("2단계 완료: 20cm 후진 완료");
      Q_EMIT logMessage("3단계: 리프트 올리기 시작");

      RCLCPP_INFO(node->get_logger(), "2단계 완료 - 3단계: 리프트 올리기 시작");

      precise_step_ = 2;
      lift_controller_->moveUp();  // 리프트 올리기 시작

      // 리프트 상태를 주기적으로 체크 (100ms 간격)
      precise_control_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&QNode::executePreciseControlStep, this));
      break;
    }

    case 2: {
      // 3단계: 리프트 높이 체크
      double current_height = lift_controller_->getCurrentHeight();
      double max_height = 0.7;

      if (current_height >= max_height - 0.1) {
        // 타이머 정지
        if (precise_control_timer_) {
          precise_control_timer_->cancel();
          precise_control_timer_ = nullptr;
        }

        lift_controller_->stop();
        Q_EMIT logMessage(QString("3단계 완료: 리프트 최대 높이 도달 (%.2fm)").arg(current_height));
        Q_EMIT logMessage(QString("정밀 제어 완료! (총 소요시간: %.1f초)").arg(elapsed / 1000.0));

        RCLCPP_INFO(node->get_logger(), "정밀 제어 시퀀스 완료 - 리프트 높이: %.2fm", current_height);

        // 정밀 제어 완료
        precise_control_mode_ = false;  // ← 정밀 제어 모드 비활성화
        setState(WorkState::COMPLETED);
        lift_performing_action_ = false;

        // OCR 결과 전송
        sendOCRResult(true, target_item_, 1.0f, "목표 물품 발견 및 픽업 완료");
        return;
      }

      // 계속 체크 (타이머는 이미 설정되어 있으므로 추가 설정 불필요)
      Q_EMIT logMessage(QString("리프트 올리는 중... (현재: %.2fm / 목표: %.2fm)").arg(current_height).arg(max_height));
      break;
    }

    default:
      Q_EMIT logMessage("정밀 제어 오류: 알 수 없는 단계");
      completePreciseControlWithError("정밀 제어 단계 오류");
      break;
  }
}

// OCR 결과 전송
void QNode::sendOCRResult(bool found, const std::string& detected_text, float confidence, const std::string& message) {
  // 성공 결과는 정밀 제어 완료 후에만 전송 가능
  if (found && lift_performing_action_) {
    RCLCPP_INFO(node->get_logger(), "정밀 제어 완료 후 OCR 결과 전송 대기");
    return;
  }

  // 실패 결과는 OCR 스캔이 활성화된 경우에만 전송
  if (!found && !ocr_scan_active_) {
    RCLCPP_DEBUG(node->get_logger(), "OCR 스캔이 이미 비활성화됨, 결과 전송 취소");
    return;
  }

  ocr_scan_active_ = false;

  // OCR 결과 메시지 생성
  auto result_msg = robot_msgs::msg::OCRResult();
  result_msg.request_id = current_request_id_;
  result_msg.item_found = found;
  result_msg.detected_text = detected_text;
  result_msg.confidence = confidence;
  result_msg.target_item_id = target_item_;
  result_msg.current_location = current_location_;
  result_msg.message = message;

  // 결과 발행
  ocr_result_pub_->publish(result_msg);

  Q_EMIT logMessage(QString("OCR 결과 전송: %1").arg(QString::fromStdString(message)));

  RCLCPP_INFO(node->get_logger(), "OCR 결과 전송 완료 (ID: %ld, 발견: %s)", current_request_id_, found ? "예" : "아니오");

  if (!found) {
    setState(WorkState::IDLE);
  }
}

// 정밀 제어 오류 처리
void QNode::completePreciseControlWithError(const std::string& error_message) {
  // 타이머 정지
  if (precise_control_timer_) {
    precise_control_timer_->cancel();
    precise_control_timer_ = nullptr;
  }

  // 모든 동작 정지
  driving_.setSpeed(0.0, 0.0);
  RobotDriving::start = false;  // ← 로봇 드라이빙 비활성화
  lift_controller_->stop();

  precise_control_mode_ = false;  // ← 정밀 제어 모드 비활성화

  Q_EMIT logMessage(QString("정밀 제어 실패: %1").arg(QString::fromStdString(error_message)));

  setState(WorkState::IDLE);
  lift_performing_action_ = false;

  // 실패 OCR 결과 전송
  sendOCRResult(false, "", 0.0f, error_message);
}

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
  // 1. 정확한 매칭
  std::string detected_lower = detected_text;
  std::string target_lower = target_text;
  std::transform(detected_lower.begin(), detected_lower.end(), detected_lower.begin(), ::tolower);
  std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);

  if (detected_lower == target_lower) {
    Q_EMIT logMessage(QString("정확한 매칭: '%1' == '%2'").arg(QString::fromStdString(detected_text)).arg(QString::fromStdString(target_text)));
    return true;
  }

  // 2. 부분 문자열 매칭
  if (detected_lower.find(target_lower) != std::string::npos || target_lower.find(detected_lower) != std::string::npos) {
    Q_EMIT logMessage(QString("부분 매칭: '%1' <-> '%2'").arg(QString::fromStdString(detected_text)).arg(QString::fromStdString(target_text)));
    return true;
  }

  // 3. 정규화된 문자열 비교
  std::string normalized_detected = normalizeString(detected_text);
  std::string normalized_target = normalizeString(target_text);

  if (!normalized_detected.empty() && !normalized_target.empty()) {
    if (normalized_detected == normalized_target) {
      Q_EMIT logMessage(QString("정규화 매칭: '%1' -> '%2'").arg(QString::fromStdString(detected_text)).arg(QString::fromStdString(normalized_detected)));
      return true;
    }

    // 4. 유사도 기반 매칭
    double similarity = calculateSimilarity(normalized_detected, normalized_target);

    Q_EMIT logMessage(QString("유사도: '%1' vs '%2' = %.1f%% (신뢰도: %.1f%%)")
                          .arg(QString::fromStdString(normalized_detected))
                          .arg(QString::fromStdString(normalized_target))
                          .arg(similarity * 100)
                          .arg(confidence * 100));

    if (similarity >= 0.85 && confidence >= 0.7) {
      Q_EMIT logMessage(QString("유사도 매칭 성공: %.1f%%").arg(similarity * 100));
      return true;
    }
  }

  return false;
}

void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;

  Q_EMIT logMessage("작업 취소 중...");

  // ROS2 타이머 정지
  if (precise_control_timer_) {
    precise_control_timer_->cancel();
    precise_control_timer_ = nullptr;
  }

  // 네비게이션 시스템에 취소 요청 전송
  auto msg = std_msgs::msg::String();
  msg.data = "CANCEL";
  search_request_pub->publish(msg);

  navigation_mode_ = false;
  precise_control_mode_ = false;  // ← 정밀 제어 모드 비활성화

  // 상태 초기화
  ocr_scan_active_ = false;
  setState(WorkState::IDLE);
  target_item_ = "";
  lift_performing_action_ = false;
  precise_step_ = 0;
  RobotDriving::start = false;  // ← 로봇 드라이빙 비활성화

  // 로봇 정지
  geometry_msgs::msg::Twist stop_twist;
  pub_motor->publish(stop_twist);

  liftStop();
  Q_EMIT logMessage("작업이 취소되었습니다");
}

void QNode::setState(WorkState new_state) {
  if (current_work_state_ != new_state) {
    current_work_state_ = new_state;
    Q_EMIT workStateChanged(static_cast<int>(new_state));
  }
}

void QNode::turtleRun() {
  if (!navigation_mode_) {
    // 정밀 제어 모드에서는 go() 호출하지 않음
    if (precise_control_mode_) {
      pub_motor->publish(driving_.motor_value_);  // 직접 publish
    } else {
      driving_.go();
      pub_motor->publish(driving_.motor_value_);
    }
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

}  // namespace robot_master