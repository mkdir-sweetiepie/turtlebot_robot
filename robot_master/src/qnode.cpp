#include "../include/robot_master/qnode.hpp"

#include <QTimer>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_master {

QNode::QNode() : current_work_state_(WorkState::IDLE), target_item_(""), lift_performing_action_(false) {
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
  rclcpp::WallRate loop_rate(50);
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

  // 네비게이션 시스템과 통신용 (새로 추가)
  search_request_pub = node->create_publisher<std_msgs::msg::String>("item_search_request", 10);
  search_result_sub = node->create_subscription<std_msgs::msg::String>("item_search_result", 10, std::bind(&QNode::searchResultCallback, this, std::placeholders::_1));

  // 기존 OCR 서비스 (유지)
  ocr_scan_service_ = node->create_service<robot_msgs::srv::OCRScan>("ocr_scan_request", std::bind(&QNode::handleOCRScanRequest, this, std::placeholders::_1, std::placeholders::_2));
}

// 문자열 정규화 함수 (공백, 특수문자 제거, 소문자 변환)
std::string QNode::normalizeString(const std::string& str) {
  std::string result;
  for (char c : str) {
    if (std::isalnum(c)) {  // 영숫자만 유지
      result += std::tolower(c);
    }
  }
  return result;
}

// 문자열 유사도 계산 (Levenshtein distance 기반)
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

// 향상된 텍스트 매칭 함수
bool QNode::isTextMatch(const std::string& detected_text, const std::string& target_text, float confidence) {
  // 1. 정확한 매칭 (대소문자 무시)
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
    Q_EMIT logMessage(QString("부분 문자열 매칭: '%1' <-> '%2'").arg(QString::fromStdString(detected_text)).arg(QString::fromStdString(target_text)));
    return true;
  }

  // 3. 정규화된 문자열 비교 (공백, 특수문자 제거)
  std::string normalized_detected = normalizeString(detected_text);
  std::string normalized_target = normalizeString(target_text);

  if (!normalized_detected.empty() && !normalized_target.empty()) {
    if (normalized_detected == normalized_target) {
      Q_EMIT logMessage(QString("정규화된 매칭: '%1' -> '%2' == '%3' -> '%4'")
                            .arg(QString::fromStdString(detected_text))
                            .arg(QString::fromStdString(normalized_detected))
                            .arg(QString::fromStdString(target_text))
                            .arg(QString::fromStdString(normalized_target)));
      return true;
    }

    // 4. 유사도 기반 매칭 (85% 이상 + 높은 신뢰도)
    double similarity = calculateSimilarity(normalized_detected, normalized_target);

    Q_EMIT logMessage(QString("유사도 계산: '%1' vs '%2' = %.2f%% (신뢰도: %.1f%%)")
                          .arg(QString::fromStdString(normalized_detected))
                          .arg(QString::fromStdString(normalized_target))
                          .arg(similarity * 100)
                          .arg(confidence * 100));

    if (similarity >= 0.85 && confidence >= 0.7) {
      Q_EMIT logMessage(QString("유사도 매칭 성공: %.2f%% (임계값: 85%%)").arg(similarity * 100));
      return true;
    }
  }

  return false;
}

void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다.");
    return;
  }

  target_item_ = item;
  setState(WorkState::WORKING);

  Q_EMIT logMessage(QString("'%1' 물품 검색 작업을 시작합니다.").arg(QString::fromStdString(item)));

  // 네비게이션 시스템에 검색 요청 전송
  auto msg = std_msgs::msg::String();
  msg.data = "START:" + item;
  search_request_pub->publish(msg);

  Q_EMIT logMessage("네비게이션 시스템에 검색 요청을 전송했습니다.");
}

void QNode::searchResultCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string result = msg->data;

  if (result.find("FOUND:") == 0) {
    // 물품 발견됨
    std::string found_item = result.substr(6);  // "FOUND:" 제거
    Q_EMIT logMessage(QString("네비게이션 시스템에서 물품 '%1'을(를) 발견했다고 보고했습니다!").arg(QString::fromStdString(found_item)));

    // 자동 리프트 동작 시작
    QTimer::singleShot(1000, [this]() { performItemFoundActions(); });

  } else if (result.find("NOT_FOUND:") == 0) {
    // 물품 찾지 못함
    std::string item = result.substr(10);  // "NOT_FOUND:" 제거
    Q_EMIT logMessage(QString("모든 거점을 검색했지만 물품 '%1'을(를) 찾지 못했습니다.").arg(QString::fromStdString(item)));
    setState(WorkState::IDLE);

  } else if (result.find("MISSION_COMPLETE") == 0) {
    // 미션 완료
    Q_EMIT logMessage("물품 검색 미션이 완료되었습니다!");
    setState(WorkState::COMPLETED);
  }
}

void QNode::handleOCRScanRequest(const std::shared_ptr<robot_msgs::srv::OCRScan::Request> request, std::shared_ptr<robot_msgs::srv::OCRScan::Response> response) {
  RCLCPP_INFO(node->get_logger(), "OCR 스캔 요청: %s에서 '%s' 물품 검색", request->current_location.c_str(), request->target_item_id.c_str());

  // 타겟 아이템 설정 및 상태 업데이트
  target_item_ = request->target_item_id;
  current_scan_request_ = request;
  setState(WorkState::WORKING);

  Q_EMIT logMessage(QString("%1에서 '%2' 물품 OCR 스캔 시작").arg(QString::fromStdString(request->current_location)).arg(QString::fromStdString(request->target_item_id)));

  // OCR 스캔 시작 (vision 시스템이 자동으로 감지)
  ocr_scan_active_ = true;
  scan_response_ = response;

  // 스캔 시작 시간 기록
  scan_start_time_ = std::chrono::steady_clock::now();

  // 12초 후 타임아웃 처리 (더 여유있게)
  QTimer::singleShot(12000, [this]() {
    if (ocr_scan_active_) {
      finishOCRScan(false, "", 0.0f, "OCR 스캔 타임아웃");
    }
  });
}

void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!ocr_scan_active_ || !vision_msg) return;

  QString detected_text = QString::fromStdString(vision_msg->ocr_text);
  float confidence = vision_msg->confidence;

  if (vision_msg->ocr_detected) {
    Q_EMIT logMessage(QString("OCR 감지됨 - 텍스트: '%1', 신뢰도: %2%%").arg(detected_text).arg(QString::number(confidence * 100, 'f', 1)));

    // 향상된 텍스트 매칭 사용
    if (isTextMatch(vision_msg->ocr_text, target_item_, confidence)) {
      Q_EMIT logMessage(QString("목표 물품 매칭 성공! 감지: '%1', 목표: '%2'").arg(detected_text).arg(QString::fromStdString(target_item_)));
      finishOCRScan(true, vision_msg->ocr_text, confidence, "목표 물품 발견");
    } else {
      // 매칭 실패했지만 계속 시도 (타임아웃까지)
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - scan_start_time_).count();

      Q_EMIT logMessage(QString("물품 불일치 ('%1' != '%2'), 계속 스캔 중... (%3초 경과)").arg(detected_text).arg(QString::fromStdString(target_item_)).arg(elapsed));
      // OCR 스캔을 즉시 종료하지 않고 계속 시도
    }
  } else {
    // 물품이 감지되지 않았지만 즉시 실패로 처리하지 않음
    Q_EMIT logMessage("물품 감지 안됨, 계속 스캔 중...");
  }
}

void QNode::finishOCRScan(bool found, const std::string& detected_text, float confidence, const std::string& message) {
  if (!ocr_scan_active_) return;

  ocr_scan_active_ = false;

  if (scan_response_) {
    scan_response_->item_found = found;
    scan_response_->detected_text = detected_text;
    scan_response_->confidence = confidence;
    scan_response_->continue_search = !found;  // 찾지 못했을 때만 계속 검색
    scan_response_->message = message;

    Q_EMIT logMessage(QString("OCR 스캔 완료: %1").arg(QString::fromStdString(message)));
  }

  if (!found) {
    setState(WorkState::IDLE);
  }
}

void QNode::performItemFoundActions() {
  if (lift_performing_action_) return;

  lift_performing_action_ = true;
  Q_EMIT logMessage("물품 발견! 180도 회전 후 후진하여 리프트 동작 시작");

  const double ROTATION_SPEED = 0.5;
  const double ROTATION_DURATION = M_PI / ROTATION_SPEED;
  const double BACKWARD_SPEED = 0.1;
  const double BACKWARD_DURATION = 2.0;

  // 180도 회전
  geometry_msgs::msg::Twist twist;
  twist.angular.z = ROTATION_SPEED;
  pub_motor->publish(twist);

  QTimer::singleShot(ROTATION_DURATION * 1000, [this, BACKWARD_SPEED, BACKWARD_DURATION]() {
    Q_EMIT logMessage("회전 완료, 후진 시작");

    // 후진
    geometry_msgs::msg::Twist backward_twist;
    backward_twist.linear.x = -BACKWARD_SPEED;
    pub_motor->publish(backward_twist);

    QTimer::singleShot(BACKWARD_DURATION * 1000, [this]() {
      Q_EMIT logMessage("후진 완료, 정지 후 리프트 올림");

      // 정지
      geometry_msgs::msg::Twist stop_twist;
      pub_motor->publish(stop_twist);

      // 리프트 올림
      liftUp();

      QTimer::singleShot(3000, [this]() {
        Q_EMIT logMessage("리프트 동작 완료");
        liftStop();
        setState(WorkState::COMPLETED);
        lift_performing_action_ = false;
      });
    });
  });
}

void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;

  Q_EMIT logMessage("작업 취소 중...");

  // 네비게이션 시스템에 취소 요청 전송
  auto msg = std_msgs::msg::String();
  msg.data = "CANCEL";
  search_request_pub->publish(msg);

  ocr_scan_active_ = false;
  setState(WorkState::IDLE);
  target_item_ = "";
  lift_performing_action_ = false;

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
  driving_.go();
  pub_motor->publish(driving_.motor_value_);
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