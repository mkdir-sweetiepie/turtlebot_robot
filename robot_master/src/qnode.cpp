#include "../include/robot_master/qnode.hpp"

#include <QTimer>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_master {

QNode::QNode() : current_work_state_(WorkState::IDLE), target_item_(""), lift_performing_action_(false), ocr_scan_active_(false), performance(false) {
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


  // 네비게이션 시스템과 통신용
  search_request_pub = node->create_publisher<std_msgs::msg::String>("item_search_request", 10);
  search_result_sub = node->create_subscription<std_msgs::msg::String>("item_search_result", 10, std::bind(&QNode::searchResultCallback, this, std::placeholders::_1));

  // OCR 토픽 통신 (서비스 완전 제거)
  ocr_request_sub_ = node->create_subscription<robot_msgs::msg::OCRRequest>("ocr_scan_request", 10, std::bind(&QNode::ocrRequestCallback, this, std::placeholders::_1));
  ocr_result_pub_ = node->create_publisher<robot_msgs::msg::OCRResult>("ocr_scan_result", 10);

  RCLCPP_INFO(node->get_logger(), "QNode 초기화 완료 (OCR 토픽 방식)");
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
    std::string found_item = result.substr(6);
    Q_EMIT logMessage(QString("물품 '%1'을(를) 발견했습니다!").arg(QString::fromStdString(found_item)));

    // 1초 후 리프트 동작 시작
    QTimer::singleShot(1000, [this]() { performItemFoundActions(); });

  } else if (result.find("NOT_FOUND:") == 0) {
    // 물품 찾지 못함
    std::string item = result.substr(10);
    Q_EMIT logMessage(QString("모든 거점을 검색했지만 물품 '%1'을(를) 찾지 못했습니다.").arg(QString::fromStdString(item)));
    setState(WorkState::IDLE);

  } else if (result.find("MISSION_COMPLETE") == 0) {
    // 미션 완료
    Q_EMIT logMessage("물품 검색 미션이 완료되었습니다!");
    setState(WorkState::COMPLETED);
  }
}

void QNode::ocrRequestCallback(const robot_msgs::msg::OCRRequest::SharedPtr msg) {
  RCLCPP_INFO(node->get_logger(), "OCR 요청 수신: %s에서 '%s' 검색 (ID: %ld)", msg->current_location.c_str(), msg->target_item_id.c_str(), msg->request_id);

  // 이전 스캔이 진행 중이면 정리
  if (ocr_scan_active_) {
    RCLCPP_WARN(node->get_logger(), "이전 OCR 스캔 강제 종료");
    ocr_scan_active_ = false;
  }

  // 새 OCR 요청 처리
  target_item_ = msg->target_item_id;
  current_request_id_ = msg->request_id;
  current_location_ = msg->current_location;
  setState(WorkState::WORKING);

  Q_EMIT logMessage(QString("%1에서 '%2' OCR 스캔 시작").arg(QString::fromStdString(msg->current_location)).arg(QString::fromStdString(msg->target_item_id)));

  // OCR 스캔 상태 설정
  ocr_scan_active_ = true;
  scan_start_time_ = std::chrono::steady_clock::now();

  // 10초 타이머 설정
  QTimer::singleShot(30000, [this]() {
    if (ocr_scan_active_) {
      Q_EMIT logMessage("OCR 스캔 타임아웃 (10초)");

      sendOCRResult(false, "", 0.0f, "OCR 스캔 타임아웃");
    }
  });

  Q_EMIT logMessage("비전 시스템에서 OCR 결과 대기 중...");
}

void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!ocr_scan_active_ || !vision_msg) {
    return;
  }

  QString detected_text = QString::fromStdString(vision_msg->ocr_text);
  float confidence = vision_msg->confidence;

  if (vision_msg->ocr_detected) {
    Q_EMIT logMessage(QString("OCR 감지: '%1' (신뢰도: %2%%)").arg(detected_text).arg(QString::number(confidence * 100, 'f', 1)));

    // 텍스트 매칭 확인
    if (isTextMatch(vision_msg->ocr_text, target_item_, confidence)) {
      Q_EMIT logMessage(QString("목표 물품 매칭 성공!"));
      performItemFoundActions();
      if (performance) {
        sendOCRResult(true, vision_msg->ocr_text, confidence, "목표 물품 발견");
      }

    } else {
      Q_EMIT logMessage("물품 불일치, 계속 스캔 중...");
    }
  }
}

void QNode::sendOCRResult(bool found, const std::string& detected_text, float confidence, const std::string& message) {
  if (!ocr_scan_active_) {
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

void QNode::performItemFoundActions() {
  if (lift_performing_action_) return;

  lift_performing_action_ = true;
  RCLCPP_INFO(node->get_logger(), "물품 발견 후 리프트 동작 시작");
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

      QTimer::singleShot(1000, [this]() {
        Q_EMIT logMessage("리프트 동작 완료");
        liftStop();
        setState(WorkState::COMPLETED);
        lift_performing_action_ = false;
      });
    });
  });
  performance = true;
}

void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;

  Q_EMIT logMessage("작업 취소 중...");

  // 네비게이션 시스템에 취소 요청 전송
  auto msg = std_msgs::msg::String();
  msg.data = "CANCEL";
  search_request_pub->publish(msg);

  // 상태 초기화
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