#include "../include/robot_master/qnode.hpp"

#include <QTimer>
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
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  ocr_scan_service_ = node->create_service<robot_msgs::srv::OCRScan>("ocr_scan_request", std::bind(&QNode::handleOCRScanRequest, this, std::placeholders::_1, std::placeholders::_2));
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

  // 10초 후 타임아웃 처리
  QTimer::singleShot(10000, [this]() {
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
    Q_EMIT logMessage(QString("OCR 감지됨 - 텍스트: %1, 신뢰도: %2%").arg(detected_text).arg(QString::number(confidence * 100, 'f', 1)));

    if (detected_text.toStdString() == target_item_) {
      Q_EMIT logMessage(QString("목표 물품 '%1' 발견!").arg(detected_text));
      finishOCRScan(true, vision_msg->ocr_text, confidence, "목표 물품 발견");

      // 물품 발견 시 자동으로 리프트 동작 시작
      QTimer::singleShot(1000, [this]() { performItemFoundActions(); });
    } else {
      Q_EMIT logMessage(QString("다른 물품 감지됨 ('%1' != '%2')").arg(detected_text).arg(QString::fromStdString(target_item_)));
      finishOCRScan(false, vision_msg->ocr_text, confidence, "목표 물품 불일치");
    }
  } else {
    Q_EMIT logMessage("물품 감지 안됨");
    finishOCRScan(false, "", 0.0f, "물품 감지 실패");
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

void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다.");
    return;
  }

  target_item_ = item;
  Q_EMIT logMessage(QString("'%1' 물품 검색 작업 시작 - 네비게이션 노드에서 거점 탐색을 시작합니다").arg(QString::fromStdString(item)));
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