// =============================================================================
// src/qnode.cpp (LiftController 통합)
// =============================================================================
#include "../include/robot_master/qnode.hpp"

namespace robot_master {

QNode::QNode() : current_work_state_(WorkState::IDLE), target_item_("") {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_master");
  initPubSub();

  // LiftController 초기화
  lift_controller_ = std::make_shared<LiftController>(node);

  // 작업 타임아웃 타이머 설정 (15초)
  work_timeout_timer_ = new QTimer(this);
  work_timeout_timer_->setSingleShot(true);
  connect(work_timeout_timer_, &QTimer::timeout, this, &QNode::onWorkTimeout);

  this->start();
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
  pub_master = node->create_publisher<robot_msgs::msg::MasterMsg>("turtle_master", 100);
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  nav_client_ = node->create_client<robot_msgs::srv::NavigateToParcel>("navigate_to_parcel");
}

// 리프트 제어 메서드들 구현
void QNode::liftUp() {
  if (lift_controller_) {
    lift_controller_->moveUp();
    Q_EMIT logMessage("리프트 올림 명령 전송");
  } else {
    Q_EMIT logMessage("오류: 리프트 컨트롤러가 초기화되지 않았습니다");
  }
}

void QNode::liftDown() {
  if (lift_controller_) {
    lift_controller_->moveDown();
    Q_EMIT logMessage("리프트 내림 명령 전송");
  } else {
    Q_EMIT logMessage("오류: 리프트 컨트롤러가 초기화되지 않았습니다");
  }
}

void QNode::liftStop() {
  if (lift_controller_) {
    lift_controller_->stop();
    Q_EMIT logMessage("리프트 정지 명령 전송");
  } else {
    Q_EMIT logMessage("오류: 리프트 컨트롤러가 초기화되지 않았습니다");
  }
}

double QNode::getLiftHeight() {
  if (lift_controller_) {
    return lift_controller_->getCurrentHeight();
  }
  return 0.0;
}

void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다");
    return;
  }

  target_item_ = item;
  setState(WorkState::WORKING);

  // 네비게이션 시작을 위한 메시지 발행
  driving_.master_msg_.item = item;
  driving_.master_msg_.slam = true;
  driving_.master_msg_.qr = false;
  driving_.master_msg_.lift = false;

  Q_EMIT logMessage(QString("물품 검색 시작: %1").arg(QString::fromStdString(item)));

  // 15초 타임아웃 설정
  work_timeout_timer_->start(15000);
}

void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) {
    return;
  }

  work_timeout_timer_->stop();
  setState(WorkState::IDLE);
  target_item_ = "";

  // 모든 작업 중지
  driving_.master_msg_.slam = false;
  driving_.master_msg_.qr = false;
  driving_.master_msg_.lift = false;

  // 리프트도 정지
  liftStop();

  Q_EMIT logMessage("작업이 취소되었습니다");
}

void QNode::setState(WorkState new_state) {
  if (current_work_state_ != new_state) {
    current_work_state_ = new_state;
    Q_EMIT workStateChanged(static_cast<int>(new_state));
  }
}

void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!vision_msg) return;

  // QR 코드 감지 로그
  if (vision_msg->ocr_detected) {
    QString qr_msg = QString("QR Code 감지: %1").arg(QString::fromStdString(vision_msg->ocr_data));
    Q_EMIT logMessage(qr_msg);

    // 작업 중이고 타겟 아이템과 일치하는 경우
    if (current_work_state_ == WorkState::WORKING && vision_msg->ocr_data == target_item_) {
      Q_EMIT logMessage("목표 물품을 찾았습니다! 리프트 동작 시작");

      // 자동으로 리프트 올리기
      liftUp();

      // 리프트 동작을 위한 메시지 설정
      driving_.master_msg_.lift = true;
      driving_.master_msg_.slam = false;  // 네비게이션 중지

      // 작업 완료 처리 (3초 후)
      QTimer::singleShot(3000, [this]() {
        work_timeout_timer_->stop();
        setState(WorkState::COMPLETED);

        // 5초 후 IDLE로 복귀
        QTimer::singleShot(5000, [this]() {
          driving_.master_msg_.lift = false;
          liftStop();  // 리프트 정지
          setState(WorkState::IDLE);
          target_item_ = "";
          Q_EMIT logMessage("작업 완료 - 대기 상태로 복귀");
        });
      });
    }
  }
}

void QNode::onWorkTimeout() {
  Q_EMIT logMessage("작업 타임아웃 - 15초 내에 물품을 찾지 못했습니다");
  cancelTask();
}

void QNode::turtleRun() {
  driving_.go();
  pub_motor->publish(driving_.motor_value_);
  pub_master->publish(driving_.master_msg_);
}

void QNode::navigateToPosition(double x, double y, double yaw) {
  if (!nav_client_->wait_for_service(std::chrono::seconds(1))) {
    Q_EMIT logMessage("내비게이션 서비스를 사용할 수 없습니다");
    return;
  }

  auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
  request->x = x;
  request->y = y;
  request->yaw = yaw;

  Q_EMIT logMessage(QString("내비게이션 요청: x=%.2f, y=%.2f, yaw=%.2f").arg(x).arg(y).arg(yaw));

  auto future = nav_client_->async_send_request(request, [this](rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
    auto response = future.get();
    QString result_msg = QString("내비게이션 결과: %1").arg(response->success ? "성공" : "실패");
    if (!response->message.empty()) {
      result_msg += QString(" - %1").arg(QString::fromStdString(response->message));
    }
    Q_EMIT logMessage(result_msg);
  });
}

}  // namespace robot_master