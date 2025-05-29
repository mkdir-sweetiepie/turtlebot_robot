#include "../include/robot_master/qnode.hpp"

#include <yaml-cpp/yaml.h>

#include <QTimer>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace robot_master {

QNode::QNode() : current_work_state_(WorkState::IDLE), target_item_(""), current_location_index_(0), at_location_(false) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_master");
  loadLocations();  // 위치 정보 로드
  initPubSub();     // 퍼블리셔와 서브스크라이버 초기화
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
    turtleRun();  // 로봇 동작 실행
    Q_EMIT dataReceived();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

// 위치 정보 로드
void QNode::loadLocations() {
  try {
    std::string config_path = ament_index_cpp::get_package_share_directory("robot_master") + "/config/parcel_locations.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    if (!config["locations"] || !config["locations"].IsSequence()) {
      RCLCPP_ERROR(node->get_logger(), "잘못된 parcel_locations.yaml 형식");
      return;
    }
    locations_.clear();
    for (const auto& loc : config["locations"]) {
      double x = loc["x"].as<double>();
      double y = loc["y"].as<double>();
      double yaw = loc["yaw"].as<double>();
      locations_.push_back({x, y, yaw});
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "parcel_locations.yaml 로드 실패: %s", e.what());
  }
}

// 퍼블리셔와 서브스크라이버 초기화
void QNode::initPubSub() {
  pub_master = node->create_publisher<robot_msgs::msg::MasterMsg>("turtle_master", 100);
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  nav_client_ = node->create_client<robot_msgs::srv::NavigateToParcel>("navigate_to_parcel");
}

// 물품 찾기 작업 시작
void QNode::startFindParcelTask(const std::string& item) {
  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다. 현재 작업을 취소하고 다시 시도하세요.");
    return;
  }
  if (item.empty()) {
    Q_EMIT logMessage("Item 입력 후 작업을 시작하세요.");
    return;
  }
  if (locations_.empty()) {
    Q_EMIT logMessage("위치 정보가 없습니다. parcel_locations.yaml 파일을 확인하세요.");
    return;
  }
  target_item_ = item;
  current_location_index_ = 0;
  setState(WorkState::WORKING);
  navigateToNextLocation();
}

// 다음 위치로 내비게이션
void QNode::navigateToNextLocation() {
  if (current_location_index_ >= locations_.size()) {
    Q_EMIT logMessage(QString("Item %1을(를) 모든 위치에서 찾지 못했습니다").arg(QString::fromStdString(target_item_)));
    setState(WorkState::IDLE);
    target_item_ = "";
    return;
  }
  if (!nav_client_->wait_for_service(std::chrono::seconds(2))) {
    Q_EMIT logMessage("내비게이션 서비스를 사용할 수 없습니다");
    setState(WorkState::IDLE);
    return;
  }
  auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
  request->x = std::get<0>(locations_[current_location_index_]);
  request->y = std::get<1>(locations_[current_location_index_]);
  request->yaw = std::get<2>(locations_[current_location_index_]);
  nav_client_->async_send_request(request, [this](rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
    auto response = future.get();
    QString msg = QString("위치 %1로 내비게이션 %2: %3").arg(current_location_index_ + 1).arg(response->success ? "시작됨" : "실패").arg(QString::fromStdString(response->message));
    Q_EMIT logMessage(msg);
    if (response->success) {
      at_location_ = true;  // 위치에 도착했음을 표시
    } else {
      setState(WorkState::IDLE);
    }
  });
  driving_.master_msg_.item = target_item_;
  driving_.master_msg_.slam = true;
}

// 비전 처리 콜백 함수
void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!vision_msg || !at_location_) return;  // 매개변수 이름 수정
  at_location_ = false;                      // 비전 데이터 처리 후 플래그 리셋
  
  if (vision_msg->ocr_detected) {
    QString detected_text = QString::fromStdString(vision_msg->ocr_text);
    float confidence = vision_msg->confidence;

    Q_EMIT logMessage(QString("OCR 감지됨 - 텍스트: %1, 신뢰도: %2, FPS: %3ms")
                      .arg(detected_text)
                      .arg(confidence, 0, 'f', 2)
                      .arg(vision_msg->fps));

    if (detected_text == target_item_) {
      Q_EMIT logMessage("목표 물품을 찾았습니다! 리프트 동작 시작");
      performItemFoundActions();
    } else {
      current_location_index_++;
      navigateToNextLocation();
    }
  } else {
    current_location_index_++;
    navigateToNextLocation();
  }
}

// 물품 발견 시 수행할 동작
void QNode::performItemFoundActions() {
  const double ROTATION_SPEED = 0.5;                       // rad/s
  const double ROTATION_DURATION = M_PI / ROTATION_SPEED;  // 180도 회전 시간
  const double BACKWARD_SPEED = 0.1;                       // m/s
  const double BACKWARD_DURATION = 2.0;                    // 후진 시간 (초)

  // 180도 회전
  geometry_msgs::msg::Twist twist;
  twist.angular.z = ROTATION_SPEED;
  pub_motor->publish(twist);

  QTimer::singleShot(ROTATION_DURATION * 1000, [this, BACKWARD_SPEED, BACKWARD_DURATION]() {
    geometry_msgs::msg::Twist backward_twist;
    backward_twist.linear.x = -BACKWARD_SPEED;  // 후진을 위한 선속도 설정 (음수)
    pub_motor->publish(backward_twist);         // 후진 명령 발행
    // 후진
    QTimer::singleShot(BACKWARD_DURATION * 1000, [this]() {
      geometry_msgs::msg::Twist stop_twist;  // 모든 속도를 0으로 초기화
      pub_motor->publish(stop_twist);        // 정지 명령 발행
      // 리프트 작업
      liftUp();
      QTimer::singleShot(3000, [this]() {
        liftStop();
        // 홈으로 복귀
        navigateToHome();
      });
    });
  });
}

// 홈으로 복귀
void QNode::navigateToHome() {
  auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
  request->x = 0.01;
  request->y = 0.0;
  request->yaw = 0.0;
  nav_client_->async_send_request(request, [this](rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      Q_EMIT logMessage("홈으로 복귀 중");
      liftStop();  // 홈에 도착하면 리프트 중지
    } else {
      Q_EMIT logMessage("홈으로의 내비게이션 시작 실패");
    }
    setState(WorkState::IDLE);
  });
}

// 작업 취소
void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;
  setState(WorkState::IDLE);
  target_item_ = "";
  current_location_index_ = 0;
  driving_.master_msg_.slam = false;
  driving_.master_msg_.qr = false;
  driving_.master_msg_.lift = false;
  liftStop();
  Q_EMIT logMessage("작업이 취소되었습니다");
}

// 상태 설정
void QNode::setState(WorkState new_state) {
  if (current_work_state_ != new_state) {
    current_work_state_ = new_state;
    Q_EMIT workStateChanged(static_cast<int>(new_state));
  }
}

void QNode::turtleRun() {
  driving_.go();
  pub_motor->publish(driving_.motor_value_);
  pub_master->publish(driving_.master_msg_);
}

// 리프트 올리기
void QNode::liftUp() {
  if (lift_controller_) {
    lift_controller_->moveUp();
    RCLCPP_INFO(node->get_logger(), "리프트 올리는 중");
  }
}

// 리프트 내리기
void QNode::liftDown() {
  if (lift_controller_) {
    lift_controller_->moveDown();
    RCLCPP_INFO(node->get_logger(), "리프트 내리는 중");
  }
}

// 리프트 중지
void QNode::liftStop() {
  if (lift_controller_) {
    lift_controller_->stop();
    RCLCPP_INFO(node->get_logger(), "리프트 중지");
  }
}

// 리프트 높이 확인
double QNode::getLiftHeight() {
  if (lift_controller_) {
    return lift_controller_->getCurrentHeight();
  }
  return 0.0;
}

}  // namespace robot_master
