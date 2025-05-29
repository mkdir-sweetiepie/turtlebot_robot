#include "../include/robot_master/qnode.hpp"

#include <yaml-cpp/yaml.h>

#include <QTimer>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace robot_master {

QNode::QNode() : current_work_state_(WorkState::IDLE), target_item_(""), current_location_index_(0), at_location_(false), via_waypoint_(false), going_home_(false) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_master");
  loadLocations();
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

void QNode::initPubSub() {
  pub_master = node->create_publisher<robot_msgs::msg::MasterMsg>("turtle_master", 100);
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  nav_client_ = node->create_client<robot_msgs::srv::NavigateToParcel>("navigate_to_parcel");
}

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
  via_waypoint_ = true;  // 첫 이동은 경유점부터
  going_home_ = false;
  navigateToNextLocation();
}

void QNode::navigateToNextLocation() {
  if (going_home_) {
    if (via_waypoint_) {
      sendNavigationRequest(std::get<0>(WAYPOINT), std::get<1>(WAYPOINT), std::get<2>(WAYPOINT), [this](bool success) {
        if (success) {
          via_waypoint_ = false;
          navigateToNextLocation();  // 홈으로 실제 이동
        } else {
          setState(WorkState::IDLE);
        }
      });
      return;
    } else {
      sendNavigationRequest(0.01, 0.0, 0.0, [this](bool success) {
        if (success) {
          Q_EMIT logMessage("홈으로 복귀 중");
          liftStop();
        } else {
          Q_EMIT logMessage("홈으로의 내비게이션 시작 실패");
        }
        setState(WorkState::IDLE);
      });
      return;
    }
  }

  if (current_location_index_ == 0 && via_waypoint_) {
    sendNavigationRequest(std::get<0>(WAYPOINT), std::get<1>(WAYPOINT), std::get<2>(WAYPOINT), [this](bool success) {
      if (success) {
        via_waypoint_ = false;
        navigateToNextLocation();  // 실제 첫 위치로 이동
      } else {
        setState(WorkState::IDLE);
      }
    });
    return;
  }

  if (current_location_index_ >= locations_.size()) {
    Q_EMIT logMessage(QString("Item %1을(를) 모든 위치에서 찾지 못했습니다").arg(QString::fromStdString(target_item_)));
    setState(WorkState::IDLE);
    target_item_ = "";
    return;
  }

  auto [x, y, yaw] = locations_[current_location_index_];
  sendNavigationRequest(x, y, yaw, [this](bool success) {
    if (success) {
      at_location_ = true;
    } else {
      setState(WorkState::IDLE);
    }
  });
}

void QNode::sendNavigationRequest(double x, double y, double yaw, std::function<void(bool)> cb) {
  if (!nav_client_->wait_for_service(std::chrono::seconds(2))) {
    Q_EMIT logMessage("내비게이션 서비스를 사용할 수 없습니다");
    setState(WorkState::IDLE);
    cb(false);
    return;
  }
  auto request = std::make_shared<robot_msgs::srv::NavigateToParcel::Request>();
  request->x = x;
  request->y = y;
  request->yaw = yaw;
  nav_client_->async_send_request(request, [this, cb](rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedFuture future) {
    auto response = future.get();
    QString msg = QString("내비게이션 %1: %2").arg(response->success ? "시작됨" : "실패").arg(QString::fromStdString(response->message));
    Q_EMIT logMessage(msg);
    cb(response->success);
  });
}

void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!vision_msg || !at_location_) return;
  at_location_ = false;

  if (vision_msg->ocr_detected) {
    QString detected_text = QString::fromStdString(vision_msg->ocr_text);
    float confidence = vision_msg->confidence;

    Q_EMIT logMessage(QString("OCR 감지됨 - 텍스트: %1, 신뢰도: %2, FPS: %3ms").arg(detected_text).arg(confidence, 0, 'f', 2).arg(vision_msg->fps));

    if (detected_text.toStdString() == target_item_) {
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

void QNode::performItemFoundActions() {
  const double ROTATION_SPEED = 0.5;
  const double ROTATION_DURATION = M_PI / ROTATION_SPEED;
  const double BACKWARD_SPEED = 0.1;
  const double BACKWARD_DURATION = 2.0;

  geometry_msgs::msg::Twist twist;
  twist.angular.z = ROTATION_SPEED;
  pub_motor->publish(twist);

  QTimer::singleShot(ROTATION_DURATION * 1000, [this, BACKWARD_SPEED, BACKWARD_DURATION]() {
    geometry_msgs::msg::Twist backward_twist;
    backward_twist.linear.x = -BACKWARD_SPEED;
    pub_motor->publish(backward_twist);
    QTimer::singleShot(BACKWARD_DURATION * 1000, [this]() {
      geometry_msgs::msg::Twist stop_twist;
      pub_motor->publish(stop_twist);
      liftUp();
      QTimer::singleShot(3000, [this]() {
        liftStop();
        navigateToHome();
      });
    });
  });
}

void QNode::navigateToHome() {
  via_waypoint_ = true;
  going_home_ = true;
  navigateToNextLocation();
}

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

void QNode::liftUp() {
  if (lift_controller_) {
    lift_controller_->moveUp();
    RCLCPP_INFO(node->get_logger(), "리프트 올리는 중");
  }
}

void QNode::liftDown() {
  if (lift_controller_) {
    lift_controller_->moveDown();
    RCLCPP_INFO(node->get_logger(), "리프트 내리는 중");
  }
}

void QNode::liftStop() {
  if (lift_controller_) {
    lift_controller_->stop();
    RCLCPP_INFO(node->get_logger(), "리프트 중지");
  }
}

double QNode::getLiftHeight() {
  if (lift_controller_) {
    return lift_controller_->getCurrentHeight();
  }
  return 0.0;
}

}  // namespace robot_master
