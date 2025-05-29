#include "../include/robot_master/qnode.hpp"

#include <yaml-cpp/yaml.h>

#include <QTimer>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_master {

QNode::QNode()
    : current_work_state_(WorkState::IDLE),
      target_item_(""),
      current_location_index_(0),
      at_location_(false),
      via_waypoint_(false),
      going_home_(false),
      navigation_active_(false),
      navigation_timeout_(false) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_master");

  loadLocations();
  initPubSub();
  lift_controller_ = std::make_shared<LiftController>(node);

  // OCR 타임아웃 타이머 초기화
  ocr_timeout_timer_ = std::make_shared<QTimer>();
  ocr_timeout_timer_->setSingleShot(true);
  connect(ocr_timeout_timer_.get(), &QTimer::timeout, this, [this]() {
    if (at_location_) {
      Q_EMIT logMessage("OCR 검사 시간 초과 - 다음 위치로 이동");
      at_location_ = false;
      current_location_index_++;
      navigateToNextLocation();
    }
  });

  // 네비게이션 타임아웃 타이머 초기화
  navigation_timeout_timer_ = std::make_shared<QTimer>();
  navigation_timeout_timer_->setSingleShot(true);
  connect(navigation_timeout_timer_.get(), &QTimer::timeout, this, [this]() {
    if (navigation_active_) {
      Q_EMIT logMessage("네비게이션 타임아웃 - 다음 위치로 시도");
      navigation_timeout_ = true;
      navigation_active_ = false;
      if (current_goal_handle_) {
        // 현재 목표 취소 시도
        navigate_client_->async_cancel_goal(current_goal_handle_);
      }
    }
  });

  start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(50);  // 20Hz → 50Hz로 증가
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
      RCLCPP_INFO(node->get_logger(), "검색 위치 로드: (%.2f, %.2f, %.2f)", x, y, yaw);
    }
    RCLCPP_INFO(node->get_logger(), "총 %zu개 검색 위치 로드 완료", locations_.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "parcel_locations.yaml 로드 실패: %s", e.what());
  }
}

void QNode::initPubSub() {
  pub_master = node->create_publisher<robot_msgs::msg::MasterMsg>("turtle_master", 100);
  pub_motor = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sub_vision = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));

  // Nav2 액션 클라이언트 초기화
  navigate_client_ = rclcpp_action::create_client<NavigateAction>(node, "navigate_to_pose");

  // 초기 위치 설정용 퍼블리셔
  initial_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  RCLCPP_INFO(node->get_logger(), "퍼블리셔 및 서브스크라이버 초기화 완료");
}

void QNode::setInitialPose(double x, double y, double z, double w) {
  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = node->now();

  initial_pose.pose.pose.position.x = x;
  initial_pose.pose.pose.position.y = y;
  initial_pose.pose.pose.position.z = 0.0;

  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = z;
  initial_pose.pose.pose.orientation.w = w;

  // 공분산 행렬 설정
  for (int i = 0; i < 36; i++) {
    initial_pose.pose.covariance[i] = 0.0;
  }
  initial_pose.pose.covariance[0] = 0.25;                  // x
  initial_pose.pose.covariance[7] = 0.25;                  // y
  initial_pose.pose.covariance[35] = 0.06853891945200942;  // yaw

  // 초기 위치 여러 번 발행
  for (int i = 0; i < 10; ++i) {
    initial_pose.header.stamp = node->now();
    initial_pose_pub_->publish(initial_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Q_EMIT logMessage(QString("초기 위치 설정 완료: (%.2f, %.2f)").arg(x).arg(y));
  RCLCPP_INFO(node->get_logger(), "초기 위치 설정 완료: (%.2f, %.2f)", x, y);
}

bool QNode::waitUntilNav2Active() {
  Q_EMIT logMessage("Nav2 시스템 활성화 대기 중...");
  RCLCPP_INFO(node->get_logger(), "Nav2 액션 서버 대기 중...");

  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(30))) {
    Q_EMIT logMessage("Nav2 액션 서버 활성화 실패 (30초 타임아웃)");
    RCLCPP_ERROR(node->get_logger(), "Nav2 액션 서버가 활성화되지 않았습니다!");
    return false;
  }

  Q_EMIT logMessage("Nav2 시스템 활성화 완료!");
  RCLCPP_INFO(node->get_logger(), "Nav2 시스템 활성화 완료!");
  return true;
}

void QNode::startFindParcelTask(const std::string& item) {
  RCLCPP_INFO(node->get_logger(), "물품 검색 작업 시작: %s", item.c_str());

  if (current_work_state_ != WorkState::IDLE) {
    Q_EMIT logMessage("다른 작업이 진행 중입니다. 현재 작업을 취소하고 다시 시도하세요.");
    return;
  }
  if (item.empty()) {
    Q_EMIT logMessage("물품 ID를 입력 후 작업을 시작하세요.");
    return;
  }
  if (locations_.empty()) {
    Q_EMIT logMessage("위치 정보가 없습니다. parcel_locations.yaml 파일을 확인하세요.");
    return;
  }

  target_item_ = item;
  current_location_index_ = 0;
  setState(WorkState::WORKING);
  via_waypoint_ = true;
  going_home_ = false;
  navigation_active_ = false;
  navigation_timeout_ = false;

  Q_EMIT logMessage(QString("'%1' 물품 검색을 시작합니다 (총 %2개 위치 검색)").arg(QString::fromStdString(item)).arg(locations_.size()));

  // 초기 위치 설정
  setInitialPose(0.0, 0.0, 0.0, 1.0);

  // Nav2 활성화 확인 후 네비게이션 시작
  QTimer::singleShot(3000, [this]() {
    if (waitUntilNav2Active()) {
      navigateToNextLocation();
    } else {
      Q_EMIT logMessage("Nav2 초기화 실패로 작업을 중단합니다.");
      setState(WorkState::IDLE);
    }
  });
}

void QNode::navigateToNextLocation() {
  if (navigation_active_) {
    RCLCPP_WARN(node->get_logger(), "이미 네비게이션이 진행 중입니다.");
    return;
  }

  if (current_work_state_ != WorkState::WORKING) {
    return;  // 작업이 취소된 경우
  }

  // 홈으로 복귀 중인 경우
  if (going_home_) {
    if (via_waypoint_) {
      Q_EMIT logMessage("홈 복귀 - 경유점으로 이동");
      auto [x, y, yaw] = WAYPOINT;
      navigateToWaypoint(x, y, yaw, [this](bool success) {
        if (success) {
          Q_EMIT logMessage("경유점 도착 완료");
          via_waypoint_ = false;
          navigateToNextLocation();
        } else {
          Q_EMIT logMessage("경유점 이동 실패");
          setState(WorkState::IDLE);
        }
      });
    } else {
      Q_EMIT logMessage("홈으로 최종 이동");
      navigateToWaypoint(0.0, 0.0, 0.0, [this](bool success) {
        if (success) {
          Q_EMIT logMessage("홈 복귀 완료! 물품 검색 작업 종료");
          liftStop();
          setState(WorkState::COMPLETED);
        } else {
          Q_EMIT logMessage("홈 복귀 실패");
          setState(WorkState::IDLE);
        }
      });
    }
    return;
  }

  // 첫 위치로 이동 전 경유점 경유
  if (current_location_index_ == 0 && via_waypoint_) {
    Q_EMIT logMessage("첫 번째 검색 위치로 가기 전 경유점 경유");
    auto [x, y, yaw] = WAYPOINT;
    navigateToWaypoint(x, y, yaw, [this](bool success) {
      if (success) {
        Q_EMIT logMessage("경유점 경유 완료");
        via_waypoint_ = false;
        navigateToNextLocation();
      } else {
        Q_EMIT logMessage("경유점 이동 실패");
        setState(WorkState::IDLE);
      }
    });
    return;
  }

  // 모든 위치 검색 완료 확인
  if (current_location_index_ >= locations_.size()) {
    Q_EMIT logMessage(QString("모든 %1개 위치에서 물품 '%2'을(를) 찾지 못했습니다").arg(locations_.size()).arg(QString::fromStdString(target_item_)));
    setState(WorkState::IDLE);
    target_item_ = "";
    return;
  }

  // 다음 검색 위치로 이동
  auto [x, y, yaw] = locations_[current_location_index_];
  Q_EMIT logMessage(QString("검색 위치 %1/%2로 이동: (%.2f, %.2f)").arg(current_location_index_ + 1).arg(locations_.size()).arg(x).arg(y));

  navigateToWaypoint(x, y, yaw, [this](bool success) {
    if (success) {
      Q_EMIT logMessage(QString("위치 %1 도착 - OCR 검사 시작").arg(current_location_index_ + 1));
      at_location_ = true;
      ocr_timeout_timer_->start(10000);  // 10초 OCR 타이머 시작
    } else {
      Q_EMIT logMessage(QString("위치 %1 이동 실패 - 다음 위치로 시도").arg(current_location_index_ + 1));
      current_location_index_++;
      navigateToNextLocation();
    }
  });
}

void QNode::navigateToWaypoint(double x, double y, double yaw, std::function<void(bool)> callback) {
  if (navigation_active_) {
    RCLCPP_WARN(node->get_logger(), "네비게이션이 이미 활성화되어 있습니다.");
    if (callback) callback(false);
    return;
  }

  if (current_work_state_ != WorkState::WORKING) {
    if (callback) callback(false);
    return;
  }

  navigation_active_ = true;
  navigation_timeout_ = false;
  current_navigation_callback_ = callback;

  RCLCPP_INFO(node->get_logger(), "네비게이션 시작: (%.2f, %.2f, %.2f)", x, y, yaw);

  auto goal_msg = NavigateAction::Goal();
  goal_msg.pose = createPoseFromCoordinates(x, y, yaw);

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&QNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&QNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&QNode::resultCallback, this, std::placeholders::_1);

  // 네비게이션 타임아웃 타이머 시작 (60초)
  navigation_timeout_timer_->start(60000);

  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

geometry_msgs::msg::PoseStamped QNode::createPoseFromCoordinates(double x, double y, double yaw) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  // 쿼터니언 변환
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = sy;
  pose.pose.orientation.w = cy;

  return pose;
}

void QNode::goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "네비게이션 목표 거부됨");
    Q_EMIT logMessage("네비게이션 목표가 거부되었습니다");
    navigation_active_ = false;
    navigation_timeout_timer_->stop();
    if (current_navigation_callback_) {
      current_navigation_callback_(false);
      current_navigation_callback_ = nullptr;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "네비게이션 목표 수락됨");
    current_goal_handle_ = goal_handle;
  }
}

void QNode::feedbackCallback(const NavigateGoalHandle::SharedPtr& /*goal_handle*/, const std::shared_ptr<const NavigateAction::Feedback> feedback) {
  if (navigation_timeout_) return;  // 타임아웃된 경우 피드백 무시

  auto current_pose = feedback->current_pose.pose;
  RCLCPP_DEBUG(node->get_logger(), "현재 위치: x=%.2f, y=%.2f", current_pose.position.x, current_pose.position.y);
}

void QNode::resultCallback(const NavigateGoalHandle::WrappedResult& result) {
  navigation_timeout_timer_->stop();

  if (navigation_timeout_) {
    // 타임아웃된 경우
    RCLCPP_WARN(node->get_logger(), "네비게이션 타임아웃으로 인한 결과 무시");
    navigation_active_ = false;
    current_goal_handle_.reset();
    if (current_navigation_callback_) {
      current_navigation_callback_(false);
      current_navigation_callback_ = nullptr;
    }
    return;
  }

  navigation_active_ = false;
  current_goal_handle_.reset();

  bool success = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "네비게이션 성공!");
      success = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node->get_logger(), "네비게이션 중단됨");
      Q_EMIT logMessage("네비게이션이 중단되었습니다");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "네비게이션 취소됨");
      Q_EMIT logMessage("네비게이션이 취소되었습니다");
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "네비게이션 알 수 없는 결과: %d", static_cast<int>(result.code));
      break;
  }

  if (current_navigation_callback_) {
    current_navigation_callback_(success);
    current_navigation_callback_ = nullptr;
  }
}

void QNode::visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg) {
  if (!vision_msg || !at_location_) return;

  // OCR 타이머 중지
  ocr_timeout_timer_->stop();
  at_location_ = false;

  if (vision_msg->ocr_detected) {
    QString detected_text = QString::fromStdString(vision_msg->ocr_text);
    float confidence = vision_msg->confidence;

    Q_EMIT logMessage(QString("OCR 감지됨 - 텍스트: %1, 신뢰도: %2%, 처리시간: %3ms").arg(detected_text).arg(QString::number(confidence * 100, 'f', 1)).arg(vision_msg->fps));

    if (detected_text.toStdString() == target_item_) {
      Q_EMIT logMessage(QString("목표 물품 '%1' 발견! 리프트 동작 시작").arg(detected_text));
      performItemFoundActions();
    } else {
      Q_EMIT logMessage(QString("다른 물품 감지됨 ('%1' != '%2'), 다음 위치로 이동").arg(detected_text).arg(QString::fromStdString(target_item_)));
      current_location_index_++;
      navigateToNextLocation();
    }
  } else {
    Q_EMIT logMessage(QString("위치 %1에서 물품 감지 안됨, 다음 위치로 이동").arg(current_location_index_ + 1));
    current_location_index_++;
    navigateToNextLocation();
  }
}

void QNode::performItemFoundActions() {
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
        Q_EMIT logMessage("리프트 동작 완료, 홈으로 복귀 시작");
        liftStop();
        navigateToHome();
      });
    });
  });
}

void QNode::navigateToHome() {
  Q_EMIT logMessage("물품 픽업 완료, 홈으로 복귀합니다");
  via_waypoint_ = true;
  going_home_ = true;
  navigateToNextLocation();
}

void QNode::cancelTask() {
  if (current_work_state_ == WorkState::IDLE) return;

  Q_EMIT logMessage("작업 취소 중...");

  // 타이머들 중지
  ocr_timeout_timer_->stop();
  navigation_timeout_timer_->stop();

  // 진행 중인 네비게이션 취소
  if (navigation_active_ && current_goal_handle_) {
    navigate_client_->async_cancel_goal(current_goal_handle_);
    navigation_active_ = false;
    current_goal_handle_.reset();
  }

  setState(WorkState::IDLE);
  target_item_ = "";
  current_location_index_ = 0;
  at_location_ = false;
  via_waypoint_ = false;
  going_home_ = false;
  navigation_timeout_ = false;
  current_navigation_callback_ = nullptr;

  driving_.master_msg_.slam = false;
  driving_.master_msg_.qr = false;
  driving_.master_msg_.lift = false;

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