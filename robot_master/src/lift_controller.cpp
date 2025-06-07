#include "../include/robot_master/lift_controller.hpp"

namespace robot_master {

LiftController::LiftController(std::shared_ptr<rclcpp::Node> node) : node_(node), current_height_(0.0), current_command_(0.0), is_moving_up_(false), is_moving_down_(false) {
  lift_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_lift_vel", 10);

  // 실제 로봇용 높이 구독자 (사용 가능한 경우에만)
  lift_height_sub_ = node_->create_subscription<std_msgs::msg::Float32>("lift_height", 10, std::bind(&LiftController::heightCallback, this, std::placeholders::_1));

  // 시뮬레이션용 높이 업데이트 타이머 (50ms마다 실행)
  height_update_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LiftController::updateHeightSimulation, this));

  last_update_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "LiftController 초기화 완료 (하이브리드 모드)");
}

LiftController::~LiftController() { stop(); }

// --------------- 리프트 제어 메서드 --------------
void LiftController::moveUp() {
  if (current_height_ >= MAX_HEIGHT) {
    RCLCPP_WARN(node_->get_logger(), "리프트가 최대 높이에 도달했습니다: %.2fm", current_height_);
    stop();
    return;
  }

  current_command_ = LIFT_SPEED;
  is_moving_up_ = true;
  is_moving_down_ = false;
  last_update_time_ = std::chrono::steady_clock::now();

  publishLiftCommand(LIFT_SPEED);
  RCLCPP_DEBUG(node_->get_logger(), "리프트 올리기 시작 (현재 높이: %.2fm)", current_height_);
}

void LiftController::moveDown() {
  if (current_height_ <= MIN_HEIGHT) {
    RCLCPP_WARN(node_->get_logger(), "리프트가 최소 높이에 도달했습니다: %.2fm", current_height_);
    stop();
    return;
  }

  current_command_ = -LIFT_SPEED;
  is_moving_up_ = false;
  is_moving_down_ = true;
  last_update_time_ = std::chrono::steady_clock::now();

  publishLiftCommand(-LIFT_SPEED);
  RCLCPP_DEBUG(node_->get_logger(), "리프트 내리기 시작 (현재 높이: %.2fm)", current_height_);
}

void LiftController::stop() {
  current_command_ = 0.0;
  is_moving_up_ = false;
  is_moving_down_ = false;

  publishLiftCommand(0.0);
  RCLCPP_DEBUG(node_->get_logger(), "리프트 정지 (현재 높이: %.2fm)", current_height_);
}

// ---------------- 높이 피드백 처리 --------------
void LiftController::heightCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  // 실제 로봇에서 높이 센서 데이터를 받는 경우
  current_height_ = static_cast<double>(msg->data);

  // 한계 체크
  if (current_height_ >= MAX_HEIGHT && is_moving_up_) {
    stop();
  } else if (current_height_ <= MIN_HEIGHT && is_moving_down_) {
    stop();
  }

  RCLCPP_DEBUG(node_->get_logger(), "실제 리프트 높이 수신: %.3fm", current_height_);
}

// ---------------- 시뮬레이션용 높이 추정 --------------
void LiftController::updateHeightSimulation() {
  auto current_time = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration<double>(current_time - last_update_time_).count();

  // 리프트가 움직이고 있는 경우에만 높이 업데이트
  if (is_moving_up_ && current_command_ > 0) {
    current_height_ += LIFT_SPEED * dt;
    if (current_height_ >= MAX_HEIGHT) {
      current_height_ = MAX_HEIGHT;
      stop();
    }
  } else if (is_moving_down_ && current_command_ < 0) {
    current_height_ -= LIFT_SPEED * dt;
    if (current_height_ <= MIN_HEIGHT) {
      current_height_ = MIN_HEIGHT;
      stop();
    }
  }

  last_update_time_ = current_time;
}

// ---------------- 리프트 명령 발행 --------------
void LiftController::publishLiftCommand(double velocity) {
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = 0.0;
  msg->linear.y = 0.0;
  msg->linear.z = velocity;  // Z축을 리프트 움직임에 사용
  msg->angular.x = 0.0;
  msg->angular.y = 0.0;
  msg->angular.z = 0.0;

  lift_cmd_pub_->publish(std::move(msg));

  if (std::abs(velocity) > 0.001) {
    RCLCPP_DEBUG(node_->get_logger(), "리프트 명령 발행: %.2f m/s", velocity);
  }
}

}  // namespace robot_master