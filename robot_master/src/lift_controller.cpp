#include "../include/robot_master/lift_controller.hpp"

namespace robot_master {

LiftController::LiftController(std::shared_ptr<rclcpp::Node> node) : node_(node), current_height_(0.0), current_command_(0.0) {
  lift_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_lift_vel", 10);
}

LiftController::~LiftController() { stop(); }

// --------------- 리프트 제어 메서드 --------------
void LiftController::moveUp() {
  if (current_height_ >= MAX_HEIGHT) {
    stop();
    return;
  }

  current_command_ = LIFT_SPEED;
  publishLiftCommand(LIFT_SPEED);
}

void LiftController::moveDown() {
  if (current_height_ <= MIN_HEIGHT) {
    stop();
    return;
  }

  current_command_ = -LIFT_SPEED;
  publishLiftCommand(-LIFT_SPEED);
}

void LiftController::stop() {
  current_command_ = 0.0;
  publishLiftCommand(0.0);
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
}

}  // namespace robot_master