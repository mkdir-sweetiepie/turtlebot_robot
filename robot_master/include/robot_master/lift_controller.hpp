#ifndef ROBOT_MASTER_LIFT_CONTROLLER_HPP
#define ROBOT_MASTER_LIFT_CONTROLLER_HPP

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace robot_master {

class LiftController {
 public:
  explicit LiftController(std::shared_ptr<rclcpp::Node> node);
  ~LiftController();

  // 리프트 제어
  void moveUp();
  void moveDown();
  void stop();

  // 현재 높이 조회
  double getCurrentHeight() const { return current_height_; }

 private:
  // 리프트 속도 및 높이 설정
  static constexpr double LIFT_SPEED = 0.3;  // m/s
  static constexpr double MAX_HEIGHT = 0.7;  // 최대 높이 (미터)
  static constexpr double MIN_HEIGHT = 0.0;  // 최소 높이 (미터)

  double current_height_ = 0.0;
  double current_command_ = 0.0;

  // 시뮬레이션용 높이 추정
  std::chrono::steady_clock::time_point last_update_time_;
  bool is_moving_up_ = false;
  bool is_moving_down_ = false;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr lift_cmd_pub_;

  // 리프트 높이 구독자 (실제 로봇용)
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lift_height_sub_;

  // 높이 업데이트 타이머 (시뮬레이션용)
  rclcpp::TimerBase::SharedPtr height_update_timer_;

  // 높이 구독 콜백 (실제 로봇용)
  void heightCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // 높이 업데이트 (시뮬레이션용)
  void updateHeightSimulation();

  void publishLiftCommand(double velocity);
};

}  // namespace robot_master

#endif  // ROBOT_MASTER_LIFT_CONTROLLER_HPP