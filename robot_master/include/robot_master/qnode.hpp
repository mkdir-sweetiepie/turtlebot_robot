// =============================================================================
// include/robot_master/qnode.hpp (LiftController 통합)
// =============================================================================
#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <QTimer>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"  // LiftController 추가
#include "rclcpp/rclcpp.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "robot_msgs/srv/navigate_to_parcel.hpp"

namespace robot_master {

class QNode : public QThread {
  Q_OBJECT

 public:
  enum class WorkState { IDLE, WORKING, COMPLETED };

  QNode();
  ~QNode();

  void startFindParcelTask(const std::string& item);
  void cancelTask();

  void liftUp();
  void liftDown();
  void liftStop();
  double getLiftHeight();

  RobotDriving driving_;
  WorkState getCurrentState() const { return current_work_state_; }

 public Q_SLOTS:

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();
  void logMessage(const QString& message);
  void workStateChanged(int state);

 protected:
  void run();

 private:
  std::shared_ptr<rclcpp::Node> node;

  // Publishers
  rclcpp::Publisher<robot_msgs::msg::MasterMsg>::SharedPtr pub_master;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;

  // Subscribers
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

  // Service clients
  rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedPtr nav_client_;

  // LiftController
  std::shared_ptr<LiftController> lift_controller_;

  // 작업 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  QTimer* work_timeout_timer_;

  void initPubSub();
  void turtleRun();
  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);
  void navigateToPosition(double x, double y, double yaw);
  void setState(WorkState new_state);

 private Q_SLOTS:
  void onWorkTimeout();
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_