#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <QTimer>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "robot_msgs/srv/navigate_to_parcel.hpp"

namespace robot_master {

class QNode : public QThread {
  Q_OBJECT

 public:
  // 작업 상태 정의
  enum class WorkState { IDLE, WORKING, COMPLETED };

  QNode();
  ~QNode();

  void startFindParcelTask(const std::string& item);  // 아이템 찾기 작업 시작
  void cancelTask();                                  // 작업 취소

  // 리프트 제어
  void liftUp();
  void liftDown();
  void liftStop();
  double getLiftHeight();

  RobotDriving driving_;                                             // 로봇 주행 제어
  WorkState getCurrentState() const { return current_work_state_; }  // 현재 작업 상태 조회

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();                      // 데이터 수신 시그널
  void logMessage(const QString& message);  // 로그 메시지 시그널
  void workStateChanged(int state);         // 작업 상태 변경 시그널

 protected:
  void run();

 private:
  std::shared_ptr<rclcpp::Node> node;

  // Sub Pub Service Client
  rclcpp::Publisher<robot_msgs::msg::MasterMsg>::SharedPtr pub_master;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;
  rclcpp::Client<robot_msgs::srv::NavigateToParcel>::SharedPtr nav_client_;
  std::shared_ptr<LiftController> lift_controller_;

  // 작업 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  size_t current_location_index_{0};
  std::vector<std::tuple<double, double, double>> locations_;
  bool at_location_{false};
  bool via_waypoint_{false};                                         // 경유점 경유 여부
  bool going_home_{false};                                           // 홈 복귀 중 여부
  const std::tuple<double, double, double> WAYPOINT{3.0, 0.0, 0.0};  // 경유점

  void initPubSub();                                                                  // 퍼블리셔, 서브스크라이버, 서비스 초기화
  void turtleRun();                                                                   // 로봇 주행 제어 루프
  void loadLocations();                                                               // 위치 목록 로드
  void navigateToNextLocation();                                                      // 다음 위치로 이동
  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);  // 비전 메시지 콜백
  void setState(WorkState new_state);                                                 // 작업 상태 변경
  void performItemFoundActions();                                                     // 물품 발견 시 동작 수행
  void navigateToHome();
  void sendNavigationRequest(double x, double y, double yaw, std::function<void(bool)> cb);  // 홈으로 복귀
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_