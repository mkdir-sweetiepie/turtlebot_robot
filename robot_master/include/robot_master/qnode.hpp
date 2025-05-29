#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <QTimer>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"

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
  // ROS2 관련
  std::shared_ptr<rclcpp::Node> node;

  // Nav2 액션 클라이언트
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;

  // Publishers & Subscribers
  rclcpp::Publisher<robot_msgs::msg::MasterMsg>::SharedPtr pub_master;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

  std::shared_ptr<LiftController> lift_controller_;

  // 작업 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  size_t current_location_index_{0};
  std::vector<std::tuple<double, double, double>> locations_;
  bool at_location_{false};
  bool via_waypoint_{false};                                         // 경유점 경유 여부
  bool going_home_{false};                                           // 홈 복귀 중 여부
  bool navigation_active_{false};                                    // 네비게이션 활성 상태
  bool navigation_timeout_{false};                                   // 네비게이션 타임아웃 상태
  const std::tuple<double, double, double> WAYPOINT{3.0, 0.0, 0.0};  // 경유점

  // 타이머 관리
  std::shared_ptr<QTimer> ocr_timeout_timer_;
  std::shared_ptr<QTimer> navigation_timeout_timer_;

  // 네비게이션 관리
  NavigateGoalHandle::SharedPtr current_goal_handle_;
  std::function<void(bool)> current_navigation_callback_;

  // 메서드
  void initPubSub();                   // 퍼블리셔, 서브스크라이버 초기화
  void turtleRun();                    // 로봇 주행 제어 루프
  void loadLocations();                // 위치 목록 로드
  void setState(WorkState new_state);  // 작업 상태 변경

  // 네비게이션 관련 메서드 (개선된 비동기 방식)
  void setInitialPose(double x, double y, double z, double w);                                  // 초기 위치 설정
  bool waitUntilNav2Active();                                                                   // Nav2 활성화 대기
  void navigateToNextLocation();                                                                // 다음 위치로 이동
  void navigateToWaypoint(double x, double y, double yaw, std::function<void(bool)> callback);  // 웨이포인트로 이동
  geometry_msgs::msg::PoseStamped createPoseFromCoordinates(double x, double y, double yaw);    // 좌표에서 Pose 생성

  // Nav2 액션 콜백들
  void goalResponseCallback(const NavigateGoalHandle::SharedPtr& goal_handle);
  void feedbackCallback(const NavigateGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const NavigateAction::Feedback> feedback);
  void resultCallback(const NavigateGoalHandle::WrappedResult& result);

  // 비전 및 동작 관련
  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);  // 비전 메시지 콜백
  void performItemFoundActions();                                                     // 물품 발견 시 동작 수행
  void navigateToHome();                                                              // 홈으로 복귀
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_