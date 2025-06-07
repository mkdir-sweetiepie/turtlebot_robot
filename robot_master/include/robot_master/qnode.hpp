#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/ocr_request.hpp"
#include "robot_msgs/msg/ocr_result.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

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
  void performItemFoundActions();                     // 물품 발견 시 동작 수행

  // 리프트 제어
  void liftUp();
  void liftDown();
  void liftStop();
  double getLiftHeight();

  RobotDriving driving_;  // 로봇 주행 제어
  WorkState getCurrentState() const { return current_work_state_; }

 Q_SIGNALS:
  void rosShutDown();
  void dataReceived();
  void logMessage(const QString& message);
  void workStateChanged(int state);

 protected:
  void run();

 private:
  // 상태 관리
  int precise_step_;  // 0: 180도 회전, 1: 20cm 후진, 2: 리프트 올리기

  // ROS2 관련
  std::shared_ptr<rclcpp::Node> node;

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

  // 네비게이션 시스템과 통신용 퍼블리셔/서브스크라이버
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_request_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_result_sub;

  // OCR 토픽 통신
  rclcpp::Subscription<robot_msgs::msg::OCRRequest>::SharedPtr ocr_request_sub_;
  rclcpp::Publisher<robot_msgs::msg::OCRResult>::SharedPtr ocr_result_pub_;

  std::shared_ptr<LiftController> lift_controller_;

  // ROS2 타이머로 교체 (QTimer 대신)
  rclcpp::TimerBase::SharedPtr precise_control_timer_;
  std::chrono::steady_clock::time_point precise_control_start_time_;

  // 작업 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  bool lift_performing_action_;
  bool ocr_scan_active_;
  bool precise_control_mode_;  // 정밀 제어 모드 플래그

  bool navigation_mode_;  // 네비게이션 모드 플래그
  bool arrive;

  // OCR 토픽 관련 상태
  int64_t current_request_id_;
  std::string current_location_;
  std::chrono::steady_clock::time_point scan_start_time_;

  // 메서드
  void initPubSub();
  void turtleRun();
  void setState(WorkState new_state);

  // 네비게이션 시스템과 통신
  void searchResultCallback(const std_msgs::msg::String::SharedPtr msg);

  // OCR 토픽 처리
  void ocrRequestCallback(const robot_msgs::msg::OCRRequest::SharedPtr msg);
  void sendOCRResult(bool found, const std::string& detected_text, float confidence, const std::string& message);

  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);

  // 직접 정밀 제어 메서드들
  void startPreciseControl();
  void executePreciseControlStep();  // ROS2 타이머 콜백으로 변경
  void completePreciseControlWithError(const std::string& error_message);

  // 향상된 텍스트 매칭 함수들
  std::string normalizeString(const std::string& str);
  double calculateSimilarity(const std::string& str1, const std::string& str2);
  bool isTextMatch(const std::string& detected_text, const std::string& target_text, float confidence);
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_