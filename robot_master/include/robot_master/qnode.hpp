#ifndef robot_master_QNODE_HPP_
#define robot_master_QNODE_HPP_

#include <QThread>
#include <QTimer>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "lift_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_driving.hpp"
#include "robot_msgs/msg/master_msg.hpp"
#include "robot_msgs/msg/vision_msg.hpp"
#include "robot_msgs/srv/ocr_scan.hpp"
#include "std_msgs/msg/string.hpp"

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
  // ROS2 관련
  std::shared_ptr<rclcpp::Node> node;

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr sub_vision;

  // 네비게이션 시스템과 통신용 퍼블리셔/서브스크라이버
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_request_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_result_sub;

  // OCR 스캔 서비스 (기존 유지)
  rclcpp::Service<robot_msgs::srv::OCRScan>::SharedPtr ocr_scan_service_;

  std::shared_ptr<LiftController> lift_controller_;

  // 작업 상태 관리
  WorkState current_work_state_;
  std::string target_item_;
  bool lift_performing_action_{false};
  bool ocr_scan_active_{false};

  // 스캔 시작 시간 추적
  std::chrono::steady_clock::time_point scan_start_time_;

  // OCR 스캔 관련
  std::shared_ptr<robot_msgs::srv::OCRScan::Request> current_scan_request_;
  std::shared_ptr<robot_msgs::srv::OCRScan::Response> scan_response_;

  // 메서드
  void initPubSub();
  void turtleRun();
  void setState(WorkState new_state);

  // 네비게이션 시스템과 통신
  void searchResultCallback(const std_msgs::msg::String::SharedPtr msg);

  // OCR 서비스 처리 (기존 유지)
  void handleOCRScanRequest(const std::shared_ptr<robot_msgs::srv::OCRScan::Request> request, std::shared_ptr<robot_msgs::srv::OCRScan::Response> response);

  void visionCallback(const std::shared_ptr<robot_msgs::msg::VisionMsg> vision_msg);
  void finishOCRScan(bool found, const std::string& detected_text, float confidence, const std::string& message);

  // 향상된 텍스트 매칭 함수들
  std::string normalizeString(const std::string& str);
  double calculateSimilarity(const std::string& str1, const std::string& str2);
  bool isTextMatch(const std::string& detected_text, const std::string& target_text, float confidence);
};

}  // namespace robot_master

#endif  // robot_master_QNODE_HPP_