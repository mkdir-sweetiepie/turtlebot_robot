#ifndef ROBOT_VISION_QNODE_HPP_
#define ROBOT_VISION_QNODE_HPP_

#include <QObject>
#include <QString>
#include <QThread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/vision_msg.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace robot_vision {

class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  virtual ~QNode();

  void run() override;
  void enableDetection(bool enabled);  // OCR 탐지 활성화 함수

  cv::Mat* imgRaw = nullptr;                      // 원본 이미지 포인터
  bool isreceived = false;                        // 이미지 수신 여부
  robot_msgs::msg::VisionMsg latest_ocr_result_;  // 최신 OCR 결과 메시지

 Q_SIGNALS:
  void rosShutDown();
  void sigRcvImg();                                                                                  // 이미지 수신 시그널
  void sigOCRResult(const QString& text, bool detected, float confidence, int ocr_processing_time);  // OCR 결과 시그널
  void sigCameraFPS(int fps);                                                                        // 카메라 FPS 시그널

 private:
  void initPubSub();
  void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img);     // usb_cam 카메라 이미지 콜백 함수
  void requestOCRInference();                                               // OCR 추론 요청 함수 (5초마다 호출)
  void ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg);  // OCR 결과 콜백 함수

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<robot_msgs::msg::VisionMsg>::SharedPtr vision_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocr_request_pub;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr ocr_result_sub;
  rclcpp::TimerBase::SharedPtr ocr_timer;

  bool ocr_processing_;                                 // OCR 처리 중인지 여부
  bool detection_enabled_;                              // OCR 탐지 활성화 여부
  sensor_msgs::msg::Image::SharedPtr latest_image_msg;  // 최신 이미지 메시지

  int camera_fps_count_;                                    // 카메라 FPS 계산을 위한 카운트
  int current_camera_fps_;                                  // 현재 카메라 FPS
  std::chrono::steady_clock::time_point last_camera_time_;  // 마지막 카메라 시간
};

}  // namespace robot_vision

Q_DECLARE_METATYPE(QString);

#endif  // ROBOT_VISION_QNODE_HPP_