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
  void enableDetection(bool enabled);

  cv::Mat* imgRaw = nullptr;
  bool isreceived = false;
  robot_msgs::msg::VisionMsg latest_ocr_result_;

 Q_SIGNALS:
  void rosShutDown();
  void sigRcvImg();
  void sigOCRResult(const QString& text, bool detected, float confidence, int ocr_processing_time);
  void sigCameraFPS(int fps);

 private:
  void initPubSub();
  void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img);
  void requestOCRInference();
  void ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<robot_msgs::msg::VisionMsg>::SharedPtr vision_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocr_request_pub;
  rclcpp::Subscription<robot_msgs::msg::VisionMsg>::SharedPtr ocr_result_sub;
  rclcpp::TimerBase::SharedPtr ocr_timer;

  bool ocr_processing_;
  bool detection_enabled_;
  sensor_msgs::msg::Image::SharedPtr latest_image_msg;

  int camera_fps_count_;
  int current_camera_fps_;
  std::chrono::steady_clock::time_point last_camera_time_;
};

}  // namespace robot_vision

Q_DECLARE_METATYPE(QString);

#endif  // ROBOT_VISION_QNODE_HPP_