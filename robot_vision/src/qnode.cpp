#include "../include/robot_vision/qnode.hpp"

#include <cv_bridge/cv_bridge.h>

namespace robot_vision {

QNode::QNode()
    : ocr_processing_(false),
      ocr_enabled_(false),  // OCR 활성화 상태 추가
      camera_fps_count_(0),
      current_camera_fps_(0),
      isreceived(false) {
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_vision");

  last_camera_time_ = std::chrono::steady_clock::now();

  initPubSub();
  this->start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(30);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

// ================ ROS 2 Publisher/Subscriber 초기화 ================
void QNode::initPubSub() {
  // === OCR 제어 구독 (Navigation에서 제어) ===
  ocr_control_sub_ = node->create_subscription<std_msgs::msg::Bool>("ocr_control", 10, std::bind(&QNode::ocrControlCallback, this, std::placeholders::_1));

  // === 이미지 관련 ===
  // OCR 요청용 이미지 퍼블리셔
  ocr_request_pub = node->create_publisher<sensor_msgs::msg::Image>("/ocr_request", 10);
  // 카메라 이미지 구독
  image_sub = node->create_subscription<sensor_msgs::msg::Image>("camera1/image_raw", 10, std::bind(&QNode::callbackImage, this, std::placeholders::_1));

  // === OCR 결과 구독 (GUI 표시용) ===
  ocr_result_sub = node->create_subscription<robot_msgs::msg::VisionMsg>("turtle_vision", 10, std::bind(&QNode::ocrResultCallback, this, std::placeholders::_1));

  // === OCR 활성화 시 이미지 전송 타이머 (1초마다) ===
  ocr_image_timer = node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&QNode::sendImageToOCR, this));

  // === 통합 로그 시스템 ===
  system_log_pub_ = node->create_publisher<robot_msgs::msg::LogMessage>("system_log", 100);

  publishSystemLog("INFO", "비전 시스템 초기화 완료 (OCR 자동 제어 연동)");
}

// ================ 통합 로그 표시 ================
void QNode::publishSystemLog(const std::string& level, const std::string& message) {
  auto log_msg = robot_msgs::msg::LogMessage();
  log_msg.node_name = "vision";
  log_msg.level = level;
  log_msg.message = message;
  log_msg.timestamp = node->now();

  system_log_pub_->publish(log_msg);
}

// ================ OCR 제어 콜백 함수 ================
void QNode::ocrControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !ocr_enabled_) {
    ocr_enabled_ = true;
    publishSystemLog("INFO", "OCR 활성화 - 이미지 전송 시작");
  } else if (!msg->data && ocr_enabled_) {
    ocr_enabled_ = false;
    ocr_processing_ = false;
    publishSystemLog("INFO", "OCR 비활성화 - 이미지 전송 중단");
  }
}

// ================ 카메라 이미지 콜백 함수 ================
void QNode::callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img) {
  if (!isreceived) {
    isreceived = true;

    // 카메라 FPS 계산
    auto current_time = std::chrono::steady_clock::now();
    camera_fps_count_++;

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_camera_time_);
    if (duration.count() >= 1000) {
      current_camera_fps_ = (camera_fps_count_ * 1000) / duration.count();
      camera_fps_count_ = 0;
      last_camera_time_ = current_time;
      Q_EMIT sigCameraFPS(current_camera_fps_);
    }

    // 이미지 처리 (GUI 표시용)
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8);

      if (imgRaw != nullptr) {
        delete imgRaw;
      }

      imgRaw = new cv::Mat(cv_ptr->image);
      latest_image_msg = msg_img;  // OCR 전송용으로 최신 이미지 보관
      Q_EMIT sigRcvImg();

    } catch (cv_bridge::Exception& e) {
      publishSystemLog("ERROR", "cv_bridge exception: " + std::string(e.what()));
    }
  }
}

// ================ OCR용 이미지 전송 함수 (1초마다 호출) ================
void QNode::sendImageToOCR() {
  // OCR이 활성화되어 있고, 처리 중이 아니며, 최신 이미지가 있을 때만 전송
  if (ocr_enabled_ && !ocr_processing_ && latest_image_msg) {
    ocr_processing_ = true;
    ocr_request_pub->publish(*latest_image_msg);
    publishSystemLog("DEBUG", "OCR용 이미지 전송");
  }
}

// ================ OCR 결과 콜백 함수 ================
void QNode::ocrResultCallback(const robot_msgs::msg::VisionMsg::SharedPtr msg) {
  ocr_processing_ = false;
  latest_ocr_result_ = *msg;

  float confidence = 0.0f;
  std::string display_text = "";

  if (msg->ocr_detected) {
    display_text = msg->ocr_text;
    confidence = msg->confidence;
    publishSystemLog("INFO", "OCR 감지: '" + msg->ocr_text + "' (신뢰도: " + std::to_string((int)(confidence * 100)) + "%%)");
  } else {
    display_text = "인식 실패";
    publishSystemLog("DEBUG", "OCR 인식 실패");
  }

  // GUI에 결과 전송
  Q_EMIT sigOCRResult(QString::fromStdString(display_text), msg->ocr_detected, confidence, msg->fps);
}

// ================ OCR 상태 확인 함수 (GUI용) ================
bool QNode::isOCREnabled() const { return ocr_enabled_; }

bool QNode::isOCRProcessing() const { return ocr_processing_; }

// ================ 수동 OCR 활성화/비활성화 (GUI에서 호출 가능) ================
void QNode::enableOCRManually(bool enabled) {
  if (enabled != ocr_enabled_) {
    auto msg = std_msgs::msg::Bool();
    msg.data = enabled;

    // 임시로 제어 메시지 퍼블리시 (테스트용)
    auto temp_pub = node->create_publisher<std_msgs::msg::Bool>("ocr_control", 10);
    temp_pub->publish(msg);

    if (enabled) {
      publishSystemLog("INFO", "OCR 수동 활성화");
    } else {
      publishSystemLog("INFO", "OCR 수동 비활성화");
    }
  }
}

// ================ 레거시 함수들 (호환성 유지) ================
void QNode::enableDetection(bool enabled) {
  // 이 함수는 이제 OCR 수동 제어로 리다이렉트
  enableOCRManually(enabled);

  if (enabled) {
    publishSystemLog("INFO", "OCR 탐지가 활성화되었습니다 (레거시 호출)");
  } else {
    publishSystemLog("INFO", "OCR 탐지가 비활성화되었습니다 (레거시 호출)");
    latest_ocr_result_ = robot_msgs::msg::VisionMsg();
  }
}

}  // namespace robot_vision