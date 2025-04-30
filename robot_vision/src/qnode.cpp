/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2025
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robot_vision/qnode.hpp"

#include <cv_bridge/cv_bridge.h>
namespace robot_vision {

QNode::QNode() {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robot_vision");
  initPubSub();
  // initUdpSocket();
  // qRegisterMetaType<cv::Mat>("cv::Mat");
  this->start(); // 소켓 초기화 후 스레드 시작
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  // if (udp_socket != -1) {
  //   close(udp_socket);
  // }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    // receiveUdpImage();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::initPubSub() {
  pubImage = node->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);
  vision_pub = node->create_publisher<robot_msgs::msg::VisionMsg>("turtle_vision", 100);
  image_sub = node->create_subscription<sensor_msgs::msg::Image>("camera1/image_raw", 10, std::bind(&QNode::callbackImage, this, std::placeholders::_1));
  // subMaster = node->create_subscription<robot_msgs::msg::MasterMsg>("turtle_master", 100, std::bind(&QNode::updateParameter, this, std::placeholders::_1));
}

void QNode::callbackImage(const sensor_msgs::msg::Image::SharedPtr msg_img)
{
  if (imgRaw == NULL && !isreceived)  // imgRaw -> NULL, isreceived -> false
  {
    try {
      // ROS2 이미지 메시지를 OpenCV Mat 형식으로 변환, 이미지 객체에 할당
      imgRaw = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);

      if (imgRaw != NULL)  // imgRaw 변환 성공
      {
        Q_EMIT sigRcvImg();  // 이미지 수신을 알리는 시그널 발생
        isreceived = true;
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// void QNode::initUdpSocket() {
//   udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
//   if (udp_socket == -1) {
//     throw std::runtime_error("Failed to create UDP socket");
//   }

//   sockaddr_in server_addr{};
//   server_addr.sin_family = AF_INET;
//   server_addr.sin_addr.s_addr = INADDR_ANY;
//   server_addr.sin_port = htons(UDP_PORT);

//   if (bind(udp_socket, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) == -1) {
//     close(udp_socket);
//     throw std::runtime_error("Failed to bind UDP socket");
//   }

//   RCLCPP_INFO(node->get_logger(), "UDP socket initialized and bound to port %d", UDP_PORT);
// }


// void QNode::receiveUdpImage() {
//   std::vector<char> buffer(BUFFER_SIZE);
//   sockaddr_in client_addr{};
//   socklen_t client_addr_len = sizeof(client_addr);

//   ssize_t received_bytes = recvfrom(udp_socket, buffer.data(), buffer.size(), 0,
//                                     reinterpret_cast<struct sockaddr*>(&client_addr), &client_addr_len);

//   if (received_bytes > 0) {
//     RCLCPP_INFO(node->get_logger(), "Received %zd bytes from %s:%d",
//                 received_bytes, inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
//     std::vector<uchar> jpeg_data(buffer.begin(), buffer.begin() + received_bytes);
//     cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

//     if (!image.empty()) {
//       RCLCPP_INFO(node->get_logger(), "Decoded image: %dx%d", image.cols, image.rows);
//       processReceivedImage(image);
//     } else {
//       RCLCPP_WARN(node->get_logger(), "Failed to decode image");
//     }
//   } else if (received_bytes == -1) {
//     RCLCPP_WARN(node->get_logger(), "UDP receive error: %s", strerror(errno));
//   } else {
//     RCLCPP_DEBUG(node->get_logger(), "No data received");
//   }
// }

// void QNode::processReceivedImage(const cv::Mat& image) {
//   {
//     std::lock_guard<std::mutex> lock(image_mutex);
//     raw_img = image.clone();
//   }

//   vision_pub->publish(robot_vision_.Vision_msg);
//   Q_EMIT imageReceived(image);

//   updateFps();
// }
// cv::Mat QNode::getLatestImage() const {
//   std::lock_guard<std::mutex> lock(image_mutex);
//   return raw_img.clone();
// }

// void QNode::updateFps() {
//   auto now = std::chrono::steady_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_update).count();

//   if (duration > 0) {
//     double current_fps = 1000.0 / duration;

//     fps_history.push_back(current_fps);
//     if (fps_history.size() > FPS_HISTORY_SIZE) {
//       fps_history.pop_front();
//     }

//     double avg_fps = std::accumulate(fps_history.begin(), fps_history.end(), 0.0) / fps_history.size();

//     Vision::now_fps = static_cast<int>(std::round(avg_fps));
//     Q_EMIT fpsUpdated(Vision::now_fps);

//     last_fps_update = now;
//   }
// }


}  // namespace robot_vision