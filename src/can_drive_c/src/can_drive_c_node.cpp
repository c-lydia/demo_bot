#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "custom_messages/msg/brt_feedback.hpp"
#include "custom_messages/msg/damiao_command.hpp"
#include "custom_messages/msg/digital_and_analog_feedback.hpp"
#include "custom_messages/msg/digital_and_solenoid_command.hpp"
#include "custom_messages/msg/encoder_feedback.hpp"
#include "custom_messages/msg/motor_command.hpp"
#include "custom_messages/msg/pwm_command.hpp"
#include "custom_messages/msg/robomaster_current_command.hpp"
#include "custom_messages/msg/robomaster_feedback.hpp"
#include "custom_messages/msg/servo_command.hpp"
#include "custom_messages/msg/swerve_command.hpp"
#include "custom_messages/msg/swerve_feedback.hpp"
#include "custom_messages/msg/vesc_command.hpp"
#include "custom_messages/msg/vesc_status_four.hpp"
#include "custom_messages/msg/vesc_status_one.hpp"
#include "custom_messages/msg/vesc_status_three.hpp"
#include "custom_messages/msg/vesc_status_two.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class CanDriveC final : public rclcpp::Node {
public:
  CanDriveC() : Node("can_drive_c"), socket_fd_(-1), running_(true), can_transmit_count_(0), can_receive_count_(0), can_error_count_(0), last_recovery_time_(this->now()) {
    setup_publishers();
    setup_subscribers();

    can_timer_ = this->create_wall_timer(1s, std::bind(&CanDriveC::can_timer_callback, this));

    if (!setup_can_interface()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize can0. Receiver thread will keep retrying.");
    }

    receiver_thread_ = std::thread(&CanDriveC::receive_loop, this);
  }

  ~CanDriveC() override {
    running_.store(false);
    
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }

    close_can_socket();
  }

private:
  static float sign(float x) {
    return x >= 0.0f ? 1.0f : -1.0f;
  }

  static int16_t unpack_i16_be(const uint8_t * data) {
    const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8U) | static_cast<uint16_t>(data[1]);
    return static_cast<int16_t>(raw);
  }

  static int32_t unpack_i32_be(const uint8_t * data) {
    const uint32_t raw = (static_cast<uint32_t>(data[0]) << 24U) | (static_cast<uint32_t>(data[1]) << 16U) | (static_cast<uint32_t>(data[2]) << 8U) | static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(raw);
  }

  static void pack_i16_be(uint8_t * dst, int16_t value) {
    const uint16_t raw = static_cast<uint16_t>(value);
    dst[0] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
    dst[1] = static_cast<uint8_t>(raw & 0xFFU);
  }

  static void pack_i32_be(uint8_t * dst, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    dst[0] = static_cast<uint8_t>((raw >> 24U) & 0xFFU);
    dst[1] = static_cast<uint8_t>((raw >> 16U) & 0xFFU);
    dst[2] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
    dst[3] = static_cast<uint8_t>(raw & 0xFFU);
  }

  static float unpack_f32_le(const uint8_t * data) {
    const uint32_t raw = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) | (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
  }

  static void pack_f32_le(uint8_t * dst, float value) {
    uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(raw));
    dst[0] = static_cast<uint8_t>(raw & 0xFFU);
    dst[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
    dst[2] = static_cast<uint8_t>((raw >> 16U) & 0xFFU);
    dst[3] = static_cast<uint8_t>((raw >> 24U) & 0xFFU);
  }

  void setup_publishers() {
    transmit_count_publisher_ = this -> create_publisher<std_msgs::msg::UInt32>("/can_transmit_count", 10);
    receive_count_publisher_ = this -> create_publisher<std_msgs::msg::UInt32>("/can_receive_count", 10);
    error_count_publisher_ = this -> create_publisher<std_msgs::msg::UInt32>("/can_error_count", 10);

    encoder_publisher_ = this->create_publisher<custom_messages::msg::EncoderFeedback>("/encoder_feedback", 10);
    digital_analog_publisher_ = this -> create_publisher<custom_messages::msg::DigitalAndAnalogFeedback>("/digital_analog_feedback", 10);
    robomaster_feedback_publisher_ = this -> create_publisher<custom_messages::msg::RobomasterFeedback>("/robomaster_feedback", 10);
    vesc_status1_publisher_ = this -> create_publisher<custom_messages::msg::VescStatusOne>("/vesc_status1", 10);
    vesc_status2_publisher_ = this -> create_publisher<custom_messages::msg::VescStatusTwo>("/vesc_status2", 10);
    vesc_status3_publisher_ = this -> create_publisher<custom_messages::msg::VescStatusThree>("/vesc_status3", 10);
    vesc_status4_publisher_ = this -> create_publisher<custom_messages::msg::VescStatusFour>("/vesc_status4", 10);
    imu_quaternion_publisher_ = this -> create_publisher<geometry_msgs::msg::Quaternion>("/imu/quaternion", 10);
    swerve_feedback_publisher_ = this -> create_publisher<custom_messages::msg::SwerveFeedback>("/swerve_feedback", 10);
    brt_feedback_publisher_ = this -> create_publisher<custom_messages::msg::BRTFeedback>("/brt_feedback", 10);
  }

  void setup_subscribers() {
    motor_subscription_ = this -> create_subscription<custom_messages::msg::MotorCommand>("/publish_motor", 10, std::bind(&CanDriveC::motor_callback, this, std::placeholders::_1));
    servo_subscription_ = this -> create_subscription<custom_messages::msg::ServoCommand>("/publish_servo", 10, std::bind(&CanDriveC::servo_command_callback, this, std::placeholders::_1));
    pwm_subscription_ = this -> create_subscription<custom_messages::msg::PwmCommand>("/publish_pwm", 10, std::bind(&CanDriveC::pwm_command_callback, this, std::placeholders::_1));
    digital_solenoid_subscription_ = this -> create_subscription<custom_messages::msg::DigitalAndSolenoidCommand>("/publish_digital_solenoid", 10, std::bind(&CanDriveC::digital_and_solenoid_command_callback, this, std::placeholders::_1));
    robomaster_current_subscription_ = this -> create_subscription<custom_messages::msg::RobomasterCurrentCommand>("/publish_robomaster_current", 10, std::bind(&CanDriveC::robomaster_current_callback, this, std::placeholders::_1));
    vesc_command_subscription_ = this -> create_subscription<custom_messages::msg::VescCommand>("/publish_vesc", 10, std::bind(&CanDriveC::vesc_command_callback, this, std::placeholders::_1));
    swerve_subscription_ = this -> create_subscription<custom_messages::msg::SwerveCommand>("/publish_swerve", 10, std::bind(&CanDriveC::swerve_callback, this, std::placeholders::_1));
    damiao_command_subscription_ = this -> create_subscription<custom_messages::msg::DamiaoCommand>("/publish_damiao", 10, std::bind(&CanDriveC::damiao_command_callback, this, std::placeholders::_1));
  }

  void can_timer_callback() {
    std_msgs::msg::UInt32 msg;

    msg.data = can_transmit_count_.exchange(0U);
    transmit_count_publisher_ -> publish(msg);

    msg.data = can_receive_count_.exchange(0U);
    receive_count_publisher_ -> publish(msg);

    msg.data = can_error_count_.exchange(0U);
    error_count_publisher_ -> publish(msg);
  }

  bool ensure_can_interface_up() {
    if (std::system("ip link show can0 2>/dev/null | grep -q 'state UP'") == 0) {
      return true;
    }

    const int rc = std::system("sudo ip link set can0 up type can bitrate 1000000");

    if (rc != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set up can0 interface");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "CAN interface can0 brought up at 1 Mbps");
    return true;
  }

  bool setup_can_interface() {
    if (!ensure_can_interface_up()) {
      return false;
    }

    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (fd < 0) {
      RCLCPP_ERROR(this -> get_logger(), "socket(PF_CAN) failed: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this -> get_logger(), "ioctl(SIOCGIFINDEX) failed: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this -> get_logger(), "bind(can0) failed: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      close_can_socket_locked();
      socket_fd_ = fd;
    }

    return true;
  }

  void close_can_socket_locked() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void close_can_socket() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    close_can_socket_locked();
  }

  int get_socket_fd() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    return socket_fd_;
  }

  void attempt_recovery() {
    const auto now = this->now();

    if ((now - last_recovery_time_).seconds() < 1.0) {
      return;
    }

    last_recovery_time_ = now;
    close_can_socket();
    (void)setup_can_interface();
  }

  void send_can_frame(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc, bool is_extended_id) {
    if (dlc > 8U) {
      RCLCPP_ERROR(this->get_logger(), "Invalid CAN DLC: %u", dlc);
      return;
    }

    struct can_frame frame {};
    frame.can_id = arbitration_id;

    if (is_extended_id) {
      frame.can_id |= CAN_EFF_FLAG;
    }

    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    int fd = get_socket_fd();

    if (fd < 0) {
      if (!setup_can_interface()) {
        can_error_count_.fetch_add(1U);
        return;
      }

      fd = get_socket_fd();

      if (fd < 0) {
        can_error_count_.fetch_add(1U);
        return;
      }
    }

    const ssize_t bytes_written = write(fd, &frame, sizeof(frame));

    if (bytes_written != static_cast<ssize_t>(sizeof(frame))) {
      can_error_count_.fetch_add(1U);
      RCLCPP_WARN(this->get_logger(), "CAN write failed: %s", std::strerror(errno));
      attempt_recovery();
      return;
    }

    can_transmit_count_.fetch_add(1U);
  }

  void receive_loop() {
    while (running_.load()) {
      const int fd = get_socket_fd();

      if (fd < 0) {
        (void)setup_can_interface();
        std::this_thread::sleep_for(500ms);
        continue;
      }

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd, &read_fds);

      struct timeval timeout {};
      timeout.tv_sec = 0;
      timeout.tv_usec = 200000;

      const int select_rc = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);

      if (select_rc < 0) {
        can_error_count_.fetch_add(1U);
        attempt_recovery();
        std::this_thread::sleep_for(100ms);
        continue;
      }

      if (select_rc == 0) {
        continue;
      }

      struct can_frame frame {};
      const ssize_t bytes_read = read(fd, &frame, sizeof(frame));

      if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
        can_error_count_.fetch_add(1U);
        continue;
      }

      process_can_frame(frame);
    }
  }

  void process_can_frame(const struct can_frame & frame) {
    if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
      can_error_count_.fetch_add(1U);
      return;
    }

    can_receive_count_.fetch_add(1U);

    const bool is_extended = (frame.can_id & CAN_EFF_FLAG) != 0U;
    const uint32_t arbitration_id = frame.can_id & (is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    const uint8_t dlc = frame.can_dlc;
    const uint8_t * data = frame.data;

    if (process_smart_driver(arbitration_id, data, dlc)) {
      return;
    }

    if (process_controller_board(arbitration_id, data, dlc)) {
      return;
    }

    if (process_sensor_board(arbitration_id, data, dlc)) {
      return;
    }

    if (process_imu_board(arbitration_id, data, dlc)) {
      return;
    }

    if (process_robomaster(arbitration_id, data, dlc)) {
      return;
    }

    if (process_vesc(arbitration_id, data, dlc)) {
      return;
    }

    if (process_swerve(arbitration_id, data, dlc)) {
      return;
    }

    if (process_damiao(arbitration_id, data, dlc)) {
      return;
    }

    if (process_brt38(arbitration_id, data, dlc)) {
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Unprocessed CAN frame id=%u dlc=%u", arbitration_id, dlc);
  }

  bool process_smart_driver(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id >= 100U && arbitration_id < 200U && dlc == 8U) {
      publish_encoder_feedback(arbitration_id, data);
      return true;
    }

    return false;
  }

  bool process_controller_board(uint32_t, const uint8_t *, uint8_t) {
    return false;
  }

  bool process_sensor_board(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id >= 100U && arbitration_id < 200U && dlc == 8U) {
      publish_encoder_feedback(arbitration_id, data);
      return true;
    }
    
    if (arbitration_id >= 500U && arbitration_id <= 510U && dlc == 8U) {
      publish_digital_analog_feedback(arbitration_id, data);
      return true;
    }
    
    return false;
  }

  bool process_imu_board(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id == 1000U && dlc == 8U) {
      geometry_msgs::msg::Quaternion q;

      const float sign_w = ((data[0] >> 7U) == 1U) ? -1.0f : 1.0f;
      q.w = sign_w * static_cast<double>(((data[0] % 128U) << 8U) + data[1]) / 32767.0;

      const float sign_x = ((data[2] >> 7U) == 1U) ? -1.0f : 1.0f;
      q.x = sign_x * static_cast<double>(((data[2] % 128U) << 8U) + data[3]) / 32767.0;

      const float sign_y = ((data[4] >> 7U) == 1U) ? -1.0f : 1.0f;
      q.y = sign_y * static_cast<double>(((data[4] % 128U) << 8U) + data[5]) / 32767.0;

      const float sign_z = ((data[6] >> 7U) == 1U) ? -1.0f : 1.0f;
      q.z = sign_z * static_cast<double>(((data[6] % 128U) << 8U) + data[7]) / 32767.0;

      imu_quaternion_publisher_->publish(q);
      return true;
    }

    return false;
  }

  bool process_robomaster(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id >= 0x201U && arbitration_id <= 0x208U && dlc == 8U) {
      custom_messages::msg::RobomasterFeedback feedback_msg;
      feedback_msg.motor_id = arbitration_id - 0x200U;
      feedback_msg.position = static_cast<float>(unpack_i16_be(data + 0));
      feedback_msg.speed = static_cast<float>(unpack_i16_be(data + 2));
      feedback_msg.current = static_cast<float>(unpack_i16_be(data + 4));
      robomaster_feedback_publisher_->publish(feedback_msg);
      return true;
    }

    return false;
  }

  bool process_vesc(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (dlc != 8U) {
      return false;
    }

    if (arbitration_id >= 0x900U && arbitration_id <= 0x907U) {
      custom_messages::msg::VescStatusOne status_one;
      status_one.esc_id = arbitration_id - 0x900U;
      status_one.rpm = static_cast<float>(unpack_i32_be(data + 0));
      status_one.current = static_cast<float>(unpack_i16_be(data + 4)) * 0.1f;
      status_one.duty = static_cast<float>(unpack_i16_be(data + 6)) * 0.001f;
      vesc_status1_publisher_ -> publish(status_one);
      return true;
    }

    if (arbitration_id >= 0xE00U && arbitration_id <= 0xE07U) {
      custom_messages::msg::VescStatusTwo status_two;
      status_two.esc_id = arbitration_id - 0xE00U;
      status_two.amp_hours = static_cast<float>(unpack_i32_be(data + 0)) * 1e-4f;
      status_two.amp_hours_charged = static_cast<float>(unpack_i32_be(data + 4)) * 1e-4f;
      vesc_status2_publisher_ -> publish(status_two);
      return true;
    }

    if (arbitration_id >= 0xF00U && arbitration_id <= 0xF07U) {
      custom_messages::msg::VescStatusThree status_three;
      status_three.esc_id = arbitration_id - 0xF00U;
      status_three.watt_hours = static_cast<float>(unpack_i32_be(data + 0)) * 1e-4f;
      status_three.watt_hours_charged = static_cast<float>(unpack_i32_be(data + 4)) * 1e-4f;
      vesc_status3_publisher_ -> publish(status_three);
      return true;
    }

    if (arbitration_id >= 0x1000U && arbitration_id <= 0x1020U) {
      custom_messages::msg::VescStatusFour status_four;
      status_four.esc_id = arbitration_id - 0x1000U;
      status_four.fet_temp = static_cast<float>(unpack_i16_be(data + 0)) * 0.1f;
      status_four.motor_temp = static_cast<float>(unpack_i16_be(data + 2)) * 0.1f;
      status_four.input_current = static_cast<float>(unpack_i16_be(data + 4)) * 0.1f;
      status_four.pid_position = static_cast<float>(unpack_i16_be(data + 6)) * 0.02f;
      vesc_status4_publisher_ -> publish(status_four);
      return true;
    }

    return false;
  }

  bool process_swerve(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id >= 250U && arbitration_id < 300U && dlc == 8U) {
      custom_messages::msg::SwerveFeedback feedback_msg;
      feedback_msg.can_id = arbitration_id;
      feedback_msg.position = unpack_f32_le(data + 0);
      feedback_msg.speed = unpack_f32_le(data + 4);
      swerve_feedback_publisher_->publish(feedback_msg);
      return true;
    }

    return false;
  }

  bool process_damiao(uint32_t, const uint8_t *, uint8_t) {
    return false;
  }

  bool process_brt38(uint32_t arbitration_id, const uint8_t * data, uint8_t dlc) {
    if (arbitration_id >= 201U && arbitration_id < 250U && dlc == 7U) {
      if (data[0] == 0x07U && data[1] == arbitration_id && data[2] == 0x01U) {
        custom_messages::msg::BRTFeedback feedback_msg;
        feedback_msg.can_id = arbitration_id;
        const uint32_t encoder_count = static_cast<uint32_t>(data[3]) | (static_cast<uint32_t>(data[4]) << 8U) | (static_cast<uint32_t>(data[5]) << 16U) | (static_cast<uint32_t>(data[6]) << 24U);
        feedback_msg.encoder_count = encoder_count;
        brt_feedback_publisher_ -> publish(feedback_msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown BRT CAN frame conflict");
      }

      return true;
    }

    return false;
  }

  void publish_encoder_feedback(uint32_t arbitration_id, const uint8_t * data) {
    custom_messages::msg::EncoderFeedback encoder_msg;
    encoder_msg.can_id = arbitration_id;
    encoder_msg.position = unpack_f32_le(data + 0);
    encoder_msg.speed = unpack_f32_le(data + 4);
    encoder_publisher_->publish(encoder_msg);
  }

  void publish_digital_analog_feedback(uint32_t arbitration_id, const uint8_t * data) {
    custom_messages::msg::DigitalAndAnalogFeedback sensor_msg;
    sensor_msg.can_id = arbitration_id;

    sensor_msg.analog1_value = static_cast<float>(((data[0] << 4U) + (data[1] >> 4U)) / 4095.0);
    sensor_msg.analog2_value = static_cast<float>((((data[1] & 0x0FU) << 8U) + data[2]) / 4095.0);
    sensor_msg.analog3_value = static_cast<float>(((data[3] << 4U) + (data[4] >> 4U)) / 4095.0);
    sensor_msg.analog4_value = static_cast<float>((((data[4] & 0x0FU) << 8U) + data[5]) / 4095.0);
    sensor_msg.analog5_value = static_cast<float>(((data[6] << 4U) + (data[7] >> 4U)) / 4095.0);

    sensor_msg.digital1_value = (data[7] & 1U) == 1U;
    sensor_msg.digital2_value = ((data[7] >> 1U) & 1U) == 1U;
    sensor_msg.digital3_value = ((data[7] >> 2U) & 1U) == 1U;
    sensor_msg.digital4_value = ((data[7] >> 3U) & 1U) == 1U;

    digital_analog_publisher_ -> publish(sensor_msg);
  }

  void motor_callback(const custom_messages::msg::MotorCommand::SharedPtr ros_msg) {
    std::array<uint8_t, 8> can_data {};

    can_data[0] = static_cast<uint8_t>(
      (ros_msg->positionmode ? 1U : 0U) +
      ((ros_msg->speedmode ? 1U : 0U) << 1U) +
      ((ros_msg->voltagemode ? 1U : 0U) << 2U) +
      ((ros_msg->stop ? 1U : 0U) << 3U) +
      ((ros_msg->reset ? 1U : 0U) << 4U)
    );

    pack_f32_le(can_data.data() + 2, ros_msg->goal);

    send_can_frame(ros_msg -> can_id, can_data.data(), 8U, false);
  }

  void pwm_command_callback(const custom_messages::msg::PwmCommand::SharedPtr ros_msg) {
    std::array<float, 4> pwm_values {
      ros_msg->pwm1_value,
      ros_msg->pwm2_value,
      ros_msg->pwm3_value,
      ros_msg->pwm4_value
    };

    std::array<uint8_t, 8> can_data {};

    for (size_t i = 0; i < pwm_values.size(); ++i) {
      const float clamped = std::clamp(pwm_values[i], 0.0f, 1.0f);
      const int32_t pwm_value_int = static_cast<int32_t>(clamped * 16383.0f);
      can_data[2U * i] = static_cast<uint8_t>((pwm_value_int >> 8U) & 0xFF);
      can_data[2U * i + 1U] = static_cast<uint8_t>(pwm_value_int & 0xFF);

      if (i == 0U) {
        can_data[0] = static_cast<uint8_t>(can_data[0] | 0x80U);
      }
    }

    send_can_frame(ros_msg->can_id, can_data.data(), 8U, false);
  }

  void digital_and_solenoid_command_callback(const custom_messages::msg::DigitalAndSolenoidCommand::SharedPtr ros_msg) {
    std::array<uint8_t, 8> can_data {};

    can_data[0] = 0x40U;
    can_data[1] = 0U;
    can_data[2] = 0U;

    const std::array<bool, 4> digital_values {
      ros_msg -> digital1_value,
      ros_msg -> digital2_value,
      ros_msg -> digital3_value,
      ros_msg -> digital4_value
    };

    const std::array<bool, 6> solenoid_values {
      ros_msg -> solenoid1_value,
      ros_msg -> solenoid2_value,
      ros_msg -> solenoid3_value,
      ros_msg -> solenoid4_value,
      ros_msg -> solenoid5_value,
      ros_msg -> solenoid6_value
    };

    for (size_t i = 0; i < digital_values.size(); ++i) {
      can_data[1] = static_cast<uint8_t>(can_data[1] | ((digital_values[i] ? 1U : 0U) << i));
    }

    for (size_t i = 0; i < solenoid_values.size(); ++i) {
      can_data[2] = static_cast<uint8_t>(can_data[2] | ((solenoid_values[i] ? 1U : 0U) << i));
    }

    send_can_frame(ros_msg->can_id, can_data.data(), 8U, false);
  }

  void servo_command_callback(const custom_messages::msg::ServoCommand::SharedPtr ros_msg) {
    std::array<float, 4> servo_values {
      ros_msg -> servo1_value,
      ros_msg -> servo2_value,
      ros_msg -> servo3_value,
      ros_msg -> servo4_value
    };

    std::array<uint8_t, 8> can_data {};

    for (size_t i = 0; i < servo_values.size(); ++i) {
      const float clamped = std::clamp(servo_values[i], 0.0f, 1.0f);
      const int32_t servo_value_int = static_cast<int32_t>(clamped * 16383.0f);
      can_data[2U * i] = static_cast<uint8_t>((servo_value_int >> 8U) & 0xFF);
      can_data[2U * i + 1U] = static_cast<uint8_t>(servo_value_int & 0xFF);

      if (i == 0U) {
        can_data[0] = static_cast<uint8_t>(can_data[0] | 0xC0U);
      }
    }

    send_can_frame(ros_msg->can_id, can_data.data(), 8U, false);
  }

  void robomaster_current_callback(const custom_messages::msg::RobomasterCurrentCommand::SharedPtr ros_msg) {
    std::array<float, 4> current_data {
      ros_msg -> current1,
      ros_msg -> current2,
      ros_msg -> current3,
      ros_msg -> current4
    };

    std::array<uint8_t, 4> motor_type {
      ros_msg -> type1,
      ros_msg -> type2,
      ros_msg -> type3,
      ros_msg -> type4
    };

    for (size_t i = 0; i < current_data.size(); ++i) {
      if (motor_type[i] == 0U) {
        constexpr float max_current = 10.0f;

        if (std::abs(current_data[i]) > max_current) {
          current_data[i] = max_current * sign(current_data[i]);
        }

        current_data[i] *= 1000.0f;
      } else if (motor_type[i] == 1U) {
        constexpr float max_current = 20.0f;

        if (std::abs(current_data[i]) > max_current) {
          current_data[i] = max_current * sign(current_data[i]);
        }

        current_data[i] *= 819.2f;
      }
    }

    std::array<uint8_t, 8> can_data {};
    pack_i16_be(can_data.data() + 0, static_cast<int16_t>(current_data[0]));
    pack_i16_be(can_data.data() + 2, static_cast<int16_t>(current_data[1]));
    pack_i16_be(can_data.data() + 4, static_cast<int16_t>(current_data[2]));
    pack_i16_be(can_data.data() + 6, static_cast<int16_t>(current_data[3]));

    send_can_frame(ros_msg->can_id, can_data.data(), 8U, false);
  }

  void vesc_command_callback(const custom_messages::msg::VescCommand::SharedPtr ros_msg) {
    uint32_t can_id = ros_msg -> vesc_id;
    float goal = ros_msg -> goal;

    if (ros_msg -> pwm_mode) {
      can_id += 0x000U;
      goal = goal * 1e5f;
      
      if (std::abs(goal) > 1e5f) {
        goal = 1e5f * sign(goal);
      }
    } else if (ros_msg -> current_mode) {
      can_id += 0x100U;
      goal = goal * 1e6f;
    } else if (ros_msg -> current_brake_mode) {
      can_id += 0x200U;
      goal = goal * 1e6f;
    } else if (ros_msg -> speed_mode) {
      can_id += 0x300U;
    } else if (ros_msg -> position_mode) {
      can_id += 0x400U;
      goal = goal * 1e6f;
    }

    std::array<uint8_t, 4> can_data {};
    pack_i32_be(can_data.data(), static_cast<int32_t>(goal));
    send_can_frame(can_id, can_data.data(), 4U, true);
  }

  void swerve_callback(const custom_messages::msg::SwerveCommand::SharedPtr ros_msg) {
    std::array<uint8_t, 8> can_data {};
    pack_f32_le(can_data.data() + 0, ros_msg->position);
    pack_f32_le(can_data.data() + 4, ros_msg->speed);

    send_can_frame(ros_msg->can_id, can_data.data(), 8U, false);
  }

  void damiao_command_callback(const custom_messages::msg::DamiaoCommand::SharedPtr ros_msg) {
    const uint32_t can_id = ros_msg->motor_id + 0x200U;

    if (ros_msg->arm) {
      const std::array<uint8_t, 8> can_data {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
      send_can_frame(can_id, can_data.data(), 8U, false);
      return;
    }

    std::array<uint8_t, 4> can_data {};
    pack_f32_le(can_data.data(), ros_msg->speed);
    send_can_frame(can_id, can_data.data(), 4U, false);
  }

  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr transmit_count_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr receive_count_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr error_count_publisher_;

  rclcpp::Publisher<custom_messages::msg::EncoderFeedback>::SharedPtr encoder_publisher_;
  rclcpp::Publisher<custom_messages::msg::DigitalAndAnalogFeedback>::SharedPtr digital_analog_publisher_;
  rclcpp::Publisher<custom_messages::msg::RobomasterFeedback>::SharedPtr robomaster_feedback_publisher_;
  rclcpp::Publisher<custom_messages::msg::VescStatusOne>::SharedPtr vesc_status1_publisher_;
  rclcpp::Publisher<custom_messages::msg::VescStatusTwo>::SharedPtr vesc_status2_publisher_;
  rclcpp::Publisher<custom_messages::msg::VescStatusThree>::SharedPtr vesc_status3_publisher_;
  rclcpp::Publisher<custom_messages::msg::VescStatusFour>::SharedPtr vesc_status4_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr imu_quaternion_publisher_;
  rclcpp::Publisher<custom_messages::msg::SwerveFeedback>::SharedPtr swerve_feedback_publisher_;
  rclcpp::Publisher<custom_messages::msg::BRTFeedback>::SharedPtr brt_feedback_publisher_;

  rclcpp::Subscription<custom_messages::msg::MotorCommand>::SharedPtr motor_subscription_;
  rclcpp::Subscription<custom_messages::msg::ServoCommand>::SharedPtr servo_subscription_;
  rclcpp::Subscription<custom_messages::msg::PwmCommand>::SharedPtr pwm_subscription_;
  rclcpp::Subscription<custom_messages::msg::DigitalAndSolenoidCommand>::SharedPtr digital_solenoid_subscription_;
  rclcpp::Subscription<custom_messages::msg::RobomasterCurrentCommand>::SharedPtr robomaster_current_subscription_;
  rclcpp::Subscription<custom_messages::msg::VescCommand>::SharedPtr vesc_command_subscription_;
  rclcpp::Subscription<custom_messages::msg::SwerveCommand>::SharedPtr swerve_subscription_;
  rclcpp::Subscription<custom_messages::msg::DamiaoCommand>::SharedPtr damiao_command_subscription_;

  rclcpp::TimerBase::SharedPtr can_timer_;

  std::mutex socket_mutex_;
  int socket_fd_;
  std::thread receiver_thread_;
  std::atomic<bool> running_;

  std::atomic<uint32_t> can_transmit_count_;
  std::atomic<uint32_t> can_receive_count_;
  std::atomic<uint32_t> can_error_count_;
  rclcpp::Time last_recovery_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanDriveC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
