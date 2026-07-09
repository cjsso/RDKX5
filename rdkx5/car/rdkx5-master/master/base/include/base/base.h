#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <thread>
#include <algorithm>
#include <csignal>
#include <stdlib.h>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "chassis_driver_msgs/msg/chassis_state.hpp"
#include "chassis_driver_msgs/srv/chassis_led.hpp"
#include "chassis_driver_msgs/srv/chassis_buzzer.hpp"
#include "chassis_driver_msgs/srv/chassis_pid.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define CHASSIS_WHEEL_SPACING  (0.11)

typedef struct {
    uint8_t head_flag;
    uint8_t frame_type;
    uint8_t data_len;
    uint8_t payload[6];
    uint8_t check_sum;
    uint8_t end_flag;
} ChassisSerialFrame;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} ImuRawData;

typedef struct {
    float supply_voltage;
    bool buzzer_active;
    bool led_light_active;
} ChassisHardwareState;

enum {
    FRAME_TYPE_SPEED_CTRL = 0x01,
    FRAME_TYPE_FEED_SPEED = 0x02,
    FRAME_TYPE_FEED_ACC   = 0x03,
    FRAME_TYPE_FEED_GYRO  = 0x04,
    FRAME_TYPE_FEED_EULER = 0x05,
    FRAME_TYPE_FEED_SENSOR= 0x06,
    FRAME_TYPE_HW_CTRL    = 0x07,
};

class ChassisDriverNode : public rclcpp::Node
{
public:
    ChassisDriverNode(std::string nodeName);
    ~ChassisDriverNode();
    
private:
    void serial_read_loop();
    bool verify_serial_frame(ChassisSerialFrame &frame);
    void parse_serial_frame(ChassisSerialFrame &frame);

    void parse_feedback_speed(ChassisSerialFrame &frame);
    void parse_feedback_gyro(ChassisSerialFrame &frame);
    void parse_feedback_acc(ChassisSerialFrame &frame);
    void parse_feedback_euler(ChassisSerialFrame &frame);
    void parse_feedback_hw_sensor(ChassisSerialFrame &frame);

    double imu_byte_to_float(uint8_t high_byte, uint8_t low_byte);
    bool imu_offset_calibrate();
    double degree_to_radian(double deg);

    void publish_odom_topic(float linear_x, float angular_z);
    void publish_imu_topic();

    bool set_buzzer_state(bool enable);
    bool set_led_light_state(bool enable);

    void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void service_buzzer_handler(
        const std::shared_ptr<chassis_driver_msgs::srv::ChassisBuzzer::Request> req,
        std::shared_ptr<chassis_driver_msgs::srv::ChassisBuzzer::Response> res);
    void service_led_handler(
        const std::shared_ptr<chassis_driver_msgs::srv::ChassisLed::Request> req,
        std::shared_ptr<chassis_driver_msgs::srv::ChassisLed::Response> res);
    void service_pid_left_handler(
        const std::shared_ptr<chassis_driver_msgs::srv::ChassisPID::Request> req,
        std::shared_ptr<chassis_driver_msgs::srv::ChassisPID::Response> res);
    void service_pid_right_handler(
        const std::shared_ptr<chassis_driver_msgs::srv::ChassisPID::Request> req,
        std::shared_ptr<chassis_driver_msgs::srv::ChassisPID::Response> res);

    void timer_period_100ms_cb();

private:
    serial::Serial serial_port_obj_;
    rclcpp::Time sys_time_now_;
    float odom_pos_x_ = 0.0f;
    float odom_pos_y_ = 0.0f;
    float odom_yaw_theta_ = 0.0f;

    float linear_correct_scale_ = 1.0f;
    float angular_correct_scale_ = 1.0f;    
    
    std::shared_ptr<std::thread> serial_read_thread_;
    ImuRawData imu_cache_data_;
    ChassisHardwareState chassis_hw_state_;

    rclcpp::TimerBase::SharedPtr timer_100ms_period_;
    bool enable_auto_stop_ = true;
    bool enable_pub_odom_tf_ = false;
    bool enable_imu_fusion_ = false;
    unsigned int auto_stop_timer_count_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_topic_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_topic_pub_;
    rclcpp::Publisher<chassis_driver_msgs::msg::ChassisState>::SharedPtr hw_state_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
   
    rclcpp::Service<chassis_driver_msgs::srv::ChassisBuzzer>::SharedPtr buzzer_srv_handle_;
    rclcpp::Service<chassis_driver_msgs::srv::ChassisLed>::SharedPtr led_srv_handle_;
    rclcpp::Service<chassis_driver_msgs::srv::ChassisPID>::SharedPtr pid_left_srv_handle_;
    rclcpp::Service<chassis_driver_msgs::srv::ChassisPID>::SharedPtr pid_right_srv_handle_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_handle_;
    std::string serial_device_name_;
};