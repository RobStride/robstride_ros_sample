#include "motor_ros2/motor_cfg.h"
#include "stdint.h"
#include <atomic>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unistd.h>
#include <vector>

class MotorControlSample : public rclcpp::Node {
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node"),
        motor(RobStrideMotor("can0", 0xFF, 0x01, 0)) {

    motor.Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    motor.enable_motor();
    usleep(1000);
    worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
  }

  ~MotorControlSample() {
    motor.Disenable_Motor(0);
    running_ = false; // 停止线程
    if (worker_thread_.joinable())
      worker_thread_.join(); // 等待线程结束
  }

  void excute_loop() {
    float position = 1.57f;
    float velocity = 0.1f;
    while (true) {
      // 自定义循环逻辑
      // 依次为速度，运控，位置模式, 电流，CSP位置

      auto [position_feedback, velocity_feedback, torque, temperature] =
          // motor.send_motion_command(0.0, position, velocity, 0.1f, 0.1f);
          // motor.RobStrite_Motor_PosPP_control(float Speed, float
          // Acceleration, float Angle);
          // motor.RobStrite_Motor_Current_control(float IqCommand, float
          // IdCommand); motor.send_velocity_mode_command(5.0f);
          motor.RobStrite_Motor_PosCSP_control(position, velocity);

      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // loop rate
    }
  }

private:
  std::thread worker_thread_;
  std::atomic<bool> running_ = true;

  RobStrideMotor motor;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}