#include "motor_ros2/motor_cfg.h"

void RobStrideMotor::init_socket() {
  socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0) {
    perror("socket");
    exit(1);
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl");
    exit(1);
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    exit(1);
  }

  struct can_filter rfilter[1];
  rfilter[0].can_id =
      (motor_id << 8) | CAN_EFF_FLAG; // Bit8~Bit15 放电机ID，高位扩展帧标志
  rfilter[0].can_mask =
      (0xFF << 8) | CAN_EFF_FLAG; // 只匹配 Bit8~Bit15 + 扩展帧标志

  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter,
                 sizeof(rfilter)) < 0) {
    perror("setsockopt filter");
    exit(1);
  }
}

void RobStrideMotor::receive_status_frame() {
  auto result = receive();
  if (!result) {
    throw std::runtime_error("No frame received.");
  }

  auto [communication_type, extra_data, host_id, data] = *result;

  // uint8_t status_mode = (extra_data >> 14) & 0x03;
  // uint8_t status_uncalibrated = (extra_data >> 13) & 0x01;
  // uint8_t status_hall_encoder_fault = (extra_data >> 12) & 0x01;
  // uint8_t status_magnetic_encoder_fault = (extra_data >> 11) & 0x01;
  // uint8_t status_overtemperature = (extra_data >> 10) & 0x01;
  // uint8_t status_overcurrent = (extra_data >> 9) & 0x01;
  // uint8_t status_undervoltage = (extra_data >> 8) & 0x01;
  // uint8_t device_id = (extra_data >> 0) & 0xFF;

  if (data.size() < 8) {
    throw std::runtime_error("Data size too small");
  }

  // std::cout << "communication_type: " << static_cast<int>(communication_type)
  // << std::endl;
  if (communication_type == Communication_Type_MotorRequest) {
    // 解析数据：高字节在前（大端序）
    uint16_t position_u16 = (data[0] << 8) | data[1];
    uint16_t velocity_u16 = (data[2] << 8) | data[3];
    uint16_t torque_i16 = (data[4] << 8) | data[5];
    uint16_t temperature_u16 = (data[6] << 8) | data[7];

    // 转换成物理量
    position_ =
        ((static_cast<float>(position_u16) / 32767.0f) - 1.0f) *
        (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
             .position);
    velocity_ =
        ((static_cast<float>(velocity_u16) / 32767.0f) - 1.0f) *
        (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
             .velocity);
    torque_ =
        ((static_cast<float>(torque_i16) / 32767.0f) - 1.0f) *
        (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
             .torque);
    temperature_ = static_cast<float>(temperature_u16) * 0.1f;
  } else if (communication_type == 17) {
    params.data = uint8_t(data[4]);
    std::cout << params.data << std::endl;
    params.index = 0X7005;
    for (int index_num = 0; index_num <= 13; index_num++) {
      if ((data[1] << 8 | data[0]) == Index_List[index_num])
        switch (index_num) {
        case 0:
          drw.run_mode.data = uint8_t(data[4]);
          std::cout << "mode data: " << static_cast<int>(data[4]) << std::endl;
          ;
          break;
        case 1:
          drw.iq_ref.data = Byte_to_float(data);
          break;
        case 2:
          drw.spd_ref.data = Byte_to_float(data);
          break;
        case 3:
          drw.imit_torque.data = Byte_to_float(data);
          break;
        case 4:
          drw.cur_kp.data = Byte_to_float(data);
          break;
        case 5:
          drw.cur_ki.data = Byte_to_float(data);
          break;
        case 6:
          drw.cur_filt_gain.data = Byte_to_float(data);
          break;
        case 7:
          drw.loc_ref.data = Byte_to_float(data);
          break;
        case 8:
          drw.limit_spd.data = Byte_to_float(data);
          break;
        case 9:
          drw.limit_cur.data = Byte_to_float(data);
          break;
        case 10:
          drw.mechPos.data = Byte_to_float(data);
          break;
        case 11:
          drw.iqf.data = Byte_to_float(data);
          break;
        case 12:
          drw.mechVel.data = Byte_to_float(data);
          break;
        case 13:
          drw.VBUS.data = Byte_to_float(data);
          break;
        }
    }
  } else {
    throw std::runtime_error("Invalid communication type");
  }
}

void RobStrideMotor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value,
                                                   char Value_mode) {
  struct can_frame frame{};

  frame.can_id =
      Communication_Type_SetSingleParameter << 24 | master_id << 8 | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 0x08;

  frame.data[0] = Index;
  frame.data[1] = Index >> 8;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;

  if (Value_mode == 'p') {
    memcpy(&frame.data[4], &Value, 4);
  } else if (Value_mode == 'j') {
    // Motor_Set_All.set_motor_mode = int(Value);
    frame.data[4] = (uint8_t)Value;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
  }

  int n = write(socket_fd, &frame, sizeof(frame));
  if (n != sizeof(frame)) {
    perror("set mode failed");
  } else {
    // std::cout << "[✓] Motor set-mode command sent." << std::endl;
  }
  receive_status_frame();
}

// 发送使能指令（通信类型3）
std::tuple<float, float, float, float> RobStrideMotor::enable_motor() {
  struct can_frame frame{};
  frame.can_id =
      (Communication_Type_MotorEnable << 24) | (master_id << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 8;
  memset(frame.data, 0, 8);

  int n = write(socket_fd, &frame, sizeof(frame));
  if (n != sizeof(frame)) {
    perror("enable_motor failed");
  } else {
    std::cout << "[✓] Motor enable command sent." << std::endl;
  }
  receive_status_frame();

  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

uint16_t RobStrideMotor::float_to_uint(float x, float x_min, float x_max,
                                       int bits) {
  if (x < x_min)
    x = x_min;
  if (x > x_max)
    x = x_max;
  float span = x_max - x_min;
  float offset = x - x_min;
  return static_cast<uint16_t>((offset * ((1 << bits) - 1)) / span);
}

// 发送运控模式（控制角度 + 速度 + KP + KD）
std::tuple<float, float, float, float>
RobStrideMotor::send_motion_command(float torque, float position_rad,
                                    float velocity_rad_s, float kp, float kd) {
  if (drw.run_mode.data != 0 && pattern == 2) {
    Disenable_Motor(0);
    usleep(1000);

    Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
    usleep(1000);

    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);

    enable_motor();
    usleep(1000);
  }
  struct can_frame frame{};
  frame.can_id =
      (Communication_Type_MotionControl << 24) |
      (float_to_uint(torque,
                     -ACTUATOR_OPERATION_MAPPING
                          .at(static_cast<ActuatorType>(actuator_type))
                          .torque,
                     ACTUATOR_OPERATION_MAPPING
                         .at(static_cast<ActuatorType>(actuator_type))
                         .torque,
                     16)
       << 8) |
      motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  // frame.can_id = 0x1200fd01;
  frame.can_dlc = 8;

  uint16_t pos = float_to_uint(
      position_rad,
      -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
           .position,
      ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
          .position,
      16);
  uint16_t vel = float_to_uint(
      velocity_rad_s,
      -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
           .velocity,
      ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
          .velocity,
      16);
  uint16_t kp_u = float_to_uint(
      kp, 0.0f,
      ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
          .kp,
      16);
  uint16_t kd_u = float_to_uint(
      kd, 0.0f,
      ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type))
          .kd,
      16);

  frame.data[0] = (pos >> 8);
  frame.data[1] = pos;
  frame.data[2] = (vel >> 8);
  frame.data[3] = vel;
  frame.data[4] = (kp_u >> 8);
  frame.data[5] = kp_u;
  frame.data[6] = (kd_u >> 8);
  frame.data[7] = kd_u;
  // 05 70 00 00 07 01 82 F9

  int n = write(socket_fd, &frame, sizeof(frame));

  if (n != sizeof(frame)) {
    perror("send_motion_command failed");
  }
  receive_status_frame();
  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

std::tuple<float, float, float, float>
RobStrideMotor::send_velocity_mode_command(float velocity_rad_s) {
  if (drw.run_mode.data != 2 && pattern == 2) {
    Disenable_Motor(0);
    std::cout << "disable motor " << std::endl;
    usleep(1000);
    Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
    usleep(1000);
    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    enable_motor();
    Set_RobStrite_Motor_parameter(0X7018, 27.0f, Set_parameter);
    usleep(1000);
    Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All.set_acc, Set_parameter);
    usleep(1000);
  }
  std::cout << "excute vel_mode" << std::endl;
  Set_RobStrite_Motor_parameter(0X700A, velocity_rad_s, Set_parameter);
  std::cout << "finish" << std::endl;
  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

float RobStrideMotor::read_initial_position() {
  struct can_frame frame{};
  auto start = std::chrono::steady_clock::now();
  while (true) {
    // float neutral_pos = 2.0f;
    // send_motion_command(neutral_pos, 0.0f, 0.0f, 0.0f);

    ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));
    if (nbytes > 0 && (frame.can_id & CAN_EFF_FLAG)) {
      uint32_t canid = frame.can_id & CAN_EFF_MASK;
      uint8_t type = (canid >> 24) & 0xFF;
      uint8_t mid = (canid >> 8) & 0xFF;
      uint8_t eid = canid & 0xFF;

      // please switch print output;
      printf("type: 0x%02X\n", type);
      printf("mid:  0x%02X\n", mid);
      printf("eid:  0x%02X\n", eid);

      if (type == 0x02 && mid == 0x01 && eid == 0xFD) {
        uint16_t p_uint = (frame.data[0] << 8) | frame.data[1];
        float pos = uint_to_float(p_uint, -4 * M_PI, 4 * M_PI, 16);
        std::cout << "[✓] Initial position read: " << pos << " rad"
                  << std::endl;
        return pos;
      }
    }
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
            .count() > 10000) {
      std::cerr << "[!] Timeout waiting for motor feedback." << std::endl;
      return 0.0f;
    }
  }
}

void RobStrideMotor::Get_RobStrite_Motor_parameter(uint16_t Index) {
  struct can_frame frame{};
  frame.can_id = (Communication_Type_GetSingleParameter << 24) |
                 (master_id << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 8;

  frame.data[0] = Index;
  frame.data[1] = Index >> 8;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  int n = write(socket_fd, &frame, sizeof(frame));

  if (n != sizeof(frame)) {
    perror("get_motor_parameter failed");
  }
  std::cout << "excute Get_motor_params" << std::endl;
  receive_status_frame();
}

// 位置模式（PP）
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_PosPP_control(float Speed, float Acceleration,
                                              float Angle) {
  if (drw.run_mode.data != 1 && pattern == 2) {
    Disenable_Motor(0);
    usleep(1000);
    Set_RobStrite_Motor_parameter(0X7005, PosPP_control_mode, Set_mode);
    usleep(1000);
    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    enable_motor();
    usleep(1000);
  }

  Motor_Set_All.set_speed = Speed;
  Motor_Set_All.set_acc = Acceleration;
  Motor_Set_All.set_angle = Angle;

  Set_RobStrite_Motor_parameter(0X7024, Motor_Set_All.set_speed, Set_parameter);
  usleep(1000);

  Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_acc, Set_parameter);
  usleep(1000);

  Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
  usleep(1000);

  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

// 电流模式
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_Current_control(float IqCommand) {
  if (drw.run_mode.data != 3) {
    Disenable_Motor(0);
    usleep(1000);
    Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);
    usleep(1000);
    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    enable_motor();
    usleep(1000);
  }

  // Store the target values
  Motor_Set_All.set_iq = IqCommand;

  Motor_Set_All.set_iq =
      float_to_uint(Motor_Set_All.set_iq, SCIQ_MIN, SC_MAX, 16);
  Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_iq, Set_parameter);
  usleep(1000);

  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

void RobStrideMotor::RobStrite_Motor_Set_Zero_control() {
  Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode,
                                Set_mode); // 设置电机模式
}

void RobStrideMotor::Disenable_Motor(uint8_t clear_error) {
  struct can_frame frame{};
  frame.can_id =
      (Communication_Type_MotorStop << 24) | (master_id << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 8;
  memset(frame.data, 0, 8);

  frame.data[0] = clear_error;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  int n = write(socket_fd, &frame, sizeof(frame));

  if (n != sizeof(frame)) {
    perror("disable_motor failed");
  } else {
    std::cout << "[✓] Motor disable command sent." << std::endl;
  }
  receive_status_frame();
}

void RobStrideMotor::Set_CAN_ID(uint8_t Set_CAN_ID) {
  Disenable_Motor(0);

  struct can_frame frame{};
  frame.can_id = (Communication_Type_Can_ID << 24) | (Set_CAN_ID << 16) |
                 (master_id << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 8;
  memset(frame.data, 0, 8);

  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  int n = write(socket_fd, &frame, sizeof(frame));

  if (n != sizeof(frame)) {
    perror("Set_ZeroPos failed");
  } else {
    std::cout << "[✓] Motor Set_ZeroPos command sent." << std::endl;
  }
}
// 5.位置模式（CSP）
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_PosCSP_control(float Speed, float Angle) {
  Motor_Set_All.set_speed = Speed;
  Motor_Set_All.set_angle = Angle;
  std::cout << "speed: " << Motor_Set_All.set_speed << std::endl;
  std::cout << "angle: " << Motor_Set_All.set_angle << std::endl;

  if (drw.run_mode.data != 5 && pattern == 2) {
    Disenable_Motor(0);
    usleep(1000);
    Set_RobStrite_Motor_parameter(0X7005, PosCSP_control_mode,
                                  Set_mode); // 设置电机模式
    usleep(1000);

    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);

    enable_motor();
    usleep(1000);

    Motor_Set_All.set_motor_mode = PosCSP_control_mode;
  }
  Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
  Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
  usleep(1000);

  return std::make_tuple(position_, velocity_, torque_, temperature_);
}

void RobStrideMotor::Set_ZeroPos() {
  Disenable_Motor(0);

  if (drw.run_mode.data != 4) {
    Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
    usleep(1000);

    Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
  }

  struct can_frame frame{};
  frame.can_id =
      (Communication_Type_SetPosZero << 24) | (master_id << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG; // 扩展帧
  frame.can_dlc = 8;
  memset(frame.data, 0, 8);

  frame.data[0] = 1;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  int n = write(socket_fd, &frame, sizeof(frame));

  if (n != sizeof(frame)) {
    perror("Set_ZeroPos failed");
  } else {
    std::cout << "[✓] Motor Set_ZeroPos command sent." << std::endl;
  }

  enable_motor();
}