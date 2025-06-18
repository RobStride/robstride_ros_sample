//
// Created by Mu Shibo on 04/2024
//

#ifndef _ROBSTRIDE_H_
#define _ROBSTRIDE_H_

#include "ros/ros.h"
#include "stdint.h"
#include "can_msgs/Frame.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f 
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

#define SC_MAX 23.0f
#define SC_MIN 0.0f
#define SV_MAX 20.0f
#define SV_MIN -20.0f
#define SCIQ_MIN -23.0f

#define Set_mode 	  'j' //设置控制模式
#define Set_parameter 'p' //设置参数
//各种控制模式
#define move_control_mode  0 //运控模式
#define Pos_control_mode   1 //位置模式
#define Speed_control_mode 2 //速度模式
#define Elect_control_mode 3 //电流模式
#define Set_Zero_mode      4 //零点模式
//通信地址
#define Communication_Type_Get_ID 0x00  //获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl 0x01  //运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02  //用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03  //电机使能运行
#define Communication_Type_MotorStop 0x04  //电机停止运行
#define Communication_Type_SetPosZero 0x06  //设置电机机械零位
#define Communication_Type_Can_ID 0x07  //更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12  //设置电机模式
#define Communication_Type_GetSingleParameter 0x11  //读取单个参数
#define Communication_Type_SetSingleParameter 0x12  //设定单个参数
#define Communication_Type_ErrorFeedback 0x15  //故障反馈帧

class data_read_write_one
{
	public:
		uint16_t index;
		float data;
};
static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};
//18通信类型可以写入的参数列表
//参数变量名  参数地址  描述  类型  字节数  单位/说明
class data_read_write
{
	
	public:
		data_read_write_one run_mode; //0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式 uint8  1byte
		data_read_write_one iq_ref; //电流模式Iq指令  float 	4byte 	-23~23A
		data_read_write_one spd_ref; //转速模式转速指令  float 	4byte 	-30~30rad/s 
		data_read_write_one imit_torque; //转矩限制  float 	4byte 	0~12Nm  
		data_read_write_one cur_kp; //电流的 Kp  float 	4byte 	默认值 0.125  
		data_read_write_one cur_ki; //电流的 Ki  float 	4byte 	默认值 0.0158  
		data_read_write_one cur_filt_gain; //电流滤波系数filt_gain  float 	4byte 	0~1.0，默认值0.1  
		data_read_write_one loc_ref; //位置模式角度指令  float 	4byte 	rad  
		data_read_write_one limit_spd; //位置模式速度设置  float 	4byte 	0~30rad/s  
		data_read_write_one limit_cur; //速度位置模式电流设置  float 	4byte 	0~23A
		//以下只可读
		data_read_write_one mechPos; //负载端计圈机械角度  float 	4byte 	rad
		data_read_write_one iqf; //iq 滤波值  float 	4byte 	-23~23A
		data_read_write_one	mechVel; //负载端转速  float 	4byte 	-30~30rad/s 	
		data_read_write_one	VBUS; //母线电压  float 	4byte 	V	
		data_read_write_one	rotation; //圈数  int16 	2byte   圈数
		data_read_write(const uint16_t *index_list=Index_List);
};
typedef struct
{
	float Angle;
	float Speed;
	float Torque;
	float Temp;
	int pattern; //电机模式（0复位1标定2运行）
}Motor_Pos_RobStrite_Info;

typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_Kp;
	float set_Ki;
	float set_Kd;
	float set_iq;
	float set_id;
	float set_acc;
}Motor_Set;

//RobStrite_Motor电机
class RobStrite_Motor
{
private:		
	uint8_t CAN_ID;						//CAN ID   (默认127(0x7f) 可以通过上位机和通信类型1查看)
	uint16_t Master_CAN_ID;		//主机ID  （会在初始化函数中设定为0x1F）
	float (*Motor_Offset_MotoFunc)(float Motor_Tar);

	Motor_Set Motor_Set_All;		//设定值

public:
	// creat publisher
	ros::Publisher command_pub;
	
	float output;
	int Can_Motor;
	Motor_Pos_RobStrite_Info Pos_Info;
	data_read_write drw;
    uint8_t error_code;
    
	RobStrite_Motor(uint8_t CAN_Id, ros::NodeHandle* node_handle, const std::string &command_tx);
	RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, ros::NodeHandle* node_handle, const std::string &command_tx);
	void RobStrite_Get_CAN_ID();
	void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);
	void Get_RobStrite_Motor_parameter(uint16_t Index);
	void RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId);
	void RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
	void RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);
	void RobStrite_Motor_Speed_control(float Current_Limits, float Accel, float Speed);
	void RobStrite_Motor_Current_control(float IqCommand, float IdCommand);
	void RobStrite_Motor_PosCSP_control(float Speed, float Angle);
	void RobStrite_Motor_Set_Zero_control();
	void Enable_Motor();
	void Disenable_Motor( uint8_t clear_error);
	void Set_CAN_ID(uint8_t Set_CAN_ID);
	void Set_ZeroPos();

};

#endif