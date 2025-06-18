//
// Created by Mu Shibo on 04/2024
//

#include "motor_control/robstride.h"

/*******************************Mathmatic Function************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

int float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float = *(float*)(&data);
    return data_float;  
}  

/*******************************RobStride Function************************************/
RobStrite_Motor::RobStrite_Motor(uint8_t CAN_Id, ros::NodeHandle* node_handle, const std::string &command_tx)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x00;	
	Motor_Set_All.set_motor_mode = move_control_mode;

    command_pub = node_handle->advertise<can_msgs::Frame>(command_tx, 100);
}

RobStrite_Motor::RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, ros::NodeHandle* node_handle, const std::string &command_tx)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x00;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;

    command_pub = node_handle->advertise<can_msgs::Frame>(command_tx, 100);
}

void RobStrite_Motor::RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId)
{
	if (uint8_t((ID_ExtId&0xFF00)>>8) == CAN_ID)
	{		
		//count_num++;
		if (int((ID_ExtId&0x3F000000)>>24) == 2)
		{
			Pos_Info.Angle =  uint16_to_float(DataFrame[0]<<8|DataFrame[1],P_MIN,P_MAX,16);
			Pos_Info.Speed =  uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
			Pos_Info.Torque = uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
			Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7])*0.1;
			error_code = uint8_t((ID_ExtId&0x3F0000)>>16);
			Pos_Info.pattern = uint8_t((ID_ExtId&0xC00000)>>22);
		}
		else if (int((ID_ExtId&0x3F000000)>>24) == 17)
		{
			for (int index_num = 0; index_num <= 13; index_num++)
			{
				if ((DataFrame[1]<<8|DataFrame[0]) == Index_List[index_num])
					switch(index_num)
					{
						case 0:
							drw.run_mode.data = uint8_t(DataFrame[4]);
							break;
						case 1:
							drw.iq_ref.data = Byte_to_float(DataFrame);
							break;
						case 2:
							drw.spd_ref.data = Byte_to_float(DataFrame);
							break;
						case 3:
							drw.imit_torque.data = Byte_to_float(DataFrame);
							break;
						case 4:
							drw.cur_kp.data = Byte_to_float(DataFrame);
							break;
						case 5:
							drw.cur_ki.data = Byte_to_float(DataFrame);
							break;
						case 6:
							drw.cur_filt_gain.data = Byte_to_float(DataFrame);
							break;
						case 7:
							drw.loc_ref.data = Byte_to_float(DataFrame);
							break;
						case 8:
							drw.limit_spd.data = Byte_to_float(DataFrame);
							break;
						case 9:
							drw.limit_cur.data = Byte_to_float(DataFrame);
							break;	
						case 10:
							drw.mechPos.data = Byte_to_float(DataFrame);
							break;	
						case 11:
							drw.iqf.data = Byte_to_float(DataFrame);
							break;	
						case 12:
							drw.mechVel.data =Byte_to_float(DataFrame);
							break;	
						case 13:
							drw.VBUS.data = Byte_to_float(DataFrame);
							break;	
					}	
			}
		}
		else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
		{
			CAN_ID = uint8_t((ID_ExtId & 0xFF00)>>8);	
		}	
	}
}

void RobStrite_Motor::RobStrite_Get_CAN_ID()
{   
    can_msgs::Frame cansendata;
    
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID;
    cansendata.dlc = 0x08;
    cansendata.data[0] = 0x00;
    cansendata.data[1] = 0x00;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

    command_pub.publish(cansendata);
}

/**********************************MIT CONTROL MODE****************************/
//0.运控模式
void RobStrite_Motor::RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	Motor_Set_All.set_Torque = Torque;
	Motor_Set_All.set_angle = Angle;	
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_Kp = Kp;
	Motor_Set_All.set_Kd = Kd;
	if (drw.run_mode.data != 0 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);		//设置电机模式
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = move_control_mode;
	}

    can_msgs::Frame cansendata;
    cansendata.is_extended = true; // 设置为扩展帧
    cansendata.is_rtr = false;
    cansendata.dlc = 0x08;

	cansendata.id = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID;
	cansendata.data[0] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16)>>8; 
	cansendata.data[1] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16); 
	cansendata.data[2] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16)>>8; 
	cansendata.data[3] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16); 
	cansendata.data[4] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16)>>8; 
	cansendata.data[5] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16); 
	cansendata.data[6] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16)>>8; 
	cansendata.data[7] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16); 
    
    command_pub.publish(cansendata);
}

//1.位置模式（PP）
void RobStrite_Motor::RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_acc   = Acceleration;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 1 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, PosPP_control_mode, Set_mode);		//设置电机模式
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = PosPP_control_mode;
	}
	Set_RobStrite_Motor_parameter(0X7024, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_acc,   Set_parameter);
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
}

//2.速度模式
void RobStrite_Motor::RobStrite_Motor_Speed_control(float Current_Limits, float Accel, float Speed) 
{
  // Store the target values
  Motor_Set_All.set_limit_cur = Current_Limits;
  Motor_Set_All.set_acc = Accel;
  Motor_Set_All.set_speed = Speed;

  // Check if we need to change the motor mode
  if (drw.run_mode.data != 2 && Pos_Info.pattern == 2) {
    Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
    Get_RobStrite_Motor_parameter(0x7005);
    Motor_Set_All.set_motor_mode = Speed_control_mode;
  }

  Motor_Set_All.set_limit_cur = float_to_uint(Motor_Set_All.set_limit_cur, SC_MIN,SC_MAX, 16);
  Motor_Set_All.set_speed = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16);
  // Set parameters
  Set_RobStrite_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
  Set_RobStrite_Motor_parameter(0X7022, Motor_Set_All.set_acc, Set_parameter);
  Set_RobStrite_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);

}
//3.电流模式
void RobStrite_Motor::RobStrite_Motor_Current_control(float IqCommand, float IdCommand) {
  // Store the target values
  Motor_Set_All.set_iq = IqCommand;
  Motor_Set_All.set_id = IdCommand;

  // Check if we need to change the motor mode
  if (drw.run_mode.data != 3 && Pos_Info.pattern == 2) {
    Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);
    Get_RobStrite_Motor_parameter(0x7005);
    Motor_Set_All.set_motor_mode = Elect_control_mode;
  }
  Motor_Set_All.set_iq = float_to_uint(Motor_Set_All.set_iq, SCIQ_MIN,SC_MAX, 16);
  Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_iq, Set_parameter);
  Set_RobStrite_Motor_parameter(0X7007, Motor_Set_All.set_id, Set_parameter);
}
//5.位置模式（CSP）
void RobStrite_Motor::RobStrite_Motor_PosCSP_control(float Speed, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 5 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, PosCSP_control_mode, Set_mode);		//设置电机模式
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = PosCSP_control_mode;
	}
	Motor_Set_All.set_speed = float_to_uint(Motor_Set_All.set_speed, SC_MIN,V_MAX, 16);
	Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);

}

void RobStrite_Motor::RobStrite_Motor_Set_Zero_control()
{
	Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//设置电机模式
}

void RobStrite_Motor::Enable_Motor()
{
    can_msgs::Frame cansendata;
    cansendata.header.stamp = ros::Time::now();
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.dlc = 0x08;
    cansendata.id = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_ID;
    cansendata.data[0] = 0x00;
    cansendata.data[1] = 0x00;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

    command_pub.publish(cansendata);
}

void RobStrite_Motor::Disenable_Motor(uint8_t clear_error)
{
    can_msgs::Frame cansendata;
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|CAN_ID;
    cansendata.dlc = 0x08;
    cansendata.data[0] = clear_error;
    cansendata.data[1] = 0x00;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

    command_pub.publish(cansendata);
}

void RobStrite_Motor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{   
    can_msgs::Frame cansendata;
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
    cansendata.dlc = 0x08;

    cansendata.data[0] = Index;
    cansendata.data[1] = Index>>8;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    
	if (Value_mode == 'p')
	{
		memcpy(&cansendata.data[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		Motor_Set_All.set_motor_mode = int(Value);
		cansendata.data[4] = (uint8_t)Value;
		cansendata.data[5] = 0x00;	
		cansendata.data[6] = 0x00;	
		cansendata.data[7] = 0x00;	
	}
    
    command_pub.publish(cansendata);
}

void RobStrite_Motor::Get_RobStrite_Motor_parameter(uint16_t Index)
{   
    can_msgs::Frame cansendata;
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
    cansendata.dlc = 0x08;
    cansendata.data[0] = Index;
    cansendata.data[1] = Index>>8;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

	command_pub.publish(cansendata);
}

void RobStrite_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);

    can_msgs::Frame cansendata;
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
    cansendata.dlc = 0x08;
    cansendata.data[0] = 0x00;
    cansendata.data[1] = 0x00;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

	command_pub.publish(cansendata);
}

void RobStrite_Motor::Set_ZeroPos()
{
	Disenable_Motor(0);

    can_msgs::Frame cansendata;
    cansendata.is_extended = true;
    cansendata.is_rtr = false;
    cansendata.id = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
    cansendata.dlc = 0x08;
    cansendata.data[0] = 1;
    cansendata.data[1] = 0x00;
    cansendata.data[2] = 0x00;
    cansendata.data[3] = 0x00;
    cansendata.data[4] = 0x00;
    cansendata.data[5] = 0x00;
    cansendata.data[6] = 0x00;
    cansendata.data[7] = 0x00;

	command_pub.publish(cansendata);

	Enable_Motor();
}

data_read_write::data_read_write(const uint16_t *index_list)
{
	run_mode.index = index_list[0];
	iq_ref.index = index_list[1];
	spd_ref.index = index_list[2];
	imit_torque.index = index_list[3];
	cur_kp.index = index_list[4];
	cur_ki.index = index_list[5];
	cur_filt_gain.index = index_list[6];
	loc_ref.index = index_list[7];
	limit_spd.index = index_list[8];
	limit_cur.index = index_list[9];
	mechPos.index = index_list[10];
	iqf.index = index_list[11];
	mechVel.index = index_list[12];
	VBUS.index = index_list[13];	
	rotation.index = index_list[14];
}




