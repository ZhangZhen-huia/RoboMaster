#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "key_task.h"


Chassis_Power_limit_t PowerLimit;
static void Chassis_power(void);
static uint16_t SuperPower_Strategy(void);


SuperPowerState_e SuperPowerState;
void chassis_power_control(void);


void chassis_power_feedback(Chassis_Power_limit_t *power_control)
{
	power_control->Chassis_Max_power = Referee_System.ext_game_robot_state.chassis_power_limit;
	Chassis_power();
}


void chassis_power_control(void)
{

		fp32 a,b,c,temp;
		fp32 consume_power = 0;
		fp32 initial_give_power[4];
		fp32 available_power;
		fp32 factor;
		

			if(SuperPower_data.voltage >= 5)
				SuperPowerState = OPEN;
			else
				SuperPowerState = CLOSE;
		
		
		/*-- 开启超电，大幅度超功率 --*/
		if((chassis_move.get_gimbal_data->rc_data.rc_key_v & KEY_PRESSED_OFFSET_SHIFT)&& (SuperPowerState == OPEN))
		{
			available_power = PowerLimit.Chassis_Max_power + SuperPower_Strategy();//获取可用功率
			
		}
		/*-- 不开启超电 --*/
		else
		{
			SuperPowerState = CLOSE;
			available_power = PowerLimit.Chassis_Max_power;
		}
		
		/*--	计算预测的功率		--*/
		for(uint8_t i=0;i<4;i++)
		{

			//3508功率计算
			initial_give_power[i] = TOQUE_COEFFICIENT_3508 * chassis_move.chassis_drive_speed_pid[i].out * chassis_move.motor_chassis[i].chassis_motor_measure->rpm	//,力矩和转速（力矩使用电流发送值代替，因为其是线性关系）
																																	 +POWER_3508_K1 * chassis_move.chassis_drive_speed_pid[i].out * chassis_move.chassis_drive_speed_pid[i].out//,力矩平方
																																	 +POWER_3508_K2 * chassis_move.motor_chassis[i].chassis_motor_measure->rpm * chassis_move.motor_chassis[i].chassis_motor_measure->rpm + POWER_CONSTANT;//,转速平方

				//正功计入消耗	，负功计入补偿
			if(initial_give_power[i] > 0)
				consume_power +=initial_give_power[i];
			else
				available_power -=initial_give_power[i];

		}
		
		
		
		/*-- 判断是否超功率 --*/
		if(consume_power > available_power)
			factor = available_power/consume_power;
		else
			factor = 1.0f;
		
		


			/*--	功率分配 	--*/
			for(uint8_t i=0;i<4;i++)
			{
				/*-- 3508超出可用的功率才进行功率重新分配 --*/
				if(factor != 1)
				{
					a = POWER_3508_K1;
					b	= TOQUE_COEFFICIENT_3508*chassis_move.motor_chassis[i].chassis_motor_measure->rpm;
					c = POWER_3508_K2*chassis_move.motor_chassis[i].chassis_motor_measure->rpm * chassis_move.motor_chassis[i].chassis_motor_measure->rpm - available_power/4.0f/*drive_initial_give_power[i] * factor*/ + POWER_CONSTANT;	//二元一次方程的c
					
					if(chassis_move.chassis_drive_speed_pid[i].out > 0)
					{
							temp=(-b+sqrt(b*b-4*a*c))/(2*a);//二元一次方程的正解
							if(temp>16000)//限制最大解
								chassis_move.chassis_drive_speed_pid[i].out = 16000;
							else
								chassis_move.chassis_drive_speed_pid[i].out = temp;
					}
					else//向后走取负解
					{
						 temp=(-b-sqrt(b*b-4*a*c))/(2*a);//二元一次方程的负解
						if(temp<-16000)//限制最大解
							chassis_move.chassis_drive_speed_pid[i].out = -16000;
						else
							chassis_move.chassis_drive_speed_pid[i].out = temp;		
					}
				}
		
		}
}



static uint16_t SuperPower_Strategy(void)
{
	if(chassis_move.chassis_mode == CHASSIS_VECTOR_FLY)
	{
		if(Referee_System.ext_power_heat_data.chassis_power_buffer >= 5)
		{
			SuperPowerState = OPEN;
			return 1000;
		}
		else
		{
			SuperPowerState = CLOSE;
			return 0;
		}
	}
	else
	{
		if(Referee_System.ext_power_heat_data.chassis_power_buffer >= 25)
		{
				SuperPowerState = OPEN;
				return (Referee_System.ext_game_robot_state.robot_level+2)*30;
		}
		else
		{
				SuperPowerState = CLOSE;
				return 0;
		}
	}
}


static void Chassis_power(void)
{
	PowerLimit.chassis_power_data_debug.data1 = chassis_move.motor_chassis[0].chassis_motor_measure->rpm;	
	PowerLimit.chassis_power_data_debug.data2 = chassis_move.motor_chassis[1].chassis_motor_measure->rpm;	
	PowerLimit.chassis_power_data_debug.data3 = chassis_move.motor_chassis[2].chassis_motor_measure->rpm;	
	
	PowerLimit.chassis_power_data_debug.data4 = chassis_move.motor_chassis[3].chassis_motor_measure->rpm;	
	PowerLimit.chassis_power_data_debug.data6 = PowerLimit.Chassis_Max_power;
}

const DebugData * get_chassis_power(void)
{
	return &PowerLimit.chassis_power_data_debug;
}




