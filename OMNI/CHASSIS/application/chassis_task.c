#include "chassis_task.h"
#include "cmsis_os.h"
#include "chassis_behaviour.h"
#include "arm_math.h"
#include "communicate_task.h"
#include "chassis_power_control.h"
#include "key_task.h"
#include "detect_task.h"

#define CHASSIS_POWER_CONTROL

static void Chassis_Debug_get_data(void);
static uint8_t chassis_motor_detect(void);


static void chassis_init(chassis_move_t *init);
static void chassis_feedback_update(chassis_move_t *feedback_update);

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_vector_to_omni_calculate(fp32 wheel_speed[4],fp32 vx_set,fp32 vy_set,fp32 wz_set);


static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

chassis_move_t chassis_move;




fp32 yaw_diff=0;

//底盘总任务
void chassis_task(void const * argument)
{
	while(gimbal_data.GimbalInit == 0)
	{
		//等待云台初始化完成
		vTaskDelay(CHASSIS_TASK_INIT_TIME);
	}
		
    //底盘初始化
    chassis_init(&chassis_move);

    //判断底盘电机是否都在线
    while (chassis_motor_detect())
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);//保证全部初始化完毕
    }
		
    while (1)
		{
			//底盘控制模式选择
			chassis_set_mode(&chassis_move);
			//底盘数据更新
			chassis_feedback_update(&chassis_move);
      //底盘控制量设置（设置遥控器输入控制量）
      chassis_set_contorl(&chassis_move);
			//底盘控制
			chassis_control_loop(&chassis_move);			

			
			if(POWER_OFF)
			{
				
				CAN_cmd_drive(0,0,0,0);

			}
			else
			{
				CAN_cmd_drive(chassis_move.chassis_drive_speed_pid[0].out,chassis_move.chassis_drive_speed_pid[1].out,chassis_move.chassis_drive_speed_pid[2].out,chassis_move.chassis_drive_speed_pid[3].out);
			}
			osDelay(1);//控制频率为1khz，与can接收中断频率一致
		}
}


static void chassis_init(chassis_move_t *init)
{
	uint8_t i=0;
	
	static float PID_SPEED_DRIVE[4][3]={DRIVE_MOTOR1_SPEED_PID_KP,DRIVE_MOTOR1_SPEED_PID_KI,DRIVE_MOTOR1_SPEED_PID_KD,
																			DRIVE_MOTOR2_SPEED_PID_KP,DRIVE_MOTOR2_SPEED_PID_KI,DRIVE_MOTOR2_SPEED_PID_KD,
																			DRIVE_MOTOR3_SPEED_PID_KP,DRIVE_MOTOR3_SPEED_PID_KI,DRIVE_MOTOR3_SPEED_PID_KD,
																			DRIVE_MOTOR4_SPEED_PID_KP,DRIVE_MOTOR4_SPEED_PID_KI,DRIVE_MOTOR4_SPEED_PID_KD};
	
	
	static float YAW_PID_MAX[2] = {YAW_PID_MAX_OUT,YAW_PID_MAX_IOUT};
	static float YAW_ANGLE_PID[3] = {YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI,YAW_ANGLE_PID_KD};
	
	
	static float SPEED_PID_MAX_OUT_DRIVE[4]={DRIVE_MOTOR1_SPEED_PID_MAX_OUT,DRIVE_MOTOR2_SPEED_PID_MAX_OUT,DRIVE_MOTOR3_SPEED_PID_MAX_OUT,DRIVE_MOTOR4_SPEED_PID_MAX_OUT}; 
	static float SPEED_PID_MAX_IOUT_DRIVE[4]={DRIVE_MOTOR1_SPEED_PID_MAX_IOUT,DRIVE_MOTOR2_SPEED_PID_MAX_IOUT,DRIVE_MOTOR3_SPEED_PID_MAX_IOUT,DRIVE_MOTOR4_SPEED_PID_MAX_IOUT}; 
	
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
	
	init->chassis_RC=get_remote_control_point();
	init->chassis_bmi088_data = get_INS_data_point();
	init->get_gimbal_data = get_gimbal_data_point();
	PID_init(&init->yaw_pid,DATA_NORMAL,YAW_ANGLE_PID,YAW_PID_MAX[0],YAW_PID_MAX[1],NONE);//初始化底盘PID
	
	//初始化驱动速度PID 并获取电机数据
	for(i=0;i<4;i++)
	{
		init->motor_chassis[i].chassis_motor_measure = get_chassis_drive_motor_measure_point(i);//获取底盘3508的数据，接收电机的反馈结构体		
		PID_init(&init->chassis_drive_speed_pid[i],DATA_NORMAL,PID_SPEED_DRIVE[i],SPEED_PID_MAX_OUT_DRIVE[i],SPEED_PID_MAX_IOUT_DRIVE[i],NONE);//初始化底盘PID
		init->drive_set_speed[i]=0.0f;
	}
	
	//用一阶滤波
  first_order_filter_init(&init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
  first_order_filter_init(&init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME_Z, chassis_z_order_filter);
	//最大 最小速度
  init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
  init->wz_max_speed = NORMAL_MAX_CHASSIS_SPEED_W;
  init->wz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_W;


}









/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_behaviour_mode_set(chassis_move_mode);
}





/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	//求航向电机的角度
	chassis_vector_to_omni_calculate(chassis_move_control_loop->drive_set_speed,chassis_move_control_loop->vx_set,chassis_move_control_loop->vy_set,chassis_move_control_loop->wz_set);


	//计算pid
	for (int i = 0; i < 4; i++)
		{
        PID_calc(&chassis_move_control_loop->chassis_drive_speed_pid[i], 
					chassis_move_control_loop->motor_chassis[i].speed,chassis_move_control_loop->drive_set_speed[i]);
		}


		#ifdef CHASSIS_POWER_CONTROL
		chassis_power_control();
		#endif


}


/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *feedback_update)
{
	uint8_t i = 0;
	
	//更新驱动电机实际速度-->额定3.15m/s  (3.54m/s,电机转不到)
	for (i = 0; i < 4; i++)
	{
		feedback_update->motor_chassis[i].speed = feedback_update->motor_chassis[i].chassis_motor_measure->rpm * (CHASSIS_MOTOR_RPM_TO_VECTOR_SEN);
	}
	
	feedback_update->wz_rad = feedback_update->chassis_bmi088_data->gyro[INS_GYRO_X_ADDRESS_OFFSET];
	
	feedback_update->chassis_yaw = feedback_update->chassis_bmi088_data->INS_angle[INS_YAW_ADDRESS_OFFSET];

	
		yaw_diff = motor_ecd_to_angle_change(yaw_motor.ecd,5900);
	
	
	
	chassis_power_feedback(&PowerLimit);
	#ifdef CHASSIS_DEBUG
	Chassis_Debug_get_data();
	#endif
}


/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
		//检查是否为空指针
    if (chassis_move_control == NULL)
    {
        return;
    }
			//设置速度
		
    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

			//根据底盘状态设置来计算vx_set，vy_set，wz_set
    chassis_behaviour_control_set(&vx_set,&vy_set,&wz_set,chassis_move_control);
    
		//跟随云台yaw,左上
		if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DIRECTION_FOLLOW_GIMBAL_YAW)
    {
			fp32 angle_diff;	

				angle_diff=rad_format(yaw_diff* PI / 180.0f);
			chassis_move_control->vx_set = vy_set * cos(angle_diff) - vx_set * sin(angle_diff);
			chassis_move_control->vy_set = vy_set * sin(angle_diff) + vx_set * cos(angle_diff);

				PID_calc(&chassis_move_control->yaw_pid,angle_diff,0);
				chassis_move_control->wz_set = chassis_move_control->yaw_pid.out;

			chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
		}
		
		//不跟随云台，左下
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TOP_FOLLOW_GIMBAL_YAW)
		{
			static uint8_t Dir = 0;
			if(chassis_move_control->last_chassis_mode != CHASSIS_VECTOR_TOP_FOLLOW_GIMBAL_YAW)
			{
				Dir++;
				Dir%=2;
			}

			 fp32 angle_diff=rad_format(yaw_diff* PI / 180.0f); 
			 
			chassis_move_control->vx_set = vy_set * cos(angle_diff) - vx_set * sin(angle_diff);
			chassis_move_control->vy_set = vy_set * sin(angle_diff) + vx_set * cos(angle_diff);
//			if(Dir)
//			{	
//				if(chassis_move_control->get_gimbal_data->rc_data.rc_key_v & KEY_PRESSED_OFFSET_SHIFT  )
//						chassis_move_control->wz_set = 2.0f;	
//				else
//					chassis_move_control->wz_set = 1.5f;
//			}
//			else
//			{
				if(chassis_move_control->get_gimbal_data->rc_data.rc_key_v & KEY_PRESSED_OFFSET_SHIFT  )
						chassis_move_control->wz_set = -1.7f;	
				else
					chassis_move_control->wz_set = -1.25;
//			}

			 chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
			 chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			 chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
		}
		
		//底盘无力，左中
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_ZERO_FORCE)
    {

				 chassis_move_control->vx_set = 0;
				 chassis_move_control->vy_set = 0;
				 chassis_move_control->wz_set = 0;	
			
    }
		
		

		//飞坡模式，左中
		else if(chassis_move_control->chassis_mode == CHASSIS_VECTOR_FLY)
		{
			fp32 angle_diff;
			
			angle_diff=rad_format(yaw_diff* PI / 180.0f);
			
			chassis_move_control->vx_set = vy_set * cos(angle_diff) - vx_set * sin(angle_diff);
			chassis_move_control->vy_set = vy_set * sin(angle_diff) + vx_set * cos(angle_diff);
			

				PID_calc(&chassis_move_control->yaw_pid,angle_diff,0);
				chassis_move_control->wz_set = chassis_move_control->yaw_pid.out;

		
			

			
			chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
		}
}




static void chassis_vector_to_omni_calculate(fp32 wheel_speed[4],fp32 vx_set,fp32 vy_set,fp32 wz_set)
{

		wheel_speed[0] =  -vx_set - vy_set +  wz_set ;
    wheel_speed[1] =   vx_set - vy_set +  wz_set ;
    wheel_speed[2] =   vx_set + vy_set +  wz_set ;
    wheel_speed[3] =  -vx_set + vy_set +  wz_set ;
}




/**
  * @brief          根据遥控器通道值，计算纵向和横移速度               
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	int16_t vx_channel, vy_channel, wz_channel;
  fp32 rc_vx=0, rc_vy=0,rc_wz=0;
	fp32 key_vx=0,key_vy=0;
	fp32 final_vx=0,final_vy=0,final_wz=0;
	
//  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
//  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
//  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
//  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_W_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);
	
  rc_deadband_limit(chassis_move_rc_to_vector->get_gimbal_data->rc_data.vx_set, vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->get_gimbal_data->rc_data.vy_set, vy_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->get_gimbal_data->rc_data.wz_set, wz_channel, CHASSIS_RC_DEADLINE);

		rc_vx = vx_channel * -CHASSIS_VX_RC_SEN;
    rc_vy = -vy_channel * -CHASSIS_VY_RC_SEN;
	  rc_wz = wz_channel * CHASSIS_WZ_RC_SEN;

    //键盘控制
    if (chassis_move_rc_to_vector->get_gimbal_data->rc_data.rc_key_v & KEY_PRESSED_OFFSET_D)
    {
        key_vx = -MAX_SPEED;
    }
    else if (chassis_move_rc_to_vector->get_gimbal_data->rc_data.rc_key_v  & KEY_PRESSED_OFFSET_A)
    {
        key_vx = MAX_SPEED;
    }

    if (chassis_move_rc_to_vector->get_gimbal_data->rc_data.rc_key_v  & KEY_PRESSED_OFFSET_S)
    {
        key_vy = -MAX_SPEED;
    }
    else if (chassis_move_rc_to_vector->get_gimbal_data->rc_data.rc_key_v  & KEY_PRESSED_OFFSET_W)
    {
        key_vy =MAX_SPEED;
    }
	
	final_vx = key_vx+rc_vx;
	final_vy = key_vy+rc_vy;
	final_wz = rc_wz;		
		
	final_wz = fp32_constrain(final_wz, chassis_move_rc_to_vector->wz_min_speed, chassis_move_rc_to_vector->wz_max_speed);
	final_vx = fp32_constrain(final_vx, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);	
	final_vy = fp32_constrain(final_vy, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);	
		
	//一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, final_vx);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, final_vy);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, final_wz);
	
  if (final_vx < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && final_vx > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
      chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (final_vy < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && final_vy > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
      chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }
  if (final_wz < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && final_wz > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
  {
      chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
  }    
	
	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
	*wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;

}


/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}



static uint8_t chassis_motor_detect(void)
{
	return 	toe_is_error(DRIVE_MOTOR1_TOE)  || toe_is_error(DRIVE_MOTOR2_TOE)  || toe_is_error(DRIVE_MOTOR3_TOE)  || toe_is_error(DRIVE_MOTOR4_TOE);
}


/* ----------------------------------------------------------------------- */
/* -------------------------------vofa调参-------------------------------- */
/* ----------------------------------------------------------------------- */
static void Chassis_Debug_get_data(void)
{
	chassis_move.chassis_debug_data.data1 = chassis_move.motor_chassis[0].speed;
	chassis_move.chassis_debug_data.data2 = chassis_move.motor_chassis[1].speed;
	chassis_move.chassis_debug_data.data3 = chassis_move.motor_chassis[2].speed;
	
	chassis_move.chassis_debug_data.data4 = chassis_move.motor_chassis[3].speed;
	
//	chassis_move.chassis_debug_data.data4 = chassis_move.drive_set_speed[0];
//	chassis_move.chassis_debug_data.data5 = chassis_move.drive_set_speed[1];
//	chassis_move.chassis_debug_data.data6 = chassis_move.drive_set_speed[2];
}
const DebugData* get_chassis_Debug(void)
{
	return &chassis_move.chassis_debug_data;
}


