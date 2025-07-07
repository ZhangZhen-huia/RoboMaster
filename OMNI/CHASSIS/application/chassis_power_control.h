#ifndef __CHASSIS_POWER_CONTROL_H
#define __CHASSIS_POWER_CONTROL_H

#include "main.h"
#include "vofa_task.h"


#define TOQUE_COEFFICIENT_3508 1.99688994e-6f

/*
k1调，使其车堵转，使裁判系统功率达到功率上限附近，超功率时加大k1.没达到时则减小
k2调，使其车小陀螺，使裁判系统功率达到功率上限附近，超功率时加大k2.没达到时则减小

调试原理：公式-P=k1*F*F+b*V*F+k2*V*V+C,(二元一次方程)
因为堵转时速度基本很小，功率主要为力矩决定，所以k1影响大。调k1
小陀螺时速度主导，则调k2
*/
#define POWER_3508_K1 			1.894e-07f
#define POWER_3508_K2			 	1.453e-07f




#define POWER_CONSTANT  4.0810f
#define CAP_POWER_OPEN  200
#define CAP_POWER_CLOSE 5

#define MAX_POWER 100



typedef enum
{
	OPEN,
	CLOSE
}SuperPowerState_e;





extern SuperPowerState_e SuperPowerState;

typedef struct
{
	fp32 Chassis_Max_power;//底盘最大功率
	DebugData chassis_power_data_debug;
}Chassis_Power_limit_t;



extern Chassis_Power_limit_t PowerLimit;
void chassis_power_control(void);
void chassis_power_feedback(Chassis_Power_limit_t *power_control);





#endif
