#ifndef __BSP_PID_H
#define	__BSP_PID_H

#include "ti_msp_dl_config.h"
// #include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>





typedef struct
{
    float target_val;               
    float actual_val;        		
    float err;             			
    float err_last;          		
    float Kp,Ki,Kd;          		
    float integral;          		
}_pid;

extern _pid pid_speed, pid_speed2;    
extern _pid pid_direct;
extern _pid pid_position,pid_position2;

void PID_reset(_pid *pid);
 void PID_param_init(void);
 void set_pid_target(_pid *pid, float temp_val);
 float get_pid_target(_pid *pid);
 void set_p_i_d(_pid *pid, float p, float i, float d);


float direct_pid_realize(_pid *pid, float actual_val);
float speed_pid_realize(_pid *pid, float actual_val);
float position_pid_realize(_pid *pid, float actual_val);


#endif
