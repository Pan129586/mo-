#include "bsp_pid.h"


_pid pid_speed, pid_speed2,low_pid_speed,low_pid_speed2;    
_pid pid_direct;
_pid pid_position,pid_position2;  
_pid pid_angle;


void PID_param_init()
{
  //角度
   pid_direct.target_val=0.0;				
    pid_direct.actual_val=0.0;
    pid_direct.err=0.0;
    pid_direct.err_last=0.0;
    pid_direct.integral=0.0;

		pid_direct.Kp = 0.65;
		pid_direct.Ki = 0.0;
		pid_direct.Kd = 0.24;


    pid_direct.target_val=0.0;				
    pid_direct.actual_val=0.0;
    pid_direct.err=0.0;
    pid_direct.err_last=0.0;
    pid_direct.integral=0.0;

		pid_direct.Kp = 0.65;
		pid_direct.Ki = 0.0;
		pid_direct.Kd = 0.24;
	
  
    pid_speed.target_val=0.0;				
    pid_speed.actual_val=0.0;
    pid_speed.err=0.0;
    pid_speed.err_last=0.0;
    pid_speed.integral=0.0;
  
		pid_speed.Kp =1.8;
		pid_speed.Ki = 1.2;
		pid_speed.Kd = 0.1;
		
			  
 
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
		pid_speed2.Kp = 2.0;
		pid_speed2.Ki = 1.2;
		pid_speed2.Kd = 0.1;
		
		
    pid_position.target_val = 0.0;				
    pid_position.actual_val = 0.0;
    pid_position.err = 0.0;
    pid_position.err_last = 0.0;
    pid_position.integral = 0.0;
  
	pid_position.Kp = 0.5;   
	pid_position.Ki = 0.0;   
	pid_position.Kd = 0.1;  
	
	 pid_position2.target_val = 0.0;				
    pid_position2.actual_val = 0.0;
    pid_position2.err = 0.0;
    pid_position2.err_last = 0.0;
    pid_position2.integral = 0.0;
  
	pid_position2.Kp = 0.5;  
	pid_position2.Ki = 0.0;   
	pid_position2.Kd = 0.1;  
	
		
	
	
	
		
#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);    
#endif
}


void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;   
}


float get_pid_target(_pid *pid)
{
  return pid->target_val;    
}


void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    
		pid->Ki = i;    
		pid->Kd = d;   
}


float yaw_pid_realize(_pid *pid, float actual_val) 
{

    pid->err = pid->target_val - actual_val;

    pid->actual_val = pid->Kp * pid->err + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    if(pid->actual_val > 150) pid->actual_val = 150; 
    else if(pid->actual_val < -150) pid->actual_val = -150;

    return pid->actual_val;
}


float position_pid_realize(_pid *pid, float actual_val) 
{

    pid->err = pid->target_val - actual_val;

    pid->integral += pid->err; 
	if(pid->err<=0.5&&-0.5<=pid->err)
	{
		pid->err=0.0;
	}
  //PD的算法
    pid->actual_val = pid->Kp * pid->err + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;
    return pid->actual_val;
}



float direct_pid_realize(_pid *pid, float actual_val) 
{

    pid->err=pid->target_val-actual_val;
  
  //   if((pid->err >= -1) && (pid->err <= 1)) 
  //   {
  //     pid->err = 0;
  //     pid->integral = 0;
  //   }
    
    pid->integral += pid->err;    

    pid->actual_val = pid->Kp*pid->err
		                  
		                  +pid->Kd*(pid->err-pid->err_last);
  

    pid->err_last=pid->err;
    // 输出限幅 (防止转弯过猛)
    if(pid->actual_val > 150)pid->actual_val=150;
    else if(pid->actual_val < -150)pid->actual_val=-150;
    
    return pid->actual_val;
}


float speed_pid_realize(_pid *pid, float actual_val)
{

    pid->err=pid->target_val-actual_val;

    // if((pid->err<1.0f ) && (pid->err>-1.0f)) 
		// {
    //   pid->err = 0.0f;
		// }
	
		
    pid->integral += pid->err;   
  //积分抗饱和限幅
	   	 if (pid->integral >= 800) {pid->integral =800;}
      else if (pid->integral < -800)  {pid->integral = -800;}

    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
    pid->err_last=pid->err;
    
    return pid->actual_val;
}


void PID_reset(_pid *pid)
{
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->actual_val = 0;
}
