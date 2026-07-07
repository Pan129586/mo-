#include "control.h"



float Baseleft = 0.0,Baseright = 0.0; //基础速度
float  MotorPWM =0.0, Motor2PWM =0.0;
float speed_target;
float speed2_target;
float variation_R;
float direct_val;
float jy_speed;


static motor_dir_t direction  = MOTOR_FWD;     // 记录电机方向
 uint16_t    dutyfactor = 0;             // 记录电机占空比
static motor_dir_t direction2  = MOTOR_FWD;     // 记录电机2方向
 uint16_t    dutyfactor2 = 0;             // 记录电机2占空比
uint8_t is_motor_en = 0, is_motor2_en = 0;            // 电机使能


float g_fTargetJourney = 0;  //存放小车左右轮所走路程和 ， 单位cm，需要在下一阶段任务中设置


float current_base_L = 0.0; 
float current_base_R = 0.0;
float base_speed_output=0.0;
float base_speed2_output=0.0;

//VOFA
float vofa_data[5];
uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};


void TIMER_TICK_INST_IRQHandler(void)
{
   switch (DL_TimerG_getPendingInterrupt(TIMER_TICK_INST))  
    {
        case DL_TIMER_IIDX_ZERO:     //倒计时模式
            
            JY61P_Poll();          // 陀螺仪轮询
            GetMotorPulse();       // 提取编码器脉冲
            Light_Turn_control();  

            if(Start_Flag == 1)
            {
                direct_val = Gray_pd_control();

                speed_target = Baseleft - direct_val;
                speed2_target = Baseright + direct_val;

                set_pid_target(&pid_speed, speed_target);
                set_pid_target(&pid_speed2, speed2_target);

                MotorPWM = speed_pid_control();
                Motor2PWM = speed2_pid_control();

                MotorOutput(MotorPWM, Motor2PWM);
            }
            else
            {
                set_pid_target(&pid_speed, 0);
                set_pid_target(&pid_speed2, 0);

                MotorPWM = speed_pid_control();
                Motor2PWM = speed2_pid_control();

                MotorOutput(MotorPWM, Motor2PWM);
            }
            break;
        default:
            break;
    }
}


	

void set_basespeed(float left_base,float right_base)
{
    Baseleft = left_base;
    Baseright = right_base;
}


//灰度控制循迹
float Gray_pd_control(void)
{
	
	float cont_val = 0.0;
	
    set_pid_target(&pid_direct, 0); 
    cont_val = direct_pid_realize(&pid_direct, Line_Num);    // 进行 PID 计算
	
	
    return cont_val;
}


float speed_pid_control(void)  
{
   
    float cont_val = 0.0;                       // 当前控制值
    int32_t actual_speed;
	
       actual_speed = g_nMotorPulse;	
    cont_val = speed_pid_realize(&pid_speed, actual_speed);    // 进行 PID 计算
    
//	 #if defined(PID_ASSISTANT_EN)
//   set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_speed, 1);                // 给通道 1 发送实际值
// 
// #endif
	
	return cont_val;
}

float speed2_pid_control(void)  
{
   
    float cont_val = 0.0;                       // 当前控制值
    int32_t actual_speed;
	
	
      actual_speed = g_nMotor2Pulse;
	  cont_val = speed_pid_realize(&pid_speed2, actual_speed);    // 进行 PID 计算
		
    
// #if defined(PID_ASSISTANT_EN)
//   set_computer_value(SEND_FACT_CMD, CURVES_CH2, &actual_speed, 1);                // 给通道 1 发送实际值
////  #else
////    printf("实际值：%d. 目标值：%.0f\n", actual_speed, get_pid_target());      // 打印实际值和目标值
// #endif
	
	return cont_val;
}


//使用uart1
void UART_SendArray(uint8_t *data, uint16_t len) 
{
    for(uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_VOFA_INST, data[i]);
    }
}

void Send_To_VOFA(float target_left, float real_left, float target_right, float real_right, float Line_Num)
{
    vofa_data[0] = target_left;
    vofa_data[1] = real_left;
    vofa_data[2] = target_right;
    vofa_data[3] = real_right;
    vofa_data[4] = Line_Num;
    
    UART_SendArray((uint8_t *)vofa_data, sizeof(vofa_data));
    UART_SendArray(vofa_tail, 4);
}


/*****************电机的控制函数***************/

void MotorOutput(int nMotorPwm,int nMotor2Pwm)//设置电机电压和方向
{
		if (nMotorPwm >= 0)    // 判断电机方向         
		{
			set_motor_direction(MOTOR_FWD);   //正方向要对应
		}
		else
		{
			nMotorPwm = -nMotorPwm;    
			set_motor_direction(MOTOR_REV);   //正方向要对应
		}
		nMotorPwm = ((nMotorPwm > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT : nMotorPwm);    // 速度上限处理
		
		
		if (nMotor2Pwm >= 0)    // 判断电机方向         
		{
			set_motor2_direction(MOTOR_FWD);   //正方向要对应
		}
		else
		{
			nMotor2Pwm = -nMotor2Pwm;    
			set_motor2_direction(MOTOR_REV);   //正方向要对应
		}
		
		nMotor2Pwm = ((nMotor2Pwm > PWM_MAX_PERIOD_COUNT) ? PWM_MAX_PERIOD_COUNT : nMotor2Pwm);    // 速度上限处理
		
		set_motor_speed(nMotorPwm);        // 设置 PWM 占空比
		set_motor2_speed(nMotor2Pwm);      // 设置 PWM 占空比
 }

 
/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor_speed(uint16_t v)    //这种还是单极性控制。。。。。如果想要更大的力，就要改成双极性的来。
{
  dutyfactor = v;
  SET_COMPAER(v);
}

/**
  * @brief  设置电机方向
  * @param  无
  * @retval 无
  */
void set_motor_direction(motor_dir_t dir)    //这个要改为平衡车的前进和后退
{
  direction = dir;
  
  if (direction == MOTOR_FWD)
  {
	SET_FWD;
  }
  else
  {
	SET_REV;
  }
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_motor_enable(void)   //这俩个使能和禁用的函数对于双极性控制来说还有效吗？
{
	is_motor_en  = 1;
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_motor_disable(void)
{
    SET_STOP;
	
	is_motor_en  = 0; 
}


/*****************电机2的控制函数***************/
/**
  * @brief  设置电机2速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor2_speed(uint16_t v)
{
  dutyfactor2 = v;   
  SET2_COMPAER(v);
}

/**
  * @brief  设置电机2方向
  * @param  无
  * @retval 无
  */
void set_motor2_direction(motor_dir_t dir)  
{
  direction2 =  dir;    
  
  if (direction2 == MOTOR_FWD) 
  {
    SET2_FWD;
  }
  else
  {
   SET2_REV;
  }
}

/**
  * @brief  使能电机2
  * @param  无
  * @retval 无
  */
void set_motor2_enable(void)
{
	is_motor2_en  = 1;
	

}

/**
  * @brief  禁用电机2
  * @param  无
  * @retval 无
  */
void set_motor2_disable(void)
{
    SET2_STOP;
	
	is_motor2_en  = 0;


}





