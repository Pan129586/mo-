#ifndef __CONTROL_H
#define __CONTROL_H

#include "ti_msp_dl_config.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"
#include "Hardware/DL_JY61P/JY61P.h"
#include "Hardware/DL_pid/bsp_pid.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
// #include "usart.h"
// #include "tim.h"
// #include "adc.h"
// #include "K230.h"
// #include "bsp_led.h"



// 正转：AIN1=高，AIN2=低
#define SET_FWD  do{ \
    DL_GPIO_setPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN); \
}while(0)

// 反转：AIN1=低，AIN2=高
#define SET_REV  do{ \
    DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN); \
}while(0)

// 刹车/停止：AIN1=低，AIN2=低
#define SET_STOP do{ \
    DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN); \
}while(0)

// 正转：BIN1=高，BIN2=低
#define SET2_FWD  do{ \
    DL_GPIO_setPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN); \
}while(0)

// 反转：BIN1=低，BIN2=高
#define SET2_REV  do{ \
    DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN); \
}while(0)

// 刹车/停止：BIN1=低，BIN2=低
#define SET2_STOP do{ \
    DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN); \
}while(0)

//PWM 输出宏定义   两个通道
#define SET_COMPAER(ChannelPulse)     DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, ChannelPulse, DL_TIMER_CC_0_INDEX)
#define SET2_COMPAER(ChannelPulse)    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, ChannelPulse, DL_TIMER_CC_1_INDEX)


 /* 编码器物理分辨率 */
#define ENCODER_RESOLUTION                     13

/* 经过倍频之后的总分辨率 */
#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4倍频后的总分辨率 */
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (1000)   
#define PWM2_PERIOD_COUNT     (1000)
/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 30)    //如果PWM弄成了满的，一些驱动板就会出现问题（硬件上的原因）
#define PWM2_MAX_PERIOD_COUNT              (PWM2_PERIOD_COUNT - 30)

/* 减速电机减速比 */
#define REDUCTION_RATIO  20
#define SPEED_PID_PERIOD  20    //这个要看定时器1的中断周期
#define R1  95                 //cm
#define R2  105
#define WheelR  2.4
#define lunju 14
#define BASE  60


typedef enum
{
  left_90,
	right_90,
	back_180
}spin_dir_t;  


/* 电机方向控制枚举 */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
}motor_dir_t;


extern unsigned char g_ucMainEventCount;
extern unsigned char g_ucMainEventCountLong;
//extern unsigned char TASK_NUM;
extern uint8_t is_motor_en;


extern int32_t  actual_speed;


extern float Baseleft,Baseright;
extern uint8_t Start_Flag;
extern volatile uint8_t tim1_20ms_flag;   //定时器的20ms标志位

extern float current_base_L;   //小车稍微大于死区的基础速度
extern float current_base_R ;
extern float base_speed_output,base_speed2_output;




void set_basespeed(float left_base,float right_base);
void set_motor_speed(uint16_t v);
void set_motor_direction(motor_dir_t dir);
void set_motor_enable(void);
void set_motor_disable(void);
void set_motor2_speed(uint16_t v);
void set_motor2_direction(motor_dir_t dir);
void set_motor2_enable(void);
void set_motor2_disable(void);
float Gray_pd_control(void);
float speed_pid_control(void);
float speed2_pid_control(void);
void MotorOutput(int nMotorPwm,int nMotor2Pwm);
float Gray_pd_control(void);

void Send_To_VOFA(float target_left, float real_left, float target_right, float real_right,float Line_Num);
#endif 

