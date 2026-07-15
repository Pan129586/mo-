#include "control.h"

#include "Hardware/DL_KEY/bsp_key.h"
#include <math.h>

#define JY61P_ISR_BYTE_BUDGET    (32U)
#define FOUR_CORNER_COUNT        (4U)
// #define LINE_BASE_SPEED          (35.0f)
#define BASE_turn_angle             (90.0f) 
#define TURN_PREPARE_TICKS 1
#define CENTER_LOST_TIME   2
#define CENTER_FOUND_TIME  2


 
float Baseleft = 0.0f;
float Baseright = 0.0f;
float MotorPWM = 0.0f;
float Motor2PWM = 0.0f;
float speed_target = 0.0f;
float speed2_target = 0.0f;
float direct_val = 0.0f;

static motor_dir_t direction = MOTOR_FWD;
static motor_dir_t direction2 = MOTOR_FWD;
static uint16_t dutyfactor = 0;
static uint16_t dutyfactor2 = 0;

uint8_t is_motor_en = 0;
uint8_t is_motor2_en = 0;

volatile uint8_t min_circle = 1;  //定义圈数
volatile uint8_t maix_circle =5;
volatile uint8_t turn_flag=0;
float turn_target_yaw=0.0f;
float min_turn_angle = 45.0f;
float turn_start_yaw = 0.0f;
volatile uint8_t flag_20ms = 0;
//速度变量
uint8_t turn_left_speed = 15;
uint8_t turn_r_speed = 60;
volatile uint8_t LINE_BASE_SPEED=60;

uint8_t normal_line_flag = 0;
static uint8_t corner_departed = 0;
uint8_t center_found_count = 0;
uint8_t corner_lock =0;
static uint8_t turn_prepare = 0;  //直角后直行变量
static uint8_t turn_prepare_count = 0;
 static uint8_t center_lost_count = 0;



volatile trace_state_t g_traceState = RUN_STATE_READY;
volatile uint8_t g_target_circle = 1;
volatile uint8_t g_compt_corner = 0;
volatile uint32_t g_run_20ms = 0;
volatile uint32_t g_trace_isr_count = 0U;

volatile uint32_t g_trace_overrun_count = 0U;

static float vofa_data[5];
static uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};


//计算当前设定的圈数一共需要跑多少个直角
static uint8_t totall_corners(void)
{
    return (uint8_t)(g_target_circle * FOUR_CORNER_COUNT);
}

static void rest_pid(void)
{
    set_pid_target(&pid_speed, 0.0f);
    set_pid_target(&pid_speed2, 0.0f);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    PID_reset(&pid_angle);
    MotorOutput(0, 0);
}



void run_data_init(void)
{
    g_traceState = RUN_STATE_READY;
    g_target_circle = 1;
    g_compt_corner = 0;
    g_run_20ms = 0;
    Start_Flag = 0;
    turn_flag =0;
    turn_target_yaw = total_yaw;

    corner_departed = 0;
    center_found_count = 0;

    turn_prepare = 0;
    turn_prepare_count = 0;
    corner_lock = 0;
    center_lost_count = 0;

    Graysensor_ResetState();
    set_basespeed(0.0f, 0.0f);
    rest_pid();

}


void run_start(void)
{
    g_compt_corner = 0;
    g_run_20ms = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    turn_flag =0;
    turn_target_yaw = total_yaw;

    corner_departed = 0;
    center_found_count = 0;
    corner_lock = 0;

     turn_prepare = 0;
    turn_prepare_count = 0;
    center_lost_count = 0;
    
    Graysensor_ResetState();
    reset_turn_count();
    rest_pid();
    set_basespeed(LINE_BASE_SPEED, LINE_BASE_SPEED);
    Start_Flag = 1; 
    g_traceState = RUN_STATE_RUNNING;    
}

void turn_start()
{
    turn_flag =1;
    turn_target_yaw =total_yaw + 90.0f;
    center_lost_count = 0;

    PID_reset(&pid_angle);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    set_pid_target(&pid_angle, turn_target_yaw);
}




void yaw_turn_control(void)
{
    float yaw_error;
    float turn_speed;

     yaw_error = turn_target_yaw - total_yaw;

    if (fabsf(yaw_error) <=BASE_turn_angle )
    {
        MotorOutput(0, 0);
        turn_flag = 0;

        PID_reset(&pid_angle);
        PID_reset(&pid_direct);
        PID_reset(&pid_speed);
        PID_reset(&pid_speed2);

        g_compt_corner++;

        if (g_compt_corner >= totall_corners())
        {
            run_stop(RUN_STATE_FINISHED);
        }

        return;
    }

    turn_speed = yaw_pid_realize(&pid_angle, total_yaw);
     speed_target  =  -turn_speed;
     speed2_target = turn_speed;

     set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();

    MotorOutput(MotorPWM, Motor2PWM);

    if (g_compt_corner >= totall_corners())
    {
        run_stop(RUN_STATE_FINISHED);
    }

}


void run_stop(trace_state_t next_state)
{
    turn_flag =0;
    turn_target_yaw = total_yaw;
    g_traceState = next_state;
    Start_Flag = 0;
    set_basespeed(0.0f, 0.0f);    
    rest_pid();
}


// void start_diff_turn()
// {
//     turn_flag = 1;
//     turn_start_yaw = total_yaw;

// }

void turn_diff_control()
{
    float turned_yaw=0.0f;
    turned_yaw = fabsf(total_yaw - turn_start_yaw);
    if(State_Value[3]==1 || State_Value[4] == 1)
    {
        center_found_flag = 1;

    }

    if(Corner_Flag == 0 && Black_Sensor_Count >=1 && Black_Sensor_Count <=2 && center_found_flag ==1)
    {
        normal_line_flag =1;
    }

    if(turn_flag ==1&&turned_yaw >= min_turn_angle && normal_line_flag ==1)
    {
        turn_flag =0;
         PID_reset(&pid_direct);

          g_compt_corner++;

          if (g_compt_corner >= totall_corners())
          {
                run_stop(RUN_STATE_FINISHED);
          }

         return;
    }

        speed_target  = turn_left_speed;
      speed2_target = turn_r_speed;

      set_pid_target(&pid_speed, speed_target);
        set_pid_target(&pid_speed2, speed2_target);

    MotorPWM  = speed_pid_control();
    Motor2PWM = speed2_pid_control();

    MotorOutput(MotorPWM, Motor2PWM);

}


void Trace_Task20ms(void)
{
     Light_Turn_control();
     (void)JY61P_PollBudget(JY61P_ISR_BYTE_BUDGET);
    GetMotorPulse();

    if (g_traceState != RUN_STATE_RUNNING) 
    {
        MotorOutput(0, 0);
        return;
    }


    g_run_20ms++;

    if ((turn_flag == 0)&& (turn_prepare == 0)&&(Corner_Rise_Flag == 1) &&(corner_lock == 0))
    {
        turn_prepare = 1;
        turn_prepare_count = 0;

        corner_lock = 1;
        corner_departed = 0;
         center_found_count = 0;
          center_lost_count = 0;

         PID_reset(&pid_speed);
        //   PID_reset(&pid_speed2);
    }

    if(turn_prepare !=0)
    {
        speed_target = Baseleft;
        speed2_target = Baseright;

         turn_prepare_count++;
          if (turn_prepare_count >= TURN_PREPARE_TICKS)
          {
            turn_prepare = 0;
            turn_prepare_count = 0;

            turn_flag = 1;
            corner_departed = 0;
            center_found_count = 0;
            center_lost_count = 0;
            

             PID_reset(&pid_speed);
            PID_reset(&pid_speed2);

          }
    }
    else if(turn_flag == 1)
    {

        speed_target  = turn_left_speed;   
        speed2_target = turn_r_speed; 

        if(State_Value[3] == 0 && State_Value[4] ==0)
        {
            if(center_lost_count <CENTER_LOST_TIME)   //连续两个周期中间两个探头都灭掉才是离开直线
            {
                center_lost_count ++;
            }
            
        }
         else 
        {
            center_lost_count =0;
        }

         if(center_lost_count >=CENTER_LOST_TIME)
        {
            corner_departed =1;
        }


        if(corner_departed ==1)
        {
            if(State_Value[3] == 1 ||State_Value[4] ==1)
            {
                if(center_found_count <CENTER_FOUND_TIME )
                {
                    center_found_count ++ ;
                }
            }
            else 
            {
                center_found_count =0;
            }

            if ( (center_found_count >= CENTER_FOUND_TIME) &&(Black_Sensor_Count >= 1))
            {
                turn_flag =0;
                turn_prepare = 0;
                turn_prepare_count = 0;

                corner_departed =0;
                center_found_count =0;
                 center_lost_count = 0;
                corner_lock = 0;
                // PID_reset(&pid_direct);
                g_compt_corner ++;

                if(g_compt_corner >= totall_corners())
                {
                    run_stop(RUN_STATE_FINISHED);
                }
            }
            
        }
    }
    else if(turn_flag == 0) 
    {

        // direct_val = Gray_pd_control();
        // // direct_val = 0;

        // speed_target = Baseleft - direct_val;
        // speed2_target = Baseright + direct_val;
        if (Black_Sensor_Count >= 1)
        {
            direct_val = Gray_pd_control();

            speed_target = Baseleft - direct_val;
            speed2_target = Baseright + direct_val;
        }
        else
        {
            // 暂时保持直行速度，不让旧误差立即把车拉反
            speed_target = Baseleft;
            speed2_target = Baseright;
        }
        
        if((State_Value[0] ==0) && ((State_Value[3]==1)||(State_Value[4]==1)))
        {
            corner_lock =0;
        }
     }

    set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();

    MotorOutput(MotorPWM, Motor2PWM);
    
}

const char *get_runstate(void)
{
    switch (g_traceState) 
    {
        case RUN_STATE_IDLE:
            return "IDLE ";
        case RUN_STATE_READY:
            return "READY";
        case RUN_STATE_RUNNING:
            return "RUN  ";
        case RUN_STATE_FINISHED:
            return "DONE ";
        case RUN_STATE_EMERGENCY_STOP:
            return "STOP ";
        default:
            return "UNKWN";
    }
}



void TIMER_TICK_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_TICK_INST)) 
    {
        case DL_TIMER_IIDX_ZERO:
            flag_20ms = 1;
            g_trace_isr_count++;
            Trace_Task20ms();
            // Send_To_VOFA(LINE_BASE_SPEED,g_fMotorSpeedCmps,LINE_BASE_SPEED,g_fMotor2SpeedCmps);
            // if ((DL_TimerG_getRawInterruptStatus(
            //          TIMER_TICK_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT) &
            //      DL_TIMERG_INTERRUPT_ZERO_EVENT) != 0U)
            // {
            //     g_trace_overrun_count++;
            // }
            break;
        default:
            break;
    }
}

void set_basespeed(float left_base, float right_base)
{
    Baseleft = left_base;
    Baseright = right_base;
}

float Gray_pd_control(void)
{
    set_pid_target(&pid_direct, 0.0f);
    return direct_pid_realize(&pid_direct, Line_Num);
}

//实际编码器反应的数值
float speed_pid_control(void)
{
    // return speed_pid_realize(&pid_speed, (float)g_nMotorPulse);
      return speed_pid_realize(&pid_speed, g_fMotor2SpeedCmps);   //cm/s的单位
 }

float speed2_pid_control(void)
{

    // return speed_pid_realize(&pid_speed2, (float)g_nMotor2Pulse);  //浮点数有影响吗
     return speed_pid_realize(&pid_speed2, g_fMotorSpeedCmps);
}

void UART_SendArray(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_VOFA_INST, data[i]);
    }
}

void Send_To_VOFA(float target_left, float real_left, float target_right,
                  float real_right)
{
    vofa_data[0] = target_left;
    vofa_data[1] = real_left;
    vofa_data[2] = target_right;
    vofa_data[3] = real_right;
    

    UART_SendArray((uint8_t *)vofa_data, sizeof(vofa_data));
    UART_SendArray(vofa_tail, 4);
}



// void MotorOutput(int nMotorPwm, int nMotor2Pwm)
// {
   
//     if (nMotorPwm > 0) 
//     {
//         set_motor_direction(MOTOR_FWD);
//     }
//     else if (nMotorPwm < 0)
//     {
//         nMotorPwm = -nMotorPwm;
//         set_motor_direction(MOTOR_REV);
//     }
//     else
//     {
//         SET_STOP;
//     }
//     if (nMotorPwm > PWM_MAX_PERIOD_COUNT) {
//         nMotorPwm = PWM_MAX_PERIOD_COUNT;
//     }

//     if (nMotor2Pwm > 0) {
//         set_motor2_direction(MOTOR_FWD);
//     }
//     else if
//     (nMotor2Pwm < 0)
//     {
//         nMotor2Pwm = -nMotor2Pwm;
//         set_motor2_direction(MOTOR_REV);
//     }
//     else
//     {
//         SET2_STOP;
//     }
//     if
//     (nMotor2Pwm > PWM2_MAX_PERIOD_COUNT)
//     {
//         nMotor2Pwm = PWM2_MAX_PERIOD_COUNT;
//     }
    
//     set_motor_speed((uint16_t)nMotorPwm);
//     set_motor2_speed((uint16_t)nMotor2Pwm);
// }


void MotorOutput(int left_pwm, int right_pwm)
{
    uint16_t left_duty;
    uint16_t right_duty;

    /*
     * left_pwm  是逻辑左轮，但实际走 Motor2 通道
     * right_pwm 是逻辑右轮，但实际走 Motor1 通道
     */

    if (left_pwm > 0)
    {
        set_motor2_direction(MOTOR_FWD);
        left_duty = (uint16_t)left_pwm;
    }
    else if (left_pwm < 0)
    {
        set_motor2_direction(MOTOR_REV);
        left_duty = (uint16_t)(-left_pwm);
    }
    else
    {
        left_duty = 0U;
        SET2_STOP;
    }

    if (right_pwm > 0)
    {
        set_motor_direction(MOTOR_FWD);
        right_duty = (uint16_t)right_pwm;
    }
    else if (right_pwm < 0)
    {
        set_motor_direction(MOTOR_REV);
        right_duty = (uint16_t)(-right_pwm);
    }
    else
    {
        right_duty = 0U;
        SET_STOP;
    }

    if (left_duty > PWM2_MAX_PERIOD_COUNT)
    {
        left_duty = PWM2_MAX_PERIOD_COUNT;
    }

    if (right_duty > PWM_MAX_PERIOD_COUNT)
    {
        right_duty = PWM_MAX_PERIOD_COUNT;
    }

    // 逻辑左轮 -> Motor2/PWMA/BIN
    set_motor2_speed(left_duty);

    // 逻辑右轮 -> Motor1/PWMB/AIN
    set_motor_speed(right_duty);
}

void set_motor_speed(uint16_t v)
{
    if (v > PWM_PERIOD_COUNT) {
        v = PWM_PERIOD_COUNT;
    }

    dutyfactor = v;
    // compare = (uint16_t)(PWM_PERIOD_COUNT - v);
    SET_COMPAER(v);
}

void set_motor_direction(motor_dir_t dir)
{
    direction = dir;
    if (direction == MOTOR_FWD) {
        SET_FWD;
    } else {
        SET_REV;
    }
}

void set_motor_enable(void)
{
    is_motor_en = 1;
}

void set_motor_disable(void)
{
    set_motor_speed(0U);
    SET_STOP;
    is_motor_en = 0;
}

void set_motor2_speed(uint16_t v)
{
    if (v > PWM2_PERIOD_COUNT) {
        v = PWM2_PERIOD_COUNT;
    }

    dutyfactor2 = v;
    // compare = (uint16_t)(PWM2_PERIOD_COUNT - v);
    SET2_COMPAER(v);
}

void set_motor2_direction(motor_dir_t dir)
{
    direction2 = dir;
    if (direction2 == MOTOR_FWD) {
        SET2_FWD;
    } else {
        SET2_REV;
    }
}

void set_motor2_enable(void)
{
    is_motor2_en = 1;
}

void set_motor2_disable(void)
{
    set_motor2_speed(0U);
    SET2_STOP;
    is_motor2_en = 0;
}
