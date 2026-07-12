#include "control.h"

#include "Hardware/DL_KEY/bsp_key.h"
#include <math.h>


#define four_corner_count        (4U)    // 跑完一圈需要的直角数
#define corner_base_speed     (30.0f)     // 遇到直角时的降速目标
// #define lost_base_speed     (50.0f)      // 脱线时的寻线速度
#define line_base_speed     (40.0f)
/* Yaw Turn Config */
#ifndef TRACE_TURN_DIR
#define TRACE_TURN_DIR             (1.0f)
#endif
#define TRACE_TURN_ANGLE_DEG       (90.0f)
#define TRACE_TURN_TOLERANCE_DEG   (6.0f)
#define TRACE_TURN_SETTLE_TICKS    (3U)
#define TRACE_REACQUIRE_SPEED      (28.0f)
#define TRACE_REACQUIRE_STEER_MAX  (12.0f)
#define TRACE_REACQUIRE_OK_TICKS   (3U)

/* Drive Feedback Protection */
#define FEEDBACK_TARGET_MIN         (15.0f)
#define FEEDBACK_ACTUAL_MIN         (3.0f)
#define FEEDBACK_REVERSE_TICKS      (5U)

 
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

volatile trace_state_t g_traceState = RUN_STATE_READY;
volatile trace_phase_t g_tracePhase = TRACE_PHASE_LINE;
volatile uint8_t g_target_circle = 1;
volatile uint8_t g_compt_corner = 0;
volatile uint32_t g_run_20ms = 0;
volatile float g_turn_target_yaw = 0.0f;
volatile float g_turn_yaw_error = 0.0f;
volatile drive_fault_t g_drive_fault = DRIVE_FAULT_NONE;
volatile uint32_t g_motor1_reverse_feedback_count = 0U;
volatile uint32_t g_motor2_reverse_feedback_count = 0U;

volatile uint32_t  time_20ms_flag=0;
volatile uint32_t g_trace_overrun_count = 0U;
static uint8_t s_turn_settle_ticks = 0U;
static uint8_t s_reacquire_ok_ticks = 0U;
static uint8_t s_motor1_reverse_ticks = 0U;
static uint8_t s_motor2_reverse_ticks = 0U;
static int8_t s_motor1_last_target_sign = 0;
static int8_t s_motor2_last_target_sign = 0;

static float vofa_data[5];
static uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};


//计算当前设定的圈数一共需要跑多少个直角
static uint8_t totall_corners(void)
{
    return (uint8_t)(g_target_circle * four_corner_count);
}

static int8_t target_sign(float value)
{
    if (value > 0.0f)
    {
        return 1;
    }
    if (value < 0.0f)
    {
        return -1;
    }
    return 0;
}

static void reset_feedback_streaks(void)
{
    s_motor1_reverse_ticks = 0U;
    s_motor2_reverse_ticks = 0U;
    s_motor1_last_target_sign = 0;
    s_motor2_last_target_sign = 0;
}

static void reset_feedback_diagnostics(void)
{
    reset_feedback_streaks();
    g_drive_fault = DRIVE_FAULT_NONE;
    g_motor1_reverse_feedback_count = 0U;
    g_motor2_reverse_feedback_count = 0U;
}

static uint8_t feedback_is_reversed(float target, float actual,
    uint8_t *reverse_ticks, int8_t *last_target_sign,
    volatile uint32_t *reverse_count)
{
    int8_t current_sign = target_sign(target);

    if (current_sign != *last_target_sign)
    {
        *last_target_sign = current_sign;
        *reverse_ticks = 0U;
    }

    if ((fabsf(target) < FEEDBACK_TARGET_MIN) ||
        (fabsf(actual) < FEEDBACK_ACTUAL_MIN) ||
        ((target * actual) >= 0.0f))
    {
        *reverse_ticks = 0U;
        return 0U;
    }

    (*reverse_count)++;
    if (*reverse_ticks < UINT8_MAX)
    {
        (*reverse_ticks)++;
    }

    return (*reverse_ticks >= FEEDBACK_REVERSE_TICKS) ? 1U : 0U;
}





static void rest_pid(void)
{
    set_pid_target(&pid_speed, 0.0f);
    set_pid_target(&pid_speed2, 0.0f);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    PID_reset(&pid_angle);
    reset_feedback_streaks();
    MotorOutput(0, 0);
}


//直角计圈防抖
#if 0
static void updata_corner_count(void)
{
    // 获取底层识别到的直角总数

    uint8_t yaw_corners = 0;
    if(turn_90_count<0)
    {
        yaw_corners = 0;
    }
    else 
    {
        yaw_corners=turn_90_count;
    }

    // 如果检测到直角信号上升沿（刚压到直角线）
    if (Corner_Rise_Flag != 0) 
    {
        // 开启防抖窗口，在此窗口时间内，允许更新计数值
        ture_wait_time = corner_wait_time;
    } 
    else if (ture_wait_time > 0U) 
    {
        ture_wait_time--;
    }

    // 只有当底层直角数增加了，并且 (还在防抖窗口内 或者 传感器当前正压着直角线)
    if ((yaw_corners > g_compt_corner) &&
        ((ture_wait_time > 0) || (Corner_Flag != 0))) 
        {
           //确认有效，更新已完成的直角总数
        g_compt_corner = yaw_corners;
        ture_wait_time = 0;  //防止同一直角重复计数
    }
}

#endif


static void reset_trace_phase(void)
{
    g_tracePhase = TRACE_PHASE_LINE;
    g_turn_target_yaw = total_yaw;
    g_turn_yaw_error = 0.0f;
    s_turn_settle_ticks = 0U;
    s_reacquire_ok_ticks = 0U;

    set_pid_target(&pid_angle, total_yaw);
    PID_reset(&pid_angle);
}

 void apply_speed_targets(float left_target, float right_target)
{
    drive_fault_t cycle_fault = DRIVE_FAULT_NONE;

    speed_target = left_target;
    speed2_target = right_target;

    set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    if (feedback_is_reversed(speed_target, (float)g_nMotorPulse,
        &s_motor1_reverse_ticks, &s_motor1_last_target_sign,
        &g_motor1_reverse_feedback_count) != 0U)
    {
        cycle_fault = (drive_fault_t)(cycle_fault |
            DRIVE_FAULT_MOTOR1_FEEDBACK_REVERSED);
    }
    if (feedback_is_reversed(speed2_target, (float)g_nMotor2Pulse,
        &s_motor2_reverse_ticks, &s_motor2_last_target_sign,
        &g_motor2_reverse_feedback_count) != 0U)
    {
        cycle_fault = (drive_fault_t)(cycle_fault |
            DRIVE_FAULT_MOTOR2_FEEDBACK_REVERSED);
    }
    if (cycle_fault != DRIVE_FAULT_NONE)
    {
        g_drive_fault = cycle_fault;
        run_stop(RUN_STATE_EMERGENCY_STOP);
        return;
    }

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();

    MotorOutput((int)MotorPWM, (int)Motor2PWM);
}

 void enter_yaw_turn(void)
{
    g_tracePhase = TRACE_PHASE_YAW_TURN;

    g_turn_target_yaw = total_yaw + (TRACE_TURN_DIR * TRACE_TURN_ANGLE_DEG);
    g_turn_yaw_error = g_turn_target_yaw - total_yaw;
    s_turn_settle_ticks = 0U;
    PID_reset(&pid_angle);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    reset_feedback_streaks();
    set_pid_target(&pid_angle, g_turn_target_yaw);
}

 void enter_reacquire(void)   //进行转弯的时候实现角度pid
{
    g_tracePhase = TRACE_PHASE_REACQUIRE;
    s_reacquire_ok_ticks = 0U;
    PID_reset(&pid_angle);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    reset_feedback_streaks();
}

 void yaw_turn_control(void)
{
    float turn_output;

    g_turn_yaw_error = g_turn_target_yaw - total_yaw;
    if (fabsf(g_turn_yaw_error) <= TRACE_TURN_TOLERANCE_DEG)
    {
        if (s_turn_settle_ticks == 0U)
        {
            PID_reset(&pid_speed);
            PID_reset(&pid_speed2);
        }
        if (s_turn_settle_ticks < UINT8_MAX)
        {
            s_turn_settle_ticks++;
        }
        MotorOutput(0, 0);

        if (s_turn_settle_ticks >= TRACE_TURN_SETTLE_TICKS)
        {
            if (g_compt_corner < UINT8_MAX)
            {
                g_compt_corner++;
            }
            if (g_compt_corner >= totall_corners())
            {
                run_stop(RUN_STATE_FINISHED);
            }
            else
            {
                g_tracePhase = TRACE_PHASE_LINE; 
                PID_reset(&pid_direct);
                // enter_reacquire();
            }
        }
        return;
    }

    if (s_turn_settle_ticks != 0U)
    {
        s_turn_settle_ticks = 0U;
        PID_reset(&pid_speed);
        PID_reset(&pid_speed2);
    }
    turn_output = yaw_pid_realize(&pid_angle, total_yaw);
    apply_speed_targets(-turn_output, turn_output);
}

//  void reacquire_line_control(void)
// {
//     float steer;

//     if ((Black_Sensor_Count > 0U) && (Corner_Flag == 0U))
//     {
//         if (s_reacquire_ok_ticks < UINT8_MAX)
//         {
//             s_reacquire_ok_ticks++;
//         }
//     }
//     else
//     {
//         s_reacquire_ok_ticks = 0U;
//     }

//     if (s_reacquire_ok_ticks >= TRACE_REACQUIRE_OK_TICKS)
//     {
//         g_tracePhase = TRACE_PHASE_LINE;
//         PID_reset(&pid_direct);
//         return;
//     }
//     steer = Gray_pd_control();
//     if (steer > TRACE_REACQUIRE_STEER_MAX)
//     {
//         steer = TRACE_REACQUIRE_STEER_MAX;
//     }
//     else if (steer < -TRACE_REACQUIRE_STEER_MAX)
//     {
//         steer = -TRACE_REACQUIRE_STEER_MAX;
//     }
//     apply_speed_targets(TRACE_REACQUIRE_SPEED - steer,
//                         TRACE_REACQUIRE_SPEED + steer);
// }


// static float select_speed(void)
// {
//     //如果处于脱线状态，切入极低速寻线模式
//     if (Lost_Line_Count > 0U) 
//     {
//         return lost_base_speed;
//     }

//     //如果遇到直角特征，或者当前车身偏离黑线很远
//     if ((Corner_Flag != 0U) || (fabsf(Line_Num) > 22.0f)) 
//     {
//         return corner_base_speed;
//     }

//     // 正常直线行驶
//     return line_base_speed;  //（60）
// }


void run_data_init(void)
{
    g_traceState = RUN_STATE_READY;
    g_target_circle = 1;
    g_compt_corner = 0;
    g_run_20ms = 0;
    Start_Flag = 0;
    reset_feedback_diagnostics();
    reset_trace_phase();
    set_basespeed(0.0f, 0.0f);
    rest_pid();
}


void run_start(void)
{
    g_compt_corner = 0;
    g_run_20ms = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    Lost_Line_Count = 0;  //发车前清除脱线计时，防止开车钱急停的现象
    reset_feedback_diagnostics();

    reset_turn_count();
    reset_trace_phase();
    rest_pid();
    set_basespeed(line_base_speed, line_base_speed);
    Start_Flag = 1; 
    g_traceState = RUN_STATE_RUNNING;    
}

void run_stop(trace_state_t next_state)
{
    Start_Flag = 0;
    set_basespeed(0.0f, 0.0f);    //基础速度？还是设置目标速度
    rest_pid();
    reset_trace_phase();
    g_traceState = next_state;
}

void Trace_Task20ms(void)
{
    
    JY61P_Poll();
    GetMotorPulse();

    if (g_traceState != RUN_STATE_RUNNING) 
    {
        MotorOutput(0, 0);
        return;
    }

    Light_Turn_control();

    g_run_20ms++;

    // if (g_tracePhase == TRACE_PHASE_YAW_TURN)
    // {
    //     yaw_turn_control();
    //     return;
    // }
    // //这是在干嘛
    // if (g_tracePhase == TRACE_PHASE_REACQUIRE)
    // {
    //     reacquire_line_control();
    //     return;
    // }


    // if (Corner_Rise_Flag == 1)   //检测到直角
    // {
    //     enter_yaw_turn();
    //     yaw_turn_control();
    //     return;
    // }

    // float base_speed = select_speed();   //获取不同情况的速度
    set_basespeed(line_base_speed, line_base_speed);

    direct_val = Gray_pd_control();   //
    // direct_val = 0;

    speed_target = Baseleft - direct_val;
    speed2_target = Baseright + direct_val;

    apply_speed_targets(speed_target, speed2_target);
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
            time_20ms_flag = 1U;
            Trace_Task20ms();
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

float speed_pid_control(void)
{
    // return speed_pid_realize(&pid_speed, (float)g_nMotorPulse);
      return speed_pid_realize(&pid_speed, g_fMotorSpeedCmps);   //cm/s的单位
 }

float speed2_pid_control(void)
{

    // return speed_pid_realize(&pid_speed2, (float)g_nMotor2Pulse);  //浮点数有影响吗
     return speed_pid_realize(&pid_speed2, g_fMotor2SpeedCmps);
}

void UART_SendArray(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_VOFA_INST, data[i]);
    }
}

void Send_To_VOFA(float target_left, float real_left, float target_right,
                  float real_right, float line_num)
{
    vofa_data[0] = target_left;
    vofa_data[1] = real_left;
    vofa_data[2] = target_right;
    vofa_data[3] = real_right;
    vofa_data[4] = line_num;

    UART_SendArray((uint8_t *)vofa_data, sizeof(vofa_data));
    UART_SendArray(vofa_tail, 4);
}

void MotorOutput(int nMotorPwm, int nMotor2Pwm)
{
   
    if (nMotorPwm > 0) {
        set_motor_direction(MOTOR_FWD);
    }
    else if (nMotorPwm < 0)
    {
        nMotorPwm = -nMotorPwm;
        set_motor_direction(MOTOR_REV);
    }
    else
    {
        SET_STOP;
    }
    if (nMotorPwm > PWM_MAX_PERIOD_COUNT) {
        nMotorPwm = PWM_MAX_PERIOD_COUNT;
    }

    if (nMotor2Pwm > 0) {
        set_motor2_direction(MOTOR_FWD);
    }
    else if
    (nMotor2Pwm < 0)
    {
        nMotor2Pwm = -nMotor2Pwm;
        set_motor2_direction(MOTOR_REV);
    }
    else
    {
        SET2_STOP;
    }
    if
    (nMotor2Pwm > PWM2_MAX_PERIOD_COUNT)
    {
        nMotor2Pwm = PWM2_MAX_PERIOD_COUNT;
    }

    set_motor_speed((uint16_t)nMotorPwm);
    set_motor2_speed((uint16_t)nMotor2Pwm);
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
