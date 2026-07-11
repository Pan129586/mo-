#include "control.h"

#include "Hardware/DL_KEY/bsp_key.h"
#include <math.h>


#define four_corner_count        (4U)    // 跑完一圈需要的直角数
#define corner_base_speed     (45.0f)     // 遇到直角时的降速目标
#define lost_base_speed     (30.0f)      // 脱线时的寻线速度
#define line_base_speed     (60.0f)  
#define lost_stop_times       (25)       // 连续脱线多少个时间周期之后认为是迷失并且刹车
#define corner_wait_time   (45)       // 直角确认的防抖时间

 
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
volatile uint8_t g_target_circle = 1;
volatile uint8_t g_compt_corner = 0;
volatile uint32_t g_run_20ms = 0;

static uint8_t ture_wait_time = 0;
volatile uint32_t  time_20ms_flag=0;

static float vofa_data[5];
static uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};


//计算当前设定的圈数一共需要跑多少个直角
static uint8_t totall_corners(void)
{
    return (uint8_t)(g_target_circle * four_corner_count);
}





static void rest_pid(void)
{
    set_pid_target(&pid_speed, 0.0f);
    set_pid_target(&pid_speed2, 0.0f);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    MotorOutput(0, 0);
}


//直角计圈防抖
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

static float select_speed(void)
{
    //如果处于脱线状态，切入极低速寻线模式
    if (Lost_Line_Count > 0U) 
    {
        return lost_base_speed;
    }

    //如果遇到直角特征，或者当前车身偏离黑线很远
    if ((Corner_Flag != 0U) || (fabsf(Line_Num) > 22.0f)) 
    {
        return corner_base_speed;
    }

    // 正常直线行驶
    return line_base_speed;  //（60）
}


void run_data_init(void)
{
    g_traceState = RUN_STATE_READY;
    g_target_circle = 1;
    g_compt_corner = 0;
    g_run_20ms = 0;
    ture_wait_time = 0;
    Start_Flag = 0;
    set_basespeed(0.0f, 0.0f);
    rest_pid();
}


void run_start(void)
{
    g_compt_corner = 0;
    g_run_20ms = 0;
    ture_wait_time = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    Lost_Line_Count = 0;  //发车前清除脱线计时，防止开车钱急停的现象

    reset_turn_count();
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
    g_traceState = next_state;
}

void Trace_Task20ms(void)
{
   
    JY61P_Poll();   //里面有死等
    GetMotorPulse();
    Light_Turn_control();   //读取灰度adc的死等

    if (g_traceState != RUN_STATE_RUNNING) 
    {
        MotorOutput(0, 0);
        return;
    }

    g_run_20ms++;  //用来计时小车跑圈的时间
    updata_corner_count();

    //完成的直角数目大于目标的直角数目
    if (g_compt_corner >= totall_corners())
    {
        run_stop(RUN_STATE_FINISHED);   //停车
        
        //该位置后面加入瞄准部分的开启激光
        return;
    }

    if (Lost_Line_Count >= lost_stop_times) 
    {
        run_stop(RUN_STATE_EMERGENCY_STOP);
        return;
    }

    float base_speed = select_speed();   //获取不同情况的速度
    set_basespeed(base_speed, base_speed);

    direct_val = Gray_pd_control();   //

    speed_target = Baseleft - direct_val;
    speed2_target = Baseright + direct_val;

    set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();
    MotorOutput((int)MotorPWM, (int)Motor2PWM);
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
            time_20ms_flag =1;   
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
    return speed_pid_realize(&pid_speed, (float)g_nMotorPulse);
}

float speed2_pid_control(void)
{
    return speed_pid_realize(&pid_speed2, (float)g_nMotor2Pulse);
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
   
    if (nMotorPwm >= 0) {
        set_motor_direction(MOTOR_FWD);
    } 
    else 
    {
        nMotorPwm = -nMotorPwm;
        set_motor_direction(MOTOR_REV);
    }
    if (nMotorPwm > PWM_MAX_PERIOD_COUNT) {
        nMotorPwm = PWM_MAX_PERIOD_COUNT;
    }

    if (nMotor2Pwm >= 0) {
        set_motor2_direction(MOTOR_FWD);
    } else {
        nMotor2Pwm = -nMotor2Pwm;
        set_motor2_direction(MOTOR_REV);
    }
    if (nMotor2Pwm > PWM2_MAX_PERIOD_COUNT) {
        nMotor2Pwm = PWM2_MAX_PERIOD_COUNT;
    }

    set_motor_speed((uint16_t)nMotorPwm);
    set_motor2_speed((uint16_t)nMotor2Pwm);
}

void set_motor_speed(uint16_t v)
{
    uint16_t compare;

    if (v > PWM_PERIOD_COUNT) {
        v = PWM_PERIOD_COUNT;
    }

    dutyfactor = v;
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
    SET_STOP;
    is_motor_en = 0;
}

void set_motor2_speed(uint16_t v)
{
    uint16_t compare;

    if (v > PWM2_PERIOD_COUNT) {
        v = PWM2_PERIOD_COUNT;
    }

    dutyfactor2 = v;
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
    SET2_STOP;
    is_motor2_en = 0;
}
