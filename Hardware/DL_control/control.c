#include "control.h"

#include "Hardware/DL_KEY/bsp_key.h"
#include <math.h>


#define four_corner_count        (4U)    // 跑完一圈需要的直角数
#define corner_base_speed     (30.0f)     // 遇到直角时的降速目标
#define lost_base_speed     (25.0f)      // 脱线时的寻线速度
#define line_base_speed     (45.0f)  
// #define lost_stop_times       (25)       // 连续脱线多少个时间周期之后认为是迷失并且刹车

/* Yaw Turn Config */
#ifndef TRACE_TURN_DIR
#define TRACE_TURN_DIR             (1.0f)
#endif
#define TRACE_TURN_ANGLE_DEG       (90.0f)
#define TRACE_TURN_TOLERANCE_DEG   (6.0f)
#define TRACE_TURN_SETTLE_TICKS    (3U)
#define TRACE_TURN_TIMEOUT_TICKS   (75U)
#define TRACE_YAW_STALE_TICKS      (25U)
#define TRACE_REACQUIRE_SPEED      (28.0f)
#define TRACE_REACQUIRE_STEER_MAX  (12.0f)
#define TRACE_REACQUIRE_OK_TICKS   (3U)
#define TRACE_REACQUIRE_TIMEOUT    (50U)

 
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
volatile float lost_stop_times = 25U;

volatile uint32_t  time_20ms_flag=0;
volatile uint32_t g_trace_overrun_count = 0U;
static uint16_t s_phase_ticks = 0U;
static uint16_t s_yaw_stale_ticks = 0U;
static uint8_t s_turn_settle_ticks = 0U;
static uint8_t s_reacquire_ok_ticks = 0U;
static uint32_t s_last_yaw_frame_count = 0U;

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
    PID_reset(&pid_angle);
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
    s_phase_ticks = 0U;
    s_yaw_stale_ticks = 0U;
    s_turn_settle_ticks = 0U;
    s_reacquire_ok_ticks = 0U;
    s_last_yaw_frame_count = jy_valid_yaw_count;

    set_pid_target(&pid_angle, total_yaw);
    PID_reset(&pid_angle);
}

 void apply_speed_targets(float left_target, float right_target)
{
    speed_target = left_target;
    speed2_target = right_target;

    set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();
    MotorOutput((int)MotorPWM, (int)Motor2PWM);
}

 void enter_yaw_turn(void)
{
    g_tracePhase = TRACE_PHASE_YAW_TURN;

    g_turn_target_yaw = total_yaw + (TRACE_TURN_DIR * TRACE_TURN_ANGLE_DEG);
    g_turn_yaw_error = g_turn_target_yaw - total_yaw;
    s_phase_ticks = 0U;
    s_yaw_stale_ticks = 0U;
    s_turn_settle_ticks = 0U;
    s_last_yaw_frame_count = jy_valid_yaw_count;
    PID_reset(&pid_angle);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    set_pid_target(&pid_angle, g_turn_target_yaw);
}

 void enter_reacquire(void)
{
    g_tracePhase = TRACE_PHASE_REACQUIRE;
    s_phase_ticks = 0U;
    s_reacquire_ok_ticks = 0U;
    PID_reset(&pid_angle);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
}

 void yaw_turn_control(void)
{
    float turn_output;

    s_phase_ticks++;
    if (jy_valid_yaw_count != s_last_yaw_frame_count)
    {
        s_last_yaw_frame_count = jy_valid_yaw_count;
        s_yaw_stale_ticks = 0U;
    }
    else if (s_yaw_stale_ticks < UINT16_MAX)
    {
        s_yaw_stale_ticks++;
    }

    if ((s_phase_ticks >= TRACE_TURN_TIMEOUT_TICKS) ||
        (s_yaw_stale_ticks >= TRACE_YAW_STALE_TICKS))
    {
        run_stop(RUN_STATE_EMERGENCY_STOP);
        return;
    }

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
                enter_reacquire();
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

 void reacquire_line_control(void)
{
    float steer;

    s_phase_ticks++;
    if ((Black_Sensor_Count > 0U) && (Corner_Flag == 0U))
    {
        if (s_reacquire_ok_ticks < UINT8_MAX)
        {
            s_reacquire_ok_ticks++;
        }
    }
    else
    {
        s_reacquire_ok_ticks = 0U;
    }

    if (s_reacquire_ok_ticks >= TRACE_REACQUIRE_OK_TICKS)
    {
        g_tracePhase = TRACE_PHASE_LINE;
        s_phase_ticks = 0U;
        PID_reset(&pid_direct);
        return;
    }
    if (s_phase_ticks >= TRACE_REACQUIRE_TIMEOUT)
    {
        run_stop(RUN_STATE_EMERGENCY_STOP);
        return;
    }

    steer = Gray_pd_control();
    if (steer > TRACE_REACQUIRE_STEER_MAX)
    {
        steer = TRACE_REACQUIRE_STEER_MAX;
    }
    else if (steer < -TRACE_REACQUIRE_STEER_MAX)
    {
        steer = -TRACE_REACQUIRE_STEER_MAX;
    }
    apply_speed_targets(TRACE_REACQUIRE_SPEED - steer,
                        TRACE_REACQUIRE_SPEED + steer);
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
    Start_Flag = 0;
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
    GetMotorPulse();
    Light_Turn_control();   //读取灰度adc的死等

    if (g_traceState != RUN_STATE_RUNNING) 
    {
        MotorOutput(0, 0);
        return;
    }

    g_run_20ms++;

    if (g_tracePhase == TRACE_PHASE_YAW_TURN)
    {
        yaw_turn_control();
        return;
    }
    if (g_tracePhase == TRACE_PHASE_REACQUIRE)
    {
        reacquire_line_control();
        return;
    }
    if (Corner_Rise_Flag == 1)   //检测到直角
    {
        enter_yaw_turn();
        yaw_turn_control();
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

char get_trace_phase_char(void)
{
    switch (g_tracePhase)
    {
        case TRACE_PHASE_LINE:
            return 'L';
        case TRACE_PHASE_YAW_TURN:
            return 'T';
        case TRACE_PHASE_REACQUIRE:
            return 'R';
        default:
            return '?';
    }
}

void TIMER_TICK_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_TICK_INST)) 
    {
        case DL_TIMER_IIDX_ZERO:
            if (time_20ms_flag != 0U)
            {
                g_trace_overrun_count++;
            }
            else
            {
                time_20ms_flag = 1U;
            }
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
    else if (nMotor2Pwm < 0) {
        nMotor2Pwm = -nMotor2Pwm;
        set_motor2_direction(MOTOR_REV);
    }
    else
    {
        SET2_STOP;
    }
    if (nMotor2Pwm > PWM2_MAX_PERIOD_COUNT) {
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

    if (v > PWM2_PERIOD_COUNT) {
        v = PWM2_PERIOD_COUNT;
    }
    dutyfactor = v;
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
