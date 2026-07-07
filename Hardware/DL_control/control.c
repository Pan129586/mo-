#include "control.h"

#include "Hardware/DL_KEY/bsp_key.h"
#include <math.h>

/* Trace Config */
#define TRACE_MIN_LAPS              (1U)
#define TRACE_MAX_LAPS              (2U)
#define TRACE_CORNER_PER_LAP        (4U)
#define TRACE_CORNER_BASE_SPEED     (45.0f)
#define TRACE_SEARCH_BASE_SPEED     (35.0f)
#define TRACE_LOST_STOP_TICKS       (25U)
#define TRACE_CORNER_WINDOW_TICKS   (45U)

/* Trace State */
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

volatile trace_state_t g_traceState = TRACE_STATE_READY;
volatile uint8_t g_traceTargetLaps = 1;
volatile uint8_t g_traceCompletedCorners = 0;
volatile uint32_t g_traceRunTicks20ms = 0;

static uint8_t s_cornerConfirmWindow = 0;

static float vofa_data[5];
static uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* Trace State Machine */
static uint8_t Trace_TargetCorners(void)
{
    return (uint8_t)(g_traceTargetLaps * TRACE_CORNER_PER_LAP);
}

static void Trace_ResetPidAndOutput(void)
{
    set_pid_target(&pid_speed, 0.0f);
    set_pid_target(&pid_speed2, 0.0f);
    PID_reset(&pid_speed);
    PID_reset(&pid_speed2);
    PID_reset(&pid_direct);
    MotorOutput(0, 0);
}

static void Trace_UpdateCornerCount(void)
{
    uint8_t yaw_corners = (turn_90_count < 0) ? 0U : (uint8_t)turn_90_count;

    if (Corner_Rise_Flag != 0U) {
        s_cornerConfirmWindow = TRACE_CORNER_WINDOW_TICKS;
    } else if (s_cornerConfirmWindow > 0U) {
        s_cornerConfirmWindow--;
    }

    if ((yaw_corners > g_traceCompletedCorners) &&
        ((s_cornerConfirmWindow > 0U) || (Corner_Flag != 0U))) {
        g_traceCompletedCorners = yaw_corners;
        s_cornerConfirmWindow = 0;
    }
}

static float Trace_SelectBaseSpeed(void)
{
    if (Lost_Line_Count > 0U) {
        return TRACE_SEARCH_BASE_SPEED;
    }

    if ((Corner_Flag != 0U) || (fabsf(Line_Num) > 22.0f)) {
        return TRACE_CORNER_BASE_SPEED;
    }

    return (float)BASE;
}

void Trace_Init(void)
{
    g_traceState = TRACE_STATE_READY;
    g_traceTargetLaps = 1;
    g_traceCompletedCorners = 0;
    g_traceRunTicks20ms = 0;
    s_cornerConfirmWindow = 0;
    Start_Flag = 0;
    set_basespeed(0.0f, 0.0f);
    Trace_ResetPidAndOutput();
}

void Trace_Start(void)
{
    g_traceCompletedCorners = 0;
    g_traceRunTicks20ms = 0;
    s_cornerConfirmWindow = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    JY61P_ResetYawTrack();
    Trace_ResetPidAndOutput();
    set_basespeed((float)BASE, (float)BASE);
    Start_Flag = 1;
    g_traceState = TRACE_STATE_RUNNING;
}

void Trace_Stop(trace_state_t next_state)
{
    Start_Flag = 0;
    set_basespeed(0.0f, 0.0f);
    Trace_ResetPidAndOutput();
    g_traceState = next_state;
}

void Trace_HandleButton(void)
{
    uint8_t key = g_nButton;

    if (key == 0U) {
        return;
    }
    g_nButton = 0;

    switch (key) {
        case KEY1_PRES:
            if (g_traceState != TRACE_STATE_RUNNING) {
                if (g_traceTargetLaps < TRACE_MAX_LAPS) {
                    g_traceTargetLaps++;
                }
                g_traceState = TRACE_STATE_READY;
            }
            break;
        case KEY2_PRES:
            if (g_traceState != TRACE_STATE_RUNNING) {
                if (g_traceTargetLaps > TRACE_MIN_LAPS) {
                    g_traceTargetLaps--;
                }
                g_traceState = TRACE_STATE_READY;
            }
            break;
        case KEY3_PRES:
            if (g_traceState != TRACE_STATE_RUNNING) {
                Trace_Start();
            }
            break;
        case KEY4_PRES:
            if (g_traceState == TRACE_STATE_RUNNING) {
                Trace_Stop(TRACE_STATE_EMERGENCY_STOP);
            } else {
                Trace_Init();
            }
            break;
        default:
            break;
    }
}

void Trace_Task20ms(void)
{
    /* 20ms Control Loop */
    JY61P_Poll();
    GetMotorPulse();
    Light_Turn_control();

    if (g_traceState != TRACE_STATE_RUNNING) {
        MotorOutput(0, 0);
        return;
    }

    g_traceRunTicks20ms++;
    Trace_UpdateCornerCount();

    if (g_traceCompletedCorners >= Trace_TargetCorners()) {
        Trace_Stop(TRACE_STATE_FINISHED);
        return;
    }

    if (Lost_Line_Count >= TRACE_LOST_STOP_TICKS) {
        Trace_Stop(TRACE_STATE_EMERGENCY_STOP);
        return;
    }

    float base_speed = Trace_SelectBaseSpeed();
    set_basespeed(base_speed, base_speed);

    direct_val = Gray_pd_control();
    speed_target = Baseleft - direct_val;
    speed2_target = Baseright + direct_val;

    set_pid_target(&pid_speed, speed_target);
    set_pid_target(&pid_speed2, speed2_target);

    MotorPWM = speed_pid_control();
    Motor2PWM = speed2_pid_control();
    MotorOutput((int)MotorPWM, (int)Motor2PWM);
}

const char *Trace_GetStateText(void)
{
    switch (g_traceState) {
        case TRACE_STATE_IDLE:
            return "IDLE ";
        case TRACE_STATE_READY:
            return "READY";
        case TRACE_STATE_RUNNING:
            return "RUN  ";
        case TRACE_STATE_FINISHED:
            return "DONE ";
        case TRACE_STATE_EMERGENCY_STOP:
            return "STOP ";
        default:
            return "UNKWN";
    }
}

void TIMER_TICK_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_TICK_INST)) {
        case DL_TIMER_IIDX_ZERO:
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
    /* Motor Output */
    if (nMotorPwm >= 0) {
        set_motor_direction(MOTOR_FWD);
    } else {
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
