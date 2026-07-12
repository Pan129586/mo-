#ifndef __CONTROL_H
#define __CONTROL_H

#include "ti_msp_dl_config.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"
#include "Hardware/DL_JY61P/JY61P.h"
#include "Hardware/DL_pid/bsp_pid.h"
#include <stdint.h>

/* Motor Direction Config: positive PWM always means vehicle forward. */
#ifndef MOTOR1_FORWARD_REVERSED
#define MOTOR1_FORWARD_REVERSED (0U)
#endif

#ifndef MOTOR2_FORWARD_REVERSED
#define MOTOR2_FORWARD_REVERSED (0U)
#endif

#if MOTOR1_FORWARD_REVERSED
#define SET_FWD  do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN); \
} while (0)
#define SET_REV  do { \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN); \
} while (0)
#else
#define SET_FWD  do { \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN); \
} while (0)
#define SET_REV  do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN); \
} while (0)
#endif

//停止（0，0）
#define SET_STOP do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN); \
} while (0)

#if MOTOR2_FORWARD_REVERSED
#define SET2_FWD  do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN); \
} while (0)

#define SET2_REV  do { \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN); \
} while (0)
#else
#define SET2_FWD  do { \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN); \
} while (0)
#define SET2_REV  do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN); \
} while (0)
#endif

#define SET2_STOP do { \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN); \
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN); \
} while (0)

 //定时器A1跟A
#define SET_COMPAER(ChannelPulse) \
    DL_TimerA_setCaptureCompareValue(PWMB_INST, (ChannelPulse), DL_TIMER_CC_0_INDEX)
#define SET2_COMPAER(ChannelPulse) \
    DL_TimerA_setCaptureCompareValue(PWMA_INST, (ChannelPulse), DL_TIMER_CC_1_INDEX)

#define PWM_PERIOD_COUNT      (1000)
#define PWM2_PERIOD_COUNT     (1000)
#define PWM_MAX_PERIOD_COUNT  (PWM_PERIOD_COUNT - 30)
#define PWM2_MAX_PERIOD_COUNT (PWM2_PERIOD_COUNT - 30)


#define ENCODER_RESOLUTION         ENCODER_BASE_RESOLUTION
#define ENCODER_TOTAL_RESOLUTION   ENCODER_X4_RESOLUTION
#define REDUCTION_RATIO            ENCODER_REDUCTION_RATIO
#define SPEED_PID_PERIOD           ENCODER_SAMPLE_PERIOD_MS
#define WheelR                     ENCODER_WHEEL_RADIUS_CM
#define lunju                      (14)
#define BASE                       (60)

typedef enum {
    RUN_STATE_IDLE = 0,
    RUN_STATE_READY,
    RUN_STATE_RUNNING,
    RUN_STATE_FINISHED,
    RUN_STATE_EMERGENCY_STOP
} trace_state_t;

typedef enum {
    TRACE_PHASE_LINE = 0,
    TRACE_PHASE_YAW_TURN,
    TRACE_PHASE_REACQUIRE
} trace_phase_t;

typedef enum {
    MOTOR_FWD = 0,
    MOTOR_REV,
} motor_dir_t;

typedef enum {
    DRIVE_FAULT_NONE = 0,
    DRIVE_FAULT_MOTOR1_FEEDBACK_REVERSED = (1U << 0),
    DRIVE_FAULT_MOTOR2_FEEDBACK_REVERSED = (1U << 1)
} drive_fault_t;


extern volatile uint8_t min_circle;
extern volatile uint8_t maix_circle;
extern volatile trace_state_t g_traceState;
extern volatile trace_phase_t g_tracePhase;
extern volatile uint8_t g_target_circle;
extern volatile uint8_t g_compt_corner;
extern volatile uint32_t  time_20ms_flag; //定时器任务标志位
extern volatile uint32_t g_trace_overrun_count;
extern volatile uint32_t g_run_20ms;
extern volatile float g_turn_target_yaw;
extern volatile float g_turn_yaw_error;
extern volatile drive_fault_t g_drive_fault;
extern volatile uint32_t g_motor1_reverse_feedback_count;
extern volatile uint32_t g_motor2_reverse_feedback_count;

extern float Baseleft;
extern float Baseright;
extern uint8_t is_motor_en;

void run_data_init(void);
// void key_change_circle(void);
void Trace_Task20ms(void);
void run_start(void);
void run_stop(trace_state_t next_state);
const char *get_runstate(void);
char get_trace_phase_char(void);

void set_basespeed(float left_base, float right_base);
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
void MotorOutput(int nMotorPwm, int nMotor2Pwm);
void Send_To_VOFA(float target_left, float real_left, float target_right,
                  float real_right, float line_num);

 void enter_yaw_turn(void);
  void yaw_turn_control(void);
 void reacquire_line_control(void);

#endif
