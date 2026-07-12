#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

#define ENCODER_BASE_RESOLUTION    (500L)
#define ENCODER_X4_RESOLUTION      (ENCODER_BASE_RESOLUTION * 4L) //手动模拟4倍
#define ENCODER_REDUCTION_RATIO    (28L)
#define ENCODER_WHEEL_PULSES_REV   (ENCODER_X4_RESOLUTION * ENCODER_REDUCTION_RATIO)
#define ENCODER_SAMPLE_PERIOD_MS   (20L)
#define ENCODER_WHEEL_RADIUS_CM    (3.25f)

extern volatile long g_lMotorPulseSigma;
extern volatile long g_lMotor2PulseSigma;
extern volatile short g_nMotorPulse;
extern volatile short g_nMotor2Pulse;
extern volatile uint32_t encoder_irq_count;
extern volatile float g_fMotorSpeedCmps;
extern volatile float g_fMotor2SpeedCmps;

void Encoder_Init(void);
void GetMotorPulse(void);

#endif
