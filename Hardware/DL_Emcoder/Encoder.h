#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

extern volatile long g_lMotorPulseSigma;
extern volatile long g_lMotor2PulseSigma;
extern volatile short g_nMotorPulse;
extern volatile short g_nMotor2Pulse;

void Encoder_Init(void);
void GetMotorPulse(void);

#endif
