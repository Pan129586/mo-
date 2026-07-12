#include "Encoder.h"

#include <stdint.h>

/* Encoder Direction */
#ifndef ENCODER_MOTOR1_DIR
#define ENCODER_MOTOR1_DIR (-1L)
#endif

#ifndef ENCODER_MOTOR2_DIR
#define ENCODER_MOTOR2_DIR (1L)
#endif


#define ENCODER_X4_EQUIVALENT_SCALE (2L)   //脉冲数值*了2
#define ENCODER_ISR_DRAIN_LIMIT      (4U)  //
#define ENCODER_PI                    (3.1415926f)


volatile long g_lMotorPulseSigma = 0;
volatile long g_lMotor2PulseSigma = 0;
volatile short g_nMotorPulse = 0;
volatile short g_nMotor2Pulse = 0;
volatile uint32_t encoder_irq_count = 0U;
volatile float g_fMotorSpeedCmps = 0.0f;
volatile float g_fMotor2SpeedCmps = 0.0f;

static volatile long s_motorPulseCount = 0;
static volatile long s_motor2PulseCount = 0;

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Encoder_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t Encoder_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

/* A-phase double-edge decoding; B-phase is direction only. */
static void Encoder_UpdateMotorCount(void)
{
    uint8_t phaseA = Encoder_ReadPin(GPIO_Encoder_PH_A_PORT,
                                     GPIO_Encoder_PH_A_PIN);
    uint8_t phaseB = Encoder_ReadPin(GPIO_Encoder_PH_B_PORT,
                                     GPIO_Encoder_PH_B_PIN);
    long delta = (phaseA == phaseB) ? 1L : -1L;

    s_motorPulseCount += delta * ENCODER_MOTOR1_DIR *
                         ENCODER_X4_EQUIVALENT_SCALE;   //
}

static void Encoder_UpdateMotor2Count(void)
{
    uint8_t phaseA = Encoder_ReadPin(GPIO_Encoder_BH_A_PORT,
                                     GPIO_Encoder_BH_A_PIN);
    uint8_t phaseB = Encoder_ReadPin(GPIO_Encoder_BH_B_PORT,
                                     GPIO_Encoder_BH_B_PIN);
    long delta = (phaseA == phaseB) ? 1L : -1L;

    s_motor2PulseCount += delta * ENCODER_MOTOR2_DIR *
                          ENCODER_X4_EQUIVALENT_SCALE;
}

static void Encoder_HandleGPIOBInterrupt(void)
{
    uint8_t handled = 0U;

    while (handled < ENCODER_ISR_DRAIN_LIMIT)
    {
        uint32_t pending = DL_GPIO_getPendingInterrupt(GPIOB);

        if (pending == DL_GPIO_IIDX_NO_INTR)
        {
            break;
        }

        encoder_irq_count++;
        handled++;
        switch (pending)
        {
            case GPIO_Encoder_PH_A_IIDX:
                Encoder_UpdateMotorCount();
                break;
            case GPIO_Encoder_BH_A_IIDX:
                Encoder_UpdateMotor2Count();
                break;
            default:
                break;
        }
    }
}

void Encoder_Init(void)
{
    uint32_t primask = Encoder_EnterCritical();

    s_motorPulseCount = 0;
    s_motor2PulseCount = 0;
    g_nMotorPulse = 0;
    g_nMotor2Pulse = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    encoder_irq_count = 0U;
    g_fMotorSpeedCmps = 0.0f;
    g_fMotor2SpeedCmps = 0.0f;

    DL_GPIO_initDigitalInputFeatures(GPIO_Encoder_PH_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_Encoder_PH_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_Encoder_BH_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_Encoder_BH_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_disableInterrupt(GPIOA,
        GPIO_Encoder_PH_B_PIN | GPIO_Encoder_BH_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA,
        GPIO_Encoder_PH_B_PIN | GPIO_Encoder_BH_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB,
        GPIO_Encoder_PH_A_PIN | GPIO_Encoder_BH_A_PIN);
    DL_GPIO_enableInterrupt(GPIOB,
        GPIO_Encoder_PH_A_PIN | GPIO_Encoder_BH_A_PIN);

    NVIC_ClearPendingIRQ(GPIO_Encoder_INT_IRQN);
    NVIC_SetPriority(GPIO_Encoder_INT_IRQN, 3U);
    NVIC_EnableIRQ(GPIO_Encoder_INT_IRQN);

    Encoder_ExitCritical(primask);
}

void GetMotorPulse(void)
{
    long motorPulse;
    long motor2Pulse;
    uint32_t primask = Encoder_EnterCritical();

    motorPulse = s_motorPulseCount;
    motor2Pulse = s_motor2PulseCount;
    s_motorPulseCount = 0;
    s_motor2PulseCount = 0;
    Encoder_ExitCritical(primask);

    g_nMotorPulse = (short)motorPulse;
    g_nMotor2Pulse = (short)motor2Pulse;
    g_lMotorPulseSigma += g_nMotorPulse;
    g_lMotor2PulseSigma += g_nMotor2Pulse;
    //把脉冲换算成速度
    g_fMotorSpeedCmps = ((float)g_nMotorPulse * 2.0f * ENCODER_PI *
        ENCODER_WHEEL_RADIUS_CM * 1000.0f) /
        ((float)ENCODER_WHEEL_PULSES_REV * (float)ENCODER_SAMPLE_PERIOD_MS);
    g_fMotor2SpeedCmps = ((float)g_nMotor2Pulse * 2.0f * ENCODER_PI *
        ENCODER_WHEEL_RADIUS_CM * 1000.0f) /
        ((float)ENCODER_WHEEL_PULSES_REV * (float)ENCODER_SAMPLE_PERIOD_MS);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case GPIO_Encoder_INT_IIDX:
            Encoder_HandleGPIOBInterrupt();
            break;
        default:
            break;
    }
}
