#include "Encoder.h"

#include <stdint.h>

//电机1的转动方向
#ifndef ENCODER_MOTOR1_DIR
#define ENCODER_MOTOR1_DIR (-1)     
#endif

#ifndef ENCODER_MOTOR2_DIR
#define ENCODER_MOTOR2_DIR (-1)
#endif



volatile long g_lMotorPulseSigma = 0;   // 电机1总累计脉冲（绝对位置）
volatile long g_lMotor2PulseSigma = 0;
volatile short g_nMotorPulse = 0;           // 电机1当前周期内的脉冲数（速度）
volatile short g_nMotor2Pulse = 0;


//底层中断内使用的计数器与状态
static volatile long s_motorPulseCount = 0;   // 电机1底层中断脉冲累加器
static volatile long s_motor2PulseCount = 0;
static volatile uint8_t s_motorState = 0;       // 电机1上一次的A/B相电平状态
static volatile uint8_t s_motor2State = 0;

static const int8_t g_encoderTransitionTable[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}



static void Encoder_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

//读取电机1引脚状态
static uint8_t Encoder_ReadMotorState(void)
{
    uint8_t phaseA = (DL_GPIO_readPins(GPIO_Encoder_PH_A_PORT,
                          GPIO_Encoder_PH_A_PIN) != 0U) ? 1U : 0U;
    uint8_t phaseB = (DL_GPIO_readPins(GPIO_Encoder_PH_B_PORT,
                          GPIO_Encoder_PH_B_PIN) != 0U) ? 1U : 0U;

    return (uint8_t)((phaseA << 1U) | phaseB);
}


static uint8_t Encoder_ReadMotor2State(void)
{
    uint8_t phaseA = (DL_GPIO_readPins(GPIO_Encoder_BH_A_PORT,
                          GPIO_Encoder_BH_A_PIN) != 0U) ? 1U : 0U;
    uint8_t phaseB = (DL_GPIO_readPins(GPIO_Encoder_BH_B_PORT,
                          GPIO_Encoder_BH_B_PIN) != 0U) ? 1U : 0U;

    return (uint8_t)((phaseA << 1U) | phaseB);
}

//更新电机1脉冲计数值
static void Encoder_UpdateMotorCount(void)
{
    uint8_t state = Encoder_ReadMotorState();
    // 计算矩阵索引：将旧状态左移2位，并与新状态进行按位或。
    int8_t delta = g_encoderTransitionTable[((uint8_t)s_motorState << 2U) | state];

    s_motorState = state;
    // 脉冲数 = 状态变化值(-1,0,1) * 电机方向系数
    s_motorPulseCount += ((long)delta * ENCODER_MOTOR1_DIR);
}

static void Encoder_UpdateMotor2Count(void)
{
    uint8_t state = Encoder_ReadMotor2State();
    int8_t delta = g_encoderTransitionTable[((uint8_t)s_motor2State << 2U) | state];

    s_motor2State = state;
    s_motor2PulseCount += ((long)delta * ENCODER_MOTOR2_DIR);
}

//GPIO A 端口产生的中断
static void Encoder_HandleGPIOAInterrupt(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
        case GPIO_Encoder_PH_B_IIDX:
            Encoder_UpdateMotorCount();
            break;
        case GPIO_Encoder_BH_B_IIDX:
            Encoder_UpdateMotor2Count();
            break;
        default:
            break;
    }
}

static void Encoder_HandleGPIOBInterrupt(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
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

void Encoder_Init(void)
{
    uint32_t primask = Encoder_EnterCritical();

    s_motorPulseCount = 0;
    s_motor2PulseCount = 0;
    g_nMotorPulse = 0;
    g_nMotor2Pulse = 0;
    g_lMotorPulseSigma = 0;
    g_lMotor2PulseSigma = 0;
    s_motorState = Encoder_ReadMotorState();
    s_motor2State = Encoder_ReadMotor2State();

    DL_GPIO_clearInterruptStatus(GPIOA,
        GPIO_Encoder_PH_B_PIN | GPIO_Encoder_BH_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB,
        GPIO_Encoder_PH_A_PIN | GPIO_Encoder_BH_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA,
        GPIO_Encoder_PH_B_PIN | GPIO_Encoder_BH_B_PIN);
    DL_GPIO_enableInterrupt(GPIOB,
        GPIO_Encoder_PH_A_PIN | GPIO_Encoder_BH_A_PIN);

    NVIC_ClearPendingIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_Encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOB_INT_IRQN);

    Encoder_ExitCritical(primask);
}


//获取脉冲数据
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
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_Encoder_GPIOA_INT_IIDX:
            Encoder_HandleGPIOAInterrupt();
            break;
        case GPIO_Encoder_GPIOB_INT_IIDX:
            Encoder_HandleGPIOBInterrupt();
            break;
        default:
            break;
    }
}
