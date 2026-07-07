#include "Encoder.h"



long g_lMotorPulseSigma =0;//电机25ms内累计脉冲总和
long g_lMotor2PulseSigma =0;//电机25ms内累计脉冲总和
short g_nMotorPulse=0,g_nMotor2Pulse=0;//全局变量， 保存电机脉冲数值

volatile short right_encoder_count = 0;    //右轮的中断计数器


void Encoder_Init(void)
{
    DL_Timer_startCounter(Left_INST);
    //左轮定时器中断
	NVIC_EnableIRQ(Left_INST_INT_IRQN);
    // 开启右轮的 GPIO 中断 
    NVIC_EnableIRQ(GPIO_QRI_R_INT_IRQN);
}


//A相和B相 的外部中断服务函数 四倍频 
void Left_INST_IRQHandler(void)
{
    // 获取当前中断状态 (看看是A相还是B相触发了中断)
    uint32_t gpioStatus = DL_GPIO_getEnabledInterruptStatus(GPIO_QRI_R_PORT, GPIO_QRI_R_PH_A_PIN | GPIO_QRI_R_PH_B_PIN);

    // 瞬间读取此刻 A 相和 B 相的真实物理电平，并归一化为 0 或 1
    uint8_t a_val = DL_GPIO_readPins(GPIO_QRI_R_PORT, GPIO_QRI_R_PH_A_PIN) ? 1 : 0;
    uint8_t b_val = DL_GPIO_readPins(GPIO_QRI_R_PORT, GPIO_QRI_R_PH_B_PIN) ? 1 : 0;

    // === 如果是 A 相发生了跳变 (上升沿或下降沿) ===
    if ((gpioStatus & GPIO_QRI_R_PH_A_PIN) == GPIO_QRI_R_PH_A_PIN) 
    {
        //清除 A 相的中断标志
        DL_GPIO_clearInterruptStatus(GPIO_QRI_R_PORT, GPIO_QRI_R_PH_A_PIN);
        // A跳变时，如果A和B电平不相等就是正转，相等就是反转
        if (a_val != b_val) 
		{
            right_encoder_count++; // 正转
        } 
		else 
		{
            right_encoder_count--; // 反转
        }
    }
    
    // === 如果是 B 相发生了跳变 (上升沿或下降沿) ===
    if ((gpioStatus & GPIO_QRI_R_PH_B_PIN) == GPIO_QRI_R_PH_B_PIN) 
    {
       
        DL_GPIO_clearInterruptStatus(GPIO_QRI_R_PORT, GPIO_QRI_R_PH_B_PIN);
        
        //B跳变时，如果A和B电平相等就是正转，不相等就是反转
        if (a_val == b_val) 
		{
            right_encoder_count++; // 正转
        } 
		else 
		{
            right_encoder_count--; // 反转
        }
    }
}



void GetMotorPulse(void)
{
    g_nMotorPulse = (short)DL_Timer_getTimerCount(Left_INST);
    DL_Timer_setTimerCount(Left_INST, 0); // 读完清零
    
    //右轮读取软件中断计数值
    g_nMotor2Pulse = right_encoder_count;
    right_encoder_count = 0;               // 读完清零
    // === 累计总脉冲 ===
    g_lMotorPulseSigma += g_nMotorPulse;
    g_lMotor2PulseSigma += g_nMotor2Pulse;
}
