
#include "IIC.h"




void iic_delay_us(uint32_t us)
{
    Delay_us(us); 
}

// 🌟 TI 专属极简模式切换：打开 SDA 的输出驱动
void I2C_SDA_OUT(void)
{
    DL_GPIO_enableOutput(GPIO_VOICE_PORT, GPIO_VOICE_SDA_PIN);
}

// 🌟 TI 专属极简模式切换：关闭 SDA 的输出驱动，使其变为纯输入
void I2C_SDA_IN(void)
{
    DL_GPIO_disableOutput(GPIO_VOICE_PORT, GPIO_VOICE_SDA_PIN);
}

void IIC_Init()
{
    IIC_SDA_H();
    IIC_SCL_H();
}

void Voice_IIC_Start(void)
{
    I2C_SDA_OUT();
    IIC_SDA_H();
    IIC_SCL_H();
    iic_delay_us(15);
    IIC_SDA_L();
    iic_delay_us(15);
    IIC_SCL_L();
}

void Voice_IIC_Stop(void)
{
    I2C_SDA_OUT();
    IIC_SCL_L();
    IIC_SDA_L();
    iic_delay_us(15);
    IIC_SCL_H();
    IIC_SDA_H();
    iic_delay_us(15);
}

void IIC_Ack(void)
{
    IIC_SCL_L();
    I2C_SDA_OUT();
    IIC_SDA_L();
    iic_delay_us(15);
    IIC_SCL_H();
    iic_delay_us(15);
    IIC_SCL_L();
}
    
void IIC_NAck(void)
{
    IIC_SCL_L();
    I2C_SDA_OUT();
    IIC_SDA_H();
    iic_delay_us(15);
    IIC_SCL_H();
    iic_delay_us(15);
    IIC_SCL_L();
}

uint8_t Voice_IIC_Wait_Ack(void)
{
    uint8_t Time=0;
    I2C_SDA_IN();
    IIC_SDA_H();
    iic_delay_us(15);
    IIC_SCL_H();
    iic_delay_us(15);
    while(READ_SDA)
    {
        Time++;
        if(Time>250)
        {
            Voice_IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_L();
    return 0;
}

void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    I2C_SDA_OUT();
    IIC_SCL_L();
    for(t=0;t<8;t++)
    {
        if ((txd & 0x80) == 0x80) 
        {
            IIC_SDA_H();  
        }
        else
        {
            IIC_SDA_L();  
        }
        
        txd<<=1;
        iic_delay_us(15);
        IIC_SCL_H();
        iic_delay_us(15);
        IIC_SCL_L();
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i,receive=0;
    I2C_SDA_IN();
    for(i=0;i<8;i++)
    {
        IIC_SCL_L();
        iic_delay_us(15);
        IIC_SCL_H();
        receive<<=1;
        if(READ_SDA) receive++;
        iic_delay_us(1);
    }
    if(!ack) IIC_NAck();
    else IIC_Ack();
    return receive;
}

void WriteOneByte(uint16_t addr, uint8_t data)
{
    Voice_IIC_Start();
    IIC_Send_Byte(addr); 
    Voice_IIC_Wait_Ack(); 
    IIC_Send_Byte(0xA0); 
    Voice_IIC_Wait_Ack(); 
    IIC_Send_Byte(data); 
    Voice_IIC_Wait_Ack(); 
    Voice_IIC_Stop(); 
}

// 语音播报
void Asr_Speak(uint8_t cmd ,uint8_t idNum)
{   
    int i;
    int addr = Asr_Addr;
    uint8_t send[2] = {0x00 , 0x00};
    if(cmd == 0xFF || cmd == 0x00)
    {
        send[0] = cmd;
        send[1] = idNum;
        
        Voice_IIC_Start(); 
        IIC_Send_Byte(addr<<1 | 0); 
        Voice_IIC_Wait_Ack(); 
        IIC_Send_Byte(ASR_SPEAK_ADDR); 
        Voice_IIC_Wait_Ack(); 
        for(i = 0; i < 2; ++i)
        {
            IIC_Send_Byte(send[i]);
            Voice_IIC_Wait_Ack(); 
        }
        Voice_IIC_Stop(); 
        Delay_ms(20); // 替换为了你的 Delay_ms
    }
}

// 获取识别结果
int Asr_Result()
{
    int result = 0;
    Voice_IIC_Start(); 
    IIC_Send_Byte(Asr_Addr<<1 | 0); 
    Voice_IIC_Wait_Ack(); 
    IIC_Send_Byte(ASR_RESULT_ADDR); 
    if (Voice_IIC_Wait_Ack()) 
    {
        Voice_IIC_Stop(); 
        return 0;
    }
    Voice_IIC_Stop();   
    Voice_IIC_Start(); 
    IIC_Send_Byte(Asr_Addr<<1 | 1); 
    Voice_IIC_Wait_Ack(); 
    result = IIC_Read_Byte(0); 
    Voice_IIC_Stop();   
    
    return result;
}