
#include "bsp_Graysensor.h"

uint8_t State_Value[8];
uint16_t sensor_values[8];

uint16_t ADC_Value_Gray;
uint16_t ADC_Value_Bat;
// float  proportion;
// uint8_t change_Flag = 0;
// uint8_t begin_Flag = 0;
// uint8_t close_Flag = 0;


float Line_Num=0;
float Last_Num = 0;
uint8_t Corner_Flag = 0;   //直角检测的标志位



// 传感器权重（Sensor0~Sensor7：左→右）   调整增益的大小
const float sensor_weight[8] = {-32, -22, -10, -5, 5, 10, 22, 32};  


//选择通道
void select_channel(uint8_t channel)
{
    // AD0 (Bit 0)
    if (channel & 0x01) DL_GPIO_setPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD0_PIN);
    else                DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD0_PIN);
    
    // AD1 (Bit 1)
    if (channel & 0x02) DL_GPIO_setPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD1_PIN);
    else                DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD1_PIN);
    
    // AD2 (Bit 2)
    if (channel & 0x04) DL_GPIO_setPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
    else                DL_GPIO_clearPins(GPIO_SENSOR_PORT, GPIO_SENSOR_AD2_PIN);
}



//读取所有灰度传感器的值
void Graysensor_Read_All(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        select_channel(i);
        Delay_us(10);
        //开启转换
        DL_ADC12_startConversion(ADC12_sensor_INST);

        //等待转换完成
        while (DL_ADC12_getPendingInterrupt(ADC12_sensor_INST) != DL_ADC12_IIDX_MEM0_RESULT_LOADED);

        sensor_values[i] = DL_ADC12_getMemResult(ADC12_sensor_INST, DL_ADC12_MEM_IDX_0);
    }
}

void Light_Turn_control(void)
{
    float sum_weight = 0.0f; 
    uint8_t sum_black = 0;  

    Graysensor_Read_All();

    // 遍历8个传感器，二值化并累加权重
    for (uint8_t i = 0; i < 8; i++)
    {
       // THRESHOLD 需根据现场光线调整
       if (sensor_values[i] > THRESHOLD) 
       {
            State_Value[i] = 1;   //知道是哪几个传感器压线
            sum_weight += sensor_weight[i];
            sum_black++;
       } 
       else 
       {
            State_Value[i] = 0;
       }
    }

    if (sum_black == 0)
    {
        //脱线的话保持上一课的状态进行跑
        Line_Num = Last_Num;
        //没到拐弯
        Corner_Flag = 0;
    }
    else if (sum_black >= 3)       //到达拐弯处
    {
        Line_Num = Last_Num;
        Corner_Flag = 1; 
    }
    else 
    {       
        Line_Num = sum_weight / sum_black; 
        Last_Num = Line_Num;

        Corner_Flag = 0; 
    }
}


