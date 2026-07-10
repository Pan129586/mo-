#include "bsp_Graysensor.h"


uint8_t State_Value[8];
uint16_t sensor_values[8];
uint16_t ADC_Value_Gray;
uint16_t ADC_Value_Bat;

float Line_Num = 0.0f;
float Last_Num = 0.0f;
uint8_t Corner_Flag = 0;
uint8_t Corner_Rise_Flag = 0;
uint8_t Lost_Line_Count = 0;
uint8_t Black_Sensor_Count = 0;

#define ADC_WAIT_TIMEOUT_COUNT 10000U

static uint8_t s_lastCornerFlag = 0;
static uint8_t s_cornerLockTicks = 0;

static const float sensor_weight[8] = {
    -32.0f, -22.0f, -10.0f, -5.0f, 5.0f, 10.0f, 22.0f, 32.0f
};

void select_channel(uint8_t channel)
{
   
    if ((channel & 0x01U) != 0U) 
    {
        DL_GPIO_setPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
    } 
    else 
    {
        DL_GPIO_clearPins(GPIO_SENSOR_AD0_PORT, GPIO_SENSOR_AD0_PIN);
    }

    if ((channel & 0x02U) != 0U) 
    {
        DL_GPIO_setPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
    } 
    else 
    {
        DL_GPIO_clearPins(GPIO_SENSOR_AD1_PORT, GPIO_SENSOR_AD1_PIN);
    }

    if ((channel & 0x04U) != 0U) 
    {
        DL_GPIO_setPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
    } 
    else 
    {
        DL_GPIO_clearPins(GPIO_SENSOR_AD2_PORT, GPIO_SENSOR_AD2_PIN);
    }
}

void Graysensor_Read_All(void)
{
    for (uint8_t i = 0; i < 8U; i++) 
    {
        uint32_t wait_count = 0U;

        select_channel(i);
        Delay_us(10);

        DL_ADC12_startConversion(ADC12_sensor_INST);        //等待adc的转化
        while (DL_ADC12_getPendingInterrupt(ADC12_sensor_INST) !=
               DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
            wait_count++;
            if (wait_count >= ADC_WAIT_TIMEOUT_COUNT) {
                sensor_values[i] = 0U;
                break;
            }
        }

        //要是后期有影响的话就删掉
        if (wait_count >= ADC_WAIT_TIMEOUT_COUNT) { 
            continue;
        }

        sensor_values[i] = DL_ADC12_getMemResult(ADC12_sensor_INST, DL_ADC12_MEM_IDX_0);
    }
}

void Light_Turn_control(void)
{
    float sum_weight = 0.0f;
    uint8_t sum_black = 0;
    uint8_t raw_corner;

    Graysensor_Read_All();

    for (uint8_t i = 0; i < 8U; i++) 
    {
        if (sensor_values[i] > THRESHOLD) 
        {
            State_Value[i] = 1;
            sum_weight += sensor_weight[i];
            sum_black++;
        } 
        else 
        {
            State_Value[i] = 0;
        }
    }

    Black_Sensor_Count = sum_black;
    Corner_Rise_Flag = 0;

    if (sum_black == 0U) 
    {
        Line_Num = Last_Num;
        Corner_Flag = 0;
        if (Lost_Line_Count < 255U) 
        {
            Lost_Line_Count++;
        }
    } 
    else 
    {
        Lost_Line_Count = 0;
        if(sum_black>= 3U)
        {
            raw_corner = 1U;
        }
        else
        {
            raw_corner=0U;
        }

        Corner_Flag = raw_corner;

        if (raw_corner != 0U) 
        {
            Line_Num = Last_Num;
        } 
        else 
        {
            Line_Num = sum_weight / (float)sum_black;
            Last_Num = Line_Num;
        }
        //消抖，只有在刚刚进入直角的时候判断才成立
        if ((raw_corner != 0U) && (s_lastCornerFlag == 0U) && (s_cornerLockTicks == 0U)) 
        {
            Corner_Rise_Flag = 1;
            s_cornerLockTicks = 10;     //在该时间之内只会记录一次上升沿变化
        }
        s_lastCornerFlag = raw_corner;
    }

    if (Corner_Flag == 0U) 
    {
        s_lastCornerFlag = 0;
    }
    if (s_cornerLockTicks > 0U) 
    {
        s_cornerLockTicks--;
    }
}
