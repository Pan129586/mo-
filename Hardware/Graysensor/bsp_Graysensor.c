#include "bsp_Graysensor.h"


volatile uint8_t State_Value[8];
volatile uint16_t sensor_values[8];
volatile uint32_t g_gray_scan_count = 0U;
volatile uint32_t g_gray_adc_timeout_count = 0U;
volatile uint32_t g_corner_event_count = 0U;
uint16_t ADC_Value_Gray;
uint16_t ADC_Value_Bat;

volatile float Line_Num = 0.0f;
float Last_Num = 0.0f;
volatile uint8_t left_black=0;
volatile uint8_t last_corner_flag=0;
volatile uint8_t Corner_Flag = 0;
volatile uint8_t Corner_Rise_Flag = 0;
volatile uint8_t Black_Sensor_Count = 0;

#define ADC_WAIT_TIMEOUT_COUNT 10000U

static uint8_t s_corner_latched = 0U;


static const float sensor_weight[8] = {
       -30.0f, -22.0f, -16.0f, -10.0f,10.0f,  16.0f,  22.0f,  30.0f
};

void Graysensor_ResetState(void)
{
    Line_Num = 0.0f;
    Last_Num = 0.0f;
    Corner_Flag = 0;
    Corner_Rise_Flag = 0;
    Black_Sensor_Count = 0;
    g_corner_event_count = 0;
    left_black =0;
    last_corner_flag =0;
}

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

        DL_ADC12_stopConversion(ADC12_sensor_INST);
        DL_ADC12_clearInterruptStatus(
            ADC12_sensor_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

        DL_ADC12_enableConversions(ADC12_sensor_INST);

        DL_ADC12_startConversion(ADC12_sensor_INST);        //等待adc的转化
        while ((DL_ADC12_getRawInterruptStatus(
                    ADC12_sensor_INST,
                    DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U)) {
            wait_count++;
            if (wait_count >= ADC_WAIT_TIMEOUT_COUNT) {
                DL_ADC12_stopConversion(ADC12_sensor_INST);
                DL_ADC12_clearInterruptStatus(
                    ADC12_sensor_INST,
                    DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
                sensor_values[i] = 0U;
                g_gray_adc_timeout_count++;
                break;
            }
        }

        //要是后期有影响的话就删掉
        if (wait_count >= ADC_WAIT_TIMEOUT_COUNT) {
            break;
        }

        sensor_values[i] = DL_ADC12_getMemResult(ADC12_sensor_INST, DL_ADC12_MEM_IDX_0);
        DL_ADC12_stopConversion(ADC12_sensor_INST);
        DL_ADC12_clearInterruptStatus(
            ADC12_sensor_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    }

    g_gray_scan_count++;
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

    left_black = State_Value[0] +State_Value[1] +State_Value[2] +State_Value[3];
   if(left_black >=3)
   {
        Corner_Flag =1;
   }
   else 
   {
        Corner_Flag =0;
   }

    if(Corner_Flag ==1&&last_corner_flag==0)
    {
        Corner_Rise_Flag =1;
        g_corner_event_count ++;
    }

    last_corner_flag = Corner_Flag; //更新

    if (sum_black == 0U)
    {
        Line_Num = Last_Num;
    }
    else
    {
        Line_Num = sum_weight / (float)sum_black;
        Last_Num = Line_Num;
    }

    
}
