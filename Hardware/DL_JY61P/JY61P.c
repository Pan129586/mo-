#include "JY61P.h"

#include <math.h>

#define JY_TURN_COMPLETE_BIAS_DEG (10.0f)

volatile uint32_t uart_rx_test_count = 0;
volatile float yaw_real = 0.0f;
volatile float total_yaw = 0.0f;
volatile float last_yaw = 0.0f;
volatile float target_yaw = 0.0f;
volatile int turn_90_count = 0;

uint8_t Start_Flag = 0;
uint8_t uart2_rxbuff = 0;
uint8_t jy_buf[11];
uint8_t jy_state = 0;
uint8_t jy_rx_buf[JY_RX_BUF_SIZE];
volatile uint16_t jy_rx_head = 0;
static uint16_t jy_rx_tail = 0;


static float JY61P_angelturn(float angle)
{
    if (angle < -180.0f) 
    {
        angle += 360.0f;
    }
    else if (angle > 180.0f) 
    {
        angle -= 360.0f;
    }

    return angle;
}

static void JY61P_ParseYaw(void)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < 10; i++) 
    {
        sum += jy_buf[i];
    }
    if (sum != jy_buf[10]) 
    {
        return;
    }

    int16_t yaw_raw = ((int16_t)jy_buf[7] << 8) | jy_buf[6];
    yaw_real = (float)yaw_raw / 32768.0f * 180.0f;
}


void reset_turn_count(void)
{
    total_yaw = 0.0f;
    turn_90_count = 0;
    target_yaw = 0.0f;
    last_yaw = yaw_real;
}

void JY61P_UpdateTurnCounter(void)
{
    float diff = JY61P_angelturn(yaw_real - last_yaw);

    if (Start_Flag == 1U) 
    {
        total_yaw += diff;
        turn_90_count = (int)((fabsf(total_yaw) +JY_TURN_COMPLETE_BIAS_DEG) / 90.0f);  //计算转过的角度个数，使用90的倍数进行计算

        if (total_yaw >= 0.0f) 
        {
            target_yaw = (float)turn_90_count * 90.0f;
        } 
        else 
        {
            target_yaw = (float)-turn_90_count * 90.0f; //
        }
    }

    last_yaw = yaw_real;
}

// static void JY61P_receiv(uint8_t ch)
// {
//     static uint8_t cnt = 0;

//     switch (jy_state) 
//     {
//         case 0:
//             if (ch == 0x55U) 
//             {
//                 jy_buf[0] = ch;
//                 jy_state = 1;
//             }
//             break;
//         case 1:
//             if (ch == 0x53U) 
//             {
//                 jy_buf[1] = ch;
//                 cnt = 0;
//                 jy_state = 2;
//             } 
//             else 
//             {
//                 jy_state = 0;
//             }
//             break;
//         case 2:
//             jy_buf[2 + cnt] = ch;
//             cnt++;
//             if (cnt >= 9U) 
//             {
//                 cnt = 0;
//                 jy_state = 0;
//                 JY61P_ParseYaw();   //进行校验和
//                 JY61P_UpdateTurnCounter();   //更新转过的角度 
//             }
//             break;
//         default:
//             cnt = 0;
//             jy_state = 0;
//             break;
//     }
// }

static void JY61P_receiv(uint8_t ch)
{
    static uint8_t cnt = 0;

    switch (jy_state) 
    {
        case 0:
            if (ch == 0x55U) 
            {
                jy_buf[0] = ch;
                jy_state = 1;
            }
            break;
        case 1:
            // 兼容加速度、角速度、角度三种包头，防止错位
            if (ch == 0x51U || ch == 0x52U || ch == 0x53U) 
            {
                jy_buf[1] = ch;
                cnt = 0;
                jy_state = 2;
            } 
            else 
            {
                jy_state = 0;
            }
            break;
        case 2:
            jy_buf[2 + cnt] = ch;
            cnt++;
            // 凑齐一帧的 11 个字节了
            if (cnt >= 9U) 
            {
                cnt = 0;
                jy_state = 0;
                
                // 只有当这一帧是角度包(0x53)的时候，才进行解析
                if (jy_buf[1] == 0x53U) 
                {
                    JY61P_ParseYaw();          // 校验并计算 yaw_real
                    JY61P_UpdateTurnCounter(); // 更新转角累计
                }
            }
            break;
        default:
            cnt = 0;
            jy_state = 0;
            break;
    }
}

//串口接收的环形数组
void JY61P_Poll(void)
{
    uint16_t dma_remain = DL_DMA_getTransferSize(DMA, 0);
    uint16_t current_head = 256 - dma_remain;
    

    while (jy_rx_tail != jy_rx_head) 
    {
        uint8_t ch = jy_rx_buf[jy_rx_tail];
        jy_rx_tail = (uint16_t)((jy_rx_tail + 1U) % JY_RX_BUF_SIZE);
        JY61P_receiv(ch);
    }
}

void UART_JY61P_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        uart_rx_test_count++;
            jy_rx_buf[jy_rx_head] = DL_UART_Main_receiveData(UART_JY61P_INST);

            jy_rx_head = (uint16_t)((jy_rx_head + 1U) % JY_RX_BUF_SIZE);
            break;
        default:
            break;
            
    }
}
