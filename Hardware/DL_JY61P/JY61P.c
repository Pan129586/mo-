#include "JY61P.h"

#include <math.h>

#define JY_TURN_COMPLETE_BIAS_DEG (10.0f)

/* Yaw State */
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

static float JY61P_NormalizeDelta(float delta)
{
    if (delta < -180.0f) {
        delta += 360.0f;
    } else if (delta > 180.0f) {
        delta -= 360.0f;
    }

    return delta;
}

static void JY61P_ParseYaw(void)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < 10; i++) {
        sum += jy_buf[i];
    }
    if (sum != jy_buf[10]) {
        return;
    }

    int16_t yaw_raw = ((int16_t)jy_buf[7] << 8) | jy_buf[6];
    yaw_real = (float)yaw_raw / 32768.0f * 180.0f;
}

void JY61P_ResetYawTrack(void)
{

    total_yaw = 0.0f;
    turn_90_count = 0;
    target_yaw = 0.0f;
    last_yaw = yaw_real;
}

void reset_turn_count(void)
{
    JY61P_ResetYawTrack();
}

void JY61P_UpdateTurnCounter(void)
{
    float diff = JY61P_NormalizeDelta(yaw_real - last_yaw);

    if (Start_Flag == 1U) {
        total_yaw += diff;
        turn_90_count = (int)((fabsf(total_yaw) + JY_TURN_COMPLETE_BIAS_DEG) / 90.0f);

        if (total_yaw >= 0.0f) {
            target_yaw = (float)turn_90_count * 90.0f;
        } else {
            target_yaw = (float)-turn_90_count * 90.0f;
        }
    }

    last_yaw = yaw_real;
}

static void JY61P_DecodeByte(uint8_t ch)
{
    /* Frame Decode */
    static uint8_t cnt = 0;

    switch (jy_state) {
        case 0:
            if (ch == 0x55U) {
                jy_buf[0] = ch;
                jy_state = 1;
            }
            break;
        case 1:
            if (ch == 0x53U) {
                jy_buf[1] = ch;
                cnt = 0;
                jy_state = 2;
            } else {
                jy_state = 0;
            }
            break;
        case 2:
            jy_buf[2 + cnt] = ch;
            cnt++;
            if (cnt >= 9U) {
                cnt = 0;
                jy_state = 0;
                JY61P_ParseYaw();
                JY61P_UpdateTurnCounter();
            }
            break;
        default:
            cnt = 0;
            jy_state = 0;
            break;
    }
}

void JY61P_Poll(void)
{
    while (jy_rx_tail != jy_rx_head) {
        uint8_t ch = jy_rx_buf[jy_rx_tail];
        jy_rx_tail = (uint16_t)((jy_rx_tail + 1U) % JY_RX_BUF_SIZE);
        JY61P_DecodeByte(ch);
    }
}

void UART_JY61P_INST_IRQHandler(void)
{
    /* UART IRQ */
    switch (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            jy_rx_buf[jy_rx_head] = DL_UART_Main_receiveData(UART_JY61P_INST);
            jy_rx_head = (uint16_t)((jy_rx_head + 1U) % JY_RX_BUF_SIZE);
            break;
        default:
            break;
    }
}
