#include "JY61P.h"


// --------------------- 全局变量 ---------------------
volatile float yaw_real = 0;        // 当前偏航角 -180 ~ 180
volatile float total_yaw = 0.0f;       // 累计总角度
volatile float last_yaw = 0;        // 上一次角度
volatile int turn_90_count = 0;          //记录转过多少个90°
volatile float target_yaw = 0.0f;           // 起步时的基准角度 (目标朝向)

uint8_t Start_Flag=0;

uint8_t uart2_rxbuff;
uint8_t jy_buf[11];  // 帧长：11字节

uint8_t jy_state=0;
uint8_t jy_rx_buf[JY_RX_BUF_SIZE];   //软件环形缓冲区
volatile uint16_t jy_rx_head = 0;    //环形缓存区的头指针
uint16_t jy_rx_tail = 0;   //cpu去读取数据的尾指针



// 解析偏航角
void parse_yaw(void)
{
    // 1. 校验和
    uint8_t sum = 0;
    for(int i=0; i<10; i++) sum += jy_buf[i];
    if(sum != jy_buf[10]) return;

    // 2. 解析偏航角
    int16_t yaw_raw = ((int16_t)jy_buf[7] << 8) | jy_buf[6];
    yaw_real = (float)yaw_raw / 32768.0f * 180.0f;

}

void car_turn_90(void)
{
    float diff = yaw_real - last_yaw;
    if (diff < -180.0f) diff += 360.0f;
    else if (diff > 180.0f)  diff -= 360.0f;
	
    if(Start_Flag == 1) 
    { 
        total_yaw += diff; 
    }
    last_yaw = yaw_real;

    // 比如 total_yaw 到达 46 度时，就会认为已经进入了第一次转弯
    turn_90_count = (int)((fabs(total_yaw) + 45.0f) / 90.0f);

    // 如果是正转就设定正的角度
    if (total_yaw >= 0) 
	{
        target_yaw = turn_90_count * 90.0f;
    } 
	else 
	{
        target_yaw = -turn_90_count * 90.0f;
    }
}


void JY61P_cicly_Group(uint8_t ch)
{
    static uint8_t cnt = 0;
    switch(jy_state)
    {
        case 0:
            if(ch == 0x55) { jy_buf[0] = ch; jy_state = 1; }   //头帧
            break;
        case 1:
            if(ch == 0x53) { jy_buf[1] = ch; jy_state = 2; cnt = 0; }
            else { jy_state = 0; }
            break;
        case 2:
            jy_buf[2 + cnt] = ch; //一共11个数据
            cnt++;
            if(cnt >= 9) 
			{
                cnt = 0;
                jy_state = 0;
                parse_yaw();    //数据接收完毕之后送去解析角度
				car_turn_90();    //更新转过多少个拐弯处
            }
            break;
        default:
            jy_state = 0;
            break;
    }
}


void JY61P_Poll(void)
{
    while (jy_rx_tail != jy_rx_head)
    {
        uint8_t ch = jy_rx_buf[jy_rx_tail];
        // 尾指针向前移动一步
        jy_rx_tail = (jy_rx_tail + 1) % JY_RX_BUF_SIZE;
        JY61P_cicly_Group(ch);
    }
}


// 重置圈数（启动时调用一次）
void reset_turn_count(void)
{
    total_yaw = 0;
    turn_90_count = 0;
	//起步时候的角度作为0°的相对基准
	last_yaw = yaw_real;
}


void UART_JY61P_INST_IRQHandler(void)
{
    // 判断是不是串口接收中断
    switch (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST)) 
    {
        case DL_UART_MAIN_IIDX_RX:    //告诉底层是接收触发的中断，不需要执行很多的判断指令
            // 从底层寄存器读出一个字节
            jy_rx_buf[jy_rx_head] = DL_UART_Main_receiveData(UART_JY61P_INST);
            // 头指针向前移动一步（如果越界就回到0）
            jy_rx_head = (jy_rx_head + 1) % JY_RX_BUF_SIZE;
            break;
        default:
            break;
    }
}