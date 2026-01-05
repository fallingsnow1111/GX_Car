#include "motor_command.h"

MOTOR_SPEED_t car_setspeed;
struct MOTOR_DATA motor1;
struct MOTOR_DATA motor2;
struct MOTOR_DATA motor3;
struct MOTOR_DATA motor4;

static uint8_t LB_send[15];
static uint8_t LF_send[15];
static uint8_t RB_send[15];
static uint8_t RF_send[15];

static uint8_t rx_dat[RXdat_maxsize]={0};
static char rx_piont=0;

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit_DMA(&huart3,buf,len);
}

void Motor_Init(void)
{
    car_setspeed.x_setpeed = 0.0f;
    car_setspeed.y_setpeed = 0.0f;
    car_setspeed.w_setpeed = 0.0f;
    motor1.target_angle = 0;
    motor1.actual_angle = 0;
    motor2.target_angle = 0;
    motor2.actual_angle = 0;
    motor3.target_angle = 0;
    motor3.actual_angle = 0;
    motor4.target_angle = 0;
    motor4.actual_angle = 0;
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // Enable USART3 interrupt
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dat, RXdat_maxsize); // Start DMA reception
}

//输入4个电机速度，单位rpm，16位有符号整数
void Motor_Send_Speed_together(float LB, float LF, float RF, float RB)
{
    static uint8_t* LB_speedptr = LB_send;
    static uint8_t* LF_speedptr = LF_send;
    static uint8_t* RB_speedptr = RB_send;
    static uint8_t* RF_speedptr = RF_send;
    uint8_t* temp[4] = {LB_speedptr, LF_speedptr, RF_speedptr, RB_speedptr};
    int tempspeed = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t var = i;
        switch (i + 1) {
            case 1:
                tempspeed = LB;
                break;
            case 2:
                tempspeed = LF;
                break;
            case 3:
                tempspeed = RF;
                break;
            case 4:
                tempspeed = RB;
                break;
            default:
                tempspeed = 0;
                break;
        }

        temp[i][0] = var + 1;   // Motor ID
        temp[i][1] = 0xF6;      // Command for speed setting

        // if (__fbs(tempspeed) > 255)
        // {
        //     //空逻辑？
        // }

        if (tempspeed > 0)
        {
            temp[i][2] = 0x00;
            temp[i][3] = (tempspeed >> 8) & 0xFF;   // High byte
            temp[i][4] = (tempspeed & 0xFF);        // Low byte
        }
        else
        {
            tempspeed = -tempspeed;
            temp[i][2] = 0x01;
            temp[i][3] = (tempspeed >> 8) & 0xFF;   // High byte
            temp[i][4] = (tempspeed & 0xFF);        // Low byte
        }
        temp[i][5] = 0xC8;     //Acceleration
        temp[i][6] = 0x00;     // Multi-machine synchronization flag
        temp[i][7] = 0x6B;     // End byte
    }
}













