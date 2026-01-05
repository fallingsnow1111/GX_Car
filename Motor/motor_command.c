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

void Motor_Send_Speed_together(float LB, float LF, float RF, float RB)
{



}













