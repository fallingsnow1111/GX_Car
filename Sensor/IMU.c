#include "IMU.h"

#include <string.h>

#include "imu_control.h"

#define BUFFER_SIZE 128

//配置命令数组
//解锁寄存器 FF AA 69 88 B5
//Z轴角度归零FF AA 76 00 00
//设置200HZ输出 FF AA 03 0B 00
//设置波特率115200 FF AA 04 06 00
//保存 FF AA 00 00 00
//重启 FF AA 00 FF 00
uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};

struct  IMU imu;
static uint8_t imu_buffer[BUFFER_SIZE];
static uint8_t imu_buf_len = 0;
uint8_t mpu_flash = 0;  //暂时不知道作用

void  U2_send(uint8_t data)
{
    uint16_t Usart2_time = 0;
    USART2->ISR = data;
    while((USART2->ISR & USART_ISR_TXE_Msk) == 0)    //USART_ISR_TXE=1表示发送寄存器为空
    {
        Usart2_time++;
        if (Usart2_time > 65535)
        {
            break;
        }
    }
    //清除串口接收端溢出错误标志位，防止外设卡死
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

void U2_writebuf(uint8_t* buf, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        U2_send(buf[i]);
    }
}

void IMU_Receive_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, imu_buffer, BUFFER_SIZE);
}

void IMU_SetZero(void)
{
    U2_writebuf(reset_z_axis,5);
    imu_run.IS_MOVING = 0;
    imu_run.LAST_ANGLE = 0;
}

void Imu_unlock_register(void)
{
    U2_writebuf(unlock_register,5);
}

void Imu_setset_baudrate_115200(void)
{
    U2_writebuf(set_baudrate_115200,5);
}

void Imu_setsave_settings(void)
{
    U2_writebuf(save_settings,5);
}
void Imu_set500hz(void)
{
    U2_writebuf(set_output_200Hz,5);
}

void USART3_IRQHandler(void)
{
    uint32_t idle_flag = 0;
    idle_flag = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);
    //IDLE标志位在检测到空闲线路时会置1
    if (idle_flag)
    {
        //两杠代表底层寄存器操作宏，一杠代表中层宏，无杠才是函数
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        HAL_UART_DMAStop(&huart2);
        imu_buf_len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        //怎么保证第0个数据是一定是0x55
        if (imu_buffer[0] == 0x55)
        {
            uint16_t sum = 0;
            //取第二个数据包，为什么？
            for (int i = 11; i < 21; i++)
            {
                sum += imu_buffer[i];
            }
            if (sum == imu_buffer[21])
            {
                if (imu_buffer[12] == 0x53)
                {
                    imu.yaw = (float)(0.9 * (180.0*(short)(imu_buffer[18]<<8 | imu_buffer[17])/32768.0) + 0.1 * imu.yaw);
                    mpu_flash = ~mpu_flash;
                }
            }
        }
        memset(imu_buffer,0,imu_buf_len);
        imu_buf_len = 0;
    }
    //重启DMA接收
    HAL_UART_Receive_DMA(&huart2, imu_buffer, BUFFER_SIZE);
    //调用HAL库中断处理函数
    HAL_UART_IRQHandler(&huart2);
}












