#ifndef __MOTOR_COMMAND_H
#define __MOTOR_COMMAND_H

#include "usart.h"
#include "dma.h"
#include "Struct_encapsulation.h"

#define RXdat_maxsize 128

// extern DMA_HandleTypeDef hdma_usart3_rx;

struct MOTOR_DATA {
    float target_angle;
    float actual_angle;
};


#endif
