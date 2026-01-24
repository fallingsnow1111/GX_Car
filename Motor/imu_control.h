#ifndef __IMU_CONTROL_H
#define __IMU_CONTROL_H

#include "Struct_encapsulation.h"
#include "pid.h"
#include "IMU.h"
#include "motor_command.h"

typedef struct IMU_RUNDATA
{
    float ANGLE;
    float LAST_ANGLE;
    float IS_MOVING;
};



#endif
