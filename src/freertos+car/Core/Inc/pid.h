#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"



void Control(void);

#endif


