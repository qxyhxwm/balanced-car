#ifndef _ENCODER_H
#define _ENCODER_H

#include"stm32f1xx.h"

//读取定时器的编码器模式计数器的值
int Read_Speed(TIM_HandleTypeDef* htim);


#endif
