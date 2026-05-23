#include"encoder.h"


//读取定时器的编码器模式计数器的值
int Read_Speed(TIM_HandleTypeDef* htim)
{
    int temp;
    temp =(short) __HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim,0);
    return temp;
}
