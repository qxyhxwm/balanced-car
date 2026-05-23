#include "sr04.h"

extern TIM_HandleTypeDef htim3;

//保存计数器的值
uint16_t count;
//保存距离的值
float distance;

void RCCdelay_us(uint32_t udelay)
{
    __IO uint32_t Delay = udelay * 72 /8;
    do
    {
        __NOP();
    }while(Delay--);
}


void GET_Distance(void)
{
    //触发信号中先拉高TTL10us 发出超声波
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
    RCCdelay_us(12);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}
	

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //超声波的中断
    if(GPIO_Pin == GPIO_PIN_2)
    {
            //上升沿触发中断 开启定时器
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_SET)
        {
            //先让定时器清0
            __HAL_TIM_SetCounter(&htim3,0);
            HAL_TIM_Base_Start(&htim3);
        }
        else
        {
            //若是下降沿 让定时器停止计时
            HAL_TIM_Base_Stop(&htim3);
            count=__HAL_TIM_GetCounter(&htim3);
            //计算距离 单位为cm
            distance = count *0.017;
        }
    }
    

    
}
