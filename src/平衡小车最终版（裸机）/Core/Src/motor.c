#include"motor.h"

#define PWM_MAX 7200
#define PWM_MIN -7200


extern TIM_HandleTypeDef htim1;



//取绝对值
int abs(int p)
{
    if(p>0) return p;
    else return -p;
}



//控制电机
//moto取值范围是-7200 ~ 7200 代表电机的转速 
void Load(int moto1,int moto2)
{
    //左电机
    //PB13 是in1 PB12是in2  01正转 10反转
    if(moto1>0)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    }

    //配置占空比 调速度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,abs(moto1));

    //右电机
    if(moto2>0)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,abs(moto2));

}
void Limit(int *motoA, int *motoB)
{
    //限制电机速度范围
    if(*motoA>PWM_MAX) *motoA=PWM_MAX;
    if(*motoA<PWM_MIN) *motoA=PWM_MIN;
    if(*motoB>PWM_MAX) *motoB=PWM_MAX;
    if(*motoB<PWM_MIN) *motoB=PWM_MIN;
}
