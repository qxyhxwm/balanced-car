#include "pid.h"

//传感器数据变量
int Encoder_Left,Encoder_Right;
//欧拉角
float pitch,roll,yaw;
//角速度值
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;


//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out;
//速度环的期望速度
int Target_Speed;
//转向环的期望转向角度
int Target_turn;
//平衡时的角度值偏移量 （机械中值）
float Med_Angle = 1;
int MOTO1,MOTO2;


float Vertical_Kp = 280,Vertical_Kd=1.8; //直立环 数量级(kp：0~1000 kd：0~10)
float Velocity_Kp = 0.4,Velocity_Ki = 0.002; //速度环 数量级（kp：0~1 ki固定为kp的200分之一）
float Turn_Kp = 10,Turn_Kd =0.6;
uint8_t stop;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern uint8_t Fore,Back,Left,Right;
extern float distance;
#define SPEED_Y 20 // 前后最大速度
#define SPEED_Z 150// 左右最大速度

//直立环PD控制器
//输入： 期望角度  真实角度  角速度 
int Vertical(float Med,float Angle,float gyro_X)
{
    int temp;
    temp=Vertical_Kp* (Angle - Med) + Vertical_Kd* gyro_X;
    return temp;
}



//速度环PI控制器
//输入： 期望速度  左编码器  右编码器
int Velocity(int Target , int encoder_L,int encoder_R)
{
    //积分值
    static int Encoder_S;
    //上次滤波后的值
    static int Err_LowOut_last ;
    //滤波系数
    static float a = 0.7;
    int Err,Err_LowOut;
    //速度环中间变量
    int temp;
    //1.计算偏差值
    Err = (encoder_L + encoder_R) - Target ; 

    Velocity_Ki = Velocity_Kp/200;

    //2.滤波
    Err_LowOut = (1-a)*Err + a*Err_LowOut_last;
    Err_LowOut_last = Err_LowOut;

    //3.积分
    Encoder_S += Err_LowOut;

    //4.积分限幅 防止积分过大
    Encoder_S = Encoder_S>20000?20000:(Encoder_S<-20000?-20000:Encoder_S);

    if (stop==1) 
    {
        Encoder_S=0;
        stop=0;
    }

    //5.速度环计算
    temp = Velocity_Kp*Err_LowOut + Velocity_Ki*Encoder_S;

    return temp;
}


//转向环PD控制器    
//输入：z向角速度  角度误差值
int Turn(float gyro_Z, int Target_turn)
{
    int temp;
    temp = Turn_Kp*Target_turn + Turn_Kd*gyro_Z;
    return temp;
}

//输入传感器的变量，输出电机的转速值
void Control(void)
{
    //保存直立环的输出值（控制电机的转速)
    int PWM_out;

    //1.读取编码器和陀螺仪（角速度和角度值）的数据
    Encoder_Left = Read_Speed(&htim2);
    Encoder_Right =-Read_Speed(&htim4);

    mpu_dmp_get_data(&pitch,&roll,&yaw);
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);

    //获取遥控的数据
    if(Fore==0&&Back==0) Target_Speed = 0; //未接受到前后信号 让目标速度为0
    if(Fore == 1)
    {
        if(distance<25)
        {
            Target_Speed--;
        }
        else
        {
            Target_Speed++;
        }
    }
    if(Back == 1) Target_Speed--;
    //前后速度限幅
    Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);

    //左右
    if(Left==0&&Right==0) Target_turn = 0;
    if(Left == 1) Target_turn -=30; //左转
    if(Right == 1) Target_turn +=30;    //右转
    Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);

    if(Left==0&&Right==0)Turn_Kd=0.6; //如果左右没有转向 则转向环的kd值为0.6
    else if((Left==1)||(Right==1))Turn_Kd=0; //如果左右有转向 则转向环的kd值为0 使得转向更加平滑


    //2.将数据传入到pid控制器中，计算出输出结果，输出左右电机转速值
    Velocity_out = Velocity(Target_Speed,Encoder_Left,Encoder_Right);
    Vertical_out = Vertical(Velocity_out+Med_Angle,roll,gyrox);
    Turn_out = Turn(gyroz,Target_turn);

    PWM_out = Vertical_out;
    MOTO1 = PWM_out-Turn_out;
    MOTO2 = PWM_out+Turn_out;
    
    Limit(&MOTO1,&MOTO2);
    Load(MOTO1,MOTO2);
}
