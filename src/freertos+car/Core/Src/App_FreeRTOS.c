/* App_FreeRTOS.c */
#include "App_FreeRTOS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "oled.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motor.h"
#include "encoder.h"
#include "sr04.h"
#include "pid.h"
#include "stdio.h"

//句柄定义
 
TaskHandle_t xBalanceTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xUARTTaskHandle = NULL;
TaskHandle_t xUltrasonicTaskHandle = NULL;

QueueHandle_t xUARTQueue = NULL;


//外部变量声明（来自 pid.c）

extern int Encoder_Left, Encoder_Right;
extern float pitch, roll, yaw;
extern short gyrox, gyroy, gyroz;
extern int MOTO1, MOTO2;
extern int Target_Speed;
extern int Target_turn;
extern uint8_t Fore, Back, Left, Right;
extern float distance;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;



//任务函数声明

static void vBalanceTask(void *pvParameters);
static void vDisplayTask(void *pvParameters);
static void vUARTTask(void *pvParameters);
static void vUltrasonicTask(void *pvParameters);


//平衡任务（合并 IMU + PID 控制）
//周期: 5ms, 优先级: 最高

static void vBalanceTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        /* ========== 1. 读取所有传感器数据 ========== */
        /* MPU6050 DMP 姿态数据 */
        mpu_dmp_get_data(&pitch, &roll, &yaw);
        
        /* 读取原始陀螺仪数据 */
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        
        /* 读取编码器值 */
        Encoder_Left = Read_Speed(&htim2);
        Encoder_Right = -Read_Speed(&htim4);
        
        /* ========== 2. PID 控制计算 ========== */
        Control();
        
        /* ========== 3. 输出到电机 ========== */
        Limit(&MOTO1, &MOTO2);
        Load(MOTO1, MOTO2);
        
        /* ========== 4. 精确延时 5ms ========== */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

/* ====================================================
 * 超声波测距任务 (周期: 50ms, 优先级: 2)
 * ==================================================== */
static void vUltrasonicTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        GET_Distance();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/* ====================================================
 * UART 蓝牙数据处理任务 (优先级: 1)
 * ==================================================== */
static void vUARTTask(void *pvParameters)
{
    uint8_t rx_data;
    
    while (1)
    {
        if (xQueueReceive(xUARTQueue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            switch (rx_data) 
            {
                case 0x00:  /* 停止 */
                    Fore = 0; Back = 0; Left = 0; Right = 0;
                    Target_Speed = 0;
                    Target_turn = 0;
                    break;
                case 0x01:  /* 前进 */
                    Fore = 1; Back = 0; Left = 0; Right = 0;
                    break;
                case 0x05:  /* 后退 */
                    Fore = 0; Back = 1; Left = 0; Right = 0;
                    break;
                case 0x03:  /* 右转 */
                    Fore = 0; Back = 0; Left = 0; Right = 1;
                    break;
                case 0x07:  /* 左转 */
                    Fore = 0; Back = 0; Left = 1; Right = 0;
                    break;
                case 0x02:  /* 原地左转 */
                    Fore = 0; Back = 1; Left = 1; Right = 0;
                    break;
                case 0x04:  /* 原地右转 */
                    Fore = 0; Back = 1; Left = 0; Right = 1;
                    break;
                default:
                    break;
            }
        }
    }
}

/* ====================================================
 * OLED 显示任务 (周期: 200ms, 优先级: 0)
 * ==================================================== */
static void vDisplayTask(void *pvParameters)
{
    uint8_t display_buf[20];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        /* 清屏并显示系统状态 */
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t*)"Balance Car OK", 16);
        
        /* 显示角度 */
        sprintf((char*)display_buf, "Pitch:%.1f", pitch);
        OLED_ShowString(0, 2, display_buf, 16);
        
        sprintf((char*)display_buf, "Roll:%.1f", roll);
        OLED_ShowString(0, 3, display_buf, 16);
        
        /* 显示编码器值 */
        sprintf((char*)display_buf, "L:%4d R:%4d", Encoder_Left, Encoder_Right);
        OLED_ShowString(0, 5, display_buf, 16);
        
        /* 显示距离 */
        if (distance < 100)
        {
            sprintf((char*)display_buf, "Dist:%.1fcm", distance);
            OLED_ShowString(0, 6, display_buf, 16);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

/* ====================================================
 * 创建所有任务并启动调度器
 * ==================================================== */
void App_FreeRTOS_Init(void)
{
    /* 1. 创建队列（UART 数据接收）*/
    xUARTQueue = xQueueCreate(10, sizeof(uint8_t));
    
    /* 2. 创建任务 */
    /* 平衡任务：最高优先级 4，栈 512，周期 5ms */
    xTaskCreate(vBalanceTask, "Balance", 512, NULL, 4, &xBalanceTaskHandle);
    
    /* 超声波任务：优先级 2 */
    xTaskCreate(vUltrasonicTask, "Ultrasonic", 256, NULL, 2, &xUltrasonicTaskHandle);
    
    /* UART 任务：优先级 1 */
    xTaskCreate(vUARTTask, "UART", 256, NULL, 1, &xUARTTaskHandle);
    
    /* 显示任务：最低优先级 0 */
    xTaskCreate(vDisplayTask, "Display", 256, NULL, 0, &xDisplayTaskHandle);
    
    /* 3. 启动调度器 */
    vTaskStartScheduler();
}
