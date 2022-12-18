/**
 * @file DR16.cpp
 * @author JIANG Yicheng (EthenJ@outlook.sg)
 * @brief
 * @version 0.1
 * @date 2022-10-25
 *
 * @copyright This file is only for HKUST Enterprize RM2023 internal competition. All Rights Reserved.
 */

#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "FreeRTOS.h"
#include "can.h"
#include "gpio.h"
#include "main.h"
#include "task.h"
#include "tim.h"
#include "usart.h"

StackType_t uxBlinkTaskStack[128];
StaticTask_t xBlinkTaskTCB;
void blink(void *pvPara)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    uint32_t i = 400;
    while (true)
    {
        for (; i < 500; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            vTaskDelay(20);
        }
        for (; i > 400; i--)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            vTaskDelay(20);
        }
    }
}

struct PID
{
    float Kp;
    float Ki;
    float Kd;

    float pout;
    float iout;
    float dout;
    float out;

    float error;
    float lastError;
    float integral;

    float dt = 0.001f;
};

static volatile float targetPos = 0;

StackType_t uxPIDTaskStack[128];
StaticTask_t xPIDTaskTCB;
void PIDTask(void *pvPara)
{
    DJIMotor::DJIMotor &motor = DJIMotor::getMotor(0x205);
    motor.setCurrentLimit(30000);
    static volatile PID pid;

    while (true)
    {
        pid.lastError = pid.error;
        pid.error = targetPos - motor.getPosition();

        pid.pout = pid.Kp * pid.error;

        pid.integral += pid.Ki * pid.error * pid.dt;
        pid.iout = pid.integral;

        pid.dout = pid.Kd * (pid.error - pid.lastError) / pid.dt;

        pid.out = pid.pout + pid.iout + pid.dout;

        motor.setOutputCurrent(pid.out);
        DJIMotor::sendMotorGroup(1);
        vTaskDelay((uint32_t)(pid.dt * 1000) == 0? 1: (uint32_t)(pid.dt * 1000));
    }
}

static volatile float step = 1;
StackType_t uxTargetUpdateTaskStack[128];
StaticTask_t xTargetUpdateTaskTCB;
void TargetUpdateTask(void *pvPara)
{
    while (true)
    {
        while (DR16::getRcData().rc.ch3 - 1024 <= -200 || DR16::getRcData().rc.ch3 - 1024 >= 200)
            vTaskDelay(1);
        while (DR16::getRcData().rc.ch3 - 1024 > -200 && DR16::getRcData().rc.ch3 - 1024 < 200)
            vTaskDelay(1);
        if (DR16::getRcData().rc.ch3 - 1024 > 0)
            targetPos += step;
        else
            targetPos -= step;
        vTaskDelay(1);
    }
}

/**
 * @brief Create user tasks
 */
void startUserTasks()
{
    DR16::init();
    DJIMotor::init();
    xTaskCreateStatic(blink, "blink", 128, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB);
    xTaskCreateStatic(PIDTask, "PID", 128, NULL, 10, uxPIDTaskStack, &xPIDTaskTCB);
    xTaskCreateStatic(TargetUpdateTask, "TargetUpdate", 128, NULL, 10, uxTargetUpdateTaskStack, &xTargetUpdateTaskTCB);
}

