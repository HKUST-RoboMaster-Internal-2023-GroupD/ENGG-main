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

static volatile float lim_max = 16384;
static volatile float lim_min = -16384;

static volatile bool rotate_valid;
static volatile float targetPos = 0;

void rotate_position(DJIMotor::DJIMotor &motor, PID &pid, float targetPos)
{
    //if (rotate_valid == false)
    //{
    //    motor.setOutputCurrent(0);
    //    return;
    //}

    float lastPos;
    static float Pos;
    float lim_min_integ;
    float lim_max_integ;

    lastPos = Pos;
    Pos = motor.getPosition();
    pid.error = targetPos - Pos;
    pid.pout = pid.Kp * pid.error;
    pid.integral += pid.Ki * pid.error * pid.dt;
    pid.iout = pid.integral;
    pid.dout = pid.Kd * (Pos - lastPos) / pid.dt;  // solve derivative kick
    // Anti-wind-up via Dynamic Integrator Clamping
    // Limits Computationa and Application
    if (lim_max > pid.pout)
        lim_max_integ = lim_max - pid.pout;
    else
        lim_max_integ = 0.0f;
    if (lim_min < pid.pout)
        lim_min_integ = lim_min - pid.pout;
    else
        lim_min_integ = 0.0f;

    // Constrain Integrator
    if (pid.iout > lim_max_integ)
        pid.iout = lim_max_integ;
    else if (pid.iout < lim_min_integ)
        pid.iout = lim_min_integ;

    pid.out = pid.pout + pid.iout + pid.dout;
    if (pid.out < lim_min)
        pid.out = lim_min;
    else if (pid.out > lim_max)
        pid.out = lim_max;
    motor.setOutputCurrent(pid.out);
}


StackType_t uxPIDTaskStack[128];
StaticTask_t xPIDTaskTCB;
void PIDTask(void *pvPara)
{
    // get and init the motors
    DJIMotor::DJIMotor &motor = DJIMotor::getMotor(0x201);
    motor.setCurrentLimit(16383);

    // set the parameter of pid
    static volatile PID pid_set;

    // the pid of the motor, will be past by reference to the function
    PID pid;

    // used to watch the behavior of the motors
    static volatile float curPos;

    while (true)
    {
        pid.Kd = pid_set.Kd;
        pid.Kp = pid_set.Kp;
        pid.Ki = pid_set.Ki;

        curPos = motor.getPosition();
        float Pos = targetPos;
        rotate_position(motor, pid, Pos);
        DJIMotor::sendMotorGroup(0);

        vTaskDelay((uint32_t)(pid_set.dt * 1000) == 0 ? 1 : (uint32_t)(pid_set.dt * 1000));
    }
}

StackType_t uxTargetUpdateTaskStack[128];
StaticTask_t xTargetUpdateTaskTCB;
void TargetUpdateTask(void *pvPara)
{
    while (true)
    {
        
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
