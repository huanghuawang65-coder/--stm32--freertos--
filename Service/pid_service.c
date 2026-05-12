#include "pid_service.h"

static int32_t PID_Service_Clamp32(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

void PID_Service_Init(PIDController_t *pid,
                      int16_t kp,
                      int16_t ki,
                      int16_t kd,
                      int16_t scale,
                      int32_t integral_limit,
                      int16_t output_min,
                      int16_t output_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->scale = (scale == 0) ? 1 : scale;
    pid->integral_min = -integral_limit;
    pid->integral_max = integral_limit;
    pid->output_min = output_min;
    pid->output_max = output_max;

    PID_Service_Reset(pid);
}

void PID_Service_Reset(PIDController_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0;
    pid->last_error = 0;
}

int16_t PID_Service_Calc(PIDController_t *pid, int16_t target, int16_t actual)
{
    int16_t error;
    int16_t derivative;
    int32_t output;

    if (pid == 0)
    {
        return 0;
    }

    /*
     * 位置式 PID：输出值直接作为 PWM 占空比。
     * 这里使用整数比例系数，避免 STM32F103 没有硬件 FPU 时引入过多浮点开销。
     */
    error = target - actual;
    pid->integral += error;
    pid->integral = PID_Service_Clamp32(pid->integral, pid->integral_min, pid->integral_max);

    derivative = error - pid->last_error;
    pid->last_error = error;

    output = ((int32_t)pid->kp * error) +
             ((int32_t)pid->ki * pid->integral) +
             ((int32_t)pid->kd * derivative);
    output /= pid->scale;

    output = PID_Service_Clamp32(output, pid->output_min, pid->output_max);

    return (int16_t)output;
}
