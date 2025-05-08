#include "pidcontroller.h"

PID_t pid;

/**********Initializing**********/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    if (!pid) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/**********Duty update**********/
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (!pid || dt <= 0.0f || isnan(dt))
        return 0.0f;

    /* 1) 오차 */
    float error       = setpoint - measurement;

    /* 2) ∫ e dt  (Anti-windup: 출력 범위 밖이면 적분 정지) */
    float candidate_i = pid->integral + error * dt;
    float p_term      = pid->kp * error;
    float d_term      = pid->kd * (error - pid->prev_error) / dt;
    float pre_output  = p_term + pid->ki * candidate_i + d_term;

    if (pre_output > pid->out_min && pre_output < pid->out_max)
        pid->integral = candidate_i;   /* 적분 허용 */

    /* 3) 최종 출력 및 상태 갱신 */
    float output = p_term + pid->ki * pid->integral + d_term;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;
    return output;
}
