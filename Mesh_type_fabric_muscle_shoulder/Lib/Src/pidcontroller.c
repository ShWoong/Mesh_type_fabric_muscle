#include "pidcontroller.h"
#include <math.h>

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

    /* 1) 오차 계산 */
	float error = setpoint - measurement;

	/* 2) 적분 후보값 */
	float cand_i = pid->integral + error * dt;

	/* 3) P, D 항 계산 */
	float p = pid->kp * error;
	float d = pid->kd * (error - pid->prev_error) / dt;

	/* 4) anti-windup 검토 */
	float pre_out = p + pid->ki * cand_i + d;
	if (pre_out > pid->out_min && pre_out < pid->out_max) {
		pid->integral = cand_i;
	}

	/* 5) 출력 계산 및 클램프 */
	float output = p + pid->ki * pid->integral + d;
	if (output > pid->out_max) output = pid->out_max;
	if (output < pid->out_min) output = pid->out_min;

	/* 6) 상태 갱신 */
	pid->prev_error = error;
	return output;
}
