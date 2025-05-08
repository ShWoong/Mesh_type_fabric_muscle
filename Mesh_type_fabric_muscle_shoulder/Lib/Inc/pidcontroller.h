#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integral;
    float prev_error;
} PID_t;


void  PID_Init(PID_t *pid,
               float kp, float ki, float kd,
               float out_min, float out_max);

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt);

#ifdef __cplusplus
}
#endif

#endif /* PIDCONTROLLER_H */
