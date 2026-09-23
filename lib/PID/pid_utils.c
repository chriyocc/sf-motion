/*
 * pid_utils.c
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#include "pid_utils.h"
#include <string.h>

/* Bit check remains valid with the firmware -ffast-math build. */
static int finite_float(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

#ifndef TWO_PI
#define TWO_PI 6.2831853f
#endif

float pi_control(PID_Controller_t *pi, float error) {
    if (error >= -pi->e_deadband && error <= pi->e_deadband) {
        error = 0.0f;
    }

    float p_term = pi->kp * error;

    float new_integral = pi->integral + error * pi->ki * pi->ts;

    float output = p_term + new_integral;

    /* Integrate only when unsaturated or when integration unwinds saturation. */
    float increment = new_integral - pi->integral;
    if (!((output > pi->out_max && increment > 0.0f) ||
          (output < pi->out_min && increment < 0.0f))) {
        pi->integral = new_integral;
    }
    output = p_term + pi->integral;
    if (output > pi->out_max) output = pi->out_max;
    else if (output < pi->out_min) output = pi->out_min;

    pi->mv = output;

    return output;
}

float pd_control(PID_Controller_t *pd, float error) {
    if (error >= -pd->e_deadband && error <= pd->e_deadband) {
        pd->last_error = 0.0f;  // Reset last_error ketika dalam deadband
        return 0.0f;
    }

    float derivative = (error - pd->last_error);

    pd->last_error = error;

    float p_term = pd->kp * error;
    float d_term = pd->kd / pd->ts * derivative;

    float output = p_term + d_term;

    // Output clamping
    if (output > pd->out_max) {
        output = pd->out_max;
    }
    else if (output < pd->out_min) {
        output = pd->out_min;
    }

    return output;
}

float pid_control(PID_Controller_t *pid, float error) {
    if (error >= -pid->e_deadband && error <= pid->e_deadband) {
        error = 0.0f;
    }

    float p_term = pid->kp * error;

    float error_derivative = (error - pid->last_error) / pid->ts;
    pid->d_filtered = (1.0f - pid->d_alpha_filter) * pid->d_filtered + pid->d_alpha_filter * error_derivative;
    // clamp derivative
    if (pid->d_filtered > pid->d_max) pid->d_filtered = pid->d_max;
    else if (pid->d_filtered < -pid->d_max) pid->d_filtered = -pid->d_max;
    float d_term = pid->d_filtered * pid->kd;
    pid->last_error = error;

    float new_integral = pid->integral + error * pid->ki * pid->ts;
    float pd_term = p_term + d_term;
    float output = pd_term + new_integral;

    /* Integrate only when unsaturated or when integration unwinds saturation. */
    float increment = new_integral - pid->integral;
    if (!((output > pid->out_max && increment > 0.0f) ||
          (output < pid->out_min && increment < 0.0f))) {
        pid->integral = new_integral;
    }
    output = pd_term + pid->integral;
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}

void pid_reset(PID_Controller_t *p) {
	p->integral = 0;
    p->last_error = 0.0f;
    p->d_filtered = 0.0f;
}

int8_t pid_set_kp(PID_Controller_t *pid, float kp) {
    if (!finite_float(kp) || kp < 0) return -1;
    pid->kp = kp;
    return 0;
}

int8_t pid_set_ki(PID_Controller_t *pid, float ki) {
    if (!finite_float(ki) || ki < 0) return -1;
    pid->ki = ki;
    return 0;
}

int8_t pid_set_kd(PID_Controller_t *pid, float kd) {
    if (!finite_float(kd) || kd < 0) return -1;
    pid->kd = kd;
    return 0;
}

int8_t pid_set_ts(PID_Controller_t *pid, float ts) {
    if (!finite_float(ts) || ts <= 0) return -1;
    pid->ts = ts;
    return 0;
}

int8_t pid_set_out_constraint(PID_Controller_t *pid, float max, float min) {
    if (!finite_float(max) || !finite_float(min) || min > max) return -1;
    pid->out_max = max;
    pid->out_min = min;
    return 0;
}

int8_t pid_set_deadband(PID_Controller_t *pid, float deadband) {
    if (!finite_float(deadband) || deadband < 0) return -1;
    pid->e_deadband = deadband;
    return 0;
}

int8_t pid_set_d_filter_fc(PID_Controller_t *pid, float fc) {
    if (!finite_float(fc) || fc <= 0) return -1;
    pid->d_fc_lpf = fc;
    float tau = 1.0f / (TWO_PI * fc);
    pid->d_alpha_filter = pid->ts / (tau + pid->ts);
    if (pid->d_alpha_filter > 1.0f) pid->d_alpha_filter = 1.0f;
    return 0;
}

int8_t pid_set_max_d(PID_Controller_t *pid, float max) {
    if (!finite_float(max) || max <= 0) return -1;
    pid->d_max = max;
    return 0;
}

float pid_get_kp(PID_Controller_t *pid) {
    return pid->kp;
}

float pid_get_ki(PID_Controller_t *pid) {
    return pid->ki;
}

float pid_get_kd(PID_Controller_t *pid) {
    return pid->kd;
}

float pid_get_ts(PID_Controller_t *pid) {
    return pid->ts;
}

float pid_get_out_max(PID_Controller_t *pid) {
    return pid->out_max;
}

float pid_get_out_min(PID_Controller_t *pid) {
    return pid->out_min;
}

float pid_get_deadband(PID_Controller_t *pid) {
    return pid->e_deadband;
}

float pid_get_d_filter_fc(PID_Controller_t *pid) {
    return pid->d_fc_lpf;
}

float pid_get_d_alpha_filter(PID_Controller_t *pid) {
    return pid->d_alpha_filter;
}

float pid_get_max_d(PID_Controller_t *pid) {
    return pid->d_max;
}