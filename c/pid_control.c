#include "pid_control.h"

#include <stddef.h>

float clip(const float x, const float a, const float b) {
  return x < a ? a : x > b ? b : x;
}

void pid_update(const float input, const float ff, const float dt,
                pid_control_t* pid, const float* derivative) {
  // Update control error
  pid->error = pid->setpoint - input;

  // Contribution from error derivative de/dt.
  // This is intended to add stabilization or damping by countering high
  // rates of change in the measured system behavior. This generally only works
  // well if d(setpoint)/dt is small, otherwise including it can cause the
  // output to spike. So we approximate de/dt as (0 - d(input)/dt). However, it
  // is up to the user whether to include d(setpoint)/dt or omit it.
  // User-provided de/dt values are not modified here.
  float derivative_term = 0.;
  if (derivative) {
    derivative_term = pid->kd * (*derivative);
  } else {
    derivative_term = -pid->kd * (input - pid->input) / dt;
  }

  // Update integrator with accumulated error and keep it bounded
  pid->error_sum += pid->ki * pid->error * dt;
  pid->error_sum = clip(pid->error_sum, pid->min_output, pid->max_output);

  // Compute output and keep it bounded.
  pid->output = ff + pid->kp * pid->error + pid->error_sum + derivative_term;
  pid->output = clip(pid->output, pid->min_output, pid->max_output);

  pid->input = input;
}
