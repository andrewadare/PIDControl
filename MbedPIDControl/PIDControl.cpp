#include "mbed.h"
#include "PIDControl.h"

PIDControl::PIDControl(float p, float i, float d, float initialSetpoint, int timestep) :
  kp(p),
  ki(i),
  kd(d),
  setpoint(initialSetpoint),
  output(0),
  minOutput(0),
  maxOutput(255),
  dt(timestep),
  integralTerm(0),
  prevInput(0),
  prevTime(0)
{
  setPID(p, i, d);
  setUpdateInterval(timestep);
  timer.start();
}

void PIDControl::update(float input, float deadband)
{
  int now = timer.read_ms();

  float error = setpoint - input;

  // Return early if it's too soon for an update or if no correction is required.
  if (now - prevTime < dt || fabs(error) < deadband)
  {
    return;
  }

  // Store integral term separately to handle a time-varying ki gain parameter.
  integralTerm += (ki * error);
  clamp(integralTerm, minOutput, maxOutput);

  // Compute PID output.
  // Note that d/dt (setpoint) is excluded from the derivative term to avoid
  // spikes from fast setpoint changes.
  output = kp * error + integralTerm - kd * (input - prevInput);
  clamp(output, minOutput, maxOutput);

  // Store for next call
  prevInput = input;
  prevTime = now;
}

void PIDControl::setPID(float p, float i, float d)
{
  kp = p;

  // The dtSeconds factor corrects for step-size dependence in the integration
  // and differentiation.
  float dtSeconds = (float)dt / 1000;
  ki = i * dtSeconds;
  kd = d / dtSeconds;
}

void PIDControl::setUpdateInterval(unsigned long updateInterval)
{
  dt = updateInterval;

  // Adjust PID parameters for the new time step size
  setPID(kp, ki, kd);
}

void PIDControl::clamp(float &x, float lo, float hi)
{
  if (x < lo) x = lo;
  if (x > hi) x = hi;
}
