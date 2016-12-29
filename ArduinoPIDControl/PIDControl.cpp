#include "PIDControl.h"

PIDControl::PIDControl(float p, float i, float d, float initialSetpoint, unsigned long timestep) :
  kp(p),
  ki(i),
  kd(d),
  setpoint(initialSetpoint),
  output(0.0),
  minOutput(-1.0),
  maxOutput(1.0),
  dt(timestep),
  integralTerm(0),
  prevInput(0),
  prevTime(0)
{
  setPID(p, i, d);
  setDtMilliseconds(timestep);
}

void PIDControl::update(float input, unsigned long currentTime)
{
  float error = setpoint - input;

  // Return early if it's too soon for an update.
  if (currentTime - prevTime < dt)
  {
    return;
  }

  // Store integral term separately to handle a time-varying ki gain parameter
  integralTerm += ki*error;

  // Avoid integral windup by constraining integral term to output limits
  integralTerm = clamped(integralTerm, minOutput, maxOutput);

  // Compute PID output.
  // Note that d(setpoint)/dt is excluded from the derivative term to avoid
  // spikes from fast setpoint changes.
  output = kp * error + integralTerm - kd * (input - prevInput);

  output = clamped(output, minOutput, maxOutput);

  // Store for next call
  prevInput = input;
  prevTime = currentTime;
}

void PIDControl::setPID(float p, float i, float d)
{
  kp = p;

  // Correct for step-size dependence in the integration and differentiation.
  ki = i * dt;
  kd = d / dt;
}

void PIDControl::setDtMilliseconds(unsigned long updateInterval)
{
  dt = updateInterval; // ms

  // Adjust PID parameters for the new time step size
  setPID(kp, ki, kd);
}

float PIDControl::clamped(float val, float low, float high)
{
  return val < low ? low : val > high ? high : val;
}

