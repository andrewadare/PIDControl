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

  // Return early if it's too soon for an update
  if (now - prevTime < dt)
  {
    return;
  }

  // Store integral term separately to handle a time-varying ki gain parameter.
  integralTerm += ki*error;

  // Limit integral windup. Output limits used here; other choices are possible.
  clamp(integralTerm, minOutput, maxOutput);

  // Compute PID output.
  if (fabs(error) > deadband)
  {
    // Derivative term kd*de/dt adds a predictive component to the correction.
    // de/dt = d(setpoint - input)/dt = -d(input)/dt if setpoint is constant.
    // Here, exclude d/dt(setpoint) altogether, assuming step-like time dependence
    // of the setpoint (contributes either nothing or large spikes to output).
    float derivativeTerm = kd*(prevInput - input);

    output = kp*error + integralTerm + derivativeTerm;
    clamp(output, minOutput, maxOutput);
  }

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
  else if (x > hi) x = hi;
}
