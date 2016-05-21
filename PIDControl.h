// Simple class for Arduino PID control using ideas from
// https://github.com/br3ttb/Arduino-PID-Library
// and
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

class PIDControl
{
public:

  PIDControl(float p, float i, float d, float initialSetpoint, unsigned long timestep);

  ~PIDControl() {}

  // Compute new output. If |error| < deadband, don't bother.
  void update(float input, float deadband = 0);

  // Set/change kp, ki, kd and scale so output is independent of choice of dt.
  void setPID(float p, float i, float d);

  // Set/change dt and update time-scaling factors for ki, kd.
  void setUpdateInterval(unsigned long updateInterval /*ms*/);

  // Coefficients for the 3 PID terms (error e defined as setpoint - input):
  // Proportional (kp*e), integral (ki*sum(e) over dt), and derivative (kd*de/dt)
  float kp;
  float ki;
  float kd;

  // Target value for output
  float setpoint;

  // Computed output value (sum of 3 terms above) and its allowed range
  float output;
  float minOutput;
  float maxOutput;

  // Parameter update interval in ms. Fixed, regardless of input rate.
  unsigned long dt;

  // Explicitly store the PID integral term.
  // For each timestep j, add ki_j * e_j to the running sum during the update.
  // This provides more correct behavior for time-dependent ki than doing
  // ki*sum(e_j), enabling real-time adjustment.
  float integralTerm;

  // Input and time from previous update used in calculation
  float prevInput;
  unsigned long prevTime;
};

#endif
