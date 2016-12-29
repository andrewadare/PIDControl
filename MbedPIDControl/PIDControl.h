// Simple PID control class for ARM Mbed environment.
// Based on https://github.com/br3ttb/Arduino-PID-Library
// and the accompanying explanations at
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//
// Differences:
// - Minimal API: everything is public to keep the library as small and simple as possible.
// - Names and comments are different (part of my learning process)
// - Doesn't rely on global variables in user code.
// - For efficiency, optionally skip PID update if error is sufficiently small.
// - No manual/auto selection (always on, but see deadband option)
// - No option for "direction". If a reverse-PID loop is needed, simply use
//   maxOutput - output instead of output.

#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#include "mbed.h"

class PIDControl
{
public:

  PIDControl(float p, float i, float d, float initialSetpoint, int timestep);

  ~PIDControl() {}

  // Compute new output. If |error| < deadband, don't bother.
  void update(float input, float deadband = 0);

  // Set/change kp, ki, kd and scale so output is independent of choice of dt.
  void setPID(float p, float i, float d);

  // Set/change dt and update time-scaling factors for ki, kd.
  void setDtMilliseconds(unsigned long updateInterval /*ms*/);

  // Utility method to constrain x within [lo, hi].
  void clamp(float &x, float lo, float hi);

  // Coefficients for the 3 PID terms (error e defined as setpoint - input):
  // Proportional (kp*e), integral (ki*sum(e) over dt), and derivative (kd*de/dt)
  // Public for convenient read access, but use setPID to assign.
  float kp;
  float ki;
  float kd;

  // Target value for output
  float setpoint;

  // Computed output value and its boundaries
  float output;
  float minOutput;
  float maxOutput;

  // Parameter update interval in ms. Fixed, regardless of input sample rate.
  // Public for convenient read access, but use setDtMilliseconds to assign.
  int dt;

  // Timer class from Mbed SDK
  Timer timer;

  // Explicitly store the PID integral term.
  // For each timestep j, add ki_j * e_j to the running sum during the update.
  // This provides more correct behavior for time-dependent ki than doing
  // ki*sum(e_j), enabling real-time adjustment.
  float integralTerm;

  // Input and time from previous update for use in calculation
  float prevInput;
  int prevTime;
};

#endif
