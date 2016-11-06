// Simple PID control class for Arduino. Based on https://github.com/br3ttb/Arduino-PID-Library
// and the accompanying explanations at
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//
// Differences:
// - Minimal API: fields are public to keep the library small and simple
// - Naming, style conventions, and comments (part of my learning process)
// - Doesn't rely on modifying global variables in user code
// - Deadband option: skip PID update if error is sufficiently small
// - No manual/auto selection (always on, but see deadband option)
// - This is a conventional reverse-acting loop. If a direct-acting loop is needed,
//   negate the output (possibly with an offset, like maxOutput - output).

#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

class PIDControl
{
public:

  /**
   * @constructor
   *
   * @param p - proportional gain coefficient (corrects error as it arises)
   * @param i - integral gain or "reset" coefficient (corrects drift)
   * @param d - derivative gain or "preact" coefficient (anticipates changes)
   * @param initialSetpoint - in units of the process variable
   * @param timestep - update interval in milliseconds
   */
  PIDControl(float p, float i, float d, float initialSetpoint, unsigned long timestep);

  ~PIDControl() {}

  /**
   * Compute new output. If |error| < deadband, output is unchanged.
   *
   * @param input - process variable (measured feedback quantity)
   * @param [deadband] - error threshold for update
   */
  void update(float input, float deadband = 0);

  /**
   * Set/change kp, ki, kd and scale so output is independent of choice of dt.
   *
   * @param p - proportional gain
   * @param i - integral gain
   * @param d - derivative gain
   */
  void setPID(float p, float i, float d);

  /**
   * Set/change dt and update time-scaling factors for ki, kd.
   *
   * @param updateInterval - in milliseconds
   */
  void setUpdateInterval(unsigned long updateInterval);

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
  // Public for convenient read access, but use setUpdateInterval to assign.
  unsigned long dt;

  // Explicitly store the PID integral term.
  // For each timestep j, add ki_j * e_j to the running sum during the update.
  // This provides more correct behavior for time-dependent ki than doing
  // ki*sum(e_j), enabling real-time adjustment.
  float integralTerm;

  // Input and time from previous update for use in calculation
  float prevInput;
  unsigned long prevTime;
};

#endif
