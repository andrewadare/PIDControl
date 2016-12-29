// Simple PID control class. Based on https://github.com/br3ttb/Arduino-PID-Library
// and the accompanying explanations at
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//
// Differences:
// - Minimal API: fields are public to keep the library small and simple
// - Platform-independent: no dependencies
// - Naming, style conventions, and comments (part of my learning process)
// - Doesn't rely on modifying global variables in user code
// - No manual/auto selection (always on; switch externally if needed.)
// - This is a conventional reverse-acting loop. If a direct-acting loop is needed,
//   negate the output (possibly with an offset).

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
   * Compute new output.
   *
   * @param input - process variable (measured feedback quantity)
   * @param currentTime - time when update is called. Units must match this.dt
   */
  void update(float input, unsigned long currentTime);

  /**
   * Return val if within limits or nearest limit if outside.
   *
   * @param  val - input value
   * @param  low - lower limit
   * @param  high - upper limit
   *
   * @return clamped value
   */
  float clamped(float val, float low, float high);

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
   * @param updateInterval - timestep value
   */
  void setDtMilliseconds(unsigned long updateInterval);

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

  // Parameter update interval. Fixed, regardless of input sample rate.
  // Public for convenient read access, but use setDtMilliseconds to assign.
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
