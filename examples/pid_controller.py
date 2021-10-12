import numpy as np


class PIDController():
    def __init__(self, p, i, d, initial_setpoint, timestep,
                 min_output=0.0, max_output=1.0):
        """
        Parameters
        ----------
        p : proportional gain coefficient (corrects error as it arises)
        i : integral gain or "reset" coefficient (corrects drift)
        d : derivative gain or "preact" coefficient (anticipates changes)
        initial_setpoint : in units of the process variable
        timestep : update interval in seconds
        """

        # Coefficients for the 3 PID terms (error e defined as setpoint - input):
        # Proportional (kp*e), integral (ki*sum(e) over dt), and derivative (kd*de/dt)
        self._kp = p
        self._ki = i
        self._kd = d

        # Target value for output
        self.setpoint = initial_setpoint

        # Computed output value and its boundaries
        self.output = 0.
        self.min_output = min_output
        self.max_output = max_output

        # Parameter update interval. Fixed, regardless of input sample rate.
        self.dt = timestep

        # Explicitly store the PID integral term.
        # For each timestep j, add ki_j * e_j to the running sum during the update.
        # This provides more correct behavior for time-dependent ki than doing
        # ki*sum(e_j), enabling real-time adjustment.
        self.integral_term = 0.

        # Input and time from previous update for use in calculation
        self.prev_input = 0.
        self.prev_time = 0.

    @property
    def kp(self):
        return self._kp

    @property
    def ki(self):
        return self._ki * self.dt

    @property
    def kd(self):
        return self._kd / self.dt

    def update(self, input, current_time, ext_int_factor=1.0):
        """
        Compute new output.

        Parameters
        ----------
        input : float
            process variable (measured feedback quantity)
        current_time : float
            Time when this function is called. Units must match self.dt
        ext_int_factor : float
            External integrator factor
        """
        error = self.setpoint - input

        # Return early if it's too soon for an update.
        if current_time - self.prev_time < self.dt:
            return

        # Store integral term separately to handle a time-varying ki gain parameter
        self.integral_term += ext_int_factor*self.ki*error

        # Avoid integral windup by constraining integral term to output limits
        self.integral_term = np.clip(self.integral_term, self.min_output,
                                     self.max_output)

        # Compute PID output.
        # Note that d(setpoint)/dt is excluded from the derivative term to avoid
        # spikes from fast setpoint changes.
        self.output = self.kp * error + self.integral_term - \
            self.kd * (input - self.prev_input)

        self.output = np.clip(self.output, self.min_output, self.max_output)

        # Store for next call
        self.prev_input = input
        self.prev_time = current_time
