// This example simply connects an AnalogOut pin to an AnalogIn pin. A scope
// is used to observe the response to instantaneous setpoint changes as the
// PID gain coefficients are varied.
// Enter a setpoint by selecting an integer in 0 (0V) - 1000 (3.3V), and change
// the PID gain parameters with the p,l,i,k,d,c keys.

#include "mbed.h"
#include "PIDControl.h"

// PID parameters
// float kp = 2, ki = 100, kd = 0;
float kp = 0.2, ki = 50, kd = 0;
float initial_setpoint = 0.5;
int timestep = 20; // ms
float input;

// Serial connection to PC over USB
Serial pc(USBTX, USBRX);
char cmd[100];

AnalogIn ain(A0);
AnalogOut aout(A3);

PIDControl pid(kp, ki, kd, initial_setpoint, timestep);

void handle_byte(char b)
{
  // kp
  if (b == 'p') // increase kp
    kp += 0.01;
  if (b == 'l') // decrease kp
    kp -= 0.01;

  // ki
  if (b == 'i') // increase ki
    ki += 1.0;
  if (b == 'k') // decrease ki
    ki -= 1.0;

  // kd
  if (b == 'd') // increase kd
    kd += 0.001;
  if (b == 'c') // decrease kd
    kd -= 0.001;

  // Input completed - convert to a float in [0,1] and update setpoint
  if (b == '\r' || b == '\n')
  {
    pid.setpoint = atof(cmd)/1000;
    pid.clamp(pid.setpoint, pid.minOutput, pid.maxOutput);
    pc.printf("\r\nsetpoint: %f\r\n", pid.setpoint);
    cmd[0] = 0; // Reset for next use
  }

  // Check if byte is a digit (0-9 = ASCII 48-57). If so, decode to decimal
  // integer value and concatenate to cmd string.
  else if (b > 47 && b < 58)
  {
    int digit = b - 48;
    pc.printf("%d", digit);
    sprintf(cmd, "%s%d", cmd, digit);
  }

  // Assume (without checking) input is one of p,l,i,k,d,c and update.
  else
  {
    pid.setPID(kp, ki, kd);
    pc.printf("kp,ki,kd: %f %f %f; p,i,d: %f %f %f; setpoint: %f; output: %f\r\n",
              kp,ki,kd,
              pid.kp,pid.ki,pid.kd,
              pid.setpoint,
              pid.output);
  }
}

int main()
{
  pid.minOutput = 0.0;
  pid.maxOutput = 1.0;

  pc.baud(115200);

  aout.write(initial_setpoint);

  while (true)
  {
    input = ain.read();
    pid.update(input);
    aout.write(pid.output);

    if (pc.readable())
    {
      handle_byte(pc.getc());
    }
  }


  return 0;
}