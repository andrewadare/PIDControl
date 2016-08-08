#include "mbed.h"
#include "PIDControl.h"

// PID parameters
float kp = 2, ki = 100, kd = 0;
float initialSetpoint = 0.5; // Half max or 512 ADC units
int timestep = 20; // ms
float input;

// Serial connection to PC over USB
Serial pc(USBTX, USBRX);
char cmd[100];

AnalogIn ain(A0);
AnalogOut aout();

PIDControl pid(kp, ki, kd, initialSetpoint, timestep);

void handle_byte(char b)
{
  // kp
  if (b == 'p') // increase kp
    kp += 0.01;
  if (b == 'l') // decrease kp
    kp -= 0.01;

  // ki
  if (b == 'i') // increase ki
    ki += 0.1;
  if (b == 'k') // decrease ki
    ki -= 0.1;

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
    pc.printf("\r\nsetpoint: %f", pid.setpoint);
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
    pc.printf("kp,ki,kd: %f %f %f; p,i,d: %f %f %f; setpoint: %f; output: %f",
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

  while (true)
  {
    input = ain.read();
    pid.update(input);
    if (pc.readable())
    {
      handle_byte(pc.getc());
    }
  }


  return 0;
}