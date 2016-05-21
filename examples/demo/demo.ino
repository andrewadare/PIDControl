#include <PIDControl.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

float kp = 1, ki = 2, kd = 3;
float setpoint = 128;
unsigned long timestep = 100; // ms

PIDControl pid(kp, ki, kd, setpoint, timestep);

void setup()
{
}

void loop()
{
  float input = analogRead(PIN_INPUT);
  pid.update(input);
  analogWrite(PIN_OUTPUT, pid.output);
}
