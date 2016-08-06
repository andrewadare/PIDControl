// This example controls a PID loop formed by the PWM ouput through an RC
// low-pass filter to the ADC input.
//
//        R = 10k
//  PWM --\/\/\/\---+---------- Vout -> ADC
//                __|__
//                _____ C = 0.1 uF
//                  |
//                 GND
//
// There is still some ripple, so a moving average is used for the PID input.
// The voltage can be watched on a scope while the setpoint and PID parameters
// are varied.

#include <PIDControl.h>

#define ADC_PIN 0 // A0
#define PWM_PIN 9

float kp = 2, ki = 100, kd = 0;

float initialSetpoint = 0.5; // Half max or 512 ADC units
unsigned long timestep = 20; // ms
unsigned int adc16 = 0;      // 16 * running average
String cmd = "";
int dutyCycle = 0;

PIDControl pid(kp, ki, kd, initialSetpoint, timestep);

void printStatus()
{
  String s = String("kp,ki,kd: ") + kp + ", " + ki + ", " + kd
             + String("; p,i,d: ") + pid.kp + ", " + pid.ki + ", " + pid.kd
             + String(" setpoint: ") + pid.setpoint
             + String(" output: ") + pid.output
             + String(" pwm: ") + dutyCycle;
  Serial.println(s);
}

void handleByte(byte b)
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

  if (b == '\r' || b == '\n')
  {
    pid.setpoint = constrain((float)cmd.toInt()/1023, pid.minOutput, pid.maxOutput);
    // pid.setpoint = constrain(cmd.toInt(), 0, 1023);
    Serial.print("\n\rsetpoint: ");
    Serial.println(pid.setpoint);
    cmd = "";
  }
  if (isDigit(b))
  {
    byte digit = b - 48;
    Serial.print(digit);
    cmd += digit;
  }
  else
  {
    pid.setPID(kp, ki, kd);
    printStatus();
  }
}

// Exponentially weighted moving average for integer data. Used for over-
// sampling noisy measurements in a time series.
// When using the result, it must be divided by 16: x16 >> 4.
void ewma(unsigned int x, unsigned int &x16)
{
  // Compute weights like 1/16*(current x) + 15/16*(prev running avg x), except
  // multiplied through by 16 to avoid precision loss from int division:
  // 16*xavg = x + 16*xavg - (16*xavg - 16/2)/16
  x16 = x + x16 - ((x16 - 8) >> 4);
}

void setup()
{
  // Clear default Timer 1 prescale setting, then reassign w.r.t. base frequency
  // See ATmega*8 datasheet Table 16-5
  TCCR1B &= !0x07;
  TCCR1B |= (1 << CS11); // 31350/8 = 3918 Hz

  Serial.begin(115200);

  dutyCycle = 255*initialSetpoint;
  analogWrite(PWM_PIN, dutyCycle);

  pid.minOutput = -1;
  pid.maxOutput = +1;

  Serial.println("");
  Serial.println("Enter setpoint [0-1023]: ");
}

void loop()
{
  // Take a measurement and fold it in to the weighted average
  ewma(analogRead(ADC_PIN), adc16);

  // Convert to an input value in the 0-1 range and compute the PID output
  float input = float(adc16 >> 4)/1023;
  pid.update(input);

  // From the PID output value (which is in the -1 to +1 range), compute an 8-bit
  // PWM duty cycle value. Scale up by 1e4 to improve accuracy of integer math.
  int output = 10000*pid.output;
  dutyCycle = map(output, 10000*pid.minOutput, 10000*pid.maxOutput, 0, 255);
  dutyCycle = constrain(dutyCycle, 0, 255);
  analogWrite(PWM_PIN, dutyCycle);

  if (Serial.available() > 0)
  {
    byte rxByte = Serial.read();
    handleByte(rxByte);
  }
}
