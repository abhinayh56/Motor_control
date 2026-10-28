#include "Timer_utils.h"
#include "TB6612FNG.h"
#include "L298N.h"
#include "Quadrature_enc.h"
#include "PID_P.h"

#define MAIN_LOOP_FREQ   250.0

Timer_utils timer(MAIN_LOOP_FREQ);
TB6612FNG motorDriver;
Quadrature_enc encoder;

double voltInMotorA = 0;
double voltInMotorB = 0;

void setup()
{
  Serial.begin(115200);

  timer.init(MAIN_LOOP_FREQ);

  motorDriver.config(7, 12, 255);
  motorDriver.configMotorA(4, 6, 5, false);
  motorDriver.configMotorB(8, 9, 10, true);

  motorDriver.init();
  motorDriver.initMotorA();
  motorDriver.initMotorB();

  motorDriver.enableMotorA();
  motorDriver.enableMotorB();

  encoder.config(5800, 2, 3, []() { encoder.isrFunction(); }, false);
  encoder.init();
}

void loop()
{
  encoder.update();
  while (Serial.available() > 0)
  {
    // voltInMotorA = Serial.readString().toDouble();
    voltInMotorB = Serial.readString().toDouble();
  }

  Serial.print(voltInMotorA);
  Serial.print("\t");
  Serial.println(voltInMotorB);

  motorDriver.commandMotorA(voltInMotorA);
  motorDriver.commandMotorB(voltInMotorB);

  timer.sleep();
}
