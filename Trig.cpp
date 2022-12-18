/*
  Trig.cpp -  Library for offloading calculations to Core1.
  Created by Joshua McCunn (mccunnj@gmail.com) December 11, 2022.
  Released to the public.
*/

#include "Arduino.h"
#include "Trig.h"

// MultiSwap::MultiSwap(int pin)
// {
//   pinMode(pin, OUTPUT);
//   _pin = pin;
// }


void Trig::functionTimer(int t)
{
  int finish = micros();
  int total = finish - t;
  Serial.print(t);
  Serial.print(" - ");
  Serial.println(finish);
  Serial.print("Function took: ");
  Serial.print( total );
  Serial.println(" microseconds to complete.");
}
float Trig::a2r(float a)//Degrees to Radians
{
  float r = a / 180.0 * PI;
  return r;
}
float Trig::r2a(float r)//Radians to Degrees
{
  return r * 180.0 / PI;
}
float Trig::otheracos(float x)//Faster acos()
{
  //int start = micros();
  float negate = float(x < 0);
  float ret = -0.0187293;
  x = abs(x);
  ret = x * -0.0187293;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret *= sqrt(1.0 - x);
  ret = ret - 2.0 * negate * ret;
  float acos = negate * PI + ret;
  //this->functionTimer(start);
  return acos;
}
int32_t Trig::prep(float f){
  return this->otheracos(f) * 100;
}