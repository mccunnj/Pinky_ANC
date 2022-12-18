/*
  Trig.h - Library for offloading calculations to Core1.
  Created by Joshua McCunn (mccunnj@gmail.com) December 11, 2022.
  Released to the public.

*/
#ifndef Trig_h
#define Trig_h

#include "Arduino.h"

class Trig
{
  public:
    //Trig(int pin);
    //Trig(Stream &p) : port(p){}
    
    float a2r(float a);//Degrees to Radians
    float r2a(float r);//Radians to Degrees
    float otheracos(float x);// 10-30microseconds faster than built-in acos 
    
    int32_t prep(float f);//Preps for fifo
    
    void functionTimer(int t);
  private:
    //int _pin;
    //void functionTimer(int t);
};

#endif