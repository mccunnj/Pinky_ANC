/*
  OctoConfig.h - Class for configuring my octopod robot.
  Created by Joshua McCunn (mccunnj@gmail.com) December 12, 2022.
  Released to the public.

*/
#ifndef OctoConfig_h
#define OctoConfig_h

#include "Arduino.h"
#include <LittleFS_Mbed_RP2040.h>


class Octo
{
  public:
    Octo(int noLegs, int thigh, int shin, int foot);  //Input the number of legs. Thigh, shin, and foot lengths in mm.
 
    float hip0Angles[8];  //Holds angle each leg is pointing when the hip is centered
    int hipRanges[8];  //Holds the ranges of hip servos
    void setHips(); //Sets hip angles and ranges
    
    void generate_HeightTable();  //Generates a table of joint angles and Z heights
    int _thighLength;//  In millimeters
    int _shinLength;  // mm
    int _footLength;  // mm
    float combinedX[91*91]; //Horizontal distances from hip joint
    float combinedZ[91*91]; //Vertical distances from hip joint
    
    void pwmScan();//Scans the I2c bus
    
  private:
    int _noLegs;
};

#endif