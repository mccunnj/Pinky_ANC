/*
  OctoConfi.cpp -  Class functions for configuring Pinky.
  Created by Joshua McCunn (mccunnj@gmail.com) December 12, 2022.
  Released to the public.
*/

#include "Arduino.h"
#include <Wire.h> //For I2C

#include "OctoConfig.h"
#include "Trig.h"


Octo::Octo(int noLegs, int thigh, int shin, int foot)
{
  _noLegs = noLegs;
  _thighLength = thigh;//  In millimeters
  _shinLength = shin;  // mm
  _footLength = foot;  // mm
  
}
void Octo::setHips(){//Sets the hip0Angle and hipRanges
  float angleOrigin = 0;
  int actualLeg = 0;
  for(int l = 0; l < this->_noLegs; l++){//Setting up leg angles and ranges of motion for hips
    angleOrigin = 22.5 + 45 * l;
    if(angleOrigin > 359){
      angleOrigin = 360 - 22.5 - 45 *l;
    }
    actualLeg = 7-4-l;
    if(actualLeg <0){
      actualLeg = 8+ actualLeg;
    }
    hip0Angles[actualLeg] = angleOrigin;
    if(actualLeg != 3 && actualLeg != 4){ //Front hip servos have lower ranges
      hipRanges[actualLeg] = 90;
    }
    else{
      hipRanges[actualLeg] = 70;
    }
  }
}
void Octo::generate_HeightTable(){
  Trig heightTrig;
  for(int kAngle = 0; kAngle <= 90; kAngle++){
    int TA = 90 - kAngle;//Thigh/ankle angle of missing angle for knee triangle
    float shinX = this->_shinLength*sin(heightTrig.a2r(TA))/sin(PI/2);
    float shinZ = this->_shinLength*sin(heightTrig.a2r(kAngle))/sin(PI/2);
    // Serial.print("kAngle:");
    // Serial.println(kAngle);
    // Serial.print(" shinX:");
    // Serial.println(shinX);
    // Serial.print(" ShinZ:");
    // Serial.println(shinZ);
    for(int aAngle = 0; aAngle <=90; aAngle++){
      // Serial.print("aAngle:");
      // Serial.println(aAngle);
      int combinedAngle = kAngle+aAngle;
      // Serial.print("CombinedAngle:");
      // Serial.println(combinedAngle);
      float compound_footX = 0;
      float compound_footZ = 0;
      int internalSF = 0;
      int internalFG = 0;
      if(combinedAngle < 90){//This adds to overall x
        internalSF = combinedAngle;
        internalFG = 90 - internalSF;
        compound_footX = this->_footLength*sin(heightTrig.a2r(internalFG))/sin(PI/2);
        compound_footZ = this->_footLength*sin(heightTrig.a2r(internalSF))/sin(PI/2);
      }
      else if(combinedAngle == 90){//not right
        compound_footX = 0;
        compound_footZ = this->_footLength;
      }
      else if(combinedAngle > 90){
        internalSF = 180 - combinedAngle;
        internalFG = 90 - internalSF;
        compound_footX = -this->_footLength*sin(heightTrig.a2r(internalFG))/sin(PI/2);
        compound_footZ = this->_footLength*sin(heightTrig.a2r(internalSF))/sin(PI/2);
      }
      if(compound_footZ < 12){  //If the robot is sitting on its ankle joint
        compound_footZ = 12;  //Ankle joint has a radius of 12 mm
      }
      // Serial.print("compoundX");
      // Serial.println(compound_footX);
      // Serial.print("compoundZ");
      // Serial.println(compound_footZ);
      // Serial.print("StorageSpot:");
      // Serial.println(kAngle*91+aAngle);
      // Serial.print("Find it@ kangle:");
      // Serial.print((kAngle*91+aAngle)/91);
      // Serial.print(" aNgle:");
      // Serial.println((kAngle*91+aAngle)%91);
      combinedX[kAngle*91+aAngle] = this->_thighLength+shinX+compound_footX;
      combinedZ[kAngle*91+aAngle] = shinZ+compound_footZ;
    }
  }
}
void Octo::pwmScan(){//Gives a list of all available I2C devices
  byte error, address;
  int nDevices;
  Serial.println(F("Scanning..."));
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("done\n"));
}
