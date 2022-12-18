/*
  RoboRun.h - Functions that are part of the run loop.
  Created by Joshua McCunn (mccunnj@gmail.com) December 12, 2022.
  Released to the public.

*/
#ifndef RoboRun_h
#define RoboRun_h

#include "Arduino.h"
#include <arduino-timer.h>

#include <Adafruit_PWMServoDriver.h>  //Needed for PCA9685s

class RoboRun
{
  public:
    RoboRun(){
    }
    //Adjust find_frontpath later for adjustable step sizes.
    void find_frontPath(float* comboX, float* comboZ);
    int FR_Angles[5]; //minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm
    
    void find_90attack(float* comboX, float* comboZ);   //Input height and table
    int atk90[3]; //knee, ankle that = 90 and have the height requirement. Third value is the x, distance away from the hipjoint.

    int legAssignment[8]; //0 = front leg(s), 1= left leg(s), 2 = rear leg(s), 3 = right leg(s)
    int hipDist2Angle[8]; //Native angle diference from desired angle.
    void orientLegs(float* hip0angle, int* hipRanges); //Once given an angle, solve movement trajectory and orders
    int numOf_leftLegs = 0;
    int numOf_rightLegs = 0;
    void hipTrajectory(int leg, int angle_desired, float* hip0angle, int* hipRanges);
    void sideOrder(); //Orders side legs
    int leftOrder[3]; //Holds the order of LEFT legs, front to back.
    int rightOrder[3];  //Holds the order of RIGHT legs, front to back.
    void phaseCycle();
     //Legs movement order:moveOrder[m]
    int moveOrder[8] = {0,3,6,1,4,7,2,5}; //Sequence of legs moves for gait
    
    void find_kaReverse(int leg, float distance, float* comboX, float* comboZ);
    int lastHeight = 101;
    int kaReverse[2];
    int lastA[8]; //last ankle positions
    int lastK[8]; //last knee positions
    int lastH[8]; //last hip position
    //Swap the lasts with below?
    int lastAngles[24] = {0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0};//This will hold the last angles from a movement [leg*3+joint]

    int heightTranslator(int height); //  Translates joystick from(100 to -100) to (50-100)
    float find_hipMovement1(int movement, int leg, int travel, float* comboX, float* comboZ);
    
    int hipCorrectorFR(int i, int assignment, float* hip0angle, int* hipRanges);//Points the front in the direction of travel. Points the rear legs the opposite.
    int heading;//Where am I headed. Angle in degrees, 0 is forward.
    
    //-----Motion and timers-----
    Timer<> timer; // create a timer with default settings
    Timer<1, micros> spt_timer; // create a timer that can hold 1 concurrent task, with microsecond resolution
    int phaseLimit = 8;  // Gait phases
    int subPhaseLimit = 50; //  # of subphases every phase.
    int subPhaseStart;  // Holds the millis() start time.
    int subPhaseTimeCount; // Count by time. In case processes take too long between ticks.
    int phaseCount = 0; //  Body phase
    //int subPhaseCount = 0;  // Outdated subphase counter
    int cycleSpeed = 100; // 0-100 speed%
    int phaseSpeed = 1000;// Speed of each phase in millis
    int phaseList[8];//current phase for leg
    
    void initTimer(int speed, int subPhases);//In %, sets cycleSpeed.
    void tick();
    bool phaseTimer();
    bool subPhaseTimer();
    static bool callbackSPT(void *p) {
      RoboRun *ptr = (RoboRun *)p;
      return ptr->subPhaseTimer();
    }
    
    static bool callbackPT(void *p) {
      RoboRun *ptr = (RoboRun *)p;
      return ptr->phaseTimer();
    }
    
    void timeCycle(int phase, int speed);
    // void locomotion1(void function(int leg, int joint, float angle));
    void locomotion1();
    uint8_t servonum = 0;
    //  -----SERVO STUFF-----
    int ServoInts[72];//Int array holding min,max,centers of servos. HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax
    Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver();//Right PCA9685
    Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x43);//Left PCA9685
    // our servo # counter
    void PWMinit();
    void moveSave(int leg, int joint, float angle);//Takes a floating angle.Move Servos using setPWM(), save last position as an angle. Takes an angle to reduce the number of maps called.
    void moveLeg(bool methodPwm, int leg, int joint, int val);
    int findJointAngle(int leg, int joint, float angle);

    
  private:

};

#endif