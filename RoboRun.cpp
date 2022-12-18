/*
  RoboRun.cpp -  Class functions for Pinky runtime.
  Created by Joshua McCunn (mccunnj@gmail.com) December 12, 2022.
  Released to the public.
*/
#include "Arduino.h"
#include "pico/multicore.h"
#include <arduino-timer.h>

#include <Adafruit_PWMServoDriver.h>  //Needed for PCA9685s
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


#include "RoboRun.h"
#include "Trig.h"

Trig trig_run;


void RoboRun::find_frontPath(float* comboX, float* comboZ){
  int minX = 200; 
  int maxX = 0;
  for(int i = 0; i < 91*91; i++){
    //Serial.println(comboZ[i]);
    if(round(comboZ[i]) == lastHeight){
      if(comboX[i] < minX){
        minX = comboX[i];
        //FR_Angles is in pinkBLE.ino
        FR_Angles[0] = i/91;//Finds the KNEE angle for closest x position.
        FR_Angles[1] = i%91;//Finds the ANKLE angle for closest xposition.
      }
      if(round(comboX[i]) > maxX){
        maxX = comboX[i];
        FR_Angles[2] =i/91;//Finds the KNEE angle for Farthest x position.
        FR_Angles[3] = i%91;//Finds the ANKLE angle for Farthest x position.
      }
    }
  }
  FR_Angles[4] = maxX - minX;//This is the size of steps for now
  if(0){         
    Serial.print("Translated height(mm from hips(without flex)):");
    Serial.println(lastHeight);
    Serial.print("minX:");
    Serial.println(minX);
    Serial.print("minXkAngle:");
    Serial.println(FR_Angles[0]);
    Serial.print("minXaAngle:");
    Serial.println(FR_Angles[1]);
    Serial.print("maxX:");
    Serial.println(maxX);
    Serial.print("maxXkAngle:");
    Serial.println(FR_Angles[2]);
    Serial.print("maxXaAngle:");
    Serial.println(FR_Angles[3]);
    Serial.print("maxTravel Front/Rear legs(mm):");
    Serial.println(FR_Angles[4]);
  }
}
//Find 90 attack is for most stable side legs
//90 degrees should put the least amount of stress on the ankle servos
void RoboRun::find_90attack(float* comboX, float* comboZ){//Finds a combo of ankle and knee joints that combine to 90 and meet the height requirement
  int bestA;
  int bestK;
  float closestH = 5;
  int bestX;
  for(int k = 0; k < 91; k++){//Knee angles
    for(int a = 0; a < 91; a++){
      if (a+k == 90 && comboZ[k*91+a] == lastHeight){
        closestH = 0;
        bestA = a;
        bestK = k;
        bestX = comboX[k*91+a];
        if(0){
          Serial.print("solved! (K_A):");
          Serial.print(bestK);
          Serial.print(" ");
          Serial.println(bestA);
        }
      }
      else if (a+k == 90){
        if(abs(lastHeight - comboZ[k*91+a]) < closestH){
          closestH = abs(lastHeight - comboZ[k*91+a]);
          bestA = a;
          bestK = k;
          bestX = comboX[k*91+a];
          if(0){
            Serial.print("Closest(K_A):");
            Serial.print(bestK);
            Serial.print(" ");
            Serial.print(bestA);
            Serial.print(" Height:");
            Serial.println(comboZ[k*91+a]);
          }
        }
      }     
    }
  }
  atk90[0] = bestK;
  atk90[1] = bestA;
  atk90[2] = bestX;
  // Serial.print("Distance from hip @ 90:");
  // Serial.println(atk90[2]);
}
void RoboRun::orientLegs(float* hip0angle, int* hipRanges){//After setting a new heading
  //Reset the side count
  numOf_leftLegs = 0;
  numOf_rightLegs = 0;
  for(int i=0; i<8; i++){
    // Serial.print("hip0s @ orientLegs: ");
    // Serial.println(hip0angle[i]);
    hipTrajectory(i, heading, hip0angle, hipRanges);//Figure out leg orientation for the angle of attack
  }
  sideOrder();// Finds the order of side legs from front to back.
  phaseCycle();//
}
void RoboRun::hipTrajectory(int leg, int angle_desired, float* hip0Angle, int* hipRanges){
  // Serial.print("Leg:");
  // Serial.print(leg);
  // Serial.print(" AngleDesired:");
  // Serial.print(angle_desired);
  int legHipRangeMin = hip0Angle[leg]-hipRanges[leg]/2;
  int legHipRangeMax = hip0Angle[leg]+hipRanges[leg]/2;
  int legHipRangeMin2 = legHipRangeMin;
  int legHipRangeMax2 = legHipRangeMax;

  //Legs 3&4 need two ranges
  if(legHipRangeMin < 0){
    legHipRangeMax2 = 359;
    legHipRangeMin2 = 360 + legHipRangeMin;
    legHipRangeMin = 0;
  }
  else if(legHipRangeMax > 359){
    legHipRangeMax2 = legHipRangeMax - 360;
    legHipRangeMin2 = 0;
    legHipRangeMax = 359;
  }
  // Serial.print(" LegMin:");
  // Serial.print(legHipRangeMin);
  // Serial.print(" LegMax:");
  // Serial.print(legHipRangeMax);
  // Serial.print(" LegMin2:");
  // Serial.print(legHipRangeMin2);
  // Serial.print(" LegMax2:");
  // Serial.println(legHipRangeMax2);
  //Is the angle desired within the legs range?
  if ( ( (angle_desired >= legHipRangeMin) && (angle_desired <= legHipRangeMax) ) || ( (angle_desired >= legHipRangeMin2) && (angle_desired <= legHipRangeMax2) )){
    //Serial.println("Front Legs, Set Hip servos to this angle");//Pulling legs
    legAssignment[leg] = 0;
  }
  //Find rear legs
  else{
    int angle_rear = angle_desired + 180;
    if (angle_rear > 359){
      angle_rear -= 360;
    }
    if ( ( (angle_rear >= legHipRangeMin) && (angle_rear <= legHipRangeMax) ) || ( (angle_rear >= legHipRangeMin2) && (angle_rear <= legHipRangeMax2) )){
      //Serial.println("Rear Legs, Set Hip servos to this angle");//Pushing legs
      legAssignment[leg] = 2;
    }
    else{//Set servos to the closest angle
      float dist_minside;
      float dist_maxside;
      //This Stuff is needed for finding the closest angle to the desired angle for dragging legs
      if(angle_desired < hip0Angle[leg]){
        dist_minside  = hip0Angle[leg]-angle_desired;//Verified@ 0,90,180,270
        dist_maxside = angle_desired + 360 - hip0Angle[leg];//Verified@ 0,90,180,270
      }
      else{
        dist_minside  = hip0Angle[leg]+ 360 - angle_desired;//Verified@ 0,90,180,270
        dist_maxside = angle_desired - hip0Angle[leg];//Verified@ 0,90,180,270
      }
      // Serial.print("HipOrigin:");
      // Serial.print(hip0Angle[leg]);
      // Serial.print(" Range on min side:");
      // Serial.print(dist_minside);
      // Serial.print(" Range on max side:");
      // Serial.println(dist_maxside);
      if(dist_minside < dist_maxside){//This is a Right leg
        //Serial.println("Move to min");
        legAssignment[leg] = 3;
        hipDist2Angle[leg] = dist_minside;
        numOf_rightLegs++;
      }
      else{//This is a left Leg
        //Serial.println("Move to max");
        legAssignment[leg] = 1;
        hipDist2Angle[leg] = dist_maxside;
        numOf_leftLegs++;
      }
    }
  }
  // Serial.print(" Assignment:");
  // Serial.println(legAssignment[leg]);
}
void RoboRun::sideOrder(){
  //Order the side legs by Angular Distance from the desired angle(front to back)
  int distChecker1 = 180;
  int distChecker2 = 180;
  int distCheckerR1 = 180;
  int distCheckerR2 = 180;
  for(int i = 0; i < 8; i++){
    // Serial.println(legAssignment[i]);
    if (legAssignment[i] == 1){//This is a leftLeg
      // Serial.println("Comparing Lefts");
      if (hipDist2Angle[i] < distChecker1){
        leftOrder[2] = leftOrder[1];
        leftOrder[1] = leftOrder[0];
        leftOrder[0] = i;
        distChecker1 = hipDist2Angle[i];
      }
      else if(hipDist2Angle[i] < distChecker2){
        leftOrder[2] = leftOrder[1];
        leftOrder[1] = i;
        distChecker2 = hipDist2Angle[i];
      }
      else{
        leftOrder[2] = i;
      }
    }
    if ( legAssignment[i] == 3){//This is a RIGHT leg
      // Serial.println("Comparing Rights");
      if (hipDist2Angle[i] < distCheckerR1){
        rightOrder[2] = rightOrder[1];
        rightOrder[1] = rightOrder[0];
        rightOrder[0] = i;
        distCheckerR1 = hipDist2Angle[i];
      }
      else if(hipDist2Angle[i] < distCheckerR2){
        rightOrder[2] = rightOrder[1];
        rightOrder[1] = i;
        distCheckerR2 = hipDist2Angle[i];
      }
      else{
        rightOrder[2] = i;
      }
    }
  }
  //If the angular difference is < 90 they are front side legs, if angular dist > 90, they are rear side legs.
  // for(int i= 0; i < numOf_leftLegs; i++){
  //   Serial.print("LeftLegs Front->Back:");
  //   Serial.print(leftOrder[i]);
  //   Serial.print(" OriginAngle:");
  //   Serial.print(hip0Angle[leftOrder[i]]);
  //   if(i == 0){//This is the most front left side leg
      
  //     Serial.print(" LeftSide_Front -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   else if(i == numOf_leftLegs-1){// This is the most rear left side leg
    
  //     Serial.print(" LeftSideRear -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   else if( i == 1 && numOf_leftLegs == 3){// This is the middle left side leg
  //     Serial.print(" LeftSide_Mid -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   Serial.println(" ");
  // }
  // for(int i= 0; i< numOf_rightLegs; i++){
  //   Serial.print("RightLegs Front->Back:");
  //   Serial.print(rightOrder[i]);
  //   Serial.print(" OriginAngle:");
  //   Serial.print(hip0Angle[rightOrder[i]]);
  //   if(i == 0){//This is the most front right side leg
  //     Serial.print(" RightSide_Front -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   else if(i == numOf_rightLegs-1){// This is the most rear left side leg
  //     Serial.print(" RightSide_Rear -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   else if( i == 1 && numOf_rightLegs == 3){// This is the middle left side leg
  //     Serial.print(" RightSide_Mid -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   Serial.println(" ");
  // }
}
void RoboRun::phaseCycle()//Legs movement order:moveOrder[m]
{
  for(int i = 0; i < 8; i++){
    if(legAssignment[i] == 0){//Start at the lowest front leg
      if(i == 0 && legAssignment[7] == 0){
        i = 7;
      }
      // Serial.print("FirstFrontLeg:");
      // Serial.println(i);
      int legPush;
      for (int m = 0; m<8; m++){
        legPush = i + m *3;
        while(legPush > 7){
          legPush -= 8;
        }
        moveOrder[m] = legPush;
      }
      i=8;//exits for loop
    }
  }
  // for(int i = 0; i < 8; i++){
  //   Serial.print("LegOrder:");
  //   Serial.println(moveOrder[i]);
  // }
}
void RoboRun::find_kaReverse(int leg, float distance, float* comboX, float* comboZ){//Input how long the end-effector(foot) should be from the hip origin, output the best angle combo to achieve this.
  ////Search through the combined x/z tables(Old as of 12/14/22):
  // float closestX = 25;
  // float closestZ = 25;
  // int bestA = 0;
  // int bestK = 45;
  // for(int k = 0; k <= 90; k++){
  //   for(int a = 0; a <= 90; a++){
  //     if (abs(LastHeight - comboZ[k*91+a]) < closestZ && abs(distance - comboX[k*91+a]) < closestX){
  //       closestX = abs(distance - comboX[k*91+a]);
  //       closestZ = abs(LastHeight - comboZ[k*91+a]);
  //       bestA = a;
  //       bestK = k;
  //     }
  //   }
  // }
  // kaReverse[0] = bestK;
  // kaReverse[1] = bestA;
  
  // Serial.print("find_kaReverse:");
  // Serial.print(" bestK:");
  // Serial.print(bestK);
  // Serial.print(" bestA:");
  // Serial.println(bestA);
  
  // Serial.print("Requested Height: ");
  // Serial.print(LastHeight);
  // Serial.print(" closestHeight:");
  // Serial.println(comboZ[kaReverse[0]*91+kaReverse[1]]);
  // Serial.print("Requested Distance: ");
  // Serial.print(distance);
  // Serial.print(" closestDistance");
  // Serial.println(comboX[kaReverse[0]*91+kaReverse[1]]);
  //------------------------------------------------------
  // //Maintaining 90(ish) degrees with the ground
  // float closestX90 = 25;
  // float closestZ90 = 25;
  // int bestA90 = 0;
  // int bestK90 = 45;
  // for(int k = 0; k <= 90; k++){
  //   for(int a = 0; a <= 90; a++){
  //     // Search only combos where A+k = 90 +-5?
  //     if ((k+a <= 92 && k+a >= 88) && abs(LastHeight - comboZ[k*91+a]) < closestZ90 && abs(distance - comboX[k*91+a]) < closestX90){
  //       //Serial.println(k+a);
  //       closestX90 = abs(distance - comboX[k*91+a]);
  //       closestZ90 = abs(LastHeight - comboZ[k*91+a]);
  //       bestA90 = a;
  //       bestK90 = k;
  //     }
  //   }
  // }
  // Serial.print(" bestK90:");
  // Serial.print(bestK90);
  // Serial.print(" bestA90:");
  // Serial.println(bestA90);
  
  // Serial.print("Requested Height: ");
  // Serial.print(LastHeight);
  // Serial.print(" closestHeight:");
  // Serial.print(comboZ[kaReverse[0]*91+kaReverse[1]]);
  // Serial.print(" closestHeight90:");
  // Serial.println(comboZ[bestK90*91+bestA90]);
 
  // Serial.print("Requested Distance: ");
  // Serial.print(distance);
  // Serial.print(" closestDistance:");
  // Serial.print(comboX[kaReverse[0]*91+kaReverse[1]]);
  // Serial.print(" closestDistance90:");
  // Serial.println(comboX[bestK90*91+bestA90]);
  
  // Serial.print("Height Deviation: ");
  // Serial.println(abs(LastHeight - comboZ[bestK90*91+bestA90]));
  // Serial.print("Dist Deviation: ");
  // Serial.println(abs(distance - comboX[bestK90*91+bestA90]));
  //----------------------------------------------------------
  
  // Only allow moves within a certain distance of the last known position
  float closestXlast = 50;
  float closestZlast = 50;
  int bestAlast = 42;
  int bestKlast = 42;  
  if (lastA[leg] < 0 && lastK[leg] < 0){
    //Serial.println("SettingAK!");//First run sets initial best KA.
    for(int k = 0; k <= 90; k++){
      for(int a = 0; a <= 90; a++){
        if (abs(lastHeight - comboZ[k*91+a]) < closestZlast && abs(distance - comboX[k*91+a]) < closestXlast){
          closestXlast = abs(distance - comboX[k*91+a]);
          closestZlast = abs(lastHeight - comboZ[k*91+a]);
          bestAlast = a;
          bestKlast = k;
        }
      }
    }
    lastA[leg] = bestAlast;
    lastK[leg] = bestKlast;
  }
  else{
    for(int k = lastK[leg] - 15; k <= lastK[leg] + 15; k++){
      if(k < 0){
        k = 0;
      }
      else if(k > 90){
        k = 90;
      }
      for(int a = lastA[leg] - 15; a <= lastA[leg] + 15; a++){
        if(a < 0){
          a = 0;
        }
        else if(a > 90){
          a = 90;
        }
        if (abs(lastHeight - comboZ[k*91+a]) < closestZlast && abs(distance - comboX[k*91+a]) < closestXlast){
          // Serial.println("FoundBest!");  //  If the best is up or down, take a step in that direction.
          closestXlast = abs(distance - comboX[k*91+a]);
          closestZlast = abs(lastHeight - comboZ[k*91+a]);
          bestAlast = a;
          bestKlast = k;
        }
      }
    }
    if(bestAlast > lastA[leg]){
      lastA[leg]++;
    }
    else if(bestAlast < lastA[leg]){
      lastA[leg]--;
    }
    if(bestKlast > lastK[leg]){
      lastK[leg]++;
    }
    else if(bestKlast < lastK[leg]){
      lastK[leg]--;
    }
  }
  // Serial.print(" bestKlast:");
  // Serial.print(lastK[leg]);
  // Serial.print(" bestAlast:");
  // Serial.println(lastA[leg]);
}
float RoboRun::find_hipMovement1(int movement, int leg, int travel, float* comboX, float* comboZ){ // find all hip positions to walk in a line
  //RightLegs
  float legLengthb;
  float angleA;
  //float angleC;
  if (movement == 1){
    legLengthb = sqrt(pow(travel,2) + pow(atk90[2],2) - (2* travel * atk90[2]) * cos(trig_run.a2r(180 - hipDist2Angle[leg] )));
    angleA = trig_run.r2a(trig_run.otheracos( (pow(legLengthb,2) + pow(atk90[2],2) - pow(travel,2)) / (2 * legLengthb * atk90[2])));
    //angleC = r2a(otheracos( (pow(legLengthb,2) + pow(travel,2) - pow(atk90[2],2)) / (2 * legLengthb * travel)));
    // Serial.print("Leg:");
    // Serial.print(leg);
    // Serial.print(" angle_A:");
    // Serial.print(angleA);
    // Serial.print(" angle_B:");
    // Serial.print(180 - hipDist2Angle[leg]);
    // // Serial.print(" angle_C:");
    // // Serial.print(angleC);
    // Serial.print(" Required legX:");
    // Serial.println(legLengthb);
    find_kaReverse(leg, legLengthb, comboX, comboZ);//Should this be called independently?
    return angleA;
  }
  else if (movement == 2){
    legLengthb = sqrt(pow(travel,2) + pow(atk90[2],2) - (2* travel * atk90[2]) * cos(trig_run.a2r( hipDist2Angle[leg] )));
    angleA = trig_run.r2a(trig_run.otheracos( (pow(legLengthb,2) + pow(atk90[2],2) - pow(travel,2)) / (2 * legLengthb * atk90[2])));
    //angleC = r2a(otheracos( (pow(legLengthb,2) + pow(travel,2) - pow(atk90[2],2)) / (2 * legLengthb * travel)));
    // Serial.print("Leg:");
    // Serial.print(leg);
    // Serial.print(" angle_A:");
    // Serial.print(angleA);
    // Serial.print(" angle_B:");
    // Serial.print(180 - hipDist2Angle[leg]);
    // // Serial.print(" angle_C:");
    // // Serial.print(angleC);
    // Serial.print(" Required legX:");
    // Serial.println(legLengthb);
    find_kaReverse(leg, legLengthb, comboX, comboZ);//Should this be called independently?
    return angleA;
  }
}
int RoboRun::heightTranslator(int height){
  return map(height, -100, 100, 50, 100);// Translates from height 5cm to 10cm
}
int RoboRun::hipCorrectorFR(int i, int assignment, float* hip0Angle, int* hipRanges){//Points the front in the direction of travel. Points the rear legs the opposite.
  // int i = leg, assignment : 0 = Front, 2 = Rear.
  int hipCorrection;
  int angle = heading;
  if(assignment == 2){
    angle = heading+180;
    if (angle > 359)
      angle -= 360;
  }
  if(abs(angle-hip0Angle[i]) < hipRanges[i]/2 ){
    hipCorrection = angle - hip0Angle[i];
  }
  else if(abs(angle - 360 - hip0Angle[i]) < hipRanges[i] / 2){//Mostly for leg 4
    hipCorrection = angle-360-hip0Angle[i];
  }
  else if(abs(angle+360-hip0Angle[i])<hipRanges[i]/2){//Mostly for leg 3
    hipCorrection = angle + 360 - hip0Angle[i];
  }
  return hipCorrection;
}

//-----Movement----
// void RoboRun::locomotion1(void my_test_fn(int leg, int joint, float angle)){
void RoboRun::locomotion1(){ 
  for(int i = 0; i < 8; i++){//Sort by move order
    int phase14 = phaseList[i];//Reduce phases from 8 to 4. Phases: 0 = UP, 1 =
    if(phase14 > 3){
      phase14 -= 4;
    }
    // Serial.print("Leg:");
    // Serial.print(moveOrder[i]);
    // Serial.print(" Phase:");
    // Serial.print(phase14);
    // Serial.print(" Assignment:");
    // Serial.println(legAssignment[moveOrder[i]]);
    switch(legAssignment[moveOrder[i]]){//0 = front leg(s), 1= left leg(s), 2 = rear leg(s), 3 = right leg(s)
      //FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
      case 0: //  Front Legs
        switch(phase14){
          case 0: //  Lift, Move servos to 0 degrees
            Serial.print("Leg:");
            Serial.print(moveOrder[i]);
            Serial.print(" @ subPhase:");
            Serial.print(subPhaseTimeCount);
            Serial.print(" -> ");
            Serial.println(map(subPhaseTimeCount, 1, subPhaseLimit, FR_Angles[0], 0));//Report on knee movement.
            //Lift Shin, Knee to 0 degrees.
            moveSave(moveOrder[i], 1, map(subPhaseTimeCount, 1, subPhaseLimit, FR_Angles[0], 0));
            //moveLeg(1, moveOrder[i], 1, findJointAngle(moveOrder[i], 1, map(subPhaseTimeCount, 1, subPhaseLimit, FR_Angles[0], 0))); 
            if(subPhaseTimeCount < subPhaseLimit/2){//0
              moveLeg(1, moveOrder[i], 2, findJointAngle(moveOrder[i], 2, map(subPhaseTimeCount, 1, subPhaseLimit/2, 0, 90)));//Tuck Foot
            }
            else{// 0.5
              //moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, 90, 0)));//Streighten Foot
              //moveLeg(1, i, 0, findJointAngle(i, 0, hipCorrectorFR(i, legAssignment[i]) )); //Align hip
            }
            break;
          case 1: //  Contact Ground
            break;
          case 2: //  Move 1
            break;
          case 3: //  Move 1
            break;
        }
        break;
      case 1: //  Left Legs
        switch(phase14){
          case 0: //  Lift, Move servos to 0 degrees
            break;
          case 1: //  Contact Ground
            break;
          case 2: //  Move 1
            break;
          case 3: //  Move 1
            break;
        }
        break;
      case 2: // Rear Legs
        switch(phase14){
          case 0: //  Lift, Move servos to 0 degrees
            break;
          case 1: //  Contact Ground
            break;
          case 2: //  Move 1
            break;
          case 3: //  Move 1
            break;
        }
        break;  
      case 3: //  Right Legs
        switch(phase14){
          case 0: //  Lift, Move servos to 0 degrees
            break;
          case 1: //  Contact Ground
            break;
          case 2: //  Move 1
            break;
          case 3: //  Move 1
            break;
        }
        break;
    }
  }
}
//  -----TIMER-----
void RoboRun::initTimer(int speed, int subPhases){//Select which timer to init
  cycleSpeed = speed;
  subPhaseLimit = subPhases;
  // Serial.print("Init at phaseSpeed:");
  // Serial.println((int)round(phaseSpeed*(1/((float)cycleSpeed/100))));
  //Create a phase timer
  timer.every((int)round(phaseSpeed*(1/((float)cycleSpeed/100))), callbackPT, this);
}
void RoboRun::tick(){
  timer.tick();//Phase Timer
  spt_timer.tick();//Subphase timer
}
bool RoboRun::phaseTimer() {
  // Serial.print("phaseTimer: ");
  // Serial.print(phaseCount);
  // Serial.print("/");
  // Serial.println(phaseLimit - 1);
  timeCycle(phaseCount, (int)round(phaseSpeed*(1/((float)cycleSpeed/100))));
  if(phaseCount == phaseLimit - 1){
    phaseCount == 0;
    return 0;
  }
  else{ // remove this task after limit reached
    return phaseCount++ <= phaseLimit - 1;
  }
}
bool RoboRun::subPhaseTimer() {
  subPhaseTimeCount = 1 + (millis() - subPhaseStart) / ((int)round(phaseSpeed*(1/((float)cycleSpeed/100)))/subPhaseLimit);
  Serial.print("subPhaseTimeCount: ");
  Serial.print(subPhaseTimeCount);
  Serial.print("/");
  Serial.println(subPhaseLimit);
  
  locomotion1();
  if (subPhaseTimeCount < subPhaseLimit){
    return 1;
  }
  else
  {
    return 0;// remove this task after limit reached
  }
}
void RoboRun::timeCycle(int phase, int speed){//Updates the phase list
  Serial.print("Phase:");
  Serial.print(phase);
  Serial.print(" @ cycleSpeed:");
  Serial.print(cycleSpeed);
  Serial.print(" phaseSpeed:");
  Serial.println(speed);
  phaseList[0] = 0; //{0,2,1,3,0,2,1,3}
  phaseList[1] = 2;
  phaseList[2] = 1;
  phaseList[3] = 3;
  phaseList[4] = 0;
  phaseList[5] = 2;
  phaseList[6] = 1;
  phaseList[7] = 3;
  for(int i = 0; i < 8; i++){
    phaseList[i] = phase + phaseList[i];
    if(phaseList[i] > 7){
      phaseList[i] -= 8;
    }
    // Serial.print("Leg:");
    // Serial.print(moveOrder[i]);
    // Serial.print(" Phase:");
    // Serial.println(phaseList[i]);
  }
  spt_timer.cancel();//Cancel the current subphase timer.
  subPhaseTimeCount = 1;
  subPhaseStart = millis();
  spt_timer.every((speed/subPhaseLimit)*1000, callbackSPT, this);
  locomotion1();
}
//-----SERVOS-----
void RoboRun::PWMinit(){
      pwmR.begin();
      pwmR.setOscillatorFrequency(27000000);
      pwmR.setPWMFreq(SERVO_FREQ);  
      pwmL.begin();
      pwmL.setOscillatorFrequency(27000000);
      pwmL.setPWMFreq(SERVO_FREQ);
}
void RoboRun::moveLeg(bool methodPwm, int leg, int joint, int val){//Move Servos using setPWM() Used for testing...
  // Serial.print("Leg:");
  // Serial.print(leg);
  // Serial.print(" Method:");
  // Serial.print(methodPwm);
  if(leg < 4){//RightBoard
    servonum = leg * 4 + joint;
    // Serial.print(" Moving pwmR, channel:");
    // Serial.print(servonum);
    // Serial.print(" Value:");
    // Serial.println(val);
    if(methodPwm){
      pwmR.setPWM(servonum, 0, val);
    }
    else
      pwmR.writeMicroseconds(servonum, val);
  }
  else{//LeftBoard
    servonum = (leg - 4) * 4 + joint;
    // Serial.print(" Moving pwmL, channel:");
    // Serial.print(servonum);
    // Serial.print(" Value:");
    // Serial.println(val);
    if(methodPwm){
      pwmL.setPWM(servonum, 0, val);
    }
    else
      pwmL.writeMicroseconds(servonum, val);
  }
}
void RoboRun::moveSave(int leg, int joint, float angle){//Takes a floating angle.Move Servos using setPWM(), save last position as an angle. Takes an angle to reduce the number of maps called.
  int val = findJointAngle(leg, joint, angle);
  // Serial.print("Leg:");
  // Serial.print(leg);
  // Serial.print(" Joint:");
  // Serial.print(joint);
  // Serial.print(" Val:");
  // Serial.println(val);

  //Serial.println("got This Far0");
  
  if(leg < 4){//RightBoard
    pwmR.setPWM(leg * 4 + joint, 0, val);
  }
  else{//LeftBoard
    pwmL.setPWM((leg - 4) * 4 + joint, 0, val);
  }
  if(joint == 2 && (angle == 90 || angle == 0)){
    moveLeg(1, leg, joint, 4096);
    //Serial.println("Endstop: 0/90");
  }
  lastAngles[leg*3+joint] = angle;

}
int RoboRun::findJointAngle(int leg, int joint, float angle){//input leg, joint, desired angle. output servo PWM signal for required angle.
  //HipMin 0, HipCenter 1, HipMax 2, KneeMin 3, KneeCenter 4, KneeMax 5, AnkleMin 6, AnkleCenter 7, AnkleMax 8  
  int mappedJoint = 999; 
  if(joint == 0 && (angle >= -45 && angle <= 45)){//Positive Values move clockwise
    if(leg != 3 && leg !=4){ //Input ranges from +45 to -45
      mappedJoint = map(angle, -45, 45, ServoInts[leg*9+0], ServoInts[leg*9+2]);
    }
    else if(angle >= -35 && angle <= 35){ // Input ranges from +35 to -35. These Servos(3&4) also run backward....
      mappedJoint = map(angle, -35, 35, ServoInts[leg*9+2], ServoInts[leg*9+0]);
    }
  }
  else if( joint == 1 && (angle >= -10 && angle <= 100)){//KneeAngles 
    mappedJoint = map(angle, 0, 90, ServoInts[leg*9+5], ServoInts[leg*9+3]);
  }
  else if( joint == 2 && (angle >= 0 && angle <=90)){//AnkleAngles All ANKLES from MAX to Min(Down)
    mappedJoint = map(angle, 0,90,ServoInts[leg*9+6],ServoInts[leg*9+8]);
  }
  if(mappedJoint != 999){
    // Serial.print(" Mapped:");
    // Serial.println(mappedJoint);
    return mappedJoint;
  }
}