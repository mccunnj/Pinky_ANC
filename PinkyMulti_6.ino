//#include <stdio.h>
//#include <mbed.h>
#include "pico/multicore.h"
//#include "pico/util/queue.h"
#include "hardware/irq.h"
#include <Wire.h> //For I2C
#include <Adafruit_PWMServoDriver.h>  //Needed for PCA9685s

//#include "WiFiNINA.h"
//#include <arduino-timer.h>  //Timers

//----Custom libraries----
#include "Trig.h"
#include "OctoConfig.h"
#include "RoboRun.h"

//Initialize custom classes
Trig trig_g;  //Global so that both cores have access. Be careful
Octo pinky(8, 75, 75, 50);  //Init pinky's config class. Legs, thigh, shin, foot length.
RoboRun pinkyRun;

//  -----LittleFS stuff----- Couldn't get this to work in the OctoConfig class.
#define LFS_MBED_RP2040_VERSION_MIN_TARGET      "LittleFS_Mbed_RP2040 v1.1.0"
#define LFS_MBED_RP2040_VERSION_MIN             1001000
#define _LFS_LOGLEVEL_          1
#define RP2040_FS_SIZE_KB       64
#define FORCE_REFORMAT          false
#include <LittleFS_Mbed_RP2040.h>
LittleFS_MBED *myFS;
char ServoFile[24] = MBED_LITTLEFS_FILE_PREFIX "/servoData.txt";//Servo data file.
//int ServoInts[72];//Int array holding min,max,centers of servos. HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax

void readCharsFromFile(const char * path){ //LittleFS reader. Modify to send to global array.
  char ServoChars[216];//Char array holding servo data for r/w.

  myFS = new LittleFS_MBED();
  if (!myFS->init()) 
  {
    Serial.println("LITTLEFS Mount Failed");
    return;
  }
  Serial.print("readCharsFromFile: "); Serial.print(path);

  FILE *file = fopen(path, "r");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }

  char c;
  int i = 0;
  while (true) 
  {
    c = fgetc(file);
    
    if ( feof(file) ) 
    { 
      break;
    }
    else{   
      Serial.print(c);
      //write to global ServoChar array.
      ServoChars[i++] = c;
    }
  }
  Serial.println(" ");

  fclose(file);
  if(myFS->unmount()){
    Serial.println( "\nReadComplete" );
  }
  for (int i = 0; i < 72; i++){
    pinkyRun.ServoInts[i] = ((ServoChars[i*3] - '0') * 100) + ((ServoChars[i*3+1] - '0') * 10) + (ServoChars[i * 3 + 2] - '0');
    // Serial.print(ServoInts[i]);
    // Serial.print(", ");
  }
}
void createRandomArray(char * servoFile){//Populates a file with an array of random chars for LittleFS to store.
  int values[72];
  char output[216];
  for(int i = 0; i < (3*3*8); i++){//Generate 72 random numbers in place of low, mid, and high for 24 servos.
    values[i] = random(50,700);
  }
  for(int i = 0; i < (3*3*8); i++){// Breaks the random numbers into an array of chars.
    Serial.print(i);
    Serial.print(": ");
    Serial.println(values[i]);
    //Convert array of ints into array of chars
    output[i*3] = ((values[i]/100) % 10) + '0';
    output[i*3+1] = ((values[i]/10) % 10) + '0';
    output[i*3+2] = (values[i] % 10) + '0';
  }  
  writeFile(servoFile, output, sizeof(output));
}
void saveServosArray(){
  char ServoChars[216];//Char array holding servo data for r/w.
  for(int i = 0; i < (3*3*8); i++){// Breaks the random numbers into an array of chars.
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(ServoInts[i]);
    //Convert array of ints into array of chars
    ServoChars[i*3] = ((pinkyRun.ServoInts[i]/100) % 10) + '0';
    ServoChars[i*3+1] = ((pinkyRun.ServoInts[i]/10) % 10) + '0';
    ServoChars[i*3+2] = (pinkyRun.ServoInts[i] % 10) + '0';
  }  
  writeFile(ServoFile, ServoChars, sizeof(ServoChars));
  //readCharsFromFile(ServoFile);
}
void writeFile(const char * path, const char * message, size_t messageSize){ //LittleFS writer.
  myFS = new LittleFS_MBED();
  if (!myFS->init()) 
  {
    Serial.println("LITTLEFS Mount Failed");
    return;
  }
  Serial.print("Writing file: "); Serial.print(path);

  FILE *file = fopen(path, "w");
  if (file){
    Serial.println(" => Open OK");
  }
  else{
    Serial.println(" => Open Failed");
    return;
  } 
  if (fwrite((uint8_t *) message, 1, messageSize, file)){
    Serial.println("* Writing OK");
  } 
  else{
    Serial.println("* Writing failed");
  }  
  fclose(file);
  if(myFS->unmount()){
    Serial.println( "WriteComplete" );
  }
}

// //  -----SERVO STUFF-----



//  -----MULTIcore-----
// this method will handle interrupt; execute stuff while the interrupt is set
void core1_irq_handler() {
  // check if there is data in FIFO
  while(multicore_fifo_rvalid()) {
      // pop the data from FIFO stack
    uint16_t raw = multicore_fifo_pop_blocking();
    uint16_t value1 = multicore_fifo_pop_blocking();
    switch(raw){
      case 0:
        digitalWrite(LED_BUILTIN, 0);
        break;
      case 1:
        digitalWrite(LED_BUILTIN, 1);
        //multicore_fifo_push_blocking(value1);
        break;
      case 2://Run this every time the height needs to change
        pinkyRun.lastHeight = (int)value1;
        multicore_fifo_push_blocking(pinkyRun.FR_Angles[4]);
        pinkyRun.find_frontPath(pinky.combinedX, pinky.combinedZ);
        pinkyRun.find_90attack(pinky.combinedX, pinky.combinedZ);
        multicore_fifo_push_blocking(pinkyRun.FR_Angles[4]);
        break;
      case 3: //Run this everytime the angle changes.
        pinkyRun.heading = (int)value1;//Set the class member for angle
        pinkyRun.orientLegs(pinky.hip0Angles, pinky.hipRanges);
        multicore_fifo_push_blocking(1);
        break;
      case 4:
        
        break;
      case 5:
        
        break;
      case 6:
        
        break;
      case 7:
        
        break;
    }
  }
  //at the end of handling the interrupt, we need to clear it
  multicore_fifo_clear_irq();
}
// this method runs on core1. In the infinite loop, you can do essentially whatever you want. We're just using the built-in loop which does nothing. Literally. It's nop instruction afiak.
void core1_entry() {
    // this will run only at the beggining of execution. We clear the interrupt flag, if it got set by a chance
    multicore_fifo_clear_irq();
    // set the SIO_IRQ_PROC1 (FIFO register set interrupt) ownership to only one core. Opposite to irq_set_shared_handler() function
    // We pass it the name of function that shall be executed when interrupt occurs
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_irq_handler);
    // enable interrupt
    irq_set_enabled(SIO_IRQ_PROC1, true);

    // do whatever
    while(1) {
        tight_loop_contents();
    }
}
void changeHeight(int height){  //  Launches HEIGHT calculations on core1
  multicore_fifo_push_blocking(2);
  multicore_fifo_push_blocking(height);
  Serial.print("Old dist:");
  Serial.println(multicore_fifo_pop_blocking());
  Serial.print("New dist:");
  Serial.println(multicore_fifo_pop_blocking());
}
void changeAngle(int angle){//  Launches ANGLE calculations on core1
  for(int i = 0; i < 8; i++){
    Serial.print(pinkyRun.moveOrder[i]);
  }
  Serial.println("");
  int start = micros();
  multicore_fifo_push_blocking(3);
  multicore_fifo_push_blocking(angle);
  multicore_fifo_pop_blocking();
  trig_g.functionTimer(start);
  for(int i = 0; i < 8; i++){
    Serial.print(pinkyRun.moveOrder[i]);
  }
  Serial.println("");
}
void setup() {
  Serial.begin(115200);
  while(!Serial){//Wait for serial to come up
    sleep_ms(100);  
  }
  readCharsFromFile(ServoFile); //Load servo ranges into Ro
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  //Initialize pca9685 boards
  pinky.pwmScan();
  pinkyRun.PWMinit();
  
  pinky.generate_HeightTable();
  pinky.setHips();
  
  //--Multicore--
  multicore_launch_core1(core1_entry);//Fire up the second core
  sleep_ms(500);
  
  pinkyRun.initTimer(100, 25);//cycleSpeed%, # of subphases
  changeHeight(100);
  changeAngle(0);
  
  Serial.println("Setup Complete");
}

int testint = 0;
float testfloat = 0.000;
bool blink = true;
uint32_t value0 = 0;

void loop() {
  // if(testint == 0){
  //   // pinkyRun.lastA[0] = -1;
  //   // pinkyRun.lastK[0] = -1;
  //   // for(int i = 0; i <= pinkyRun.FR_Angles[4]; i++){
  //   //   //int start = micros();
  //   //   pinkyRun.find_hipMovement1(1, 0, i, pinky.combinedX, pinky.combinedZ);
  //   //   //trig_g.functionTimer(start);
  //   // }
  // }

  // if(pinkyRun.subPhaseCount <= 50){
  //   // push the read value onto the FIFO stack
  //   multicore_fifo_push_blocking(blink);
  //   multicore_fifo_push_blocking(testint);
    
  //   Serial.print("Sent:");
  //   Serial.print(testint);
  //   if(multicore_fifo_pop_timeout_us(1000, &value0)){
  //     Serial.print(" Recieved:");
  //     Serial.println(value0);
  //   }
  // }
  
  pinkyRun.tick();

  // blink = !blink;
  // testint ++;
  // if(testint == 8){
  //   testint = 0;
  // }
  // testfloat += 0.1;
  // sleep_ms(250);
}