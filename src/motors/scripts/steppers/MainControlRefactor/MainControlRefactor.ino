#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <TeensyThreads.h>
#include <numeric>
#include<string>
#include <algorithm>


#define CMD_BYTES 8

int led = 13;

volatile int cmd = 0;

const uint8_t sleepPin = 14;

uint8_t inverted[4] = {1, 0, 0, 0};

uint8_t homingPins[4] = {20, 3, 32, 33};
const uint8_t CSPins[4] = {23, 0, 29, 36};
const uint8_t FaultPins[4] = {1, 30, 35, 22}; //currently unused
const uint8_t StallPin[4] = {2, 31, 34,21}; //currently unused
const uint8_t stepPins[4] = {19,5,28,38};
const uint8_t dirPins[4] = {18, 4, 27, 37};


const double degrees_per_step = 1.8;
const double microstep = 1; //2^1, n = 0 - 8 //was 4
const double degreesM1 = 360;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const double StepPeriodUs = 800; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

bool isHomed=false;

// for manually homing
bool isHoming = false;
int beingHomed = 0;

HighPowerStepperDriver drivers[4];

int currentPositions[4] = {90, 90, 90, 90};
int desiredPositions[4] = {90, 90, 90, 90};

Threads::Mutex stepLock;

void recieve_command(){
  while(1){ 
    if(Serial.available() > 0) {
      char read_byte = Serial.read();
      switch(read_byte){
        case 0x1:{
          send_msg("init cmd recieved", false);
          home_all();
          send_msg("init_all processed");
          break;
        }
        case 0x2:{
          send_msg("align_cmd_recieved", false);
          int degs[4] = {0,0,0,0};
          for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
              while(Serial.available() < 1){}
              uint8_t read_byte = Serial.read();
              //send_msg(read_byte, false);
              //send_msg(String(degs[i]), false);
              degs[i] = int(degs[i] | (unsigned char)(read_byte) << (3-j)*8);
            }
            //send_msg(String(degs[i]),false);
          }

          if(isHoming) {
            desiredPositions[beingHomed] = degs[beingHomed];
          }
          else {
            align_all(degs);
          }

          for(int j = 0; j <4; j++){
            send_msg(String(desiredPositions[j])+" "+String(currentPositions[j]), false);
          }
          send_msg("desired angles sent");
          break;
        }
        case 0x6:{
          send_msg("blink cmd recieved", false);
          while(Serial.available() < 1){}
          int num_blinks = Serial.read();
          send_msg("blinking " + (String)num_blinks + " times\n", false);
          for(int i = 0; i < num_blinks; i++){
              digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
              delay(1000);               // wait for a second
              digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
              delay(1000);               // wait for a second
          }
          send_msg("blink processed");
          break;
        }

        case 0x7: {
          send_msg("home cmd recieved\n", false);
          while(Serial.available() < 1){}
          int driver_to_home = Serial.read();
          home(driver_to_home);
          send_msg("home processed");
          break;
        }

        case 0x9: {
          if(isHoming) { break; }
          send_msg("manual home cmd recieved\n", false);
          while(Serial.available() < 1){}
          beingHomed = Serial.read();
          isHoming = true;
          send_msg("start manual home completed");
          break;
        }

        case 0xA: {
          send_msg("stop manual home cmd recieved\n", false);
          while(Serial.available() < 1){}
          int driver_to_home = Serial.read();
          isHoming = false;
          desiredPositions[beingHomed] = 90;
          currentPositions[beingHomed] = 90;
          send_msg("stop manual home completed");
        }

        case 0xB: {
          send_msg("fake init received\n", false);
          
          desiredPositions[0] = -90;
          desiredPositions[1] = 270;
          desiredPositions[2] = 270;
          desiredPositions[3] = -90;

          int count = 0;
          while(currentPositions[1] != 270 || currentPositions[2] != 270 || currentPositions[0] != -90 || currentPositions[3] != -90){
            if(count % 1000 == 0){
              send_msg(String(currentPositions[0]) + " " + String(currentPositions[1]) + " " + String(currentPositions[2]) + " " + String(currentPositions[3]), false);
            }
            count++;
          }

          for(int i = 0; i < 4; i++){
            desiredPositions[i] = 90;
            currentPositions[i] = 90;
          }

          send_msg("fake init completed");
          break;
        }

        case 0xC: {
          send_msg("switch report request received\n", false);
          
          send_msg(String(digitalRead(homingPins[0])), false);
          send_msg(String(digitalRead(homingPins[1])), false);
          send_msg(String(digitalRead(homingPins[2])), false);
          send_msg(String(digitalRead(homingPins[3])), false);

          send_msg("switch report completed");
          break;
        }

        default: {
          Serial.println("recieved some wacky bytes");
          break;
        }
      }
    }
  }
}

void home_all(){
  bool homed[4] = {false, false, false, false};
  for(int i = 0; i < 4; i++){
    desiredPositions[i] = 90;
    currentPositions[i] = 90;
  }
  
  while(std::accumulate(homed, homed + 4, 0) < 4){
    if(Serial.available() > 0){
      int cmd = Serial.read();
      if(cmd == 0x8){ //cancel
        send_msg("init canceled", false);
        return;
      }
    }
    

    delay(100);
    for(int i = 0; i < 4; i++){
      if(homed[i]==false){
        if(digitalRead(homingPins[i]) == 0){
          desiredPositions[i] += 1;
        }
        else{
          send_msg(String(i) + " homed", false);
          homed[i] = true;
          desiredPositions[i] = 90;
          currentPositions[i] = 90;
        }
      }
    }
  }

  isHomed = true;
}

void home(int driver_num) {
  while(digitalRead(homingPins[driver_num]) == 0) {
    desiredPositions[driver_num] += 1;
    delay(100);
  }
  desiredPositions[driver_num] = 90;
  currentPositions[driver_num] = 90;
}


void align_all(int degs[]){
  for(int i = 0; i<4; i++){
    //threads.addThread(turn_degrees, new int[2]{degs[i]-currentPositions[i], i}); 
    desiredPositions[i] = degs[i];
  }
}

void send_msg(String msg){
  send_msg(msg, true);
}

void send_msg(String msg, bool terminate){
  if(!terminate){
    Serial.print(msg + '\n');
  }
  else{
    Serial.print(msg + '\0');
  }
}

void turn_degrees(int* args){
  int degrees = args[0];
  int driverNum = args[1];
  double steps = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
  if(degrees>=0){
    drivers[driverNum].setDirection(0);
  }
  else{
    drivers[driverNum].setDirection(1);
  }
  for(unsigned int x = 0; x <= steps; x++)
  {
    drivers[driverNum].step();
    delayMicroseconds(StepPeriodUs);
  }
}


void init_drivers(){
  for(int i = 0; i < 4; i++){
    pinMode(homingPins[i], INPUT);
  
    drivers[i].setChipSelectPin(CSPins[i]); 
    delay(1);
    drivers[i].resetSettings();
    drivers[i].clearStatus();
    drivers[i].setDecayMode(HPSDDecayMode::AutoMixed);
    drivers[i].setCurrentMilliamps36v4(2800); //4000 max
    double steps_approx = abs(degreesM1/(degrees_per_step/(gear_ratio*microstep)));
    double time_approx = (steps_approx*StepPeriodUs)/1000000;
    float rpm_approx = (60/time_approx)*degreesM1/float(360);  
    drivers[i].setStepMode(microstep);
    
    drivers[i].enableDriver();
  }
}

void alignControllerThread(){
  int diffs[4] = {0, 0, 0, 0};
  double steps[4] = {0.0, 0.0, 0.0, 0.0};
  while(true){
    for(int i = 0; i<4; i++){
      diffs[i] = currentPositions[i] - desiredPositions[i];
      
      if(diffs[i]){
        //send_msg(String(diffs[i]), false);
        int degrees = diffs[i]>0?-1:1;
        steps[i] = fabs(degrees/(degrees_per_step/(gear_ratio*microstep)));
        //send_msg("setting steps: "+String(steps[i]),false);
        
        currentPositions[i] += degrees;
        if(degrees>=0){
          //drivers[driverNum].setDirection(0);
          digitalWrite(dirPins[i], inverted[i]?HIGH:LOW);
        }
        else{
          //drivers[driverNum].setDirection(1);
          digitalWrite(dirPins[i], inverted[i]?LOW:HIGH);
        }
      }
      else{
        steps[i] = 0;
      }
    }

    while(!isZeros(steps, 4)){
        //send_msg("correcting", false);
        for(int i = 0; i < 4; i++){
          if(steps[i] > 0) digitalWrite(stepPins[i], HIGH);
          //send_msg(String(steps[i]), false);
        }
        delayMicroseconds(StepPeriodUs);
        for(int i = 0; i < 4; i++){
          if(steps[i] > 0) digitalWrite(stepPins[i], LOW);
          if(steps[i] > 0) steps[i]--; 
        }
        delayMicroseconds(StepPeriodUs);
    }

    
//    for(int i = 0; i<4; i++){
//      diffs[i] = currentPositions[i] - desiredPositions[i];
//      if(diffs[i]){
//        int degrees = diffs[i]>0?-1:1;
//        int driverNum = i;
//        steps[i] = abs(degrees/(degrees_per_step/(gear_ratio*microstep)));
//        if(degrees>=0){
//          //drivers[driverNum].setDirection(0);
//          digitalWrite(dirPins[driverNum], LOW);
//        }
//        else{
//          //drivers[driverNum].setDirection(1);
//          digitalWrite(dirPins[driverNum], HIGH);
//        }
//        for(unsigned int x = 0; x <= steps[i]; x++)
//        {
//          //drivers[driverNum].step();
//          digitalWrite(stepPins[driverNum], HIGH);
//          delayMicroseconds(StepPeriodUs);
//          digitalWrite(stepPins[driverNum], LOW);
//          delayMicroseconds(StepPeriodUs);
//        }
//        currentPositions[i] += degrees;
//      }
//    }
//    for(int i = 0; i<4; i++){
//      send_msg(String(diffs[i])+" "+String(steps[i]), false);
//    }
  }
  
}

bool isZeros(double* arr, int len){
  for(int i = 0; i < len; i++){
    if(arr[i] > 0.0){
      return false;
    }
  }
  return true;
}

bool twoArrEqual(uint8_t arr1[], uint8_t arr2[]) 
{ 
  for (int i = 0; i < 4; i++){
    if (arr1[i] != arr2[i]) {
      return false;
    }
  }
  
  return true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);

  Serial.begin(115200);
  for(int i = 0; i < 4; i++){
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(200);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(200);               // wait for a second
  }
  
  SPI.begin();

  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);

  for(int i = 0; i < 4; i++){
      pinMode(stepPins[i],OUTPUT);
      pinMode(dirPins[i],OUTPUT);
  }

  init_drivers();
  threads.addThread(alignControllerThread);
  
//  pinMode(FaultPin, INPUT);
//  pinMode(StallPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  recieve_command();
  // for(int i = 0; i < 4; i++){
  //   desiredPositions[i]++;
  // }
}
