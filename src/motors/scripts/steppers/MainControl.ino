#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <TeensyThreads.h>
#include <numeric>

#define CMD_BYTES 8
#define LED_PIN 13

volatile int cmd = 0;

const uint8_t sleepPin = 14;

uint8_t homingPins[4] = {3, 32, 33, 20};
const uint8_t CSPins[4] = {0, 29, 36, 23};
const uint8_t FaultPins[4] = {1, 30, 35, 22}; //currently unused
const uint8_t StallPin[4] = {2, 31, 34,21}; //currently unused

uint8_t currentPositions[4] = {180, 180, 180, 180};

const double degrees_per_step = 1.8;
const double microstep = 2; //2^1, n = 0 - 8 //was 4
const double degreesM1 = 360;
const double gear_ratio = 46.656; //old stepper motor gear ratio was 4.25

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const double StepPeriodUs = 500; //best value so far 800
const uint16_t StepPeriodMs = 1; //was 2

bool isHomed=false;

HighPowerStepperDriver drivers[4];

Threads::Mutex stepLock;

void recieve_command(){
  while(1){ 
    if(Serial.available() > 0) {
      char read_byte = Serial.read();
      
      switch(read_byte){
        case 0x1:
          send_msg("init cmd recieved", false);
          home_all();
          send_msg("init_all processed");
          break;

        case 0x2:
          
          uint8_t degs[4];
          for(int i = 0; i < 4; i++){
            while(Serial.available() < 1){}
            char read_byte = Serial.read();
            send_msg(read_byte, false);
            degs[i] = read_byte;
          }

          align_all(degs);
          send_msg("align_cmd_recieved");
          break;
          
        case 0x6:
          send_msg("blink cmd recieved\n", false);
          while(Serial.available() < 1){}
          int num_blinks = Serial.read();
          send_msg("blinking " + (String)num_blinks + " times\n", false);
          for(int i = 0; i < num_blinks; i++){
              digitalWrite(LED_PIN, HIGH);
              delay(600);
              digitalWrite(LED_PIN, LOW);
              delay(600);
          }
          send_msg("blink processed");
          break;
          
         default:
          Serial.println("recieved some wacky bytes");
          break;
      }
    }
  }
}

void home_all(){
  bool homed[4] = {false, false, false, false};

  while(std::accumulate(homed, homed + 4, 0) < 4){
    for(int i = 0; i < 4; i++){
      if(homed[i]==false){
        if(digitalRead(homingPins[i]) == 0){
          threads.addThread(turn_degrees, new int[2]{2, i});
        }
        else{
          homed[i] = true;
        }
      }
    }
  }

  isHomed = true;
}

void align_all(uint8_t degs[]){

  for(int i = 0; i<4; i++){
    threads.addThread(turn_degrees, new int[2]{degs[i]-currentPositions[i], i});

    currentPositions[i] = degs[i];
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

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  
  delay(1000);
  
  SPI.begin();
  
  //digitalWrite(SleepPin, HIGH);

  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);

  init_drivers();
  
//  pinMode(FaultPin, INPUT);
//  pinMode(StallPin, INPUT);
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

void loop() {
  // put your main code here, to run repeatedly:
  recieve_command();
}
