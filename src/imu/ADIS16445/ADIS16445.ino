#include <SPI.h>

SPISettings settings(2000000, MSBFIRST, SPI_MODE3);
const int slavePin = 10;

int16_t PROD_ID_REG, PROD_ID, GLOB_CMD, DIAG_STAT, XGYRO_OUT, YGYRO_OUT, ZGYRO_OUT, XACCL_OUT, YACCL_OUT, ZACCL_OUT, TEMP_OUT; 

void setup() {
  PROD_ID_REG = 0x5600;
  GLOB_CMD = 0x3E00;

  Serial.begin(115200);
  
  pinMode(slavePin, OUTPUT);
  SPI.begin();

  SPI.beginTransaction(settings);
  digitalWrite(slavePin, LOW);
  PROD_ID = 0x0000;

  //test SPI comms by getting product ID
  while(PROD_ID != 0x403D){
    SPI.transfer16(PROD_ID_REG);
    delayMicroseconds(20);
    PROD_ID = SPI.transfer16(PROD_ID_REG);
  }
}

void loop(){
  //Burst read. Start with glob command, read next 8 16 bit values.
  SPI.beginTransaction(settings);
  digitalWrite(slavePin, LOW);
  SPI.transfer16(GLOB_CMD);
  DIAG_STAT = SPI.transfer16(0);
  XGYRO_OUT = SPI.transfer16(0);
  YGYRO_OUT = SPI.transfer16(0);
  ZGYRO_OUT = SPI.transfer16(0);
  XACCL_OUT = SPI.transfer16(0);
  YACCL_OUT = SPI.transfer16(0);
  ZACCL_OUT = SPI.transfer16(0);
  TEMP_OUT = SPI.transfer16(0);
  digitalWrite(slavePin, HIGH);
  SPI.endTransaction();

  Serial.println(String(DIAG_STAT)+"|"+String(XGYRO_OUT)+"|"+String(YGYRO_OUT)+"|"+String(ZGYRO_OUT)+"|"+String(XACCL_OUT)+"|"+String(YACCL_OUT)+"|"+String(ZACCL_OUT)+"|"+String(TEMP_OUT));
}
