// This code is meant to run on an Arduino and 
// conects to the RIO over SPI and a Rev Colour sensor 
// over I2C because I2C is broken on the RIO at the moment

#include <SPI.h>
#include <Wire.h>
#include <pins_arduino.h>

//These values are the readings of the whitepoint of the led. Take the back of vinyl sticker and put it againts face of sensor
#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f

//Calculate the balancing factors
#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN) 
#define BAL_BLUE (LED_GREEN/LED_BLUE)

#define I2C_ADDR 0x52
 
uint8_t dataFromSensor[12];
volatile uint8_t rawIR[2];
volatile uint8_t rawRed[2];
volatile uint8_t rawGreen[2];
volatile uint8_t rawBlue[2];

uint16_t ir = 0;
uint16_t red = 0;
uint16_t green = 0;
uint16_t blue = 0;

volatile byte receivedData;
volatile bool newRequest;

void setup() {
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  newRequest = false;
  SPI.attachInterrupt();

  Wire.begin();
  i2cWrite(0x00, 0b0110);  //enable light sensor and activate rgb mode
  i2cWrite(0x04, 0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms

  Serial.begin(9600);
}

ISR (SPI_STC_vect) {
  receivedData = SPDR;
  switch (receivedData)
  {
  case 0:
    SPDR = *rawIR;
    break;
  
  case 1:
    SPDR = *rawRed;
    break;

  case 2:
    SPDR = *rawGreen;
    break;

  case 3:
    SPDR = *rawBlue;
    break;
  
  default:
    SPDR = 0;
    break;
  }
}

void loop() {
  i2cRead(0x0A, dataFromSensor, 12);

  rawIR[0] = dataFromSensor[1];
  rawIR[1] = dataFromSensor[0];

  rawRed[0] = dataFromSensor[10];
  rawRed[1] = dataFromSensor[9];

  rawGreen[0] = dataFromSensor[4];
  rawGreen[1] = dataFromSensor[3];

  rawBlue[0] = dataFromSensor[7];
  rawBlue[1] = dataFromSensor[6];

/*
  ir = (rawIR[0] << 8) | rawIR[1];
  red = (rawRed[0] << 8) | rawRed[1];
  green = (rawGreen[0] << 8) | rawGreen[1];
  blue = (rawBlue[0] << 8) | rawBlue[1];
  
  red *= BAL_RED;
  green *= BAL_GREEN;
  blue *= BAL_BLUE;

  Serial.print(ir);
  Serial.print(" ");
  Serial.print(red);
  Serial.print(" ");
  Serial.print(green);
  Serial.print(" ");
  Serial.print(blue);
  Serial.println(" ");*/
}

void i2cWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cRead(uint8_t reg, uint8_t *val, uint16_t len){
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, len);
  for(uint8_t i = 0; i < len; i++){
    val[i] = Wire.read();
  }
}
