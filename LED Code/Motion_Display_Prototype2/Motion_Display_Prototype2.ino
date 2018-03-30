/******************************************************************************
 * File name: Motion_Display_Prototype2
 * Purpose: Colors entire LED strip one color based on accelerometer data
 * Author: Sam Morrison
 * 
 * Created: 2/19/2018
 * Date last modified: 3/9/2018
 *
 ******************************************************************************/
#include <FastLED.h>
#include<Wire.h>
#include <sys/types.h>
#include <sys/wait.h>

#define NUM_LEDS 6 //number of LED's on strip
#define LED_PIN 1 //digital pin for data output
#define TEST_PIN 2

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
const int address = 0b1101000;
int done = 1;


CRGB led[NUM_LEDS]; //create LED array

void set_led(int x, int y, int z);
void setupMPU();
void recordAccelRegisters();
void processAccelData();
void printData();

/*
 * void setup();
 * Description: initialize LED strip to green
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void setup() { 
  Serial.begin(9600);
  Wire.begin();
  /*
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    led[i] = CRGB(0, 50, 0);
  }
  FastLED.show();
  */
  
  setupMPU();
  
  Serial.println("Ready");
}

/*
 * void set_led(int x, int y, int z) 
 * Description: set all LED's based on accelerometer input
 * 
 * Inputs: 
 *      Parameters:
 *          int x: x-axis of movement
 *          int y: y-axis of movement
 *          int z: z-axis of movement
 *      
 * Outputs:
 *      Returns: void
*/
void set_led(int x, int y, int z) { 
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    led[i] = CRGB(x, y, z);
  }
  FastLED.show();
}

void setupMPU(){
  Wire.beginTransmission(address); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(address); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(address); //I2C address of the MPU
  Wire.write(0x1D); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(address); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 1638.40;
  gForceY = accelY / 1638.40; 
  gForceZ = accelZ / 1638.40;
}


int convert_to_LED(long data) {
  int led_data = (int) (data * 50);
  return led_data;
}

void printData() {
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}

void loop() {
  // receive accelerometer data
  // set LED from data
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  recordAccelRegisters();
  printData();
  for (int i = 0; i < NUM_LEDS; i++) {
    led[i] = CRGB(gForceX, gForceY, gForceZ);
    digitalWrite(TEST_PIN, HIGH);
    digitalWrite(TEST_PIN, LOW);
  }
  FastLED.show();
  
  //delay(100);
  //Serial.println("Assigned colors");
}
