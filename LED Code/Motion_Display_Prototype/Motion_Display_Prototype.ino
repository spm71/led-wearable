/******************************************************************************
 * File name: Motion_Display_Prototype2
 * Purpose: Colors entire LED strip one color based on accelerometer data
 * Author: Sam Morrison
 * 
 * Created: 2/19/2018
 * Date last modified: 4/6/2018
 *
 ******************************************************************************/
#include <FastLED.h>
#include<Wire.h>
#include <sys/types.h>
#include <sys/wait.h>

// number of LED's on strip
#define NUM_LEDS_ARMS 16 
#define NUM_LEDS_LEGS 24

// digital pins for LED data output
#define LED_PIN_1 5 
#define LED_PIN_2 6
#define LED_PIN_3 7
#define LED_PIN_4 8

// digital pins for motion data input
#define MOTION_PIN_1 0 
#define MOTION_PIN_2 1
#define MOTION_PIN_3 2
#define MOTION_PIN_4 3

// variables for acceleration data
long accelX, accelY, accelZ;
float gForceX_1, gForceY_1, gForceZ_1;
float gForceX_2, gForceY_2, gForceZ_2;
float gForceX_3, gForceY_3, gForceZ_3;
float gForceX_4, gForceY_4, gForceZ_4;

// variables for gyroscopic data
long gyroX, gyroY, gyroZ;
float rotX_1, rotY_1, rotZ_1;
float rotX_2, rotY_2, rotZ_2;
float rotX_3, rotY_3, rotZ_3;
float rotX_4, rotY_4, rotZ_4;

// i2c address for active MPU
const int address = 0b1101000;

// declare previous gyro values
float prev_rotX_1, prev_rotY_1, prev_rotZ_1;
float prev_rotX_2, prev_rotY_2, prev_rotZ_2;
float prev_rotX_3, prev_rotY_3, prev_rotZ_3;
float prev_rotX_4, prev_rotY_4, prev_rotZ_4;
int inputx, inputy, inputz;

//LED arrays
CRGB led_1[NUM_LEDS_ARMS];
CRGB led_2[NUM_LEDS_ARMS];
CRGB led_3[NUM_LEDS_LEGS];
CRGB led_4[NUM_LEDS_LEGS];

// function prototypes
void set_led(CRGB led, int led_count, int x, int y, int z);
void setup_MPU();
void record_accel_registers(int sensor);
void record_gyro_registers(int sensor);
void process_accel_data(int sensor);
void process_gyro_data(int sensor);
void print_data();
void scan_motion(int sensor);

/*
 * void setup();
 * Description: initialize LED strips to green and set up MPUs
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
  
   // initialize LED strips
  FastLED.addLeds<NEOPIXEL, LED_PIN_1>(led_1, NUM_LEDS_ARMS);
  FastLED.addLeds<NEOPIXEL, LED_PIN_2>(led_2, NUM_LEDS_ARMS);
  FastLED.addLeds<NEOPIXEL, LED_PIN_3>(led_3, NUM_LEDS_LEGS);
  FastLED.addLeds<NEOPIXEL, LED_PIN_4>(led_4, NUM_LEDS_LEGS);
  
  // set all LED strips to low green
  for (int i = 0; i < NUM_LEDS_ARMS; i++) {
    led_1[i] = CRGB(0, 10, 0);
    led_2[i] = CRGB(0, 10, 0);
    led_3[i] = CRGB(0, 10, 0);
    led_4[i] = CRGB(0, 10, 0);
  }
  FastLED.show();

  // declare motion sensors as outputs
  pinMode (MOTION_PIN_1, OUTPUT);
  pinMode (MOTION_PIN_2, OUTPUT);
  pinMode (MOTION_PIN_3, OUTPUT);
  pinMode (MOTION_PIN_4, OUTPUT);

  // set one motion sensor as active, others as inactive
  digitalWrite(MOTION_PIN_1,LOW);
  digitalWrite(MOTION_PIN_2,HIGH);
  digitalWrite(MOTION_PIN_3,HIGH);
  digitalWrite(MOTION_PIN_4,HIGH);
  // run setup function for first MPU
  setup_MPU();
  // repeat process for three other MPU's
  digitalWrite(MOTION_PIN_1,HIGH);
  digitalWrite(MOTION_PIN_2,LOW);
  setup_MPU();
  
  digitalWrite(MOTION_PIN_2,HIGH);
  digitalWrite(MOTION_PIN_3,LOW);
  setup_MPU();

  digitalWrite(MOTION_PIN_3,HIGH);
  digitalWrite(MOTION_PIN_4,LOW);
  setup_MPU();

  digitalWrite(MOTION_PIN_4,HIGH);
  
  Serial.println("Ready");
}

/*
 * void loop();
 * Description: continuously receive motion data and write to LEDs
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void loop() {
  
  //double start = millis();
  
  // first limb
  scan_motion(MOTION_PIN_1); //obtain motion data
  
  //catch glitches, assigning current values to previous values
  if (rotX_1 > 400)
    rotX_1 = prev_rotX_1;
  if (rotY_1 > 400)
    rotY_1 = prev_rotY_1;
  if (rotZ_1 > 400)
    rotZ_1 = prev_rotZ_1;

   //cast rotation values to integers
   inputx = (int) rotX_1;
   inputy = (int) rotY_1;
   inputz = (int) rotZ_1;
   
   for (int i = 0; i < NUM_LEDS_ARMS; i++) {
    led_1[i] = CRGB(inputx/10, inputy/10, inputz/10); // Set LED values, adjusting for relative brightness
   }

  // second limb
  scan_motion(MOTION_PIN_2);
  if (rotX_2 > 400)
    rotX_2 = prev_rotX_2;
  if (rotY_2 > 400)
    rotY_2 = prev_rotY_2;
  if (rotZ_2 > 400)
    rotZ_2 = prev_rotZ_2;

   inputx = (int) rotX_2;
   inputy = (int) rotY_2;
   inputz = (int) rotZ_2;
   for (int i = 0; i < NUM_LEDS_ARMS; i++) {
    led_2[i] = CRGB(inputx/10, inputy/10, inputz/10);
   }

  // third limb
  scan_motion(MOTION_PIN_3);
  if (rotX_3 > 400)
    rotX_3 = prev_rotX_3;
  if (rotY_3 > 400)
    rotY_3 = prev_rotY_3;
  if (rotZ_3 > 400)
    rotZ_3 = prev_rotZ_3;

   inputx = (int) rotX_3;
   inputy = (int) rotY_3;
   inputz = (int) rotZ_3;
   for (int i = 0; i < NUM_LEDS_LEGS; i++) {
    led_3[i] = CRGB(inputx/10, inputy/10, inputz/10);
   }

  // fourth limb
  scan_motion(MOTION_PIN_4);
  if (rotX_4 > 400)
    rotX_4 = prev_rotX_4;
  if (rotY_4 > 400)
    rotY_4 = prev_rotY_4;
  if (rotZ_4 > 400)
    rotZ_4 = prev_rotZ_4;

   inputx = (int) rotX_4;
   inputy = (int) rotY_4;
   inputz = (int) rotZ_4;
   for (int i = 0; i < NUM_LEDS_LEGS; i++) {
    led_4[i] = CRGB(inputx/10, inputy/10, inputz/10);
   }

  //print_data();
  
  FastLED.show(); // bit bang to LED's

  // find processing time and print it to console
  /*double elapsed_time = millis() - start; 
  
  Serial.print("Time elapsed: ");
  Serial.print(elapsed_time);
  Serial.println("ms");
  */
  
}

/*
 * void set_led(CRGB led, int led_count, int x, int y, int z) 
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
void set_led(CRGB led, int led_count, int x, int y, int z) { 
  for (int i = 0; i < led_count; i++) {
    led[i] = CRGB(x, y, z);
  }
  FastLED.show();
}

/*
 * void setup_MPU();
 * Description: Initializes I2C connection with active address
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void setup_MPU(){
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

/*
 * void record_accel_registers();
 * Description: record 3-axis acceleration data
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void record_accel_registers(int sensor) {
  Wire.beginTransmission(address); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  process_accel_data(sensor);
}

/*
 * void record_gyro_registers();
 * Description: record 3-axis gyroscope data
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void record_gyro_registers(int sensor) {
  Wire.beginTransmission(address); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(address,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  process_gyro_data(sensor);
}

/*
 * void process_accel_data();
 * Description: modify acceleration data to reflect number of g's
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void process_accel_data(int sensor){
  if (sensor == MOTION_PIN_1) {
    gForceX_1 = accelX / 1638.40;
    gForceY_1 = accelY / 1638.40; 
    gForceZ_1 = accelZ / 1638.40;
  }
  else if (sensor == MOTION_PIN_2) {
    gForceX_2 = accelX / 1638.40;
    gForceY_2 = accelY / 1638.40; 
    gForceZ_2 = accelZ / 1638.40;
  }
  else if (sensor == MOTION_PIN_2) {
    gForceX_3 = accelX / 1638.40;
    gForceY_3 = accelY / 1638.40; 
    gForceZ_3 = accelZ / 1638.40;
  }
  else if (sensor == MOTION_PIN_2) {
    gForceX_4 = accelX / 1638.40;
    gForceY_4 = accelY / 1638.40; 
    gForceZ_4 = accelZ / 1638.40;
  }
  
}

/*
 * void process_gyro_data();
 * Description: modify gyroscopic data to reflect number of degrees per second
 * 
 * Inputs: 
 *      Parameters: none
 *      
 * Outputs:
 *      Returns: void
*/
void process_gyro_data(int sensor) {
  if (sensor == MOTION_PIN_1) {
    rotX_1 = gyroX / 131.0;
    rotY_1 = gyroY / 131.0; 
    rotZ_1 = gyroZ / 131.0;
  }
  else if (sensor == MOTION_PIN_2) {
    rotX_2 = gyroX / 131.0;
    rotY_2 = gyroY / 131.0; 
    rotZ_2 = gyroZ / 131.0;
  }
  else if (sensor == MOTION_PIN_3) {
    rotX_3 = gyroX / 131.0;
    rotY_3 = gyroY / 131.0; 
    rotZ_3 = gyroZ / 131.0;
  }
  else if (sensor == MOTION_PIN_4) {
    rotX_4 = gyroX / 131.0;
    rotY_4 = gyroY / 131.0; 
    rotZ_4 = gyroZ / 131.0;
  }
}

/*
 * int convert_to_LED(long data);
 * Description: convert values to a format fit for the LED strip
 * 
 * Inputs: 
 *      Parameters: 
 *        long data: input data from motion sensor
 *      
 * Outputs:
 *      Returns:
 *        int led_data: formatted value for LED
*/
int convert_to_LED(long data) {
  int led_data = (int) (data * 50);
  return led_data;
}

/*
 * void print_data();
 * Description: print all motion data values to test sensors
 * 
 * Inputs: 
 *      Parameters: 
 *        long data: input data from motion sensor
 *      
 * Outputs:
 *      Returns:
 *        int led_data: formatted value for LED
*/
void print_data() {
  Serial.print("Gyro (deg)");
  Serial.print(" X1=");
  Serial.print(rotX_1);
  Serial.print(" Y1=");
  Serial.print(rotY_1);
  Serial.print(" Z1=");
  Serial.print(rotZ_1);

  Serial.print(" X2=");
  Serial.print(rotX_2);
  Serial.print(" Y2=");
  Serial.print(rotY_2);
  Serial.print(" Z2=");
  Serial.print(rotZ_2);

  Serial.print(" X3=");
  Serial.print(rotX_3);
  Serial.print(" Y3=");
  Serial.print(rotY_3);
  Serial.print(" Z3=");
  Serial.print(rotZ_3);

  Serial.print(" X4=");
  Serial.print(rotX_4);
  Serial.print(" Y4=");
  Serial.print(rotY_4);
  Serial.print(" Z4=");
  Serial.println(rotZ_4);
  
  //Serial.print(" Accel (g)");
  //Serial.print(" X=");
  //Serial.print(gForceX_1);
  //Serial.print(" Y=");
  //Serial.print(gForceY_1);
  //Serial.print(" Z=");
  //Serial.println(gForceZ_1);
}

/*
 * void scan_motion();
 * Description: sets a motion sensor and captures data from it
 * 
 * Inputs: 
 *      Parameters: 
 *        int sensor: pin number for motion sensor
 *      
 * Outputs:
 *      Returns:
*/
void scan_motion(int sensor) {
  digitalWrite(sensor,LOW); // set current sensor to active
  //Serial.print("scanning MPU");
  //Serial.println(sensor);
  //record_accel_registers(sensor);
  record_gyro_registers(sensor); // record gyro data
  digitalWrite(sensor,HIGH); // set sensor back to inactive
}

