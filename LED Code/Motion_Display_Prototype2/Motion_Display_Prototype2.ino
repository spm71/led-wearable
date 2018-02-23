/******************************************************************************
 * File name: Motion_Display_Prototype2
 * Purpose: Colors entire LED strip one color based on accelerometer data
 * Author: Sam Morrison
 * 
 * Created: 2/19/2018
 * Date last modified: 2/23/2018
 *
 ******************************************************************************/
#include <FastLED.h>

#define NUM_LEDS 6 //number of LED's on strip
#define LED_PIN 2 //digital pin for data output

CRGB led[NUM_LEDS]; //create LED array

void set_led(int x, int y, int z);

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
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    led[i] = CRGB(0, 100, 0);
  }
  FastLED.show();
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


void loop() {
  // receive accelerometer data
  // set LED from data

}
