/* LedMotionDisplay
 * Description: Colors entire LED strip one color based on accelerometer data
 * Author: Sam Morrison
 * Date last modified: 2/21/2018
 */

#include <PololuLedStrip.h>
 
// Create an ledStrip object and specify the pin it will use.
PololuLedStrip<12> ledStrip;

#define LED_COUNT 60 // constant LED count, must change for multi-strip setup
rgb_color colors[LED_COUNT]; // will have to initialize for each strip
void setup() {
  // put your setup code here, to run once:

}

//takes in accelerometer data and assigns each LED in strip
void setLED(float x, float y,float z) { 
  rgb_color color;
  color.red = x;
  color.green = y;
  color.blue = z;
  for(uint16_t i = 0; i < LED_COUNT; i++) {
    colors[i] = color; // assigns color to each pixel in array
  }
  ledStrip.write(colors, LED_COUNT); // writes to strip
}

void loop() {
  // put your main code here, to run repeatedly:
  while(1) {
    //receive accelerometer data
    //setLED(x, y, z); // set LED color
  }
}
