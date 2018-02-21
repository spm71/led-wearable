/* Motion_Display_Prototype2
 * Description: Colors entire LED strip one color based on accelerometer data
 * Author: Sam Morrison
 * Date last modified: 2/21/2018
 */

#include <bitswap.h>
#include <chipsets.h>
#include <color.h>
#include <colorpalettes.h>
#include <colorutils.h>
#include <controller.h>
#include <cpp_compat.h>
#include <dmx.h>
#include <FastLED.h>
#include <fastled_config.h>
#include <fastled_delay.h>
#include <fastled_progmem.h>
#include <fastpin.h>
#include <fastspi.h>
#include <fastspi_bitbang.h>
#include <fastspi_dma.h>
#include <fastspi_nop.h>
#include <fastspi_ref.h>
#include <fastspi_types.h>
#include <hsv2rgb.h>
#include <led_sysdefs.h>
#include <lib8tion.h>
#include <noise.h>
#include <pixelset.h>
#include <pixeltypes.h>
#include <platforms.h>
#include <power_mgt.h>

#define NUM_LEDS 10
#define LED_PIN 2

CRGB led[NUM_LEDS];

void set_led(int x, int y, int z);

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    led[i] = CRGB(0, 255, 0);
  }
  FastLED.show();
}

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
