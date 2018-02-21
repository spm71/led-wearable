/* LedMotionDisplay
 *
 */
 
// Create an ledStrip object and specify the pin it will use.
PololuLedStrip<12> ledStrip;

#define LED_COUNT 60
rgb_color colors[LED_COUNT];
void setup() {
  // put your setup code here, to run once:

}

void setLED(float x, float y,float z) {
  rgb_color color;
  color.red = x;
  color.green = y;
  color.blue = z;
  for(uint16_t i = 0; i < LED_COUNT; i++) {
    colors[i] = color;
  }
  ledStrip.write(colors, LED_COUNT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(1) {
    //receive accelerometer data
    setLED(x, y, z);
  }
}
