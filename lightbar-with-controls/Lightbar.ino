// Arduino Nano
// ATMega328P (old bootloader)



#import "FastLED.h"

const int NUM_LEDS = 31;
const int LED_PIN_NUM = 6;
const int MIDDLE = NUM_LEDS / 2;

const int STARTSTOP_PIN = 11;
const int SPEED_PIN = A2;
const int HUE_PIN = A3;
const int BRIGHT_PIN = A4;
const int DEBOUNCE_TIME = 150;   // ms

CRGB leds[NUM_LEDS];


static bool goingForward = true;
static int loc = MIDDLE;

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN_NUM>(leds, NUM_LEDS);
  Serial.begin(9600);

  pinMode(STARTSTOP_PIN, INPUT_PULLUP);
}


 void setLEDHB(int led, uint8_t hue,  uint8_t brightness = 255) {
  if ((led >= 0) && (led < NUM_LEDS)) {
    leds[led] = CHSV(hue, 255, brightness);
  }
 }

 void setLEDColor(int led, int color) {
  if ((led >= 0) && (led < NUM_LEDS)) {
    leds[led] = color;
  }
 }
 

int showLights(uint8_t hue, uint8_t brightness = 255) {
   int incr;
   
   Serial.print(loc);
   Serial.print("\n>>> ");
   
   if ((loc == 0) || (loc == NUM_LEDS-1)) {
     goingForward = !goingForward;    // Reverse at ends
     }

   incr = goingForward ? 1 : -1;
   setLEDHB(loc,hue,brightness);

   // Update it

  FastLED.show();

  // Set it all back
  setLEDColor(loc,CRGB::Black);
    
  return loc + incr;
}

void stopped(uint8_t hue, int brightness = 255) {
  loc = MIDDLE;
  
  setLEDHB(loc,hue,brightness);
  setLEDHB(loc+1,hue,brightness);  
  setLEDHB(loc-1,hue,brightness);
  
  FastLED.show();
  
  setLEDColor(loc,CRGB::Black);
  setLEDColor(loc+1,CRGB::Black);
  setLEDColor(loc-1,CRGB::Black);

}

/*
 * ----------------------------------------------------------------------------
 */

bool isRunning() {
  static int lastSwitchState = HIGH;
  static bool currentlyRunning = false;
  
  int currState = digitalRead(STARTSTOP_PIN);

  if (currState != lastSwitchState) {
    delay(DEBOUNCE_TIME);     // Wait a smidge to avoid bounces
    
    if (currState == LOW) { 
       Serial.println("STARTSTOP pushed");
       currentlyRunning = ! currentlyRunning;
       }
  }
   
  return currentlyRunning;
}

/*
 * ----------------------------------------------------------------------------
 */
   

void loop() {
  int hue = analogRead(HUE_PIN) >> 2;
  long int speedSwitch = 1023-analogRead(SPEED_PIN);  // Returns 0 to 1023
  long int delayLen = 180 + speedSwitch << 4;
  int brightness = analogRead(BRIGHT_PIN) >> 2;
  int fadeout = 0;
  int fadeDelay = delayLen >> 2;    // 1/4 of a delay
  int newloc;

  if (brightness < 10) brightness = 10;
  if (delayLen < 1) delayLen = 1;
  
  // hue - hue & 255;      // Remove last few bits of color
  
  hue = hue + 170;
  if (hue > 255) { hue = hue - 265; }
  
  // delayLen = analogRead(SPEED_PIN) >> 2;
  // brightness = 255 - (analogRead(BRIGHT_PIN) >> 2);
  
  if (isRunning()) {
      newloc = showLights(hue, brightness);

    if (delayLen > 2048)
      delay(delayLen >> 8);
    else
      delayMicroseconds(delayLen);
      
      loc = newloc;
  }
  else {
     stopped(hue, brightness);
  }
}
