// SparkFun Pro Micro
// Atmega 32U4 (5V, 16Mhz)

#define RTC 1

#include "HID-Project.h"

#include <ctype.h>
#include <EEPROM.h>
#include <TimeLib.h>    // Install library: Time by Michael Margolis
#include <SoftPWM.h>    // Install library: softpwm by Brett Hagman

#define VERSION "v2.1.1"

#ifdef RTC
#include <Wire.h>
#include "DS3231.h"

DS3231 myClock;

#endif

struct prog_settings {
  uint32_t magicNum;
  uint32_t mouseMoveRate;    // in secs
  uint32_t numPixelsToMove;  //
  uint32_t yellowButtonDur; // in secs
  uint32_t greenButtonDur;  // in secs
  uint32_t blueButtonOffTime;  // in secs
  uint32_t debugLevel; 
  char     initialButton;
};

prog_settings ourSettings;

const uint32_t CUR_MAGIC_NUM = 12354;

const uint32_t DEF_MOUSE_MOVE_RATE = 5U * 60U;    // once every 5 minutes
const uint32_t DEF_GREEN_DUR = 4U * 60U * 60U;   // 4 Hrs
const uint32_t DEF_YELLOW_DUR = 2U * 60U * 60U;   // 2 Hrs
const uint32_t DEF_NUM_PIXELS = 1;
const uint32_t DEF_BLUE_TIME_OFF = 17U * 3600U;    // 5:00pm
const char     DEF_INITIAL_BUTTON = 'r';



const int RED_LED_PIN       = 4;
const int BLUE_LED_PIN      = 5;
const int GREEN_LED_PIN     = 6;

const int GREEN_BUTTON_PIN  = 18;
const int YELLOW_BUTTON_PIN = 19;
const int RED_BUTTON_PIN    = 20;
const int BLUE_BUTTON_PIN   = 21;  
const int BLACK_BUTTON_PIN  =  8;
const int WHITE_BUTTON_PIN  =  7;

const int SHIFT_KEY         = WHITE_BUTTON_PIN;
  
enum button_mode { btnContinuous, btnOnce, btnRepeat };
enum program_mode { progInitMode, progDoNothingMode, progMoveDur, progMoveUntil };

enum led_mode { ledInactive, ledSolid, ledBreath, ledBlink, ledRotate };

struct one_button {
  int pinNum;
  bool currState;
  unsigned long debouncePeriod;
  button_mode mode;
  bool fakeInput;
};

struct led_state {
  unsigned int red = 0;
  unsigned int green = 0;
  unsigned int blue = 0;
  led_mode mode = ledInactive;
};

bool mouseIsInit = false;
program_mode CUR_MODE = progInitMode;

const int MAX_LED_STATES = 2;
led_state LED_STATES[MAX_LED_STATES];

const int NUM_BUTTONS = 6;
one_button BUTTONS[NUM_BUTTONS];

time_t NEXT_MOUSE_MOVE = 0;
time_t NEXT_MODE_END = 0;  // Will be changed in setup()
bool   END_WITH_SLEEP = false;

/*
 * -----------------------------------------------------------------------
 */

void get_settings(bool force_reset = false) {
  EEPROM.get(0, ourSettings);

  if ((ourSettings.magicNum != CUR_MAGIC_NUM) || (force_reset)) {
    if (force_reset) 
        debugout(1, F("Reseting EEPROM to defaults"));
    else
        debugout(1, F("Invalid MAGIC_NUMBER, setting EEPROM to defaults"));

    ourSettings.magicNum = CUR_MAGIC_NUM;
    ourSettings.mouseMoveRate = DEF_MOUSE_MOVE_RATE;
    ourSettings.numPixelsToMove = DEF_NUM_PIXELS;
    ourSettings.yellowButtonDur = DEF_YELLOW_DUR;
    ourSettings.greenButtonDur = DEF_GREEN_DUR;
    ourSettings.blueButtonOffTime = DEF_BLUE_TIME_OFF; 
    ourSettings.initialButton = DEF_INITIAL_BUTTON;
    ourSettings.debugLevel = 3;
    save_settings();
  }
}


void save_settings() {
  EEPROM.put(0, ourSettings);
}

/*
 * -----------------------------------------------------------------------
 */

template<typename T> void debugout(int level, const T *s) {
  if (ourSettings.debugLevel >= level) {
    show_cur_time(-2); 
    Serial.println(s);
  }
}

void debugout2(int level, const char *s, const char *s2) {
  if (ourSettings.debugLevel >= level) {
    show_cur_time(-2); 
    Serial.print(s);
    Serial.print(" ");
    Serial.println(s2);
  }
}
template<typename T> void debugout(int level, const T *s, const uint32_t thisTime) {
  if (ourSettings.debugLevel >= level) {
    
    show_cur_time(-2); 
    Serial.print(s);
    Serial.print(" ");
    show_time(thisTime);
  }
}

/*
 * -----------------------------------------------------------------------
 */
void initButton(int num, int pinNum, button_mode mode) {
    pinMode(pinNum, INPUT_PULLUP);
    
    BUTTONS[num].pinNum = pinNum;
    BUTTONS[num].currState = digitalRead(pinNum);
    BUTTONS[num].debouncePeriod = 0; 
    BUTTONS[num].mode = mode;
    BUTTONS[num].fakeInput = false;
}

bool initMouse() {
  bool oldVal = mouseIsInit;

  if (!mouseIsInit) {
    Mouse.begin();
    System.begin();
    // Keyboard.begin();
    mouseIsInit = true;
  }
  return oldVal;
}
/*
 * -------------------------------------------------------
 */
void setInitialMode() {
  if (ourSettings.initialButton == 'g') 
    BUTTONS[0].fakeInput = true;
   
  if (ourSettings.initialButton == 'y') 
     BUTTONS[1].fakeInput = true;

  if (ourSettings.initialButton == 'b') 
     BUTTONS[3].fakeInput = true;
  
}

/*
 * -------------------------------------------------------
 */

unsigned int zero_to_255(unsigned int n) {
  if (n < 0) n = 0;
  if (n > 255) n = 255;

  return n;
}


void set_leds(unsigned int red, unsigned int green, unsigned int blue, 
              led_mode mode = ledSolid, unsigned int group = 0) {

      red   = zero_to_255(red);
      green = zero_to_255(green);
      blue  = zero_to_255(blue);

      // Override mode if the color is black
      if ((red == 0) && (green == 0) && (blue == 0)) mode = ledInactive;
      
      if (group < 0) group = 0;
      if (group >= MAX_LED_STATES) group = MAX_LED_STATES -1;

      LED_STATES[group].red = red;
      LED_STATES[group].blue = blue;
      LED_STATES[group].green = green;
      LED_STATES[group].mode = mode;
      }

void clear_leds(unsigned group = 0) {
  if (group < 0) group = 0;
  if (group > 1) group = 1;
      
  LED_STATES[group].mode = ledInactive;
}

void breath_leds(unsigned int & red, unsigned int & green, unsigned int & blue) {
  int desired;
  int shift;
  
  desired = (millis() >> 10) % 5;

  if (desired != 0) {
    if ((desired == 1) || (desired == 4)) { shift = 1; }
    if ((desired == 2) || (desired == 3)) { shift = 2; }
    
    red    = red   >> shift;
    green  = green >> shift;
    blue   = blue  >> shift;
  }
}

void choose_one_mode(int mode_count, unsigned int & red, unsigned int & green, unsigned int & blue) {
  int count = 0;
  int desired;
  
  if (mode_count < 1) mode_count = 1;   // Safety check: should not happen
  
  desired = (millis() >> 9) % mode_count;
  
  // Find that desired index (ignore any modes that are in ledInactive state)
  for (int i=0;i<MAX_LED_STATES;i++) {
    
    if (LED_STATES[i].mode != ledInactive) {
      if (count == desired) {
        red   = LED_STATES[i].red;
        green = LED_STATES[i].green;
        blue  = LED_STATES[i].blue;
      }
      
      count++;
    }
  }
}

void get_desired_led_state(unsigned int & red, unsigned int & green, unsigned int & blue) {
  led_mode desired_mode = ledInactive;
  int mode_count = 0;
  red = green = blue = 0;
  int curr_sec;
  
  for(int i=0;i<MAX_LED_STATES;i++) {
    
  if (LED_STATES[i].mode != ledInactive) {
    red   += LED_STATES[i].red;
    green += LED_STATES[i].green;
    blue  += LED_STATES[i].blue;

    desired_mode = LED_STATES[i].mode;
    
    mode_count++;
    }
    
  }

  if (mode_count > 1) { desired_mode = ledRotate; }
  
  switch (desired_mode) {
    
    case ledRotate : choose_one_mode(mode_count, red, green, blue);
                     break;
                     
    case ledBreath:  breath_leds(red, green, blue); break;
  }

  red   = zero_to_255(red);
  green = zero_to_255(green);
  blue  = zero_to_255(blue);


}

void check_led_state() {
  static unsigned int curr_red;
  static unsigned int curr_blue;
  static unsigned int curr_green;

  unsigned int desired_red;
  unsigned int desired_blue;
  unsigned int desired_green;

  get_desired_led_state(desired_red, desired_green, desired_blue);

  if (desired_red != curr_red) {
    SoftPWMSet(RED_LED_PIN, desired_red);
    curr_red = desired_red;
  }

    if (desired_green != curr_green) {
    SoftPWMSet(GREEN_LED_PIN, desired_green);
    curr_green = desired_green;
  }

    if (desired_blue != curr_blue) {
    SoftPWMSet(BLUE_LED_PIN, desired_blue);
    curr_blue = desired_blue;
  }
}
/*
 * -------------------------------------------------------
 */
bool get_input(int num) {
  bool newState;
  bool didRead = false;
  
  // Are we in a quiet period?
  if (BUTTONS[num].debouncePeriod == 0) {
      newState = !digitalRead(BUTTONS[num].pinNum); // Use inverse since PULLUP
      
      if (newState != BUTTONS[num].currState) {
           BUTTONS[num].currState = newState;
           BUTTONS[num].debouncePeriod = 100;
           didRead = true;
      }
  }
        
  else 
     newState = BUTTONS[num].currState;

   // Take into account the "one shot" mode
   if (BUTTONS[num].mode == btnOnce)
      newState = newState & didRead;

   // Ignore button presses during initialization
   
   if (CUR_MODE == progInitMode) { 
      newState = false; 
      BUTTONS[num].debouncePeriod = 1000;  // Wait 1 second (after init) 
      }

   if (BUTTONS[num].fakeInput) { newState = true; }
   
   return newState;
        
}


void do_set_time(char *s) {
  int hh,mm;
  int v;
  int i = 0;
  
  bool ok = true;

  // Format [h]h:mm
  
  // HH
  if (isdigit(s[i])) { hh = (s[i++] - '0'); } else { ok = false; }
  if (ok && (isdigit(s[i]))) { hh = hh * 10 + (s[i++] - '0'); } 

  // :
  if (s[i] == ':') { i++; } else { ok = false; }

  // MM
  if (isdigit(s[i])) { mm = (s[i++] - '0'); } else { ok = false; }
  if (ok && (isdigit(s[i]))) { mm = mm * 10 + (s[i++] - '0'); } else { ok = false; }

  // Do some error checking on the range of hours and minutes
  if (hh > 23) { ok = false; }
  if (mm > 59) { ok = false; }
  
  if (ok) {
    Serial.print(F("Setting time to "));
    Serial.print(hh);
    Serial.print(":");
    Serial.println(mm);

#ifdef RTC
    myClock.setClockMode(false);  // Set to 24hr mode
    
    myClock.setHour(hh);
    myClock.setMinute(mm);
    myClock.setSecond(0);
    
    // Ignore the date part
    
#endif

    setTime(hh,mm,0,0,1,0);  // Ignore the date part
  }
  else {
    Serial.println(F("Error! Invalid time - Clock not changed"));
  }
}



/*
 * -----------------------------------------------------------------------------------------------------
 */

void do_show_help() {
  Serial.println(F("Commands:"));

  Serial.println(F("d [<#>]                    - View/set the debug level"));
  Serial.println(F("g [<#|hh:mm:ss>]           - View/set the duration of the green button"));
  Serial.println(F("h                          - Show this page"));
  Serial.println(F("i [b|g|r|y]                - Set the initial (startup) mode"));
  Serial.println(F("l [<r> <g> <b> [<blink#>]] - View/set the LED manually")); 
  Serial.println(F("m [hh:mm:ss]               - View/set how often the mouse moves")); 
  Serial.println(F("n [<#>]                    - View/set the number of pixels the mouse moves")); 
  Serial.println(F("o [<hh:mm:ss>]             - View/set the blue button off time"));
  Serial.println(F("r!                         - Reset all settings (but not the clock)"));
  Serial.println(F("s                          - Show all settings"));
  Serial.println(F("t [<hh:mm:ss>]             - View/set the internal clock"));  
  Serial.println(F("v                          - Show firmware version number"));
  Serial.println(F("y [<#|hh:mm:ss>]           - View/set the duration of the yellow button"));

}

bool get_next_word(char *s, char * & currWord, char * & restOfString) {
  bool hasMore = true;
  
  // Zoom past blank spaces
  while ((s != NULL) && (s[0] != '\0') && isspace(s[0])) {
    s++;
  }

  currWord = s;
 
 // End of string?  (or NULL passed?)  Just return pointers to the NUL char
 if ((s == NULL) || (s[0] == '\0')) {
  restOfString = s;
  hasMore = false;
 }
 else {
  // Zoom past non-blank spaces
  while ((s[0] != '\0') && !isspace(s[0])) {
    s++;
  }

  /*
   * Space between words?  Convert to a NUL and continue
   */
  if (s[0] != '\0') {
    s[0] = '\0'; // 
    s++;
  }
  
  // Zoom past blank spaces
  while ((s[0] != '\0') && isspace(s[0])) {
    s++;
  }
  
  restOfString = s;

  // Last word or not?
  hasMore = (s[0] != '\0');
 }
 
 return hasMore;
}

 /*
  * -----------------------------------------------------------------------------------------------------
  */
  
template<typename T> bool parse_str(const char *s, T &val, unsigned int num_mult = 1) {
  bool ok = true;
  int i = 0;
  char ch;
  T result,result3, result2;
  int num_colons = 0;
  result3 = result2 = result = 0;
  
  while ((s[i] != '\0') && (ok)) {
    ch = s[i++];

    if (isspace(ch)) { 
      /* do nothing, ignore */
    }
    else if (isdigit(ch)) {
        result = result * 10 + (ch - '0');
      }
    else if (ch == ':') {
      if (num_colons++ < 2) {
        result3 = result2;
        result2 = result;
        result = 0;
      }
      else 
         ok = false;  /* more than two colons is not allowed! */
      }
    else {
      /* Everything else is bad */
      ok = false;
    }
    


  }

  if (ok)
     if (num_colons == 1)
        val = result2 * 60 + result;
     else if (num_colons == 2)
        val = result3 * 3600 + result2 * 60 + result; 
     else if (num_mult > 1)
        val = val * num_mult;
     else
        val = result;
     
  return ok;
}


void do_show_set_init_mode(const char *arg, const char *prompt) {
  if (arg[0] == '\0') {
    Serial.print(prompt);
    Serial.print(F(" = ")); 
    Serial.println(ourSettings.initialButton);
    }
  else {
    if ((arg[0] == 'b') || (arg[0] == 'y') || (arg[0] == 'g') || (arg[0] == 'r')) {
      ourSettings.initialButton = arg[0];
      save_settings();
    }
    else {
      Serial.println(F("Invalid initial mode - must be b,g,r, or y"));
    }
  }
}

void show_time(uint32_t result) {
  uint32_t result2;
  uint32_t result3;

  result3 = result / 3600;
  result = result % 3600;
              
  result2 = result / 60;
  result = result % 60;

  if (result3 < 10) Serial.print("0");
  Serial.print(result3);
  Serial.print(":");
  if (result2 < 10) Serial.print("0");
                   
  Serial.print(result2, DEC);
  Serial.print(":");
  
  if (result < 10) Serial.print("0");
  
  Serial.println(result);
}

template<typename T> void do_show_set_setting(T &setting, const char *arg, 
const char *prompt, bool show_as_time = false) {
            T result;
            
            /*
             * Show or set?  No argument = show
             */
            if (arg[0] == '\0') {
              Serial.print(prompt);
              Serial.print(F(" = ")); 
              
              if (show_as_time) 
                show_time(setting);
              else
                Serial.println(setting, DEC);
              }
              
            /*
             * Set!
             */
            else {
             if (parse_str(arg, result, show_as_time ? 60 : 1)) {
                setting = result;  
                save_settings();
             }
             else {
              Serial.println("Invalid value for setting.");
             }
              
          }
     }
 

 template<typename T1, typename T2> void show_setting(const T1 *s, T2 val, bool show_as_time = false) {
  int len = 0;
  int i = 0;
  int j;
  int hh,mm,ss;
  bool is12hr,isPM;

  T2 result1, result2;
  
  Serial.print(s);
  // Find the length
  while (s[i++] != '\0'); 

  for(j=i;j<40;j++) Serial.print(" ");
  Serial.print(": ");

  if (!show_as_time)
    Serial.println(val);
  else {
      show_time(val);
    }
 }

void show_t_setting(const char *s, time_t when) {
 uint32_t hh, mm, ss;
 uint32_t then;
 
  if (when == 0) {
   show_setting(s,F("Never (currently Idle)")); 
  }
  else {
    hh = hour(when);
    mm = minute(when);
    ss = second(when);
    

    then = hh * 3600 + mm * 60 + ss;
    //Serial.print(hh); Serial.print(":"); Serial.print(mm); Serial.print(":"); Serial.print(ss); Serial.print("="); Serial.println(then);
    
    show_setting(s, then, true);
    }
  }

/*
 * ---------------------------------
 */

void do_show_set_led(char *s) {
  unsigned int val;
  char *rstr, *bstr, *gstr, *mstr;
  unsigned int rint, bint, gint, mint;
  bool ok = true;
  
  if (s[0] == '\0') {
    get_desired_led_state(rint, gint, bint);
    
    Serial.print(F("LED state is R="));
    Serial.print(rint);
    Serial.print(F(", G="));
    Serial.print(gint);
    Serial.print(F(", B="));
    Serial.println(bint);
    }
  else {
     get_next_word(s, rstr, s);
     get_next_word(s, gstr, s);
     get_next_word(s, bstr, s);
     get_next_word(s, mstr, s);
     
     if ((rstr == NULL) || (gstr == NULL) || (bstr == NULL)) { ok = false; }
     if (ok) {
      ok &= parse_str(rstr, rint);
      ok &= parse_str(gstr, gint);
      ok &= parse_str(bstr, bint);
     }

     if (mstr != NULL) {
      parse_str(mstr, mint);
     }
     else {
          mint = (int) ledSolid;   
        }
     
     rint = zero_to_255(rint);
     gint = zero_to_255(gint);
     bint = zero_to_255(bint);

     if (ok) {
        Serial.print(F("Setting the LEDs to R="));
        Serial.print(rint);
        Serial.print(F(", G="));
        Serial.print(gint);
        Serial.print(F(", B="));
        Serial.print(bint);
        
        if (mint != (int) ledSolid) {
          Serial.print(F(", M="));
          Serial.print(mint);
        }
        
        Serial.println("");
        
        clear_leds(1);
        set_leds(rint, gint, bint, mint);
     }
     else {
      Serial.println(F("Usage: led <r> <g> <b>"));
     }
  }
}


/*
 * ----------------------------------------
 */
void do_reset(const char *arg) {
  if ((arg[0] == '!') && (arg[1] == '\0')) {
    
  get_settings(true); 

  CUR_MODE = progDoNothingMode;
  NEXT_MOUSE_MOVE = 0;

  Serial.println(F("Settings reset to default (clock unchanged)")); 
  
  }
 else {
  Serial.println(F("To reset, you must have the ! at the end of the command"));
 }
}

void show_pinout() {
  for(int i=0;i<NUM_BUTTONS;i++){
    Serial.print(F("Button #"));
    Serial.print(i);
    Serial.print(F(" uses pin "));
    Serial.print(BUTTONS[i].pinNum);
    Serial.print(F(" (current state="));
    Serial.print(digitalRead(BUTTONS[i].pinNum));
    Serial.println(F(")"));
  }
}
 /*
  * -----------------------------------------------------------------------------------------------------
  */

void do_command(char *s, int s_len) {
  char cmd = s[0];
  char *arg;
  int i = 1;
  
  debugout2(3, "Cmd: ", s);

  // Skip any space between command and the rest
  while ((i < s_len) && isspace(s[i])) { i++; }
  
  // Create a pointer to the remaining word
  arg = &s[i];

  switch (cmd) {
    case 'c' : show_pinout(); break;
    
    case 'd' : do_show_set_setting(ourSettings.debugLevel,arg, "Debug Level"); break;

    case 'g' : do_show_set_setting(ourSettings.greenButtonDur, arg, "Green Button Dur",true); break;

    case 'h' : 
    case '?' : do_show_help(); break;

    case 'i' : do_show_set_init_mode(arg, "Initial Mode"); break;

    case 'l' : do_show_set_led(arg); break;
    
    case 'm' : do_show_set_setting(ourSettings.mouseMoveRate, arg, "Mouse Move Rate", true); break;

    case 'n' : do_show_set_setting(ourSettings.numPixelsToMove,arg, "Num Pixels to Move"); break;

    case 'o' : do_show_set_setting(ourSettings.blueButtonOffTime,arg, "Blue Button Off Time",true); break;

    case 'r'  : do_reset(arg); break;

    case 's'  : show_state(); break;
    
    case 't' : if (arg[0] != '\0') do_set_time(arg); else show_cur_time(0);  break;
    
    case 'v' : Serial.print(F("Version ")); Serial.println(VERSION); 
               Serial.print(F("Compiled on " __DATE__ " " __TIME__));
               break;

    case 'y' : do_show_set_setting(ourSettings.yellowButtonDur, arg, "Yellow Button Dur", true); break;

    default  : Serial.println(F("Invalid command - press h for help, or s to view all settings."));
  }
}

void readSerial() {
  static char serial_buffer[21];
  static int serial_len = 0;

  char ch;
  
  while (Serial.available() > 0) {
    ch = Serial.read();

    if ((ch == '\r') || (ch == '\n')) {
      serial_buffer[serial_len] = '\0';
      
      if (serial_len > 0) {
        Serial.println("");
        do_command(serial_buffer,serial_len);
        serial_len = 0;
        }
        
      Serial.println("");
      Serial.print("> ");
    }

  /*
   *  Handle the BS or DEL key
   */
   if (((ch == '\010') || (ch == '\177')) && (serial_len > 0)) {
    Serial.print("\010 \010");
    serial_len--;
   }

   if ((isprint(ch)) && (serial_len < 20)) {
    
    // Don't allow a leading space
    if ((ch != ' ') || (serial_len > 0)) {
      Serial.print(ch);
      serial_buffer[serial_len++] = ch;
      }
   }
   
  }
}

/*
 * Handle the button (de)boucning
 */
void handle_debounce() {
    static unsigned long last_millis = 0;
     
    if (millis() != last_millis) {
      for(int i=0;i<NUM_BUTTONS;i++) {
        if (BUTTONS[i].debouncePeriod > 0) 
           BUTTONS[i].debouncePeriod--;
      }
    }
}

/*
 * See if the mode should end - if so, go into "idle"
 * (also handle special case for Init->Idle)
 */
void check_mode_end() {

    if ((NEXT_MODE_END != 0) && (NEXT_MODE_END <= now())) {
          NEXT_MODE_END = 0;
          
          if (CUR_MODE == progInitMode) { 
            debugout(1,F("Enabling USB Mouse Mode"));
            initMouse(); 
            setInitialMode();
          }
          else
            debugout(1,F("Stopped"));
            
            if (END_WITH_SLEEP) {
              debugout(1,F("Going to Sleep"));
              END_WITH_SLEEP = false;
              sleep_now();
            }
    }
    
     if (NEXT_MODE_END == 0) {
          CUR_MODE = progDoNothingMode;
          NEXT_MOUSE_MOVE = 0;
          END_WITH_SLEEP = false;

    }
}
/*
 * Is it time to move the mouse?  If so, then do it...
 */
void check_mouse_move() {
  static bool move_mouse_right = true;
  
     if ((NEXT_MOUSE_MOVE > 0) && (ourSettings.mouseMoveRate > 0) && (NEXT_MOUSE_MOVE <= now())) {
            debugout(3,F("Move mouse"));
            
            Mouse.move( move_mouse_right ? 
              ourSettings.numPixelsToMove : -ourSettings.numPixelsToMove, 0, 0);
            
            move_mouse_right = !move_mouse_right;
            NEXT_MOUSE_MOVE = now() + ourSettings.mouseMoveRate;   
          }
     }

#ifdef RTC

time_t ds3231_time() {
  bool is12hr, isPMtime;
  bool century;
  
  time_t newTime;
  tmElements_t rawTime;

  rawTime.Second = myClock.getSecond();
  rawTime.Minute = myClock.getMinute();
  rawTime.Hour   = myClock.getHour(is12hr, isPMtime);
  rawTime.Wday   = myClock.getDoW();
  rawTime.Day    = myClock.getDate();
  rawTime.Month  = myClock.getMonth(century);
  rawTime.Year   = myClock.getYear();

  newTime = makeTime(rawTime);
  
  return newTime;
 }

#endif

time_t compute_end_time(time_t seconds_of_day) {
  time_t right_now, next_time;

  right_now = now();
  
  next_time = right_now - elapsedSecsToday(right_now);
  next_time += seconds_of_day;

  /*
   * Assume tomorrow if in the past by more than 10 minutes)
   */
  if (next_time < (right_now + 10*60)) {
    next_time += SECS_PER_DAY;
  }

  return next_time;
}

void show_cur_time(int extraspaces) {
    time_t t = now();
    uint32_t h,m,s;
    
    h = hour(t);
    m = minute(t);
    s = second(t);

    /* 
     *  Don't display the title portion if extraspaces is negative
     */
    if (extraspaces >= 0) {
      Serial.print(F("Current time "));
      for(int i=0;i<extraspaces;i++) Serial.print(" ");
      Serial.print(F(": "));
    }
    
    if (h < 10) { Serial.print("0"); }
    Serial.print(h, DEC);
    Serial.print(":");
        
    if (m < 10) { Serial.print("0"); }
    Serial.print(m, DEC);
    Serial.print(":");

    if (s < 10) { Serial.print("0"); }
    Serial.print(s, DEC);

    if (extraspaces >= 0)
      Serial.println("");

    if (extraspaces <= -2) 
      Serial.print(" - ");
      
}


void show_state() {
    Serial.print(F("Current Mode                           : "));

    switch (CUR_MODE) {
      case progInitMode :      Serial.println(F("Initializing")); break;
      case progDoNothingMode : Serial.println(F("Idle")); break;
      case progMoveDur :       Serial.println(F("Move-Duration")); break;
      case progMoveUntil :     Serial.println(F("Move-Until")); break;
      
      default :                Serial.println(F("???")); break;
    }

    show_setting("Power On Button", ourSettings.initialButton);
    
    show_t_setting("Time when mode ends   (hh:mm:ss)", NEXT_MODE_END);
    show_t_setting("Time when mouse moves (hh:mm:ss)", NEXT_MOUSE_MOVE);
    show_setting("System Sleep at mode end?",          END_WITH_SLEEP);
    Serial.println("");
    
    show_setting("Yellow Button Duration (hh:mm:ss) [y]", ourSettings.yellowButtonDur, true);
    show_setting("Green Button Duration  (hh:mm:ss) [g]", ourSettings.greenButtonDur, true);
    show_setting("Blue Button Off Time   (hh:mm:ss) [b]", ourSettings.blueButtonOffTime, true);

    show_setting("Mouse Move Rate (mm:ss) [m]", ourSettings.mouseMoveRate, true);
    show_setting("Num Pixels to Move [n]", ourSettings.numPixelsToMove);
    show_setting("Debug Level [d]", ourSettings.debugLevel);
    
    Serial.println("");
    
    show_cur_time(26);


}

void setup() {
  // put your setup code here, to run once:
  initButton(0, GREEN_BUTTON_PIN,  btnOnce);
  initButton(1, YELLOW_BUTTON_PIN, btnOnce);
  initButton(2, RED_BUTTON_PIN,    btnOnce);
  initButton(3, BLUE_BUTTON_PIN,   btnOnce);
  initButton(4, BLACK_BUTTON_PIN,   btnOnce);
  initButton(5, SHIFT_KEY,         btnOnce);

  //pinMode(RED_LED_PIN, OUTPUT);
  //pinMode(GREEN_LED_PIN, OUTPUT);
  //pinMode(BLUE_LED_PIN, OUTPUT);
  SoftPWMBegin();

  SoftPWMSet(RED_LED_PIN,  0);
  SoftPWMSet(GREEN_LED_PIN,0);
  SoftPWMSet(BLUE_LED_PIN, 0);


  SoftPWMSetFadeTime(ALL, 1000, 1000);
  
  Serial.begin(9600);

  get_settings();
  
#ifdef RTC
  Wire.begin();
  
  myClock.setClockMode(false); // Set to 24-hr mode
  
  setSyncProvider(ds3231_time);
  setSyncInterval(60);
#endif

  NEXT_MODE_END = now() + 15;

}

bool is_shifted() {
  return !digitalRead(SHIFT_KEY);
}

void sleep_now() {
  System.write(SYSTEM_SLEEP);
}

void wake_now() {
  System.write(SYSTEM_WAKE_UP);
}

void check_buttons() {
  
  if (get_input(0)) {
    wake_now();
    debugout(1,F("Go - Green, duration ="), ourSettings.greenButtonDur);
    CUR_MODE = progMoveDur;
    NEXT_MODE_END = now() + ourSettings.greenButtonDur;
    NEXT_MOUSE_MOVE = now() + ourSettings.mouseMoveRate;
    END_WITH_SLEEP = is_shifted();    // Do we sleep when the timer goes off?

    set_leds(  0, 16, 0, END_WITH_SLEEP ? ledBreath : ledSolid);
  }

  if (get_input(1)) {
    wake_now();
    debugout(1,F("Go - Yellow, duration ="),ourSettings.yellowButtonDur);
    CUR_MODE = progMoveDur;
    NEXT_MODE_END = now() + ourSettings.yellowButtonDur;
    NEXT_MOUSE_MOVE = now() + ourSettings.mouseMoveRate;  
    END_WITH_SLEEP = is_shifted();    // Do we sleep when the timer goes off?

    set_leds(  0, 16, 0, END_WITH_SLEEP ? ledBreath : ledSolid);
  }

  if (get_input(2)) {
    debugout(1,F("Stop"));
    
    CUR_MODE = progDoNothingMode;
    NEXT_MODE_END = 0;
    NEXT_MOUSE_MOVE = 0;
    END_WITH_SLEEP = false;

    clear_leds(0);
  }

  if (get_input(3)) { 
    wake_now();
    debugout(1,F("Go - Blue, until ="), ourSettings.blueButtonOffTime);
   
    CUR_MODE = progMoveUntil;
    NEXT_MODE_END = compute_end_time( ourSettings.blueButtonOffTime );
    //NEXT_MODE_END = now() + ourSettings.yellowButtonDur;
    NEXT_MOUSE_MOVE = now() + ourSettings.mouseMoveRate;
    END_WITH_SLEEP = is_shifted();    // Do we sleep when the timer goes off?

    set_leds(  16, 0, 0, END_WITH_SLEEP ? ledBreath : ledSolid);
  }

  if (get_input(4)) {
    if (is_shifted()) {
      debugout(1,F("Immediate Wake"));
      wake_now(); 
      }
    else {
      debugout(1,F("Immediate Sleep"));
      sleep_now();
      }
  }

  if (get_input(5)) {
    debugout(1,F("Shift Key (no action)"));
    }

  if (is_shifted())
    set_leds(0, 0, 16, ledSolid, 1);
  else 
    clear_leds(1);
  
  
}

void loop() {
  check_mode_end();
  readSerial();
  
  if (CUR_MODE != progInitMode) {
    check_buttons(); 
    handle_debounce();
    check_mouse_move();
    check_led_state();
  }
}
