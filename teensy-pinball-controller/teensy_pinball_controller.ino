#include <Wire.h>
/* include <i2c_t3.h> */
#include <ADC.h>

#if defined(__MK20DX256__)

#define NEEDKEYROUTINES 1
#define LAYOUT_US_ENGLISH 1

#include <usb_keyboard.h>
#include <usb_keyboard.c>

#define KEYTYPE uint16_t

#else

#define KEYTYPE char

#endif


#ifndef PITCHES
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define PITCHES 1
#endif

const int MAXCMDLINE = 255;
const int MAXCMDARGS = 10;
const int MAXKEYMODES = 6;              
const int MAXJOYSTICKAXIS = 3;
const int NUMJOYSTICKSAMPLES = 3;
const int LED_OUTPUT_PIN = 13;

const int SMALL_ANALOG_VALUE = 5;
const int MEDIUM_ANALOG_VALUE = 20;

const long DEBOUNCE_TIME_MS = 20L;
const unsigned int DEFAULT_TONE_DURATION = 250;

bool ENABLE_KEYBOARD = true;
bool ENABLE_SERIAL_DEBUG = false;
bool ENABLE_NUDGE = false;
bool DISABLE_EVERYTHING = false;

const int ANALOG_INVALID_DATA = -9999999;

// Boot with Right White Flipper pressed for SERIAL_DEBUG (and no KEYBOARD)
// Boot with Right Red Flipper pressed for NORUN mode

const int ACCEL_X_PIN = A2;
const int ACCEL_Y_PIN = A3;

const int SPEAKER_PIN = 20;

const int B_LFLIP_WHITE = 0;
const int B_LFLIP_RED = 1;
const int B_START = 2;
const int B_GREEN = 3;
const int B_LAUNCH = 4;
const int B_RFLIP_WHITE = 5;
const int B_RFLIP_RED = 6;

const int R_RED = 7;    
const int R_ORANGE = 8;
const int R_YELLOW = 9;
const int R_GREEN = 10;
const int R_BLUE = 11;
const int R_WHITE = 12;

const int ARRAYSIZE = 13;

/*
 * Define 4 keys to be shifts so buttons can have multiple functions
 */
 
const int SHIFT1 =  B_LFLIP_WHITE;
const int SHIFT2 =  B_LFLIP_RED;
const int SHIFT3 =  B_RFLIP_WHITE;
const int SHIFT4 =  B_RFLIP_RED;

/*
 *==================================================================================================================
 */

enum buttonModes { 
        btnReady, btnDebounce, btnForceReleased };

enum buttonType { 
        btnUndefined, btnKeypress, btnJoystick, btnFunction, btnSound };

enum buttonFuncts { 
      funcNewMode=0x1, 
      funcAccelOn=0x2, 
      funcAccelOff=0x4, 
      funcAccelToggle=0x8, 
      funcRecalibrate=0x10 };

struct keyOut {
        buttonType type;
        
        union { 
                KEYTYPE keycode;
                int num; 
                buttonFuncts func; 
        };
        
        union {
          int auxnum;
          };
          
        int maxTime;
};

struct keyInfo {
        const __FlashStringHelper *keyName = NULL;
        const char *altname = NULL;
        int group;
};

struct jsInfoRec {
        int numerator = 1;
        int divisor = 1;
        int deadzone = 2;
        int calibrationStep = 0;        // 0 = not started, 1..4 = ignoring first values, 5..8 collect, 9 = done!
        float calibrationOffset = ANALOG_INVALID_DATA;
        long lastval = 0;
        long hist[NUMJOYSTICKSAMPLES];
};

/*
 *==================================================================================================================
 */

volatile bool buttonPressed[ARRAYSIZE];          // Current state
volatile buttonModes buttonMode[ARRAYSIZE];     // Are we in debounce mode (aka should we ignore input right now)
volatile unsigned long lastPressTime[ARRAYSIZE];   // Used to ignore extra bounces during this period

keyOut keyArray[ARRAYSIZE][MAXKEYMODES];
keyInfo keyData[ARRAYSIZE];

jsInfoRec jsInfo[MAXJOYSTICKAXIS];

bool joyChanged = false;
unsigned long joyNextReport = 0;

int scanGroup = 0;
int maxScanGroup = 0;
int keyMode = 0;        // 0=TPA normal, 1=TPA w/ nudge
int maxMode = 0;

ADC *adc = new ADC();
volatile long curr_accel_x_value = ANALOG_INVALID_DATA;
volatile long curr_accel_y_value = ANALOG_INVALID_DATA;
volatile unsigned int x_count = 0;
volatile unsigned int y_count = 0;
/*
 *==================================================================================================================
 */


// the i2c address
#define VCNL4000_ADDRESS 0x13
// commands and constants
#define VCNL4000_COMMAND 0x80
#define VCNL4000_PRODUCTID 0x81
#define VCNL4000_IRLED 0x83
#define VCNL4000_AMBIENTPARAMETER 0x84
#define VCNL4000_AMBIENTDATA 0x85
#define VCNL4000_PROXIMITYDATA 0x87
#define VCNL4000_SIGNALFREQ 0x89
#define VCNL4000_PROXINITYADJUST 0x8A
#define VCNL4000_3M125 0
#define VCNL4000_1M5625 1
#define VCNL4000_781K25 2
#define VCNL4000_390K625 3
#define VCNL4000_MEASUREAMBIENT 0x10
#define VCNL4000_MEASUREPROXIMITY 0x08
#define VCNL4000_AMBIENTREADY 0x40
#define VCNL4000_PROXIMITYREADY 0x20


/*
 *------------------------------------------------------------------------------------
 *
 *        I2C stuff
 *
 *------------------------------------------------------------------------------------
 */
 
 
// Read 1 byte from the VCNL4000 at 'address'
uint8_t read8(uint8_t address)
{
        // uint8_t data;

        write0(address);

        delayMicroseconds(170); // delay required

        Wire.requestFrom(VCNL4000_ADDRESS, 1);
        while(!Wire.available());

        return Wire.read();

}


// Read 2 byte from the VCNL4000 at 'address'
uint16_t read16(uint8_t address)
{
        uint16_t data;

        Wire.beginTransmission(VCNL4000_ADDRESS);
        Wire.write(address);
        Wire.endTransmission();

        Wire.requestFrom(VCNL4000_ADDRESS, 2);
        while(!Wire.available());

        data = Wire.read();
        data <<= 8;
        while(!Wire.available());
        data |= Wire.read();

        return data;
}

// write 1 byte
void write8(uint8_t address, uint8_t data)
{
        Wire.beginTransmission(VCNL4000_ADDRESS);
        Wire.write(address);
        Wire.write(data);
        Wire.endTransmission();
}

void write0(uint8_t address) {
        Wire.beginTransmission(VCNL4000_ADDRESS);
        Wire.write(address);
        Wire.endTransmission();
}


/*
 *=----------------------------------------------------------------------------------
 */
 
 /*
  *  timeExpired - see if a future time has occurred (handles wrap of longs)
  */
  
bool timeExpired(unsigned long currTime, unsigned long endTime) {
        return ((long) (currTime - endTime) >= 0);
}

/*
 *------------------------------------------------------------------------------------
 *
 *        Teensy Keyboard
 *
 *------------------------------------------------------------------------------------
 */


#ifdef NEEDKEYROUTINES

bool keycode_is_modifier(uint16_t keycode) {      
        return ((keycode & 0x8000) != 0);
}

uint16_t ascii_to_keycode(uint16_t ch) {
        
        if ((ch >= 20) && (ch <= 127)) { 
                ch = tolower(ch);
                ch = keycodes_ascii[ch - 32] | 0x4000;
                }
                
        return ch;
}

char keycode_to_ascii(uint16_t keycode) {
        char ch = '\0';
        
        keycode &= ~0x4000;
        
        for(int i=0;i<=127-32;i++) { if (keycodes_ascii[i] == keycode) { ch = i + 32; } }
        
        return ch;
        }
 
void key_press(uint16_t keycode) {
        uint8_t lsb = keycode & 0xff;
        int i = 0;
        
        if (keycode_is_modifier(keycode)) {
                keyboard_modifier_keys |= lsb;
                }
        else {
                // Find an empty slot (or a slot with the same key)
                while((keyboard_keys[i] != 0) && (keyboard_keys[i] != keycode) && (i < 5)) { i++; }

                // Note: if the list is full, the last entry gets overwritten so this key is always sent!
                keyboard_keys[i] = lsb;
                }
                
        usb_keyboard_send();
}

void key_release(uint16_t keycode) {
        uint8_t lsb = keycode & 0xff;
        
        if (keycode_is_modifier(keycode)) {
                keyboard_modifier_keys &= ~lsb;
                 Serial.print("Modifier is now "); printHexByte(keyboard_modifier_keys); Serial.println("");
                }
        else {
                for(int i=0;i<=5;i++) {
                        if (keyboard_keys[i] == lsb) { keyboard_keys[i] = 0; }
                }
        }
        
        usb_keyboard_send();
}

#endif

/*
 *------------------------------------------------------------------------------------
 *
 *        Command-line stuff
 *
 *------------------------------------------------------------------------------------
 */


int hexcharToInt(char ch) {
        int result = -1;

        if ((ch >= '0') && (ch <= '9')) {
                result = ch - '0';
        }

        if ((ch >= 'A') && (ch <= 'F')) {
                result = ch - 'A' + 10;
        }

        if ((ch  >= 'a') && (ch <= 'f')) {
                result = ch - 'a' + 10;
        }

        return result;
}

/*
 * Convert a string (representing a hex numebr) into a integer
 *
 *	Returns -1 if an error has occurred
 */

uint16_t hexstrToInt(const char *s) {
        int result = 0;
        int i = 0;
        int temp;

        /*
	 * Iterate until the string is complete, or an error is detected
         	 */
        while ((s[i] != '\0') && (result != -1)) {
                // Ignore a leading $ (but error out if $ is elsewhere)
                if ((s[i] != '$') || (i != 0)) {
                        temp = hexcharToInt(s[i]);
                        if (temp == -1) { 
                                result = -1; 
                        }
                        else {
                                result = (result << 4) + temp;	// Multiply by 16 and add new digit
                        }
                }

                i++;
        }

        return result;
}

int parsenum(const char *s) {
        int result = 0;

        if (s == NULL) {
                result = 0;
                }
        else if (strcasecmp(s,"on") == 0) { 
                result = 1; 
        }
        else if (strcasecmp(s,"off") == 0) { 
                result = 0; 
        }
        else if (s[0] == '$') {
                result = hexstrToInt(s);
                }
        else {
                result = atoi(s);
        }

        return result;
}



/*
 * Convert a string to a character (either it's the first character, or it's a $ indicating
 * a hexadecimal #
 *
 * Also allows $$ to be used for a $ char
 */
char keystrToChar(char *s) {
        char ch = 0;

        if (s == NULL) { } 
        else if ((s[0] == '$') && (s[1] == '$')) { 
                ch = '$'; 
        }
        else if (s[0] == '$') { 
                ch = hexstrToInt(s); 
        }
        else { 
                ch = s[0]; 
        }

        return ch;
}


uint16_t keystrToKeycode(char *s) {
        uint16_t num = 0;
        
        if (s == NULL) { }
        else if ((s[0] == '$') && (s[1] == '$')) { 
                num = ascii_to_keycode('$'); 
                }
        else if (s[0] == '$') { 
                num = hexstrToInt(s); 
        }
        else { 
                num = ascii_to_keycode(s[0]); 
        }
        
        return num;
        }

const char *getKeyName(int pinNum) {
      const char *result = NULL;
      static char defaultname[] = {'i','n','p','u','t','-','#','#','\0'};

      result  = keyData[pinNum].altname;
      if (result == NULL) { result = (const char *) keyData[pinNum].keyName; }

      if (result == NULL) {
              defaultname[7] = '0' + (pinNum % 10);
              defaultname[6] = '0' + (pinNum / 10);
              result = defaultname;
              }
      
      return result;
      }

/*
 *==================================================================================================================
 */

void printFourNum(int i) {
        if (i < 1000) {
                Serial.print(" ");
                }
        if (i < 100) {
                Serial.print(" ");
                }
        if (i < 10) { 
                Serial.print(" "); 
                }
                
        Serial.print(i);
}


void printTwoNum(int i) {
        if (i < 10) { 
                Serial.print(" "); 
        }
        Serial.print(i);
}

void printHexByte(int i) {
        i = i & 0xff;		// Last byte only
        Serial.print("$");
        if (i < 16) { 
                Serial.print("0"); 
        }
        Serial.print(i, HEX);
}

void printHexWord(uint16_t num) {
        uint16_t msb;
        
        msb = (num & 0xff00) >> 8;
        num = (num & 0xff);
        
        Serial.print("$");
        if (msb < 16) { Serial.print("0"); }
        Serial.print(msb,HEX);
        
        if (num < 16) { Serial.print("0"); }
        Serial.print(num,HEX);
        }

void printThreeNum(int i, bool padleft = false) {
        if (!padleft) { 
                Serial.print(i); 
        }

        if (i < 100) { 
                Serial.print(" "); 
        }
        if (i < 10) { 
                Serial.print(" "); 
        }

        if (padleft) { 
                Serial.print(i); 
        }
}

void printTime(int t) {
        if (t < 0) { 
                Serial.print(F("    "));
        }
        else {
                Serial.print("/");
                printThreeNum(t);
        }      
}

void printKey(int i, int j) {
               
#ifdef NEEDKEYROUTINES
       KEYTYPE keycode = keyArray[i][j].keycode;
       char ch = keycode_to_ascii(keycode);
#else
       KEYTYPE ch = keyArray[i][j].keycode;
#endif

        if (isgraph(ch) && ((int) ch < 128)) {
                Serial.print("\"");
                Serial.print(ch);
                Serial.print("\"  ");
        }
        else {
                
#if NEEDKEYROUTINES
        printHexWord(keycode);
#else
        printHexByte((int) ch);
        Serial.print(F("  "));
#endif          
        }
}


void printKeyItem(int i, int j, bool showUndef = false) {
        switch(keyArray[i][j].type) {
                
        case btnUndefined :
                if (showUndef) {
                        Serial.print(F("Undefined    "));
                }
                else {
                        Serial.print(F("             "));
                        }
                break;

        case btnKeypress :
                Serial.print(F("Key "));
                printKey(i,j);
                printTime(keyArray[i][j].maxTime);
                break;

        case btnJoystick :
                Serial.print(F("Btn "));
                printTwoNum(keyArray[i][j].num);
                printTime(keyArray[i][j].maxTime);
                Serial.print(F("   "));
                break;

        case btnFunction :
                Serial.print(F("Fun "));
                printHexByte(keyArray[i][j].func);
                printTime(keyArray[i][j].maxTime);
                Serial.print(F("  "));
                break;
                
        case btnSound :
                Serial.print("Snd ");
                printFourNum(keyArray[i][j].num);
                // printTime(keyArray[i][j].auxnum);
                Serial.print(F("  "));
                break;
        }

}


void printMode(int pinNum, int modeNum) {
        Serial.print(F("Mode "));
        Serial.print(modeNum);
        Serial.print(F(" = "));
        printKeyItem(pinNum,modeNum,true);
        Serial.println("");
        }


void printInputMesg(int pinNum, const __FlashStringHelper *mesg) {
        
        if (ENABLE_SERIAL_DEBUG) {
                Serial.print(getKeyName(pinNum));
                Serial.println(mesg);
                }
        }

/*
 *==================================================================================================================
 */

/*
 *  Mode 0 - buttons->keyboard, no serial messages
 *  Mode 1 - buttons->keyboard, serial messages
 *  Mode 2 - serial messages (no buttons->keyboard)
 *  Mode 3 - special - just say "doing nothing"
 */

void handleTestCmd(int numargs, char **args) {
        int newmode = 0;

        // Do we have an argument? if so, use it!
        if (numargs > 1) { 
                newmode = parsenum(args[1]); 
        }

        ENABLE_KEYBOARD = (newmode < 2);
        ENABLE_SERIAL_DEBUG = ((newmode == 1) || (newmode == 2));
        DISABLE_EVERYTHING =  (newmode == 3);
}

/*
 *==================================================================================================================
 */
 
void handleModeCmd(int numargs, char **args) {
        int newMode;

        if (numargs < 2) {
                Serial.print("Current mode: ");
                Serial.println(keyMode);
                }
        else {
                newMode = atoi(args[1]);
                if ((newMode >=0) && (newMode <= maxMode) && (numargs == 2)) {
                        keyMode = newMode;
                }
                else {
                        Serial.println(F("Usage: mode [<#>]"));
                }
        }

}


void recalibrate_all() {
        for(int i=0;i<MAXJOYSTICKAXIS;i++) {
                jsInfo[i].calibrationStep = 0;     // Causes the next pool of the port to recalibrate
                }
        }

void handleCalibrateCmd(int numargs, char **args) {
        recalibrate_all();
        Serial.println(F("Requested re-calibration of all axis"));
}

/*
 * Small echo command - mostly to test the parser
 */

void handleEchoCmd(int numargs, char **args) {
        if (numargs == 2) {
                if (args[1][0] == '$') {
                        Serial.println(hexstrToInt(args[1]));
                }
                else {
                        Serial.println(args[1]);
                }
        }
        else if (numargs > 2) {
                for(int i=1;i<numargs;i++) {
                        Serial.print(i);
                        Serial.print(F(" : "));
                        Serial.println(args[i]);
                        }
                }
        }

void handleNudgeCmd(int numargs, char **args) {
        int newmode = 0;

        // Do we have an argument? if so, use it!
        if (numargs == 2) { 
                newmode = parsenum(args[1]); 
                ENABLE_NUDGE = (newmode != 0);
        }

        Serial.print(F("Nudging is "));
        Serial.println(ENABLE_NUDGE ? F("on") : F("off"));
}

/*
 *==================================================================================================================
 */

void printJoystick(int jsNum) {
        Serial.print(F("Axis "));
                Serial.print(jsNum+1);
                Serial.print(F(" has multiplier of "));
                Serial.print(jsInfo[jsNum].numerator);
                Serial.print("/");
                Serial.print(jsInfo[jsNum].divisor);
                Serial.print(F(", and a deadzone of "));
                Serial.println(jsInfo[jsNum].deadzone);
                Serial.print(F("      It is calibrated to zero with "));
                Serial.print(jsInfo[jsNum].calibrationOffset);
                Serial.print(F(", and currently has a value of "));
                Serial.println(jsInfo[jsNum].lastval);
}

void handleJoystickCmd(int numargs, char **args) {
        bool isError = false;
        int jsNum = -1;   // Make the compiler happy
        int numer;
        int denom;
        int deadzone;

        /*
         * We need at least two arguments
         */
        if (numargs > 1) {
                jsNum = atoi(args[1]) - 1;
                    
                if ((jsNum < 0) || (jsNum >= MAXJOYSTICKAXIS)) { 
                        isError = true; 
                        }
                }

        if  (isError) { 
                // We'll report at the end
        } 
        else if (numargs == 1) {
                for(int i=0;i<MAXJOYSTICKAXIS;i++) { printJoystick(i); Serial.println(""); }
                }
                
        else if(numargs == 2) {
               printJoystick(jsNum);
                }
                
        else if (numargs != 5) { 
                        isError = true; 
                }
        else {
                numer = atoi(args[2]);
                denom = atoi(args[3]);
                deadzone = atoi(args[4]);
        
                if (numer <= 0) { 
                        isError = true; 
                }
                if (denom <= 0) { 
                        isError = true; 
                }

                if (deadzone < 0) { 
                        isError = true; 
                }

                if ((!isError) && (jsNum >= 0)) {
                        jsInfo[jsNum].numerator = numer;
                        jsInfo[jsNum].divisor = denom;
                        jsInfo[jsNum].deadzone = deadzone;
                        Serial.print(F("New values set for axis "));
                        Serial.println(jsNum+1);
                }

        }

        if (isError) {
                Serial.println(F("Usage: joystick [<axis-#> [<numerator> <divisor> <deadzone>]]"));
        }

}

/*
 *==================================================================================================================
 */
 
void handleDebugCmd(int numargs, char **args) {     
        Serial.print("accel_last_x = "); 
        Serial.println(curr_accel_x_value++);
        Serial.print(F("X Count = "));          
        Serial.println(x_count);

        Serial.print("accel_last_y = "); 
        Serial.println(curr_accel_y_value++);       
        Serial.print(F("Y Count = "));          
        Serial.println(y_count);

        Serial.print(F("Last JS X value = "));
        Serial.println(jsInfo[0].lastval);


        Serial.print(F("Last JS Y value = "));
        Serial.println(jsInfo[1].lastval);
}


void handleHelpCmd() {
        Serial.println(F("Commands:\n"));
        Serial.println(F("calibrate          - Recalibrate the joystick center"));
        Serial.println(F("echo <word>        - Echo back the word (or words if in quotes)"));
        Serial.println(F("label <#> [\"name\"] - Label an input with a name"));
        Serial.println(F("mode [<#>]         - View or change the current key mode"));
        Serial.println(F("nudge [on|off]     - View or change nudging via the accelerometer"));
        Serial.println(F("play <#>           - Play a tone to the speaker"));
        Serial.println(F("test <0..4>        - Set level of testing/debugging (0=no testing)"));
        Serial.println(F("view               - View current button table"));
        Serial.println(F("~                  - Panic, disable normal processing"));
        
        Serial.println("");
                
        Serial.println(F("joystick [<axis-#> [<numerator> <divisor> <deadzone>]]                - View or change joystick params"));   
        Serial.println(F("set <input#> [<mode#> <button|keyboard|function|undefined> <num|char> - View or change an input"));
}

/*
 *==================================================================================================================
 */
 

void handleViewCmd() {
        int len;
        const char *s;
        
        handleModeCmd(1,NULL);
        handleNudgeCmd(1,NULL);
        
        Serial.println(F("Current button mappings:"));
        Serial.print(F("## | Name    "));

        for(int i=0;i<=maxMode;i++) {
                Serial.print(F(" | "));

                for(int j=0;j<13;j++) {
                        Serial.print((i+1) % 10);
                }
        }
        Serial.println(" |");

        for(int i=0;i<ARRAYSIZE;i++) {
                // Print the pin #
                if (i < 10) { 
                        Serial.print("0"); 
                }
                Serial.print(i);
                Serial.print(" | ");
                s = getKeyName(i);
                len = strlen(s);
                
                if (len <= 8) {
                        Serial.print(s);
                        for(int i=len+1;i<=8;i++) { Serial.print(" "); }
                        }
                else {
                        Serial.write(s,8);
                        }
        
                for(int j=0;j<=maxMode;j++) {
                        Serial.print(" | ");
                        printKeyItem(i,j);
                }
                Serial.println(" |");
        }
}


/*
 *==================================================================================================================
 */

void handleSetCmd(int numargs, char **args) {
        bool isError = false;
        int pinNum;
        int modeNum = -1;     // Make the compiler happy
        int num;
        int auxnum;
        KEYTYPE keycode;

        isError =  (numargs < 2);
        
        // Make the comparison of the 3rd arg case insensitve
        if (numargs >= 4) { args[3][0] = tolower(args[3][0]); }
        
        if (!isError) {
                pinNum = parsenum(args[1]);
                isError = ((pinNum < 0) || (pinNum >= ARRAYSIZE));
                }

        if ((!isError) && (numargs > 2)) {
                modeNum = parsenum(args[2]);
                
                isError = ((modeNum < 0) || (modeNum >= MAXKEYMODES));
                }
                
        if (isError) { }
        
        else if (numargs == 2) {
                for(int i=0;i<=maxMode;i++) { printMode(pinNum,i); }
                }
                
        else if (numargs == 3) {
                printMode(pinNum,modeNum);
                }

        else if ((numargs == 4) && (args[3][0] == 'u')) {
                keyArray[pinNum][modeNum].type = btnUndefined;
                }
        else if ((numargs == 5) && (args[3][0] == 'b')) {
                num = parsenum(args[4]);
                if ((num > 0) && (num <= 32) && (modeNum >= 0)) {
                        keyArray[pinNum][modeNum].type = btnJoystick;
                        keyArray[pinNum][modeNum].num = num;
                        keyArray[pinNum][modeNum].auxnum = -1;
                        }
                else {
                        Serial.println(F("Invalid joystick button number"));
                        }
                }
        else if ((numargs >= 5) && (args[3][0] == 'f')) {
                num = parsenum(args[4]);
                
                auxnum = -1;
                if (numargs == 6) { auxnum = parsenum(args[5]); }
                  
                keyArray[pinNum][modeNum].type = btnFunction;
                keyArray[pinNum][modeNum].num = num;   
                keyArray[pinNum][modeNum].auxnum = auxnum;
                }
        else if ((numargs >= 5) && (args[3][0] == 's')) {
                num = parsenum(args[4]);
                
                auxnum = -1;
                if (numargs == 6) { auxnum = parsenum(args[5]); }
                
                keyArray[pinNum][modeNum].type = btnSound;
                keyArray[pinNum][modeNum].num = num;
                keyArray[pinNum][modeNum].auxnum = auxnum;               
                }
        else if ((numargs == 5) && (args[3][0] == 'k')) {
#ifdef NEEDKEYROUTINES
                keycode = keystrToKeycode(args[4]);
#else
                keycode = keystrToChar(args[4]);
#endif
                if (keycode != 0) {
                        keyArray[pinNum][modeNum].type = btnKeypress;
                        keyArray[pinNum][modeNum].keycode = keycode;
                        keyArray[pinNum][modeNum].auxnum = -1;
                        }
                else {
                        Serial.println(F("Invalid keyboard keyname/keynumber"));
                        }
                }
        else {
                // Everything else is wrong
                isError = true;
                }
                
        if (isError) {
                Serial.println(F("Usage: set <inputNum> [<modeNum> <keyboard|button|function|undefined> <num|char>]"));
                }    
}


void handlePlayTone(int numargs, char **args) {
   int num = parsenum(args[1]);
   int dur = DEFAULT_TONE_DURATION;

   if (numargs > 2) {
      dur = parsenum(args[2]);
      }

   if (num > 0) {
           tone(SPEAKER_PIN, num, dur);
           }
   else {
           noTone(SPEAKER_PIN);
   }
}

void handleLabelCmd(int numargs, char **args) {
        bool isError = ((numargs < 2) || (numargs > 3));
        int pinNum;
        int len;
        
        if (!isError) {
                pinNum = parsenum(args[1]);
                isError = ((pinNum < 0) || (pinNum >= ARRAYSIZE));
                }
        
        if (!isError) {
                if (numargs == 2) {
                        Serial.print("Label: ");
                        Serial.println(getKeyName(pinNum));
                        }
                else {
                        len = strlen(args[2]);
                        
                        /*
                         * Release old memory if an existing altname exists
                         */
                        if (keyData[pinNum].altname != NULL) {
                                free((void *) keyData[pinNum].altname);
                                keyData[pinNum].altname = NULL;
                                }
                                
                        if (len > 0) {
                                keyData[pinNum].altname = (const char *) malloc(len + 1);
                                strncpy((char *) keyData[pinNum].altname, args[2], len+1);
                                }
                                
                        }
                }
        if (isError) {
                Serial.println(F("Usage: label <input#> [\"new name\"]"));
                }
        }
/*
 *==================================================================================================================
 */

void handleCmd(int numargs, char **args) {
        char cmd = tolower(args[0][0]);              // Get first char of first argument

        //Serial.println(2);

        switch (cmd) {
        case 'c' :
                handleCalibrateCmd(numargs, args);
                break;

        case 'd' :
                handleDebugCmd(numargs, args);
                break;

        case 'e' : 
                handleEchoCmd(numargs, args);
                break;

        case 'h' :
        case '?' : 
                handleHelpCmd();
                break;

        case 'j' :
                handleJoystickCmd(numargs,args);
                break;

        case 'l' :
                handleLabelCmd(numargs, args);
                break;
                
        case 'm' :
                handleModeCmd(numargs, args);
                break;


                
        case 'n' :
                handleNudgeCmd(numargs, args);
                break;

        case 'p' :
                handlePlayTone(numargs, args);

        case 's' :
                handleSetCmd(numargs, args);
                break;
                
        case 't' :
                handleTestCmd(numargs, args);
                break;

        case 'v' :
                handleViewCmd();
                break;

        case '!' :
        case '/' :
                // Assume comments, don't do anything
                break;
                
        case '~' :
                // ~~~PANIC MODE~~~
                DISABLE_EVERYTHING = 1;
                break;


        default :
                Serial.println(F("? Unknown command"));
        }
}

/*
 * Convert the command string into a series of words
 */
void parseCmdLine(char *cmd, int cmdlen, char ** args,int &argnum) {
        int i = 0;
        bool inspace = true;
        bool inquote = false;

        argnum = 0;

        while(i < cmdlen) {
                /*
		 * Are we in the white spaces (that is not protected within a quote?)
                 		 */
                if (isspace(cmd[i]) && (!inquote)) {
                        cmd[i] = '\0';              // NUL the white spaces
                        inspace = true;
                        inquote = false;
                }       
                /* 
                 				 * Is this the ending quote?  If so, treat it like the end of the quote
                 				 */
                else if ((inquote) && (cmd[i] == '"') && 
                        (isspace(cmd[i+1]) || (cmd[i+1] == '\0'))) {
                        cmd[i] = '\0';
                        inspace = true;
                        inquote = false;
                }
                else {
                        // Start of new word?, store it!
                        if ((inspace) && (argnum < MAXCMDARGS)) { 
                                if ((cmd[i] == '"') && (cmd[i+1] != '\0')) {
                                        inquote = true;
                                        i++;
                                }
                                args[argnum++] = &cmd[i];
                        }
                        inspace = false;
                }

                i++;
        }
}

void handleSerialInput() {
        static char cmd[MAXCMDLINE];
        static int cmdlen = 0;
        char newChar;

        char * args[MAXCMDARGS];
        int numargs;

        if (Serial.available()) {
                newChar = Serial.read();

                // Time to execute something?    
                if ((newChar == '\n') || (newChar == '\r')) {
                        //Serial.print(F(" [O"));
                        cmd[cmdlen] = '\0';
                        cmd[MAXCMDLINE -1] = '\0';
                        parseCmdLine(cmd,cmdlen,args,numargs);
                        //Serial.print(F("K]"));
                        Serial.println("");

                        if (numargs > 0) { 
                                //Serial.println(1);
                                handleCmd(numargs,args); 
                        }

                        Serial.print("> ");
                        cmdlen = 0;
                        }
                else if ((newChar == 127) || (newChar == '\b')) {
                        /* BS or DEL - delete last char */
                        if (cmdlen > 0) {
                                cmdlen--;                   // Erase the character
                                cmd[cmdlen] = '\0';
                                Serial.print("\b \b");
                                }
                        }
                else if (newChar == 18) {
                        /* Ctrl-R - reprint line */
                        Serial.println("^R");
                        Serial.print("> ");
                        Serial.print(cmd);
                        }
                else if (newChar == 21) {
                        /* Ctrl-U - ignore line */
                        if (cmdlen > 0) {
                                Serial.println("^U");
                                cmdlen = 0;
                                cmd[cmdlen] = '\0';
                                Serial.print("> ");
                                }
                        }
                else {
                        if (cmdlen < MAXCMDLINE) { 
                                cmd[cmdlen++] = newChar;
                                cmd[cmdlen] = '\0';
                                Serial.print(newChar);
                        }
                }
        }

}



/*
 *==================================================================================================================
 */
void addKeyOut(int pinNum, int keyMode, KEYTYPE keycode, long maxTime = -1) {

        keyMode--;

        // if ((maxTime >= 0) && (maxTime <= DEBOUNCE_TIME_MS)) {   maxTime = DEBOUNCE_TIME_MS + 1;  }
        if (keyMode >= MAXKEYMODES) { 
                keyMode = MAXKEYMODES - 1; 
        }
        if (keyMode < 0) { 
                keyMode = 0; 
        }

        // Update maxMode if necessary
        if (keyMode > maxMode) { 
                maxMode = keyMode; 
        }

#ifdef NEEDKEYROUTINES
        if ((keycode & 0xff00) == 0) {
                /* Assume it is an (ascii) char that needs converted */
                keyArray[pinNum][keyMode].keycode = ascii_to_keycode(keycode);
        }
        else {
                keyArray[pinNum][keyMode].keycode = keycode;
        }
#else
        keyArray[pinNum][keyMode].keycode = keycode;
#endif
        keyArray[pinNum][keyMode].type = btnKeypress;
        keyArray[pinNum][keyMode].maxTime = maxTime;
}


void addKeyJoy(int pinNum, int keyMode, int num, long maxTime = -1) {

        // Sanity check the joystick button number
        if (num < 1) { 
                num = 1; 
        }
        if (num > 32) { 
                num = 32; 
        }

        keyMode--;
        if (keyMode >= MAXKEYMODES) { 
                keyMode = MAXKEYMODES - 1; 
        }
        if (keyMode < 0) { 
                keyMode = 0; 
        }

        // Update maxMode if necessary
        if (keyMode > maxMode) { 
                maxMode = keyMode; 
        }

        keyArray[pinNum][keyMode].num = num;
        keyArray[pinNum][keyMode].type = btnJoystick;
        keyArray[pinNum][keyMode].maxTime = maxTime;
}

void addKeyFunction(int pinNum, int keyMode, int f, int auxnum = -1) {
        keyMode--;
        if (keyMode >= MAXKEYMODES) { 
                keyMode = MAXKEYMODES - 1; 
        }
        if (keyMode < 0) { 
                keyMode = 0; 
        }

        // Update maxMode if necessary
        if (keyMode > maxMode) { 
                maxMode = keyMode; 
        }

        keyArray[pinNum][keyMode].type = btnFunction;
        keyArray[pinNum][keyMode].func = (buttonFuncts) f;
        keyArray[pinNum][keyMode].auxnum = auxnum;
}

void addKeySound(int pinNum, int keyMode, int frequency, int duration = -1) {
          keyMode--;
        if (keyMode >= MAXKEYMODES) { 
                keyMode = MAXKEYMODES - 1; 
        }
        if (keyMode < 0) { 
                keyMode = 0; 
        }

        // Update maxMode if necessary
        if (keyMode > maxMode) { 
                maxMode = keyMode; 
        }
        
        keyArray[pinNum][keyMode].type = btnSound;
        keyArray[pinNum][keyMode].num = frequency;
        keyArray[pinNum][keyMode].auxnum = duration;
}


void addKeyDescr(int pinNum, int group, const __FlashStringHelper *kname = NULL) {
        keyData[pinNum].keyName = kname;
        keyData[pinNum].group = group;

        // See if a new max in # of scan groups
        if (maxScanGroup < group) {
                maxScanGroup = group;
        }
}

void addKeys() {
        addKeyDescr(B_LFLIP_WHITE,  0, F("L Flip White"));
        addKeyOut(B_LFLIP_WHITE, 1, KEY_LEFT_SHIFT);
        addKeySound(B_LFLIP_WHITE, 3, NOTE_B5);
        
        addKeyDescr(B_LFLIP_RED,  0, F("L Flip Red"));
        addKeyOut(B_LFLIP_RED, 1, KEY_LEFT_CTRL);
        addKeyOut(B_LFLIP_RED, 2, 'D', 15);
        addKeySound(B_LFLIP_RED, 3, NOTE_A5);

        addKeyDescr(B_RFLIP_WHITE,  0, F("R Flip White"));
        addKeyOut(B_RFLIP_WHITE, 1, KEY_RIGHT_SHIFT);
        addKeySound(B_RFLIP_WHITE, 3, NOTE_F6);
        
        addKeyDescr(B_RFLIP_RED,  0, F("R Flip Red"));
        addKeyOut(B_RFLIP_RED, 1, KEY_RIGHT_CTRL);
        addKeyOut(B_RFLIP_RED, 2, 'A', 15);
        addKeySound(B_RFLIP_RED, 3, NOTE_G6);

        addKeyDescr(B_START, 1, F("Start"));
        addKeyOut(B_START, 1, KEY_RETURN);
        addKeySound(B_START, 3, NOTE_C6);

        addKeyDescr(B_GREEN, 1, F("Green"));
        addKeyOut(B_GREEN, 1, KEY_BACKSPACE);
        addKeySound(B_GREEN, 3, NOTE_D6);
        
        addKeyDescr(B_LAUNCH, 0, F("Launch"));
        addKeyOut(B_LAUNCH, 1, ' ');
        addKeySound(B_LAUNCH, 3, NOTE_E6);

        addKeyDescr(R_RED, 2, F("Red R"));
        addKeyOut(R_RED, 1, 'C');
        addKeySound(R_RED, 3, NOTE_A6);
        
        addKeyDescr(R_ORANGE, 2, F("Orange R"));
        addKeyOut(R_ORANGE, 1, 'V');
        addKeySound(R_ORANGE, 3, NOTE_B6);

        addKeyDescr(R_YELLOW, 3, F("Yellow R"));
        addKeyOut(R_YELLOW, 1, 'H');
        addKeySound(R_YELLOW, 3, NOTE_C7);
        
        addKeyDescr(R_GREEN, 4, F("Green R"));
        addKeyFunction(R_GREEN, 1, funcAccelToggle);
        addKeySound(R_GREEN, 3, NOTE_D7);       
        
        addKeyDescr(R_BLUE, 3, F("Blue R"));
        addKeyOut(R_BLUE, 1, KEY_ESC);
        addKeySound(R_BLUE, 3, NOTE_E7);
        
        addKeyDescr(R_WHITE, 4, F("White R"));
        addKeyFunction(R_WHITE, 1, funcNewMode);          // Note: Blue is the keymode


}
/*
 *==================================================================================================================
 */

void handleSound(int frequency, int duration) {
    
   if (duration <= 0) { duration = DEFAULT_TONE_DURATION; }  // Duration is in ms
   if (duration > 10000) { duration = 10000; } // Max out at 10 seconds, please
   
   noTone(SPEAKER_PIN);  // Just in case
   
   tone(SPEAKER_PIN, frequency, duration);
   }
   
/*
 *==================================================================================================================
 */

/*
 *==================================================================================================================
 */

bool inIgnoreTime(unsigned long currTime, int pinNum) {
        bool result = false;  // Assume we aren't ignoring

        //  Are we already debouncing?
        if (buttonMode[pinNum] == btnDebounce) {
                //  Hase the alloted time expired?
                if (timeExpired(currTime,lastPressTime[pinNum] + DEBOUNCE_TIME_MS)) {
                        buttonMode[pinNum] = btnReady;    // Exited mode - change it back
                }
                else {
                        result = true;    //  Both conditions met
                }
        }

        return result;
}



bool isForceReleaseTime(unsigned long currTime, int pinNum) {
        bool result = false;
        long maxTime;



        /*
   *  1 - Button is currently being pressed
         *  2 - Mode is not already forced released
         *  2 - maxTime > 0
         *  3 - maxTime has elapsed
         */

        // First, which key are we using?  The mode's key or the default key?
        if (keyArray[pinNum][keyMode].keycode != 0) {
                maxTime = keyArray[pinNum][keyMode].maxTime;
        }
        else {
                maxTime = keyArray[pinNum][0].maxTime;
        }

        if ((buttonPressed[pinNum] == true) && (maxTime > 0) && (buttonMode[pinNum] != btnForceReleased)) {
                result = (timeExpired(currTime,lastPressTime[pinNum] + maxTime));
        }

        return result;
}

bool buttonChanged(unsigned long currTime, int pinNum, bool &buttonIsPressed) {
        bool ignore, force;
        bool result = false;
        buttonModes lastMode;

        ignore = inIgnoreTime(currTime, pinNum);
        force = isForceReleaseTime(currTime, pinNum);

        if (force) {
                buttonIsPressed = false;      // Force the change to be "off"
                result = true;                // Tell the caller that yes, there is a change
                buttonMode[pinNum] = btnForceReleased;
        }
        else if (!ignore) {
                lastMode = buttonMode[pinNum];        // Save the prev state
                buttonIsPressed = (digitalRead(pinNum) == LOW);

                if (buttonIsPressed != buttonPressed[pinNum]) {
                        result = (lastMode != btnForceReleased); // Silently ignore any ON->OFF mode that occurs during btnForcedRelease

                        buttonPressed[pinNum] = buttonIsPressed;   // Record new state
                        buttonMode[pinNum]    = btnDebounce;  // Software debounce mode
                        lastPressTime[pinNum] = currTime; // Set up to handle debounce (and max presstime)
                }
        }

        return result;
}


void handleButton(unsigned long currTime, int pinNum) {
        bool buttonIsPressed;
        // const __FlashStringHelper *s = keyData[pinNum].keyName;
        int myMode = keyMode;

        if (keyArray[pinNum][myMode].type == btnUndefined) { 
                myMode = 0; 
        }

        if (keyArray[pinNum][myMode].type != btnUndefined) {
                if (buttonChanged(currTime, pinNum, buttonIsPressed)) {

                        // Are we pressing or releasing a button?
                        if (buttonIsPressed) {

                                printInputMesg(pinNum, F(" is now pressed"));

                                if ((ENABLE_KEYBOARD) && (keyArray[pinNum][myMode].type == btnKeypress)) {
#ifdef NEEDKEYROUTINES
                                        key_press(keyArray[pinNum][myMode].keycode);
#else
                                        Keyboard.press(keyArray[pinNum][myMode].keycode);
#endif

                                }

                                else if (keyArray[pinNum][myMode].type == btnJoystick) {
                                        Joystick.button(keyArray[pinNum][myMode].num, 1);
                                        joyChanged = true;        // Joystick needs reported
                                        joyNextReport = currTime;      // Force joystick report now!
                                }

                                else if (keyArray[pinNum][myMode].type == btnFunction) {
                                        handleFunction(keyArray[pinNum][myMode].func, keyArray[pinNum][myMode].auxnum);
                                }
                                
                                else if (keyArray[pinNum][myMode].type == btnSound) {
                                        handleSound(keyArray[pinNum][myMode].num, keyArray[pinNum][myMode].auxnum);
                                }

                        }

                        else {

                                printInputMesg(pinNum, F(" is now released"));

                                if ((ENABLE_KEYBOARD) && (keyArray[pinNum][myMode].type == btnKeypress)) {
#ifdef NEEDKEYROUTINES
                                        key_release(keyArray[pinNum][myMode].keycode);
#else
                                        Keyboard.release(keyArray[pinNum][myMode].keycode);
#endif
                                }

                                if (keyArray[pinNum][myMode].type == btnJoystick) {
                                        Joystick.button(keyArray[pinNum][myMode].num, 0);
                                        joyChanged = true;                // Joystick needs reported
                                        joyNextReport = currTime;         // Force joystick report now!
                                }
                        }
                }
        }
}

/*
 *==============================================================================================
 */

/* 
 * Pressing SHIFT1 and SHIFT2 and the NewMode key will lock the mode (and cannot be switched
 * until the mode lock is disabled (same SHIFT1 + SHIFT2 + NewMode
 */
 
void handleNewMode(int auxnum) {
        static bool modeLocked = false;    // Can I switch modes?
       
        if (buttonPressed[SHIFT1] && buttonPressed[SHIFT2]) {
            modeLocked = !modeLocked;
            Serial.print(F("Mode lock is now "));
            Serial.println(modeLocked ? "on" : "off");
        }
        else if (modeLocked) {
          Serial.println(F("Cannot switch modes, mode lock is cureently on"));
          }
        else {
          if (auxnum >= 0) {
             keyMode = auxnum;
             }
             
          else { 
                keyMode++; 
                }
          
          if (keyMode > maxMode) {
                keyMode = 0;
                }   
          }
} 


void handleFunction(int f, int auxnum) {
        
        if (f&funcNewMode) {
               handleNewMode(auxnum);
               }
        
        if (f & funcAccelOff) { 
                ENABLE_NUDGE = false; 
        }
        
        if (f & funcAccelOn)  { 
                ENABLE_NUDGE = true; 
        }
        
        if (f & funcAccelToggle) { 
                ENABLE_NUDGE = !ENABLE_NUDGE; 
                }
                
        if (f & funcRecalibrate) {
                recalibrate_all();
                }
         

}


void handle_lflip_int() {
        unsigned long currTime = millis();    //  Just read the value once per loop

        handleButton(currTime, B_LFLIP_WHITE);
}

/*
 *--------------------------------------------------------------------------------
 */

/*
 * A small integration to account for drift
 */
 
void handle_analog_drift(int i, int newval, float numsamples) {
        if ((jsInfo[i].calibrationOffset != ANALOG_INVALID_DATA) && (abs(newval-jsInfo[i].calibrationOffset) <= MEDIUM_ANALOG_VALUE)) {
                jsInfo[i].calibrationOffset = ((numsamples * jsInfo[i].calibrationOffset) + newval) / numsamples;
                }                              
        }

bool joystick_is_centered(int i, int deadzone = -1) {
        if (deadzone < 0) { deadzone = jsInfo[i].deadzone; }
        
        return (abs(jsInfo[i].lastval) <= deadzone);
        }
/*
 *--------------------------------------------------------------------------------
 */

int compute_joystick_val(int i, long val) {
        val = val - jsInfo[i].calibrationOffset;
        
        // Slowly recalibrate as needed
        handle_analog_drift(i,val,1000);        
        
#ifdef SMOOTH
        for(int j=0;j<NUMJOYSTICKSAMPLES;j++) {

                val = val + jsInfo[i].hist[j];

                if (j < NUMJOYSTICKSAMPLES-1) { 
                        jsInfo[i].hist[j] = jsInfo[i].hist[j+1];
                }
                else {
                        jsInfo[i].hist[j] = newval;
                }
        }

        val = (val / (NUMJOYSTICKSAMPLES+1));
#endif

        // If this is in the deadzone - ignore it
        if ((val <= jsInfo[i].deadzone) && (-val <= jsInfo[i].deadzone)) { 
                val = 0;                        // Dead center
        }

        if (jsInfo[i].numerator != 1) { 
                val = val * jsInfo[i].numerator; 
        }

        if (jsInfo[i].divisor != 1) { 
                val = val / jsInfo[i].divisor; 
        }

        /* 
         * Check max/min values
         */
        if (val < -512) { 
                val = -512; 
        }
        if (val > 511)  { 
                val = 511; 
        }

        return val;
}

void handle_joystick_calibration(int i, volatile long &origValue) {

        if (ENABLE_SERIAL_DEBUG) { 
                Serial.print(F("Calibration of ")); 
                Serial.print(i+1); 
                Serial.print(F(" at step ")); 
                Serial.print(jsInfo[i].calibrationStep);
                Serial.print(F(", using a value of "));
                Serial.println(origValue);
        }

        if (jsInfo[i].calibrationStep <= 4) {
                jsInfo[i].hist[0] = 0;              // Reset the accumulator
                origValue = ANALOG_INVALID_DATA;        // Wait until the next datapoint is ready
        }
        else if (jsInfo[i].calibrationStep <= 8) {
                jsInfo[i].hist[0] += origValue;        // Start collecting the next group of samples
                origValue = ANALOG_INVALID_DATA;
        }
        else if (jsInfo[i].calibrationStep == 9) {
                jsInfo[i].hist[0] = jsInfo[i].hist[0] >> 2;        // Divide by 4
                jsInfo[i].calibrationOffset = jsInfo[i].hist[0];  // Use the average
                
                Serial.print(F("Calibration of axis "));
                Serial.print(i+1);
                Serial.println(F(" is complete"));
        }

        jsInfo[i].calibrationStep++;
}



/*
 * Smooth out the previous samples, and also insert the
 * current sample into the mix
 */

int process_joystick_data(volatile long &origValue, int i) {
        long val = origValue;

        // The first request is going to be "assumed" to be the joystick at rest
        //  force it to be...
        if (jsInfo[i].calibrationStep < 10) {
                handle_joystick_calibration(i,origValue);
                val = ANALOG_INVALID_DATA;
        }
        else {
                val = compute_joystick_val(i,val);
        }

        return val;
}

bool check_plunger(unsigned long currTime) {
        uint16_t uz;
        long z;
        static long oldz = 0;
        static unsigned long holdTime = 0;
        bool plunger_changed = false;

        if (getVNCL4000ProxDistanceInSteps(uz, currTime)) {

                // z = uz >> 5;
                z = uz;

                // Serial.print("Z (pre) = ");                 Serial.println(z);
                z = process_joystick_data(z,2);
                // Serial.print("Z (post) = ");                 Serial.println(z);

                /*
                 * If plunger is going forward - don't report it until it stops
                 */
                if (z > oldz) { 
                        oldz = z; 
                        holdTime = currTime + 60;
                        }
                else if (!timeExpired(currTime,holdTime)) {
                        // Wait it out
                        }
                else if ((z != ANALOG_INVALID_DATA) && (z != jsInfo[2].lastval)) {
                        Joystick.Z(z+512);

                        joyChanged = true;
                        plunger_changed = true;
                        jsInfo[2].lastval = z;
                        holdTime = currTime;                // Just in case...

                        oldz = z;
                }
        }

        return plunger_changed;

}


void check_accelerometer() {
        int x; 
        int y;

        while (curr_accel_x_value == ANALOG_INVALID_DATA) {
                Serial.println("Waiting for initial value from X NUDGE");
                delay(1000);
        }

        while(curr_accel_y_value == ANALOG_INVALID_DATA) {
                Serial.println("Waiting for initial value from Y NUDGE");
                delay(1000);
        }

        x = process_joystick_data(curr_accel_x_value,0);
        y = process_joystick_data(curr_accel_y_value,1);

        if ((x != ANALOG_INVALID_DATA) && (x != jsInfo[0].lastval)) {
                Joystick.X(x+512);
                joyChanged = true;
                jsInfo[0].lastval = x;
        }

        if ((y != ANALOG_INVALID_DATA) && (y != jsInfo[1].lastval)) {
                Joystick.Y(y+512);
                joyChanged = true;
                jsInfo[1].lastval = y;
        }
}


void handle_rflip_int() {
        unsigned long currTime = millis();    //  Just read the value once per loop

        handleButton(currTime, B_RFLIP_WHITE);
}


/*
 *======================================================================================================
 */

void initADC() {

        pinMode(ACCEL_X_PIN, INPUT);
        pinMode(ACCEL_Y_PIN, INPUT);

        // ADC_0 
        adc->setAveraging(4, ADC_0); // set number of averages
        adc->setResolution(10, ADC_0); // set bits of resolution
        adc->setConversionSpeed(ADC_LOW_SPEED,ADC_0); // change the conversion speed
        adc->setSamplingSpeed(ADC_LOW_SPEED,ADC_0); // change the sampling speed

        // ADC_1
        adc->setAveraging(4, ADC_1); // set number of averages
        adc->setResolution(10, ADC_1); // set bits of resolution
        adc->setConversionSpeed(ADC_LOW_SPEED, ADC_1); // change the conversion speed
        adc->setSamplingSpeed(ADC_LOW_SPEED, ADC_1); // change the sampling speed
        // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
        // see the documentation for more information


        adc->startContinuous(ACCEL_X_PIN, ADC_0);
        adc->startContinuous(ACCEL_Y_PIN, ADC_1);

        adc->enableInterrupts(ADC_0);
        adc->enableInterrupts(ADC_1);
}

void adc0_isr(void) {
        x_count++;
        curr_accel_x_value = adc->analogReadContinuous(ADC_0);
}

void adc1_isr(void) {
        y_count++;
        curr_accel_y_value = adc->analogReadContinuous(ADC_1);
} 

void performanceCheck(int currTime) {
        static unsigned long nextTime = 0;
        static unsigned long loopCount = 0;
        static unsigned long nextReport = 0;

        if (ENABLE_SERIAL_DEBUG) {
                loopCount++;

                if (timeExpired(currTime,nextTime)) {

                        if (timeExpired(currTime,nextReport)) {
                                Serial.print("! At ");
                                Serial.print(currTime);
                                Serial.print(", ");
                                Serial.print(loopCount);  
                                Serial.println(F(" iterations of the main loop in 10ms"));
                                nextReport = currTime + 15000;  // Report every 15 seconds
                        }
                        
                        nextTime = currTime + 10;
                        loopCount = 0;
                }
        }
}

void joystickCheck(int currTime) {
        // static unsigned int lastForcedReport = 0;

        /*
         * Do we have a queued change and is it time?
         */
        if ((joyChanged) && (timeExpired(currTime,joyNextReport))) {
                Joystick.send_now();
                joyChanged = false;
                joyNextReport = currTime + 50;
        }
}


/*
  *=====================================================================================================
 */


void scanButtons() {
        static unsigned long nudgingAllowedTime = 0; 
        unsigned long currTime = millis();    //  Just read the value once per loop

        // Make sure that scanGroup is a valid scanGroup #

        if ((scanGroup < 1) || (scanGroup > maxScanGroup)) {
                scanGroup = 1;
        }

        // Handle group 0 and the current group

        for (int i = 0; i < ARRAYSIZE; i++) {
                if ((keyData[i].group == 0) || (keyData[i].group == scanGroup)) {
                        handleButton(currTime, i); 
                } // if

        } // for

        // Once a rotation, look for serial input
        if (scanGroup == 1) { 
                handleSerialInput(); 
        }


        /*
         * Don't allow nudging if the plunger is active - it makes too much "nudge noise"
         */
        if (!joystick_is_centered(2,50)) {
                nudgingAllowedTime = currTime + 1000;
                }

        if ((scanGroup == 2) && (ENABLE_NUDGE) && (timeExpired(currTime, nudgingAllowedTime))) { 
                check_accelerometer(); 
        }

        if (scanGroup == 3) { check_plunger(currTime); } 

        scanGroup++;

        joystickCheck(currTime);
        performanceCheck(currTime);           
}

/*
 *==================================================================================================================
 */


/*
 * See https://github.com/adafruit/VCNL4000/blob/master/vcnl4000.pde
 */

void initVNCL4000() {
        Wire.begin();
        uint8_t rev = read8(VCNL4000_PRODUCTID);

        if ((rev & 0xF0) != 0x10) {
                while (1) {
                        Serial.println("VNCL 4000 sensor not found :(");
                        delay(2000);
                }
        }

        write8(VCNL4000_IRLED, 10); // set to 10 * 10mA = 100mA (200mA max)
        //  Range 0..20
        //write8(VCNL4000_SIGNALFREQ, 3);	// What frequency to use when sending IR for proximity

}

/* 
 *		states
 *			once			write8 - xmit (no delay) 
 *			repeat			read8 request (no delay), wait usecs, read8 did you do it?
 *			once			read16 gimme it	
 *			
 */

uint16_t getVNCL4000ProxDistance() {
        uint8_t cmdResult;

        write8(VCNL4000_COMMAND, VCNL4000_MEASUREPROXIMITY);
        cmdResult = read8(VCNL4000_COMMAND);

        while(!(cmdResult&VCNL4000_PROXIMITYREADY)) {
                delay(10);
                cmdResult = read8(VCNL4000_COMMAND);
        }

        return read16(VCNL4000_PROXIMITYDATA);
}


bool getVNCL4000ProxDistanceInSteps(uint16_t &val, unsigned long currTime) {
        static int stepNum = 0;
        static unsigned long nextTime = 0;
        static uint16_t data;
        uint8_t cmdResult;
        bool result = false;                // Assume not yet

        if (timeExpired(currTime,nextTime)) {

                // Serial.print("Step #"); Serial.println(stepNum);

                /* Start with the write8 command */
                if (stepNum == 0) {
                        /*
                         * Force a wait between transmissions 
                         */
                        write8(VCNL4000_COMMAND, VCNL4000_MEASUREPROXIMITY);

                        stepNum++;        // Always go to next step
                        nextTime = currTime + 2;
                }

                /* Do the "read8" asynchronously */
                else if (stepNum == 1) {
                        /*
                         * Force a wait between transmissions 
                         */

                        write0(VCNL4000_COMMAND);

                        nextTime = currTime + 2;
                        stepNum++;        // Always go to next step
                }

                else if (stepNum == 2) {
                        /*
                         * Force a wait between transmissions 
                         */
                         
                        Wire.requestFrom(VCNL4000_ADDRESS,1);
  
                        nextTime = currTime + 2;
                        stepNum++;        // Always go to next step

                }

                /* Allow steps 2,3,4 to happen together if it can */

                if (stepNum == 3) {
                        if (Wire.available() > 0) { 
                                cmdResult = Wire.read();

                                /*
                                 * If the prox data is not ready, just fall back to step 1 and ask again
                                 */
                                if (cmdResult&VCNL4000_PROXIMITYREADY) { 
                                        nextTime = currTime + 2;
                                        stepNum++; 
                                }
                                else {
                                        nextTime = currTime + 10;        // Add a 10ms delay
                                        stepNum = 1;
                                }
                        }

                }

                /* Do the "read16" asynchronously */

                if (stepNum == 4) {
                        /*
                 * Force a wait between transmissions 
                         */

                        write0(VCNL4000_PROXIMITYDATA);
                        nextTime = currTime + 2;
                        stepNum++;

                }

                else if (stepNum == 5) {
                        /*
                         * Force a wait between transmissions 
                         */

                        Wire.requestFrom(VCNL4000_ADDRESS,2);
                        nextTime = currTime + 2;
                        stepNum++;        // Always go to next step

                }

                /* Allow steps 5,6,7 to happen together if it can */

                if (stepNum == 6) {
                        if (Wire.available() > 0) {
                                data = Wire.read() << 8;
                                stepNum++;
                        }
                }

                if (stepNum == 7) {
                        if (Wire.available() > 0) {
                                data = data | Wire.read();

                                val = data;                                // Store it
                                result = true;                             // Indicate success

                                nextTime = currTime + 10;                // wait 10ms until next sample
                                stepNum = 0;
                        }
                }
        }

        return result;
}

/*
 *==================================================================================================================
 */

void setup() {
        Serial.begin(9600);

        // Set everything to input w/ pullup
        pinMode(B_LFLIP_WHITE, INPUT_PULLUP);
        pinMode(B_LFLIP_RED, INPUT_PULLUP);
        pinMode(B_START, INPUT_PULLUP);
        pinMode(B_GREEN, INPUT_PULLUP);
        pinMode(B_LAUNCH, INPUT_PULLUP);
        pinMode(B_RFLIP_WHITE, INPUT_PULLUP);
        pinMode(B_RFLIP_RED, INPUT_PULLUP);

        pinMode(R_RED, INPUT_PULLUP);
        pinMode(R_ORANGE, INPUT_PULLUP);
        pinMode(R_YELLOW, INPUT_PULLUP);
        pinMode(R_GREEN, INPUT_PULLUP);
        pinMode(R_BLUE, INPUT_PULLUP);
        pinMode(R_WHITE, INPUT_PULLUP);


        // Check for special modes
        DISABLE_EVERYTHING = (digitalRead(B_RFLIP_RED) == LOW);
        ENABLE_SERIAL_DEBUG = (digitalRead(B_RFLIP_WHITE) == LOW);

        // ENABLE_SERIAL_DEBUG = true;   //  TEST TEST TEST

        // Currently serial mode and keyboard mode are opposites
        if (ENABLE_SERIAL_DEBUG) {
                ENABLE_KEYBOARD = false;
        }

        // Initialize the arrays
        for (int i = 0; i < ARRAYSIZE; i++) {
                buttonPressed[i] = false;
                buttonMode[i] = btnReady;
                keyData[i].group = -1;            // Key is disabled by default

                //  Reset the key array
                for (int j = 0; j < MAXKEYMODES; j++) {
                        keyArray[i][j].keycode = 0;
                        keyArray[i][j].type = btnUndefined;
                        keyArray[i][j].maxTime = -1;
                }
        }

        addKeys();

        initVNCL4000();
        initADC();

        Keyboard.begin();

        Joystick.begin();
        Joystick.useManualSend(true);

        jsInfo[2].numerator = 2;
        jsInfo[2].divisor = 3;

        jsInfo[0].numerator = 8;
        jsInfo[1].numerator = 8;
        
        Joystick.X(512);
        Joystick.Y(512);
        Joystick.Z(512);
        
        Joystick.Zrotate(512);
        Joystick.sliderRight(512);
        Joystick.sliderLeft(512);
        Joystick.hat(-1);

        if (!DISABLE_EVERYTHING) {
                attachInterrupt(B_LFLIP_WHITE, handle_lflip_int, CHANGE);
                attachInterrupt(B_RFLIP_RED, handle_rflip_int, CHANGE);
        }

}


/*
 *==================================================================================================================
 */

void loop() {
        // Don't do anything if RUNODE is HIGH (not grounded)

        if (DISABLE_EVERYTHING) {
                Serial.print(F("Booted with RFLIP_RED pressed or in test mode 4 - in safe mode (everything is disabled).  Time (in ms) since boot = "));
                Serial.println(millis());

                // Do a pattern with the light for a total of 5 seconds
                // 1 on, 2 off, 3 on, 4 off, 5 off
                digitalWrite(LED_OUTPUT_PIN, HIGH);
                delay(1000);
                digitalWrite(LED_OUTPUT_PIN, LOW);
                delay(1000);
                digitalWrite(LED_OUTPUT_PIN, HIGH);
                delay(1000);
                digitalWrite(LED_OUTPUT_PIN, LOW);
                delay(2000);
        }
        else {
                scanButtons();
        }
}


