#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <pins_arduino.h>
#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define DATAX 45
#define TIME_DATAX DATAX
#define SYNC_DATAX DATAX
#define GRAPH_Y0 40
#define GRAPH_X0 20
#define GRAPH_WIDTH 108
#define GRAPH_HEIGHT 20
#define ICON_HEIGHT   16
#define ICON_WIDTH    16

//#include "icons.h"

// ticker definition
#define TICKRATE 25 // ms
#define TICKDIV1 8
#define TICKDIV2 32

// buttons
#define BUTTONCOUNT 4
#define PIN_BUTTON0 7
#define PIN_BUTTON1 8
#define PIN_BUTTON2 9
#define PIN_BUTTON3 10
const uint8_t buttonpin[BUTTONCOUNT] = {7,8,9,10};

// debounce handling (buttons + m0)
#define INPUTSTATE_BUTTON0 0x01
#define INPUTSTATE_BUTTON1 0x02
#define INPUTSTATE_BUTTON2 0x04
#define INPUTSTATE_BUTTON3 0x08
#define INPUTSTATE_M0 0x10
//#define INPUTSTATE_M00 0x20 //reserved
//#define INPUTSTATE_H0 0x40  /reserved
//#define INPUTSTATE_H00 0x80 //reserved
volatile uint8_t lastinputstate = 0x00;
volatile uint8_t ticker_debounce[BUTTONCOUNT+1] = {0}; // buttongs + M0
void (*debouncefunc[])() = {&button0, &button1, &button2, &button3, &m0low};
#define DEBOUNCETICKS (50/TICKRATE) // 50ms approx

// serial
#define RX_BAUD 9600
#define RX_UBRR (F_CPU/16/RX_BAUD-1)
#define CR 0x0D
#define LF 0x0A
#define SERIALBUFFERSIZE 84
#define P_MESSAGEID 0
#define P_UTCTIME 1
#define P_STATUS 2
char serialbuffer[SERIALBUFFERSIZE] = "";
volatile uint8_t serialbufferindex = 0;

// advance outputs
#define PIN_ADV_M 14
#define PIN_ADV_H 15

// adressable led output
#define PIN_LEDA 17
#define NUM_LEDS 1
#define FLASHSETTING_SLOW (1000/TICKRATE)
#define FLASHSETTING_FAST (300/TICKRATE)
#define FLASHSETTING_NONE 0
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LEDA, NEO_GRB+NEO_KHZ800);
volatile uint8_t flashsetting = 0;
volatile uint32_t lampcolor = pixels.Color(0,0,0);
volatile uint8_t flashticker = 0;
volatile uint8_t subdiv = 0;
volatile bool flashon = false;

// zero state inputs
#define PIN_M0 2
#define PIN_M00 3
#define PIN_H0 4
#define PIN_H00 5

// run control / monitor
#define PIN_RUN_IN 6
#define PIN_RUN_OUT 16

// state variables
#define FLAG_RUN_OK 0 // set when i know i can control run output
#define FLAG_GPS_HASTIME 1 // set when last gps message included time
#define FLAG_GPS_HASFIX 2 // set when last gps message included fix
#define FLAG_GPS_OLDFIX 3 // set when a gps message since boot included fix
#define FLAG_TIME_SYNCED 4 // set when sync routine was completed since boot
#define FLAG_TIME_DRIFT 5 // set when measured drift of clock exceeds threshold
#define FLAG_TIME_ERROR 6  // set when inputs are inconsistent with tracked time
#define FLAG_DISPLAYCHANGE 7 // set when there are changes in the display buffer

volatile uint8_t flags = 0x00;

inline void setFlag(uint8_t flag) {
  flags |= (1<<flag);
}

inline void clearFlag(uint8_t flag) {
  flags &= ~(1<<flag);
}

inline bool isFlag(uint8_t flag) {
  return (bool)(flags & (1<<flag));
}

// display modes
#define MODE_MAIN 0
#define MODE_TIME 1
#define MODE_DRIFT 2
#define MODE_SYNC 3
#define MODECOUNT 4
volatile uint8_t displaymode = MODE_MAIN;
#define SETTINGCHANGETIMEOUT (5000/TICKRATE/TICKDIV2) // 5 seconds
volatile uint8_t settingchangetimeout = 0;

// sync states
#define SYNC_IDLE 0
#define SYNC_BEGIN 1
#define SYNC_ZERO_M0 2
#define SYNC_ZERO_H 3
#define SYNC_SET_H 4
#define SYNC_SET_M 5
#define SYNC_WAITMARK 6
#define SYNC_ERROR 7
#define SYNCCOUNT 8
volatile uint8_t syncstate = SYNC_IDLE;
const char syncstring0[] PROGMEM = " IDLE"; // space to center in 6 chars
const char syncstring1[] PROGMEM = "BEGIN";
const char syncstring2[] PROGMEM = "ZERO M";
const char syncstring3[] PROGMEM = "ZERO H";
const char syncstring4[] PROGMEM = "SET H";
const char syncstring5[] PROGMEM = "SET M";
const char syncstring6[] PROGMEM = " WAIT"; // space to center in 6 chars
const char syncstring7[] PROGMEM = "ERROR";
const char* const syncstrings[] PROGMEM = {syncstring0, syncstring1, syncstring2, syncstring3, syncstring4, syncstring5, syncstring6, syncstring7};

// sync parameters
#define SYNC_PULSETICKS (50/TICKRATE) // 50ms
#define SYNC_PULSEFREQ 4 // Hz
#define SYNC_PULSEPERIOD (1000/TICKRATE/SYNC_PULSEFREQ) // ms/sec / ms/tick / (cyc/sec) = tick/cyc
#define SYNC_RUNRETRYTICKS (100/TICKRATE) // 100ms
#define SYNC_RUNTIMEOUT (5000/SYNC_RUNRETRYTICKS) // 5 sec
#define SYNC_MAXZERO_M 10 // max presses to zero M0
#define SYNC_MAXZERO_H 24 // max presses to zero H0 + H00

// sync variables
volatile uint8_t ticker_sync = 0;
volatile uint8_t ticker_adv = 0;
volatile uint16_t marktime = 0;
volatile uint8_t advcount = 0;
volatile uint8_t timeoutcount = 0;

// time
#define MINUTESPERDAY (60*24)
#define SECONDSPERDAY (60*60*24)
#define TZ_INC 15 // minutes
#define GPSTIMEOUT (5000/TICKRATE/TICKDIV2) // 5 seconds
volatile uint8_t hour_local = 0; //local
volatile uint8_t minute_local = 0; //local
volatile uint8_t second_local = 0;
volatile int16_t tzoffset_minutes = 0;
int16_t EEMEM tzoffset_ee = 0;
volatile uint16_t timetracked = 0; // what we believe the clock to read, minutes from midnight
volatile uint8_t gpstimeout = 0;

void setLocalTime(uint8_t hour_utc, uint8_t minute_utc, uint8_t second_utc) {
  int16_t hm = hour_utc*60 + minute_utc;
  hm += tzoffset_minutes;
  hm %= MINUTESPERDAY;
  if (hm < 0) {
    hm += MINUTESPERDAY;
  }
  hour_local = hm/60;
  minute_local = hm %60;
  second_local = second_utc;
}

// drift
//int16_t drift = 0; // last recorded drift, in seconds
#define DRIFT_HISTORY_LENGTH 144 // 10 min interval for 24h
#define DRIFT_THRESHOLD 30 // seconds before drift warning
int16_t drift_history[DRIFT_HISTORY_LENGTH] = {0};
uint8_t drift_history_index = 0; // index for drift history (circular buffer)
volatile int16_t drift_current = 0; // value to be updated live

int16_t drifthistory_getat(uint8_t i) {
  return drift_history[(DRIFT_HISTORY_LENGTH+drift_history_index-i)%DRIFT_HISTORY_LENGTH];
}

int16_t drifthistory_getlast() {
  return drift_history[drift_history_index];
}

void drifthistory_append(int16_t v) {
  drift_history_index++;
  drift_history_index%=DRIFT_HISTORY_LENGTH;
  drift_history[drift_history_index] = v;
}

int16_t drifthistory_getmaxmagnitude() {
  int16_t max = 0;
  for (uint8_t i = 0; i < DRIFT_HISTORY_LENGTH; i++) {
    if (abs(drift_history[i]) > max) {
      max = abs(drift_history[i]);
    }
  }
  return max;
}

void RxInit() {
  UBRR0H = (uint8_t)(RX_UBRR>>8);
  UBRR0L = (uint8_t)(RX_UBRR);
  UCSR0B = (1<<RXEN0)|(1<<RXCIE0);
  UCSR0C = (3<<UCSZ00);
}

ISR(USART_RX_vect) {
  char c = UDR0;
  if (c == '$') {
    serialbufferindex = 0;
  }
  serialbuffer[serialbufferindex] = c;
  serialbufferindex++;
  serialbufferindex%=SERIALBUFFERSIZE;
  if (c == LF) {
    processGPS();
  }
}

void setup() {

  // setup io
  pinMode(PIN_BUTTON0, INPUT_PULLUP);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  pinMode(PIN_BUTTON3, INPUT_PULLUP);
  pinMode(PIN_ADV_M, OUTPUT);
  pinMode(PIN_ADV_H, OUTPUT);
  pinMode(PIN_LEDA, OUTPUT);
  pinMode(PIN_M0, INPUT_PULLUP);
  pinMode(PIN_M00, INPUT_PULLUP);
  pinMode(PIN_H0, INPUT_PULLUP);
  pinMode(PIN_H00, INPUT_PULLUP);
  pinMode(PIN_RUN_IN, INPUT_PULLUP);
  pinMode(PIN_RUN_OUT, OUTPUT);

  // load eeprom settings
  int16_t tzo_temp = eeprom_read_word(&tzoffset_ee);
  if ((tzo_temp < MINUTESPERDAY/2) && (tzo_temp > -1*MINUTESPERDAY/2)) {
    tzoffset_minutes = (tzo_temp / TZ_INC) * TZ_INC;
  }

  // setup led
  pixels.clear();

  // setup display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // splash
  displaySplash();
  display.display(); // force update display (loop including updated not yet be running)
  // LED startup
  pixels.setPixelColor(0, pixels.Color(0,0,0));
  pixels.show();
  delay(500);  
  pixels.setPixelColor(0, pixels.Color(255,0,0));
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0,255,0));
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0,0,255));
  pixels.show();
  delay(500);

  // check run output
  if (digitalRead(PIN_RUN_IN)) {
    clearFlag(FLAG_RUN_OK);
  } else {
    digitalWrite(PIN_RUN_OUT, 1); // run
    delay(100);
    if (digitalRead(PIN_RUN_IN)) {
      setFlag(FLAG_RUN_OK);
    } else {
      clearFlag(FLAG_RUN_OK);
    }
    digitalWrite(PIN_RUN_OUT, 0); // run off
  }

  displaymode = MODE_MAIN;
  displayMain(); 

  // attach button interrupts
  // BUTTON0  D7  PD7  PCINT23
  // BUTTON1  D8  PB0  PCINT0
  // BUTTON2  D9  PB1  PCINT1
  // BUTTON3  D10 PB2  PCINT2 
  // M0       D2  PD2  PCINT18
  PCICR =  0b00000101;
  PCMSK2 = 0b10000100; // PCINT 18, 23
  PCMSK0 = 0b0000111; // PCINT 0, 1, 2
  
  // setup serial for GPS
  //Serial.begin(9600);
  RxInit();

}


ISR(PCINT0_vect) {
  for (uint8_t i=1; i<BUTTONCOUNT; i++) { // buttons 1,2,3
    if (!digitalRead(buttonpin[i])) { // low = pressed
      if (lastinputstate & (1<<i)) { // & laststate = high
        ticker_debounce[i] = DEBOUNCETICKS; // start debounce timer
      }
      lastinputstate &= ~(1<<i); // clear state bit
    } else {
      ticker_debounce[i] = 0; // cancel debounce timer
      lastinputstate |= (1<<i); // set state bit
    }
  }
}

ISR(PCINT2_vect) {
  if (!digitalRead(buttonpin[0])) { // low = pressed; button 0 only
    if (lastinputstate & INPUTSTATE_BUTTON0) {
      ticker_debounce[0] = DEBOUNCETICKS; // start debounce timer
    }
    lastinputstate &= INPUTSTATE_BUTTON0; // clear state bit
  } else {
    ticker_debounce[0] = 0; // cancel debounce timer
    lastinputstate |= INPUTSTATE_BUTTON0; // set state bit
  }
  if (!digitalRead(PIN_M0)) { // low = zero
    if (lastinputstate & INPUTSTATE_M0) { // & laststate = high (wasn't zero)
      ticker_debounce[BUTTONCOUNT] = DEBOUNCETICKS; // start debounce timer
    }
    lastinputstate &= INPUTSTATE_M0; // clear state bit
  } else {
    ticker_debounce[BUTTONCOUNT] = 0; // cancel debounce timer
    lastinputstate |= INPUTSTATE_M0; // set state bit
  }
}

void syncBegin() {
  timeoutcount = 0;
  ticker_sync = 0;
  ticker_adv = 0;
  syncstate = SYNC_BEGIN;
}

void button0(void) {
  switch (displaymode) {
    case MODE_TIME:
      tz_increase();
      displayUpdateTimezone();
      displayUpdateTime();
      break;
    case MODE_SYNC:
      // start sync
      syncBegin();
      displayUpdateSync();
      break;
  }
}

void button1(void) {
  switch (displaymode) {
    case MODE_TIME:
      tz_decrease();
      displayUpdateTimezone();
      displayUpdateTime();
      break;
    case MODE_SYNC:
      // start zeroing
      //zeroBegin();
      //displayUpdateSync();
      break;
  }
}

void button2(void) {
  displaymode++;
  displaymode %= MODECOUNT;
  switch (displaymode) {
    case MODE_TIME:
      displayTimes();
      break;
    case MODE_DRIFT:
      displayDrift();
      break;
    case MODE_SYNC:
      displaySync();
      break;
    case MODE_MAIN:
      displayMain();
      break;
  }
}

void button3() {
  // no actions for b3
}

void m0low() {
  if ((syncstate == SYNC_IDLE) && (isFlag(FLAG_TIME_SYNCED))) {
    // should be running in sync
    timetracked += 10; // add ten minutes
    timetracked %= MINUTESPERDAY; // around the clock
    bool checkok = true;
    // TO-DO check zero signals
    if (!digitalRead(PIN_M00) != !((timetracked / 10) % 6)) {
      checkok = false;
    }
    if (!digitalRead(PIN_H0) != !((timetracked / 60) % 10)) {
      checkok = false;
    }
    if (!digitalRead(PIN_H00) != !(timetracked / 600)) {
      checkok = false;
    }        
    if (checkok) {
      clearFlag(FLAG_TIME_ERROR);
    } else {
      setFlag(FLAG_TIME_ERROR);
    }
    drift_current = (timetracked - (hour_local*60 + minute_local))*60 - second_local; // update live drift (pushed into array by GPS clock)
    if (abs(drift_current) > DRIFT_THRESHOLD) {
      setFlag(FLAG_TIME_DRIFT);
    } else {
      clearFlag(FLAG_TIME_DRIFT);
    }
    displayUpdateDrift();
  }
}

void tz_increase() {
  tzoffset_minutes += TZ_INC;
  if (tzoffset_minutes > (12*60)) {
    tzoffset_minutes = (12*60);
  }
  settingchangetimeout = SETTINGCHANGETIMEOUT;
}

void tz_decrease() {
  tzoffset_minutes -= TZ_INC;
  if (tzoffset_minutes < (-12*60)) {
    tzoffset_minutes = (-12*60);
  }
  settingchangetimeout = SETTINGCHANGETIMEOUT;
}



void displayUpdateTime() {
  /*
  if (displaymode == MODE_TIME) { // only if in correct mode
    display.fillRect(TIME_DATAX, 20, SCREEN_WIDTH-TIME_DATAX,8,SSD1306_BLACK);
    display.setCursor(TIME_DATAX,20);
    char stime[] = " 00:00:00";
    formatTimeHHMMSS(&stime[1], hour_local, minute_local, second_local);
    display.print(stime);
    display.fillRect(TIME_DATAX, 40, SCREEN_WIDTH-TIME_DATAX,8,SSD1306_BLACK);
    display.setCursor(TIME_DATAX,40);
    display.print(F("F="));
    if (isFlag(FLAG_GPS_HASFIX)) {
      display.print(F("Y"));
    } else {
      display.print(F("N"));
    }
    display.print(F(" T="));
    if (isFlag(FLAG_GPS_HASTIME)) { 
      display.print(F("Y"));
    } else {
      display.print(F("N"));
    }
    setFlag(FLAG_DISPLAYCHANGE);
  } else
  */
  //if (displaymode == MODE_MAIN) {
    // headline time display
    display.fillRect(0,0,96,16,SSD1306_BLACK);
    display.setCursor(0,0);
    display.setTextSize(2);
    char s[] = "00:00:00";
    formatTimeHHMMSS(s, hour_local, minute_local, second_local);
    display.print(s);
    setFlag(FLAG_DISPLAYCHANGE);
  //}
}

void displayUpdateTimezone() {
  /*
  if (displaymode == MODE_TIME) { // only if in correct mode
    display.fillRect(TIME_DATAX, 30, SCREEN_WIDTH-TIME_DATAX,8,SSD1306_BLACK);
    display.setCursor(TIME_DATAX,30);
    if (tzoffset_minutes >= 0) {
      display.print(F("+"));
    } else {
      display.print(F("-"));
    }
    char s[] = "00:00";
    formatTimeHHMM(s, abs(tzoffset_minutes)/60, abs(tzoffset_minutes)%60);
    display.print(s);
    setFlag(FLAG_DISPLAYCHANGE);
  } else
  */
 if (displaymode == MODE_TIME) {
  display.fillRect(0,40,SCREEN_WIDTH,SCREEN_HEIGHT-40,SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(28,40);
  if (tzoffset_minutes >= 0) {
      display.print(F("+"));
    } else {
      display.print(F("-"));
    }
  char s[] = "00:00";
  formatTimeHHMM(s, abs(tzoffset_minutes)/60, abs(tzoffset_minutes)%60);
  display.print(s);
 }
 //if (displaymode == MODE_MAIN) {
    // headline timezone display
    display.fillRect(96,0,SCREEN_WIDTH-96,16,SSD1306_BLACK);
    display.setCursor(100,0);
    display.setTextSize(1);
    if (tzoffset_minutes >= 0) {
      display.print(F("UTC+"));
    } else {
      display.print(F("UTC-"));
    }
    char s[] = "00:00";
    formatTimeHHMM(s, abs(tzoffset_minutes)/60, abs(tzoffset_minutes)%60);
    display.setCursor(97,8);
    display.print(s);
    setFlag(FLAG_DISPLAYCHANGE);
  //}
}

void displayTimes() {

  display.clearDisplay();
  // top line
  //display.drawBitmap(0, 0, icon_clock, ICON_WIDTH, ICON_HEIGHT, 1);
  //display.setTextSize(2);  
  //display.setCursor(20,0);
  //display.print(F("TIME"));
  // local time
  //display.setTextSize(1);
  //display.setCursor(0,20);
  //display.print(F("TIME:"));
  // offset
  display.setCursor(4,25);
  display.print(F("UP/DN TO SET OFFSET:"));
  // gps status
  //display.setCursor(0,40);
  //display.print(F("GPS:"));
  // update
  displayUpdateTime();
  displayUpdateTimezone();
  //displayUpdateIcons();
  setFlag(FLAG_DISPLAYCHANGE);
}

#define GPS_CARD_X0 2
#define GPS_CARD_Y0 18
#define CARD_WIDTH 40
#define CARD_HEIGHT 21
void displayUpdateGPS() {
  if (displaymode == MODE_MAIN) {
    // gps card
    display.fillRect(GPS_CARD_X0, GPS_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_BLACK);
    display.drawRect(GPS_CARD_X0, GPS_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(GPS_CARD_X0+11, GPS_CARD_Y0+2);
    display.print(F("GPS"));
    bool invert = false;
    if (isFlag(FLAG_GPS_HASFIX)) {
      display.setCursor(GPS_CARD_X0+2+9, GPS_CARD_Y0+11);
      display.print(F("FIX"));
    } else if (isFlag(FLAG_GPS_HASTIME)) {
      display.setCursor(GPS_CARD_X0+2+6, GPS_CARD_Y0+11);
      display.print(F("TIME"));
      invert = true;
    } else {
      display.setCursor(GPS_CARD_X0+2+3, GPS_CARD_Y0+11);
      display.print(F("ERROR"));
      invert = true;
    }
    if (invert) {
      display.fillRect(GPS_CARD_X0+1,GPS_CARD_Y0+1,CARD_WIDTH-2,CARD_HEIGHT-2,SSD1306_INVERSE);
    }
    setFlag(FLAG_DISPLAYCHANGE);
  }
}

#define RUN_CARD_X0 45
#define RUN_CARD_Y0 18
void displayUpdateRun() {
  if (displaymode == MODE_MAIN) {
    // gps card
    display.fillRect(RUN_CARD_X0, RUN_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_BLACK);
    display.drawRect(RUN_CARD_X0, RUN_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(RUN_CARD_X0+11, RUN_CARD_Y0+2);
    display.print(F("RUN"));
    if (isFlag(FLAG_RUN_OK)) {
      display.setCursor(RUN_CARD_X0+2+12, RUN_CARD_Y0+11);
      display.print(F("OK"));
    } else {
      display.setCursor(RUN_CARD_X0+2+3, RUN_CARD_Y0+11);
      display.print(F("ERROR"));
      display.fillRect(RUN_CARD_X0+1,RUN_CARD_Y0+1,CARD_WIDTH-2,CARD_HEIGHT-2,SSD1306_INVERSE);
    }
    setFlag(FLAG_DISPLAYCHANGE);
  }  
}

void displayMain() {
  display.clearDisplay();
  displayUpdateTime(); // draw headline time
  displayUpdateTimezone(); // draw headline timezone
  displayUpdateGPS(); // draw gps card
  displayUpdateSync(); // draw sync card
  displayUpdateRun(); // draw run card
  displayUpdateDrift(); // draw drift card
}

#define DRIFT_CARD_X0 45
#define DRIFT_CARD_Y0 41
void displayUpdateDrift() {
  if (displaymode == MODE_DRIFT) {
    displayDrift(); // just do the whole schebang
  } else if (displaymode == MODE_MAIN) {
    // drift card
    display.fillRect(DRIFT_CARD_X0, DRIFT_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_BLACK);
    display.drawRect(DRIFT_CARD_X0, DRIFT_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(DRIFT_CARD_X0+2+3, DRIFT_CARD_Y0+2);
    display.print(F("TRACK"));
    if (isFlag(FLAG_TIME_ERROR)) {
      display.setCursor(DRIFT_CARD_X0+2+3, DRIFT_CARD_Y0+11);
      display.print(F("ERROR"));
    } else {
      display.setCursor(DRIFT_CARD_X0+2, DRIFT_CARD_Y0+11);
      if (drift_current >= 0) {
        display.print("+");
      }
      display.print(drift_current);
      display.print("s");
    }
    if (isFlag(FLAG_TIME_ERROR) || isFlag(FLAG_TIME_DRIFT)) {
      display.fillRect(DRIFT_CARD_X0+1,DRIFT_CARD_Y0+1,CARD_WIDTH-2,CARD_HEIGHT-2,SSD1306_INVERSE);
    }    
    setFlag(FLAG_DISPLAYCHANGE);    
  }
}

void displayDrift() {
  display.clearDisplay();
  // top line
  //display.drawBitmap(0, 0, icon_drift, ICON_WIDTH, ICON_HEIGHT, 1);
  //display.setTextSize(2);
  //display.setCursor(20,0);
  //display.print(F("DRIFT"));
  // determine scale
  int16_t fullscale = (drifthistory_getmaxmagnitude()/20 + 1)*20;
  // graph
  //display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0,GRAPH_WIDTH,SSD1306_WHITE); // horizontal axis
  display.drawFastVLine(GRAPH_X0, GRAPH_Y0-GRAPH_HEIGHT-2,GRAPH_Y0+GRAPH_HEIGHT+2,SSD1306_WHITE); // vertical axis
  display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0-GRAPH_HEIGHT,2,SSD1306_WHITE); // top tick
  display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0+GRAPH_HEIGHT,2,SSD1306_WHITE); // bottom tick
  display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0-(GRAPH_HEIGHT/2),2,SSD1306_WHITE); // top intermediate tick
  display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0+(GRAPH_HEIGHT/2),2,SSD1306_WHITE); // bottom intermediate tick  
  display.setTextSize(1);
  display.setCursor(0, GRAPH_Y0-GRAPH_HEIGHT-4);
  display.print(F("+"));
  display.print(fullscale);
  display.setCursor(0, GRAPH_Y0-(GRAPH_HEIGHT/2)-4);
  display.print(F("+"));
  display.print(fullscale/2);
  display.setCursor(0, GRAPH_Y0+GRAPH_HEIGHT-4);
  display.print(F("-"));
  display.print(fullscale);
  display.setCursor(0, GRAPH_Y0+(GRAPH_HEIGHT/2)-4);
  display.print(F("-"));
  display.print(fullscale/2);
  // data
  for (uint8_t i = 0; i < DRIFT_HISTORY_LENGTH; i++) {
    display.drawPixel(GRAPH_X0+(i*GRAPH_WIDTH)/DRIFT_HISTORY_LENGTH,GRAPH_Y0-drifthistory_getat(DRIFT_HISTORY_LENGTH-i-1)*GRAPH_HEIGHT/fullscale,SSD1306_WHITE);
  }
  //displayUpdateIcons();
  /*
  // print the offset over the top
  display.setCursor(96,16);
  int16_t dhl = drifthistory_getlast();
  if (dhl >=- 0) {
    display.print("+");
  }
  display.print(dhl);
  display.setCursor(96,26);
  display.print(drift_current);
  display.setCursor(80,46);
  char s[] = "00:00";
  formatTimeHHMM(s, timetracked/60, timetracked%60);
  display.print(s);
  */
  displayUpdateTime();
  displayUpdateTimezone();
  // update
  setFlag(FLAG_DISPLAYCHANGE);
}

#define SYNC_CARD_X0 2
#define SYNC_CARD_Y0 41

void displayUpdateSync() {
  if (displaymode == MODE_SYNC) { // only update if in the correct mode
    display.fillRect(SYNC_DATAX, 40, SCREEN_WIDTH-SYNC_DATAX,18,SSD1306_BLACK);
    display.setCursor(SYNC_DATAX, 40);
    display.setTextSize(1);
    display.print((__FlashStringHelper *)pgm_read_word(&syncstrings[syncstate]));
    display.setCursor(SYNC_DATAX, 50);
    if (syncstate > SYNC_ZERO_H) {
      char s[] = "00:00";
      formatTimeHHMM(s, marktime/60, marktime%60);
      display.print(s);
    }
    setFlag(FLAG_DISPLAYCHANGE);
  } else if (displaymode == MODE_MAIN) {
    // sync card
    display.fillRect(SYNC_CARD_X0, SYNC_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_BLACK);
    display.drawRect(SYNC_CARD_X0, SYNC_CARD_Y0,CARD_WIDTH,CARD_HEIGHT,SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(SYNC_CARD_X0+2+6, SYNC_CARD_Y0+2);
    display.print(F("SYNC"));
    display.setCursor(SYNC_CARD_X0+2, SYNC_CARD_Y0+11);
    display.print((__FlashStringHelper *)pgm_read_word(&syncstrings[syncstate]));
    if (syncstate != SYNC_IDLE) {
      display.fillRect(SYNC_CARD_X0+1,SYNC_CARD_Y0+1,CARD_WIDTH-2,CARD_HEIGHT-2,SSD1306_INVERSE);
    }
    setFlag(FLAG_DISPLAYCHANGE);
  }
}

void displaySync() {
  display.clearDisplay();
  // top line
  //display.drawBitmap(0, 0, icon_run, ICON_WIDTH, ICON_HEIGHT, 1);
  //display.setTextSize(2);
  //display.setCursor(20,0);
  //display.print(F("RUN SYNC"));
  // instructions
  display.setTextSize(1);
  display.setCursor(16,25);
  display.print(F("PRESS UP TO SYNC"));
  display.setCursor(0,40);
  display.print(F("STATUS:"));
  display.setCursor(0,50);
  display.print(F("MARK:"));
  displayUpdateSync();
  displayUpdateTime();
  displayUpdateTimezone();
  //displayUpdateIcons();
  // update
  setFlag(FLAG_DISPLAYCHANGE);
}

/*
void displayUpdateIcons() {
  if (displaymode != MODE_MAIN) {
    display.fillRect(0,0,SCREEN_WIDTH,ICON_HEIGHT,SSD1306_BLACK);
    if (isFlag(FLAG_RUN_OK)) {
      display.drawBitmap(0,0,icon_run,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(0,0,icon_norun,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    }
    if (isFlag(FLAG_GPS_HASTIME)) {
      display.drawBitmap(16,0,icon_clock,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(16,0,icon_notime,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    }
    if (isFlag(FLAG_GPS_HASFIX)) {
      display.drawBitmap(32,0,icon_signal,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(32,0,icon_nosignal,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    }
    if (isFlag(FLAG_TIME_SYNCED)) {
      display.drawBitmap(48,0,icon_sync,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(48,0,icon_nosync,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } 
    if (isFlag(FLAG_TIME_DRIFT)) {
      display.drawBitmap(64,0,icon_driftbad,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(64,0,icon_driftok,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } 
    if (isFlag(FLAG_TIME_ERROR)) {
      display.drawBitmap(80,0,icon_zerobad,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    } else {
      display.drawBitmap(80,0,icon_zerook,ICON_WIDTH,ICON_HEIGHT,SSD1306_WHITE);
    }
    setFlag(FLAG_DISPLAYCHANGE); 
  }
}
*/

void formatTimeHHMMSS(char* buf, uint8_t h, uint8_t m, uint8_t s) {
  sprintf(&buf[0], "%02d", h);
  buf[2] = ':';
  sprintf(&buf[3], "%02d", m);
  buf[5] = ':';
  sprintf(&buf[6], "%02d", s);
}

void formatTimeHHMM(char* buf, uint8_t h, uint8_t m) {
  sprintf(&buf[0], "%02d", h);
  buf[2] = ':';
  sprintf(&buf[3], "%02d", m);
}

void displaySplash() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("DEKA CTRL"));
  display.setTextSize(1);
  display.setCursor(0,26);
  display.print(F(__DATE__));
  display.setCursor(0,36);
  display.print(F(__TIME__));
  setFlag(FLAG_DISPLAYCHANGE);
}

void processGPS() {
  if ((serialbuffer[0] == '$') && (serialbuffer[3] == 'R') & (serialbuffer[4] == 'M') && (serialbuffer[5] == 'C')) {
    //GPRMC
    uint8_t i0 = 0;
    uint8_t p = 0;
    gpstimeout = 0;
    for (uint8_t i = 0; i < serialbufferindex; i++) {
      if (serialbuffer[i] == ',') {
        switch (p) {
          case P_UTCTIME:
            if (i-i0 > 6) { // enough characters for there to be time
              uint8_t h = (serialbuffer[i0+1]-'0')*10 + (serialbuffer[i0+2]-'0');
              uint8_t m = (serialbuffer[i0+3]-'0')*10 + (serialbuffer[i0+4]-'0');
              uint8_t s = (serialbuffer[i0+5]-'0')*10 + (serialbuffer[i0+6]-'0');
              setLocalTime(h,m,s);
              setFlag(FLAG_GPS_HASTIME);
              if ((isFlag(FLAG_TIME_SYNCED)) && (s==0) && (m%10==0)) { // in sync, and it's a 10-minute line
                drifthistory_append(drift_current); // append last measured drift history
                displayUpdateDrift();
              }
              displayUpdateTime();
              displayUpdateGPS();
            } else {
              clearFlag(FLAG_GPS_HASTIME);
            }
            break;
          case P_STATUS:
            if (serialbuffer[i0+1] == 'A') {
              setFlag(FLAG_GPS_HASFIX);
              setFlag(FLAG_GPS_OLDFIX);
            } else {
              clearFlag(FLAG_GPS_HASFIX);
            }
            break;
        }
        i0 = i;
        p++;
      }
    }
  }
}

void setLamp(uint32_t newcolor, uint8_t newflashsetting) {
    if ((newflashsetting == 0) && ((newcolor != lampcolor) || (flashsetting != 0))) { // change, and new color is constant
      pixels.setPixelColor(0,newcolor);
      pixels.show();
    }
    lampcolor = newcolor;
    flashsetting = newflashsetting;
}

inline bool syncRunning() {
  return (bool)((syncstate > SYNC_IDLE) && (syncstate < SYNC_ERROR));
}

void loop() {
  delay(TICKRATE); // ticker
  subdiv++;
  if (subdiv%TICKDIV1==0) {
    // settingtimeout
    if (settingchangetimeout > 0) {
      settingchangetimeout--;
      if (settingchangetimeout==0) {
        eeprom_update_word(&tzoffset_ee, tzoffset_minutes);
      }
    }
    if (subdiv%TICKDIV2==0) {
      //displayUpdateIcons();
      // update LED status
      if (syncRunning()) {
        // sync is running
        setLamp(pixels.Color(0,0,255), FLASHSETTING_FAST); // blue flash
      } else if ((!isFlag(FLAG_GPS_HASTIME)) || (!isFlag(FLAG_RUN_OK)) || (!isFlag(FLAG_TIME_SYNCED))) {
        // error
        setLamp(pixels.Color(255,0,0), FLASHSETTING_FAST); // red, quick flash
      } else if ((!isFlag(FLAG_GPS_HASFIX)) || isFlag(FLAG_TIME_DRIFT)) {
        // warning
        setLamp(pixels.Color(128,128,0), FLASHSETTING_SLOW); // orange, slow flash
      } else {
        // ok
        setLamp(pixels.Color(0,255,0), FLASHSETTING_NONE); // green, steady
      }
      //pixels.show();
      if (gpstimeout > GPSTIMEOUT) {
        // has been a while since last GPS message received, assume dead/reset
        clearFlag(FLAG_GPS_HASFIX);
        clearFlag(FLAG_GPS_HASTIME);
        clearFlag(FLAG_GPS_OLDFIX);
        if (syncRunning()) {
          syncstate = SYNC_ERROR;
          displayUpdateSync();
        }
        displayUpdateGPS();
      } else {
        gpstimeout++;
      }
    }
    if (subdiv==0) { // max interval; 255*tickdiv (10ms: 2.5sec, 20ms: 5 sec, 25ms: 6.3 sec)
      // check run state from time to time
      if (digitalRead(PIN_RUN_OUT) == digitalRead(PIN_RUN_IN)) { // out=high=pull run voltage down (i.e. active), in=high=run voltage low (i.e. active)
        setFlag(FLAG_RUN_OK); // if reading is what we want, no reason to report error!
      } else {
        // not consistent
        clearFlag(FLAG_RUN_OK);
      }
      displayUpdateRun();
      // auto sync if conditions right and no sync currently
      if ((!syncRunning()) && (!isFlag(FLAG_TIME_SYNCED)) && isFlag(FLAG_GPS_HASTIME) && isFlag(FLAG_GPS_HASFIX) && isFlag(FLAG_RUN_OK)) {
        syncBegin();
      }
    }
  }
  if (flashsetting > 0) {
    if (flashticker == 0) {
      flashon = !flashon;
      if (flashon) {
        pixels.setPixelColor(0,lampcolor);
        pixels.show();
      } else {
        pixels.setPixelColor(0,0); // off
        pixels.show();
      }
      flashticker = flashsetting;
    }
    flashticker--;
  }
  // button debouncing
  for (uint8_t i = 0; i < BUTTONCOUNT+1; i++) {
    if ((ticker_debounce[i]) > 0) {
      ticker_debounce[i]--;
      if (ticker_debounce[i] == 0) {
        debouncefunc[i]();
      }
    }
  }
  // if pending display update, update the display
  if (isFlag(FLAG_DISPLAYCHANGE)) {
    display.display();
    clearFlag(FLAG_DISPLAYCHANGE);
  }
  // sync process
  if (syncstate != SYNC_IDLE) {
    if (ticker_adv == 0) { // reset advance pins (low --> inactive)
      digitalWrite(PIN_ADV_H, 0);
      digitalWrite(PIN_ADV_M, 0);
    } else {
      ticker_adv--;
    }
    if (ticker_sync == 0) {
      switch (syncstate) {
        case SYNC_BEGIN:
          // initialise parameters, pick a mark time, stop running, etc.
          if (isFlag(FLAG_GPS_HASTIME) && isFlag(FLAG_GPS_OLDFIX)) {
            marktime = hour_local * 60 + minute_local + 2; // worst case - update after zeroing
            marktime %= MINUTESPERDAY;
            if (digitalRead(PIN_RUN_IN)) { // high = grounded (running)
              timeoutcount++; // started at zero in begin() routine
              digitalWrite(PIN_RUN_OUT, 0); // not run
              if (timeoutcount > SYNC_RUNTIMEOUT) {
                syncstate = SYNC_ERROR;
                clearFlag(FLAG_RUN_OK);
                displayUpdateSync();
                displayUpdateRun();
              }
              ticker_sync = SYNC_RUNRETRYTICKS; // 100ms until resample
            } else {
              // low = voltage on reset line (not running); proceed
              setFlag(FLAG_RUN_OK);
              syncstate = SYNC_ZERO_M0;
              displayUpdateSync();
              displayUpdateRun();
              timeoutcount = 0; // reset for use as zero limiter
            }
          } else {
            // gps time is no good; go to error rather than start the sync
            syncstate = SYNC_ERROR;
            displayUpdateSync();
          }
          break;
        case SYNC_ZERO_M0:
          if (digitalRead(PIN_M0)) { // M0 high = not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_M) {
              // too many presses; error!
              syncstate = SYNC_ERROR;
              displayUpdateSync();
            } else {
              // press the advance button
              digitalWrite(PIN_ADV_M, 1); // high --> active
              ticker_adv = SYNC_PULSETICKS; // pulse
              ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action
            }
          } else {
            // low = M0 is zero
            syncstate = SYNC_ZERO_H; // continue on next tick
            displayUpdateSync();
            timeoutcount = 0;
          }
          break;
        case SYNC_ZERO_H:
          if (digitalRead(PIN_H0) || digitalRead(PIN_H00)) { // either H0 or H00 not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_H) {
              // too many presses; error!
              syncstate = SYNC_ERROR;
              displayUpdateSync();
            } else {
              digitalWrite(PIN_ADV_H, 1); // high --> active
              ticker_adv = SYNC_PULSETICKS; // pulse
              ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action          
            }
          } else {
            // both H0 and H00 are zero
            uint8_t timetoset = (marktime/60 + marktime%60) / SYNC_PULSEFREQ + 1; // worst case 21 seconds
            marktime = hour_local*60 + minute_local;
            if (second_local < 60-timetoset) { // will make it before next minute change
              marktime += 1; // next minute
            } else
              marktime += 2; // the one after
            marktime%=MINUTESPERDAY; // roll around midnight (unlikely...)
            advcount = marktime / 60; // number of presses = hour
            syncstate = SYNC_SET_H; // continue on next tick
            displayUpdateSync();
            timeoutcount = 0; // not sure whether i'll need this again, but zero anyway
          }
          break;
        case SYNC_SET_H:
          if (advcount == 0) {
            advcount = marktime % 60; // number of presses = minute
            syncstate = SYNC_SET_M;
            displayUpdateSync();
          } else {
            advcount--;
            digitalWrite(PIN_ADV_H, 1); // high --> active
            ticker_adv = SYNC_PULSETICKS; // pulse
            ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action     
          }
          break;
        case SYNC_SET_M:
          if (advcount == 0) {
            syncstate = SYNC_WAITMARK;
            displayUpdateSync();
          } else {
            advcount--;
            digitalWrite(PIN_ADV_M, 1); // high --> active
            ticker_adv = SYNC_PULSETICKS; // pulse
            ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action   
          }
          break;
        case SYNC_WAITMARK:
          if ((hour_local*60 + minute_local) == (marktime)) {
            digitalWrite(PIN_RUN_OUT, 1); // high = run
            timetracked = (marktime/10)*10; // init tracking time to the 10 mins before mark time
            setFlag(FLAG_TIME_SYNCED);
            syncstate = SYNC_IDLE;
            displayUpdateSync();
          }
          break;
        case SYNC_IDLE:
        case SYNC_ERROR:
        default:
          break;
      }
    } else {
      ticker_sync--;
    }
  }
}