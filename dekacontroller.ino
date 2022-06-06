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
#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 20
#define ICON_HEIGHT   16
#define ICON_WIDTH    16

#include "icons.h"

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
#define DEBOUNCETICKS 5 // x10ms approx

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
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LEDA, NEO_GRB+NEO_KHZ800);
//volatile uint8_t flashsetting = 0;
//volatile uint8_t flashticker = 0;

// zero state inputs
#define PIN_M0 2
#define PIN_M00 3
#define PIN_H0 4
#define PIN_H00 5

// run control / monitor
#define PIN_RUN_IN 6
#define PIN_RUN_OUT 16

// state variables
#define FLAG_RUN_OK 0
#define FLAG_GPS_HASTIME 1
#define FLAG_GPS_HASFIX 2
#define FLAG_GPS_OLDFIX 3
#define FLAG_TIME_SYNCED 4
#define FLAG_TIME_DRIFT 5
#define FLAG_DISPLAYCHANGE 6
#define FLAG_RESERVED 7
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
#define MODE_TIME 0
#define MODE_DRIFT 1
#define MODE_SYNC 2
#define MODECOUNT 3
volatile uint8_t displaymode = MODE_TIME;
#define SETTINGCHANGETIMEOUT 62 // x80ms (c. 5sec)
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
const char syncstring0[] PROGMEM = "Idle";
const char syncstring1[] PROGMEM = "Begin";
const char syncstring2[] PROGMEM = "Zero Min";
const char syncstring3[] PROGMEM = "Zero Hour";
const char syncstring4[] PROGMEM = "Set Hour";
const char syncstring5[] PROGMEM = "Set Min";
const char syncstring6[] PROGMEM = "Wait Mark";
const char syncstring7[] PROGMEM = "Error";
const char* const syncstrings[] PROGMEM = {syncstring0, syncstring1, syncstring2, syncstring3, syncstring4, syncstring5, syncstring6, syncstring7};

// sync parameters
#define SYNC_PULSETICKS 5 // x10ms
#define SYNC_PULSEFREQ 4 // Hz
#define SYNC_PULSEPERIOD (100/SYNC_PULSEFREQ) // x10ms
#define SYNC_RUNTIMEOUT 50 // x100ms
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
volatile uint8_t hour_local = 0; //local
volatile uint8_t minute_local = 0; //local
volatile uint8_t second_local = 0;
volatile int16_t tzoffset_minutes = 0;
int16_t EEMEM tzoffset_ee = 0;
volatile uint16_t timetracked = 0; // what we believe the clock to read, minutes from midnight

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
int16_t drift_history[DRIFT_HISTORY_LENGTH] = {0};
uint8_t drift_history_index = 0; // index for drift history (circular buffer)
volatile int16_t drift_current = 0; // value to be updated live

int16_t drifthistory_getat(uint8_t i) {
  return drift_history[(drift_history_index-i)%DRIFT_HISTORY_LENGTH];
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


  displaymode = MODE_TIME;
  displayTimes(); 

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
      displayUpdateSyncState();
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
      //displayUpdateSyncState();
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
    drift_current = timetracked - (hour_local*60 + minute_local); // update live drift (pushed into array by GPS clock)
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
  if (displaymode == MODE_TIME) { // only if in correct mode
    display.fillRect(TIME_DATAX, 20, display.width()-TIME_DATAX,8,SSD1306_BLACK);
    display.setCursor(TIME_DATAX,20);
    char stime[] = " 00:00:00";
    formatTimeHHMMSS(&stime[1], hour_local, minute_local, second_local);
    display.print(stime);
    display.fillRect(TIME_DATAX, 40, display.width()-TIME_DATAX,8,SSD1306_BLACK);
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
  }
}

void displayUpdateTimezone() {
  if (displaymode == MODE_TIME) { // only if in correct mode
    display.fillRect(TIME_DATAX, 30, display.width()-TIME_DATAX,8,SSD1306_BLACK);
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
  }
}

void displayTimes() {

  display.clearDisplay();
  // top line
  display.drawBitmap(0, 0, icon_clock, ICON_WIDTH, ICON_HEIGHT, 1);
  display.setTextSize(2);  
  display.setCursor(20,0);
  display.print(F("TIME"));
  // local time
  display.setTextSize(1);
  display.setCursor(0,20);
  display.print(F("TIME:"));
  // offset
  display.setCursor(0,30);
  display.print(F("OFFSET:"));
  // gps status
  display.setCursor(0,40);
  display.print(F("GPS:"));
  // update
  displayUpdateTime();
  displayUpdateTimezone();
  setFlag(FLAG_DISPLAYCHANGE);
}



void displayDrift() {
  display.clearDisplay();
  // top line
  display.drawBitmap(0, 0, icon_drift, ICON_WIDTH, ICON_HEIGHT, 1);
  display.setTextSize(2);
  display.setCursor(20,0);
  display.print(F("DRIFT"));
  // determine scale
  int16_t fullscale = (drifthistory_getmaxmagnitude()/20 + 1)*20;
  // graph
  display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0,GRAPH_WIDTH,SSD1306_WHITE); // horizontal axis
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
    display.drawPixel(GRAPH_X0+(i*GRAPH_WIDTH)/DRIFT_HISTORY_LENGTH,GRAPH_Y0+drifthistory_getat(i)*GRAPH_HEIGHT/fullscale,SSD1306_WHITE);
  }
  // update
  setFlag(FLAG_DISPLAYCHANGE);
}


void displayUpdateSyncState() {
  if (displaymode == MODE_SYNC) { // only update if in the correct mode
    display.fillRect(SYNC_DATAX, 40, display.width()-SYNC_DATAX,18,SSD1306_BLACK);
    display.setCursor(SYNC_DATAX, 40);
    display.setTextSize(1);
    display.print((__FlashStringHelper *)pgm_read_word(&syncstrings[syncstate]));
    display.setCursor(SYNC_DATAX, 50);
    if (syncstate > SYNC_IDLE) {
      char s[] = "00:00";
      formatTimeHHMM(s, marktime/60, marktime%60);
      display.print(s);
    }
    setFlag(FLAG_DISPLAYCHANGE);
  }
}

void displaySync() {
  display.clearDisplay();
  // top line
  display.drawBitmap(0, 0, icon_run, ICON_WIDTH, ICON_HEIGHT, 1);
  display.setTextSize(2);
  display.setCursor(20,0);
  display.print(F("RUN SYNC"));
  // instructions
  display.setTextSize(1);
  display.setCursor(0,25);
  display.print(F("PRESS UP TO START"));
  display.setCursor(0,40);
  display.print(F("STATUS:"));
  display.setCursor(0,50);
  display.print(F("MARK:"));
  displayUpdateSyncState();
  // update
  setFlag(FLAG_DISPLAYCHANGE);
}

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
              if ((isFlag(FLAG_TIME_SYNCED)) && (s==0) && (h%10==0)) { // in sync, and it's a 10-minute line
                drifthistory_append(drift_current); // append last measured drift history
              }
              displayUpdateTime();
            } else {
              clearFlag(FLAG_GPS_HASTIME);
            }
            break;
          case P_STATUS:
            if (serialbuffer[i0+1] == 'A') {
              setFlag(FLAG_GPS_HASFIX);
              setFlag(FLAG_GPS_OLDFIX);
            } else if (serialbuffer[i+1] == 'V') {
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


volatile uint8_t subdiv = 0;
//bool flashon = false;

void loop() {
  delay(10); // ticker 10ms
  subdiv++;
  if (subdiv%8==0) {
    // settingtimeout
    if (settingchangetimeout > 0) {
      settingchangetimeout--;
      if (settingchangetimeout==0) {
        eeprom_update_word(&tzoffset_ee, tzoffset_minutes);
      }
    }
    // update LED status (every 320ms approx)
    if (subdiv%64==0) {
      if ((!isFlag(FLAG_GPS_HASTIME)) || (!isFlag(FLAG_RUN_OK)) || (!isFlag(FLAG_TIME_SYNCED))) {
        // error
        pixels.setPixelColor(0, pixels.Color(255,0,0));
      } else if ((!isFlag(FLAG_GPS_HASFIX)) || isFlag(FLAG_TIME_DRIFT)) {
        // warning
        pixels.setPixelColor(0, pixels.Color(128,128,0));
      } else {
        // ok
        pixels.setPixelColor(0, pixels.Color(0,255,0));
      }
      pixels.show();
    }
  }
  /*
  if (flashsetting > 0) {
    if (flashticker == 0) {
      flashon = !flashon;
      if (flashon) {
        FastLED.setBrightness(255);
        FastLED.show();
      } else {
        FastLED.setBrightness(0);
        FastLED.show();
      }
      flashticker = flashsetting;
    }
    flashticker--;
  }
  */
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
          marktime = hour_local * 60 + minute_local + 2; // worst case - update after zeroing
          marktime %= MINUTESPERDAY;
          if (digitalRead(PIN_RUN_IN)) { // high = grounded (running)
            timeoutcount++; // started at zero in begin() routine
            digitalWrite(PIN_RUN_OUT, 0); // not run
            if (timeoutcount > SYNC_RUNTIMEOUT) {
              syncstate = SYNC_ERROR;
              displayUpdateSyncState();
            }
            ticker_sync = 10; // 100ms until resample
          } else {
            // low = voltage on reset line (not running); proceed
            syncstate = SYNC_ZERO_M0;
            displayUpdateSyncState();
            timeoutcount = 0; // reset for use as zero limiter
          }
          break;
        case SYNC_ZERO_M0:
          if (digitalRead(PIN_M0)) { // M0 high = not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_M) {
              // too many presses; error!
              syncstate = SYNC_ERROR;
              displayUpdateSyncState();
            } else {
              // press the advance button
              digitalWrite(PIN_ADV_M, 1); // high --> active
              ticker_adv = SYNC_PULSETICKS; // pulse
              ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action
            }
          } else {
            // low = M0 is zero
            syncstate = SYNC_ZERO_H; // continue on next tick
            displayUpdateSyncState();
            timeoutcount = 0;
          }
          break;
        case SYNC_ZERO_H:
          if (digitalRead(PIN_H0) || digitalRead(PIN_H00)) { // either H0 or H00 not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_H) {
              // too many presses; error!
              syncstate = SYNC_ERROR;
              displayUpdateSyncState();
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
            advcount = marktime / 60; // number of presses = hour
            syncstate = SYNC_SET_H; // continue on next tick
            displayUpdateSyncState();
            timeoutcount = 0; // not sure whether i'll need this again, but zero anyway
          }
          break;
        case SYNC_SET_H:
          if (advcount == 0) {
            advcount = marktime % 60; // number of presses = minute
            syncstate = SYNC_SET_M;
            displayUpdateSyncState();
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
            displayUpdateSyncState();
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
            timetracked = marktime; // init tracking time to mark time
            setFlag(FLAG_TIME_SYNCED);
            syncstate = SYNC_IDLE;
            displayUpdateSyncState();
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

/*
void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}


#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}

*/
