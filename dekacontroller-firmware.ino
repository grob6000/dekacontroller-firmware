/*    
 *    dekacontroller-firmware.ino
 *    Copyright (C) 2022 grob6000 (https://github.com/grob6000)
 *    This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <pins_arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <Adafruit_NeoPixel.h>

#include "dekacontroller_types.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // no reset
#define SCREEN_ADDRESS 0x3C
volatile bool displaybusy = false; // don't modify the buffer while the display routine is running

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 500000L, 500000L);
#define DATAX 45
#define TIME_DATAX DATAX
#define SYNC_DATAX DATAX
#define GRAPH_Y0 40
#define GRAPH_X0 20
#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 20
#define ICON_HEIGHT   16
#define ICON_WIDTH    16

// card dimensions
#define CARD_WIDTH 52
#define CARD_WIDTH_DOUBLE ((CARD_WIDTH*2)+CARD_MARGIN)
#define CARD_HEIGHT 21
#define CARD_MARGIN 2 // margin between cards
#define CARD_X0_0 ((SCREEN_WIDTH - CARD_WIDTH*2 - CARD_MARGIN*1)/2)
#define CARD_X0_1 (CARD_X0_0 + CARD_WIDTH + CARD_MARGIN)
//#define CARD_X0_2 (CARD_X0_1 + CARD_WIDTH + CARD_MARGIN)
#define CARD_Y0_0 (16+CARD_MARGIN)
#define CARD_Y0_1 (CARD_Y0_0 + CARD_HEIGHT + CARD_MARGIN)
#define CARD_PAD 2 // internal padding of card
#define CARD_MAX_CHARS ((CARD_WIDTH - (CARD_PAD*2)) / 6)

// specific cards
#define GPS_CARD_X0 CARD_X0_0
#define GPS_CARD_Y0 CARD_Y0_0
#define SYNC_CARD_X0 CARD_X0_0
#define SYNC_CARD_Y0 CARD_Y0_1
#define RUN_CARD_X0 CARD_X0_1
#define RUN_CARD_Y0 CARD_Y0_0
#define DRIFT_CARD_X0 CARD_X0_1
#define DRIFT_CARD_Y0 CARD_Y0_1

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
#define UART_BAUD 9600
#define UART_UBRR (F_CPU/16/UART_BAUD-1)
#define CR 0x0D
#define LF 0x0A
#define RXBUFFERSIZE 84
#define MSGLEN 14
#define TXBUFFERSIZE MSGLEN+6
#define P_MESSAGEID 0
#define P_UTCTIME 1
#define P_STATUS 2
char serialbuffer[RXBUFFERSIZE] = "";
volatile uint8_t rxbufferindex = 0;
char txbuffer[TXBUFFERSIZE] = "";
volatile uint8_t txbufferindex = 0;

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

#define SETTINGCHANGETIMEOUT (5000/TICKRATE/TICKDIV2) // 5 seconds
volatile uint8_t settingchangetimeout = 0;
#define ESPTIMEOUT (65000/TICKRATE/256) // timeout before 65 seconds; normally will be clocked by GPS
volatile uint8_t esptimeout = 0;

volatile ModeStruct mode = {
  .displaymode=Main,
  .syncstate=None
};

volatile ChangeFlags flags = {
  .displayrefresh=false,
  .forcemessage=false,
  .timechange=false,
  .gpschange=false,
  .tzchange=false,
  .syncchange=false,
  .driftchange=false,
  .runchange=false
};

const char syncstring0[] PROGMEM = "OK";
const char syncstring1[] PROGMEM = "BEGIN";
const char syncstring2[] PROGMEM = "ZERO MIN";
const char syncstring3[] PROGMEM = "ZERO HR";
const char syncstring4[] PROGMEM = "SET HR";
const char syncstring5[] PROGMEM = "SET MIN";
const char syncstring6[] PROGMEM = "WAIT MK";
const char str_error[] PROGMEM = "ERROR";
const char str_none[] PROGMEM = "NONE";
const char* const syncstrings[] PROGMEM = {syncstring0, syncstring1, syncstring2, syncstring3, syncstring4, syncstring5, syncstring6, str_error, str_none};

volatile StatusStruct status = {
  .run_ok=false,
  .gps_hastime=false,
  .gps_hasfix=false,
  .gps_oldfix=false,
  .gps_hascomms=false,
  .time_drift=false,
  .time_error=false,
};

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
#define DRIFT_HISTORY_LENGTH 24 // 10 min interval for 6h
#define DRIFT_THRESHOLD_WARNING 30 // seconds before drift warning (30 secs)
#define DRIFT_THRESHOLD_ERROR 1800 // seconds before deciding failed (30 mins)
#define DRIFT_HISTORY_YLIM_MIN 10 // minimum extent of y-axis (+/-) for history graph & interval by which this is increased
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

void UartInit() {
  UBRR0H = (uint8_t)(UART_UBRR>>8);
  UBRR0L = (uint8_t)(UART_UBRR);
  UCSR0B = (1<<RXEN0)|(1<<RXCIE0)|(1<<TXEN0)|(1<<TXCIE0);
  UCSR0C = (3<<UCSZ00);
}

ISR(USART_RX_vect) {
  char c = UDR0;
  if (c == '$') {
    rxbufferindex = 0;
  }
  serialbuffer[rxbufferindex] = c;
  rxbufferindex++;
  rxbufferindex%=RXBUFFERSIZE;
  if (c == LF) {
    processGPS();
  }
}

ISR(USART_TX_vect) {
  if ((txbuffer[txbufferindex] != 0) && (txbufferindex < TXBUFFERSIZE)) {
    UDR0 = txbuffer[txbufferindex];
    txbufferindex++;
  }
}

void uartSend(char *msg) {
  //truncate
  if (strlen(msg) > TXBUFFERSIZE) {
    msg[TXBUFFERSIZE] = 0;
  }
  strcpy(txbuffer, msg);
  if (UCSR0A && (1<<UDRE0)) { // if data register is ready (no transmission in progress)
    // start transmission (interrupt will finish)
    UDR0 = txbuffer[0];
    txbufferindex = 1;
  } // otherwise skips the transmission
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
  // SH110X_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SCREEN_ADDRESS, false)) {
    // failed display - alternate red/orange flash and block
    for (;;) {
      pixels.setPixelColor(0, pixels.Color(255,0,0));
      pixels.show();
      _delay_ms(200);
      pixels.setPixelColor(0, pixels.Color(255,255,0));
      pixels.show();
      _delay_ms(200);
    }
  }

  // splash
  displaySplash();
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
    status.run_ok = false;
  } else {
    digitalWrite(PIN_RUN_OUT, 1); // run
    delay(100);
    if (digitalRead(PIN_RUN_IN)) {
      status.run_ok = true;
    } else {
      status.run_ok = false;
    }
    digitalWrite(PIN_RUN_OUT, 0); // run off
  }
  flags.runchange = true;

  mode.displaymode = Main;
  flags.displayrefresh = true;

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
  UartInit();

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
  mode.syncstate = Begin;
  flags.syncchange = true;
}

void button0(void) {
  switch (mode.displaymode) {
    case Timezone:
      tz_increase();
      break;
    case Sync:
      // start sync
      syncBegin();
      break;
    default:
      // no action
      break;
  }
}

void button1(void) {
  switch (mode.displaymode) {
    case Timezone:
      tz_decrease();
      break;
    case Sync:
    default:
      // no action
      break;
  }
}

void button2(void) {
  // advance display mode
  switch (mode.displaymode) {
    case Main:
      mode.displaymode = Timezone;
      break;
    case Timezone:
      mode.displaymode = Drift;
      break;
    case Drift:
      mode.displaymode = Sync;
      break;
    case Sync:
    default:
      mode.displaymode = Main;
      break;
  }
  flags.displayrefresh = true;
}

void drawCard(uint8_t x0, uint8_t y0, const __FlashStringHelper *strtop, char *strbottom, bool highlight = false) {
    // clear and draw card
    display.fillRect(x0, y0,CARD_WIDTH,CARD_HEIGHT,SH110X_BLACK);
    display.drawRect(x0, y0,CARD_WIDTH,CARD_HEIGHT,SH110X_WHITE);
    
    display.setTextSize(1);
    char tempstr[CARD_MAX_CHARS+1] = "";

    // print top string
    PGM_P p = reinterpret_cast<PGM_P>(strtop);
    size_t n;
    for (n = 0; n < CARD_MAX_CHARS; n++) {
      unsigned char c = pgm_read_byte(p++);
      tempstr[n] = c;
      if (c == 0) break;
    }
    display.setCursor(x0+(CARD_WIDTH/2)-(n*3), y0+CARD_PAD);
    display.print(tempstr);

    // print bottom string
    for (n = 0; n < CARD_MAX_CHARS; n++) {
      unsigned char c = strbottom[n];
      tempstr[n] = c;
      if (c == 0) break;
    }
    display.setCursor(x0+(CARD_WIDTH/2)-(n*3), y0+CARD_PAD+9);
    display.print(tempstr);    

    // invert card body if highlighted
    if (highlight) {
      display.fillRect(x0+1,y0+1,CARD_WIDTH-2,CARD_HEIGHT-2,SH110X_INVERSE);
    }
}

void drawCard2(uint8_t x0, uint8_t y0, const __FlashStringHelper *strtop, __FlashStringHelper *strbottom, bool highlight = false) {
    char tempstr[CARD_MAX_CHARS+1] = "";
    PGM_P p = reinterpret_cast<PGM_P>(strbottom);
    size_t n;
    for (n = 0; n < CARD_MAX_CHARS; n++) {
      unsigned char c = pgm_read_byte(p++);
      tempstr[n] = c;
      if (c == 0) break;
    }
    drawCard(x0, y0, strtop, tempstr, highlight);
}

// master display function
void refreshDisplay(void) {
  if (!displaybusy) { 
    displaybusy = true;
    if (flags.displayrefresh) {
      display.clearDisplay(); // clear only if full refresh is called
    }
    if (flags.displayrefresh || flags.timechange) {
      // update big time in header row (all screens)
      display.fillRect(0,0,96,16,SH110X_BLACK);
      display.setCursor(0,0);
      display.setTextSize(2);
      char s[] = "00:00:00";
      formatTimeHHMMSS(s, hour_local, minute_local, second_local);
      display.print(s);
      display.setTextSize(1);
    }
    if (flags.displayrefresh || flags.tzchange) {
      // headline timezone display (all screens)
      display.fillRect(96,0,SCREEN_WIDTH-96,16,SH110X_BLACK);
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
    }
    // bottom part of screen depends on mode
    switch (mode.displaymode) {
      case Timezone:
        if (flags.displayrefresh) {
          // update timezone editor display, static parts
          display.setCursor(4,25);
          display.setTextSize(1);
          display.print(F("UP/DN TO SET OFFSET:"));
        }
        if (flags.displayrefresh || flags.tzchange) {
          // update big timezone
          display.fillRect(0,40,SCREEN_WIDTH,SCREEN_HEIGHT-40,SH110X_BLACK);
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
          display.setTextSize(1);
        }
        break;
      case Drift:
        if (flags.displayrefresh || flags.driftchange) {
          if (!flags.displayrefresh) {
            // clear graph space (if not already blanked)
            display.fillRect(0,16,SCREEN_WIDTH,SCREEN_HEIGHT-16,SH110X_BLACK);
          }
          // determine scale
          int16_t fullscale = (drifthistory_getmaxmagnitude()/DRIFT_HISTORY_YLIM_MIN + 1)*DRIFT_HISTORY_YLIM_MIN;
          // graph
          display.drawFastVLine(GRAPH_X0, GRAPH_Y0-GRAPH_HEIGHT-2,GRAPH_Y0+GRAPH_HEIGHT+2,SH110X_WHITE); // vertical axis
          display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0-GRAPH_HEIGHT,2,SH110X_WHITE); // top tick
          display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0+GRAPH_HEIGHT,2,SH110X_WHITE); // bottom tick
          display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0-(GRAPH_HEIGHT/2),2,SH110X_WHITE); // top intermediate tick
          display.drawFastHLine(GRAPH_X0-2,GRAPH_Y0+(GRAPH_HEIGHT/2),2,SH110X_WHITE); // bottom intermediate tick  
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
          // print value next to graph
          char driftdisplay[8] = "";
          uint8_t l = snprintf_P(driftdisplay, 8, PSTR("%+ds"), drift_current);
          display.setCursor(SCREEN_WIDTH - (SCREEN_WIDTH-GRAPH_X0-GRAPH_WIDTH)/2 - l*3,GRAPH_Y0-4);
          display.print(driftdisplay);
          // data
          for (uint8_t i = 0; i < DRIFT_HISTORY_LENGTH; i++) {
            display.drawPixel(GRAPH_X0+(i*GRAPH_WIDTH)/DRIFT_HISTORY_LENGTH,GRAPH_Y0-drifthistory_getat(DRIFT_HISTORY_LENGTH-i-1)*GRAPH_HEIGHT/fullscale,SH110X_WHITE);
          }
        }
        break;
      case Sync:
        if (flags.displayrefresh) { // static parts
          display.setTextSize(1);
          display.setCursor(16,25);
          display.print(F("PRESS UP TO SYNC"));
          display.setCursor(0,40);
          // data
          display.print(F("STATUS:"));
          display.setCursor(0,50);
          display.print(F("MARK:"));
        }
        if (flags.displayrefresh || flags.syncchange) {
          if (!flags.displayrefresh) {
            display.fillRect(DATAX, 40, SCREEN_WIDTH-DATAX,18,SH110X_BLACK);
          }
          display.setCursor(DATAX, 40);
          display.setTextSize(1);
          display.print((__FlashStringHelper *)pgm_read_word(&syncstrings[mode.syncstate]));
          display.setCursor(DATAX, 50);
          if (mode.syncstate > ZeroH) {
            char s[] = "00:00";
            formatTimeHHMM(s, marktime/60, marktime%60);
            display.print(s);
          }
          
        }
        break;
      case Main:
        // gps card
        if (flags.displayrefresh || flags.gpschange) {
          if (status.gps_hastime) {
            if (status.gps_hasfix) {
              drawCard2(GPS_CARD_X0, GPS_CARD_Y0, F("GPS"), F("OK"), false);
            } else {
              drawCard2(GPS_CARD_X0, GPS_CARD_Y0, F("GPS"), F("NOFIX"), true);
            }
          } else if (status.gps_hascomms) {
            drawCard2(GPS_CARD_X0, GPS_CARD_Y0, F("GPS"), F("INIT"), true);
          } else {
            drawCard2(GPS_CARD_X0, GPS_CARD_Y0, F("GPS"), F("COMMS"), true);
          }
        }
        // run card
        if (flags.displayrefresh || flags.runchange) {
          if (status.run_ok) {
            drawCard2(RUN_CARD_X0, RUN_CARD_Y0, F("RUN"), F("OK"), false);
          } else {
            drawCard2(RUN_CARD_X0, RUN_CARD_Y0, F("RUN"), F("ERROR"), true);
          }
        }
        // drift card
        if (flags.displayrefresh || flags.driftchange) {
          if (status.time_error) {
            drawCard2(DRIFT_CARD_X0, DRIFT_CARD_Y0, F("DRIFT"), F("ERROR"), false);
          } else {
            char driftstr[CARD_MAX_CHARS+1] = "";
            snprintf_P(driftstr, CARD_MAX_CHARS+1, PSTR("%+ds"), drift_current);
            drawCard(DRIFT_CARD_X0, DRIFT_CARD_Y0, F("DRIFT"), driftstr, false);
          }
        }
        // sync card
        if (flags.displayrefresh || flags.syncchange) {
          drawCard2(SYNC_CARD_X0, SYNC_CARD_Y0, F("SYNC"), (__FlashStringHelper *)pgm_read_word(&syncstrings[mode.syncstate]), (mode.syncstate != Ok));
        }
        break;
    }
    display.display();
    displaybusy = false;
  }
}

void button3() {
  // no actions for b3
  syncBegin(); // this button forces a sync
  return;
}

void m0low() {
  if (mode.syncstate == Ok) {
    // should be running in sync
    timetracked += 10; // add ten minutes
    timetracked %= MINUTESPERDAY; // around the clock
    bool checkok = true;
    // check zero signals
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
      int16_t drift_minutes = timetracked - (hour_local*60 + minute_local);
      if (drift_minutes < -1*(MINUTESPERDAY/2)) {
        drift_minutes += MINUTESPERDAY;
      } else if (drift_minutes > (MINUTESPERDAY/2)) {
        drift_minutes -= MINUTESPERDAY;
      }
      drift_current = (drift_minutes)*60 - second_local; // update live drift (pushed into array by GPS clock)
      if (abs(drift_current) < DRIFT_THRESHOLD_WARNING) {
        status.time_drift = false;
      } else if (abs(drift_current) < DRIFT_THRESHOLD_ERROR) {
        status.time_drift = true;
      } else {
        status.time_error = true;
      }
    } else {
      status.time_error = true;
    }
    flags.driftchange = true;
  }
}

void tz_increase() {
  tzoffset_minutes += TZ_INC;
  if (tzoffset_minutes > (12*60)) {
    tzoffset_minutes = (12*60);
  }
  flags.tzchange = true;
  settingchangetimeout = SETTINGCHANGETIMEOUT;
}

void tz_decrease() {
  tzoffset_minutes -= TZ_INC;
  if (tzoffset_minutes < (-12*60)) {
    tzoffset_minutes = (-12*60);
  }
  flags.tzchange = true;
  settingchangetimeout = SETTINGCHANGETIMEOUT;
}

const char format2d[] PROGMEM = "%02d";

void formatTimeHHMMSS(char* buf, uint8_t h, uint8_t m, uint8_t s) {
  sprintf_P(&buf[0], format2d, h);
  buf[2] = ':';
  sprintf_P(&buf[3], format2d, m);
  buf[5] = ':';
  sprintf_P(&buf[6], format2d, s);
}

void formatTimeHHMM(char* buf, uint8_t h, uint8_t m) {
  sprintf_P(&buf[0], format2d, h);
  buf[2] = ':';
  sprintf_P(&buf[3], format2d, m);
}

void displaySplash() {
  if (!displaybusy) { // bail if display is currently writing
    displaybusy = true;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print(F("DEKA CTRL"));
    display.setTextSize(1);
    display.setCursor(0,26);
    display.print(F(__DATE__));
    display.setCursor(0,36);
    display.print(F(__TIME__));
    display.setTextSize(1);
    display.display(); // force update display (loop including updated not yet be running)
    displaybusy = false;
  }
}

void processGPS() {
  if ((serialbuffer[0] == '$') && (serialbuffer[3] == 'R') & (serialbuffer[4] == 'M') && (serialbuffer[5] == 'C')) {
    //GPRMC
    uint8_t i0 = 0;
    uint8_t p = 0;
    status.gps_hascomms = true; // have communications with GPS
    gpstimeout = 0;
    bool madechanges = false;
    for (uint8_t i = 0; i < rxbufferindex; i++) {
      if (serialbuffer[i] == ',') {
        switch (p) {
          case P_UTCTIME:
            if (i-i0 > 6) { // enough characters for there to be time
              uint8_t h = (serialbuffer[i0+1]-'0')*10 + (serialbuffer[i0+2]-'0');
              uint8_t m = (serialbuffer[i0+3]-'0')*10 + (serialbuffer[i0+4]-'0');
              uint8_t s = (serialbuffer[i0+5]-'0')*10 + (serialbuffer[i0+6]-'0');
              setLocalTime(h,m,s);
              if (!status.gps_hastime) { madechanges = false; }
              status.gps_hastime = true;
              if (s==0) {
                if ((m==0) && (mode.syncstate==Ok)) { // in sync, and it's a on the hour
                  drifthistory_append(drift_current); // append last measured drift history
                  flags.driftchange = true;
                }
                flags.forcemessage = true; // 1 minute interval esp messaging
              }
              flags.timechange = true;
            } else {
              if (status.gps_hastime) { madechanges = false; }
              status.gps_hastime = false;
            }
            break;
          case P_STATUS:
            if (serialbuffer[i0+1] == 'A') {
              if (!status.gps_hasfix) { madechanges = false; }
              status.gps_hasfix = true;
              status.gps_oldfix = true;
            } else {
              if (status.gps_hasfix) { madechanges = false; }
              status.gps_hasfix = false;
            }
            break;
        }
        // check if any flags were altered, in which case update gps tile
        if (madechanges) {
          flags.gpschange = true;
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
  return (bool)((mode.syncstate > Ok) && (mode.syncstate < Error));
}

void doMessage() {
  // new message format
  // [status.02x][mode.02x][io.02x][drift.04x][offset.04x]*[csum]
  char msg[] = "00000000000000*00\r\n";
  // status
  sprintf_P(&msg[0],(const char*)F("%02X"),*((uint8_t*)&status));
  // mode
  sprintf_P(&msg[2],(const char*)F("%02X"),*((uint8_t*)&mode));
  // io
  uint8_t io = 0;
  if (!digitalRead(PIN_M0)) {io |= (1<<0);}
  if (!digitalRead(PIN_M00)) {io |= (1<<1);}
  if (!digitalRead(PIN_H0)) {io |= (1<<2);}
  if (!digitalRead(PIN_H00)) {io |= (1<<3);}
  if (digitalRead(PIN_RUN_OUT)) {io |= (1<<4);}
  if (digitalRead(PIN_RUN_IN)) {io |= (1<<5);}
  sprintf_P(&msg[4],(const char*)F("%02X"),io);  
  // drift
  sprintf_P(&msg[6],(const char*)F("%04X"),(uint16_t)drift_current);
  // offset
  sprintf_P(&msg[10],(const char*)F("%04X"),(uint16_t)tzoffset_minutes);

  // checksum
  msg[MSGLEN] = '*'; // because sprintf adds a null
  uint8_t cs = 0;
  for (uint8_t i = 0; i<strlen(msg); i++) {
    if (msg[i]=='*') {
      break;
    }
    cs^=msg[i];
  }
  sprintf_P(&msg[MSGLEN+1],(const char*)F("%02X"),cs);
  msg[MSGLEN+3] = '\r'; // because sprintf adds a null
  
  // send
  uartSend(msg); // send the message on tx
  esptimeout = 0; // reset esp message timeout

}

void loop() {
  delay(TICKRATE); // ticker
  subdiv++;
  if (subdiv%TICKDIV1==0) {
    // settingtimeout
    if (settingchangetimeout > 0) {
      settingchangetimeout--;
      if (settingchangetimeout==0) {
        int16_t oldvalue = eeprom_read_word(&tzoffset_ee);
        if (oldvalue != tzoffset_minutes) {
          eeprom_write_word(&tzoffset_ee, tzoffset_minutes);
          mode.syncstate = None; // this will automatically resync the clock
          flags.syncchange = true;
        }
      }
    }

    if (subdiv%TICKDIV2==0) {
      // update LED status
      if (syncRunning()) {
        // sync is running
        setLamp(pixels.Color(0,0,255), FLASHSETTING_FAST); // blue flash
      } else if ((!status.gps_hastime) || (!status.run_ok) || (mode.syncstate>=Error) || status.time_error) {
        // error
        setLamp(pixels.Color(255,0,0), FLASHSETTING_FAST); // red, quick flash
      } else if ((!status.gps_hasfix) || status.time_drift) {
        // warning
        setLamp(pixels.Color(255,128,0), FLASHSETTING_SLOW); // orange, slow flash
      } else {
        // ok
        setLamp(pixels.Color(0,0,0), FLASHSETTING_NONE); // off, steady (green is ugly)
      }
      //pixels.show();
      if (gpstimeout > GPSTIMEOUT) {
        // has been a while since last GPS message received, assume dead/reset
        status.gps_hasfix = false;
        status.gps_hastime = false;
        status.gps_oldfix = false;
        status.gps_hascomms = false;
        if (syncRunning()) {
          mode.syncstate = Error;
          flags.syncchange = true;
        }
        flags.gpschange = true;
      } else {
        gpstimeout++;
      }
    }
    if (subdiv==0) { // max interval; 255*tickdiv (10ms: 2.5sec, 20ms: 5 sec, 25ms: 6.3 sec)
      // check run state from time to time
      if (digitalRead(PIN_RUN_OUT) == digitalRead(PIN_RUN_IN)) { // out=high=pull run voltage down (i.e. active), in=high=run voltage low (i.e. active)
        if (status.run_ok == false) {
          flags.runchange = true;
        }
        status.run_ok = true; // if reading is what we want, no reason to report error!
      } else {
        // not consistent
        if (status.run_ok) {
          flags.runchange = true;
        }
        status.run_ok = false;
      }
      flags.displayrefresh = true; // full refresh of display
      // auto sync if conditions right and no sync currently
      if ((mode.syncstate == None) && status.gps_hastime && status.gps_oldfix && status.run_ok) {
        syncBegin();
      }
      // esp message timeout (if no GPS, still update once every minute or so)
      esptimeout++;
      if (esptimeout > ESPTIMEOUT) {
        esptimeout = 0;
        flags.forcemessage = true;
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
  // sync process
  if (syncRunning()) {
    if (ticker_adv == 0) { // reset advance pins (low --> inactive)
      digitalWrite(PIN_ADV_H, 0);
      digitalWrite(PIN_ADV_M, 0);
    } else {
      ticker_adv--;
    }
    if (ticker_sync == 0) {
      switch (mode.syncstate) {
        case Begin:
          // initialise parameters, pick a mark time, stop running, etc.
          if (status.gps_hastime && status.gps_oldfix) {
            marktime = hour_local * 60 + minute_local + 2; // worst case - update after zeroing
            marktime %= MINUTESPERDAY;
            if (digitalRead(PIN_RUN_IN)) { // high = grounded (running)
              timeoutcount++; // started at zero in begin() routine
              digitalWrite(PIN_RUN_OUT, 0); // not run
              if (timeoutcount > SYNC_RUNTIMEOUT) {
                mode.syncstate = Error;
                status.run_ok = false;
                flags.syncchange = true;
                flags.runchange = true;
              }
              ticker_sync = SYNC_RUNRETRYTICKS; // 100ms until resample
            } else {
              // low = voltage on reset line (not running); proceed
              status.run_ok = true;
              mode.syncstate = ZeroM0;
              flags.syncchange = true;
              flags.runchange = true;
              timeoutcount = 0; // reset for use as zero limiter
            }
          } else {
            // gps time is no good; go to error rather than start the sync
            mode.syncstate = Error;
            flags.syncchange = true;
          }
          break;
        case ZeroM0:
          if (digitalRead(PIN_M0)) { // M0 high = not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_M) {
              // too many presses; error!
              mode.syncstate = Error;
              flags.syncchange = true;
            } else {
              // press the advance button
              digitalWrite(PIN_ADV_M, 1); // high --> active
              ticker_adv = SYNC_PULSETICKS; // pulse
              ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action
            }
          } else {
            // low = M0 is zero
            mode.syncstate = ZeroH; // continue on next tick
            flags.syncchange = true;
            timeoutcount = 0;
          }
          break;
        case ZeroH:
          if (digitalRead(PIN_H0) || digitalRead(PIN_H00)) { // either H0 or H00 not zero
            timeoutcount++;
            if (timeoutcount > SYNC_MAXZERO_H) {
              // too many presses; error!
              mode.syncstate = Error;
              flags.syncchange = true;
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
            mode.syncstate = SetH; // continue on next tick
            flags.syncchange = true;
            timeoutcount = 0; // not sure whether i'll need this again, but zero anyway
          }
          break;
        case SetH:
          if (advcount == 0) {
            advcount = marktime % 60; // number of presses = minute
            mode.syncstate = SetM;
            flags.syncchange = true;
          } else {
            advcount--;
            digitalWrite(PIN_ADV_H, 1); // high --> active
            ticker_adv = SYNC_PULSETICKS; // pulse
            ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action     
          }
          break;
        case SetM:
          if (advcount == 0) {
            mode.syncstate = WaitMark;
            flags.syncchange = true;
          } else {
            advcount--;
            digitalWrite(PIN_ADV_M, 1); // high --> active
            ticker_adv = SYNC_PULSETICKS; // pulse
            ticker_sync = SYNC_PULSEPERIOD; // until next pulse/action   
          }
          break;
        case WaitMark:
          if ((hour_local*60 + minute_local) == (marktime)) {
            digitalWrite(PIN_RUN_OUT, 1); // high = run
            timetracked = (marktime/10)*10; // init tracking time to the 10 mins before mark time
            status.time_error = false; // clear only when sync successful
            mode.syncstate = Ok;
            flags.syncchange = true;
            drift_current = 0; // reset drift to zero
          }
          break;
        case Ok:
        case Error:
        case None:
        default:
          break;
      }
    } else {
      ticker_sync--;
    }
  }
  // if message content has changed, send a message
  if (flags.forcemessage || flags.driftchange || flags.gpschange || flags.runchange || flags.syncchange) {
    doMessage();
  }
  // if any flags are set, update display
  if (flags.displayrefresh || flags.timechange || flags.driftchange || flags.gpschange || flags.runchange || flags.syncchange || flags.tzchange) {
    refreshDisplay();
  }

  // reset all flags
  flags.displayrefresh = false;
  flags.driftchange = false;
  flags.forcemessage = false;
  flags.gpschange = false;
  flags.runchange = false;
  flags.syncchange = false;
  flags.timechange = false;
  flags.tzchange = false;
}
