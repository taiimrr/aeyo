/*
  Mega 2560 + 1602A (parallel, 4-bit) + DS1307 + Rotary Encoder + Buzzer Alarm Clock

  Features:
  - Shows current time from DS1307 on 1602A LCD
  - Rotary encoder sets alarm hour/min (short presses navigate)
  - Long press on encoder button toggles Alarm ON/OFF (saved to EEPROM)
  - Alarm triggers ONLY at :00 seconds (HH:MM:00)
  - Alarm will NOT trigger while you are setting the alarm
  - Prevents "immediate ring" right after saving the alarm time
  - Buzzer beeps in a pattern while ringing; short press stops it

  Rotary (as you wired):
    CLK -> D18
    DT  -> D19
    SW  -> D6   (important: avoid D4 if LCD uses it)

  RTC DS1307 (Mega I2C):
    SDA -> 20
    SCL -> 21
    VCC -> 5V
    GND -> GND

  Buzzer:
    Signal -> D8
    GND    -> GND
    (If 3-pin module: S->D8, +->5V, -->GND)
*/

#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

// ---------- LCD (match your wiring) ----------
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ---------- RTC ----------
RTC_DS1307 rtc;

// ---------- Rotary ----------
const int PIN_CLK = 18;   // CLK -> D18 (interrupt capable)
const int PIN_DT  = 19;   // DT  -> D19
const int PIN_SW  = 6;    // SW  -> D6

// ---------- Buzzer ----------
const int PIN_BUZZER = 8; // buzzer signal pin

// Choose buzzer mode:
// true  = passive buzzer (tone() beeps)
// false = active buzzer/module (digital HIGH/LOW beeps)
const bool USE_TONE_BUZZER = true;

// ---------- EEPROM addresses ----------
const int EE_ALARM_H  = 0;
const int EE_ALARM_M  = 1;
const int EE_ALARM_EN = 2;

// ---------- UI state ----------
enum UiState { SHOW_CLOCK, SET_HOUR, SET_MIN, RINGING };
UiState state = SHOW_CLOCK;

// ---------- Encoder handling ----------
volatile int encoderDelta = 0;
volatile unsigned long lastEncUs = 0;

// ---------- Button handling ----------
bool btnDown = false;
unsigned long btnDownMs = 0;
unsigned long lastBtnDebounce = 0;
bool lastBtnRead = HIGH;

// ---------- Display timing ----------
unsigned long lastDraw = 0;

// ---------- Alarm ----------
int  alarmHour = 7;
int  alarmMin  = 30;
bool alarmEnabled = true;

// Trigger guard (prevents retriggering multiple times)
long lastTriggeredMinuteKey = -1;

// ---------- Buzzer pattern ----------
unsigned long lastBeepToggle = 0;
bool beepOn = false;

// ---------- Helpers ----------
void saveAlarm();

void loadAlarm() {
  int h = EEPROM.read(EE_ALARM_H);
  int m = EEPROM.read(EE_ALARM_M);
  int e = EEPROM.read(EE_ALARM_EN);

  // defaults if EEPROM was uninitialized
  if (h < 0 || h > 23) h = 7;
  if (m < 0 || m > 59) m = 30;

  // EEPROM often reads 255 initially; accept only 0/1
  if (e != 0 && e != 1) e = 1;

  alarmHour = h;
  alarmMin = m;
  alarmEnabled = (e == 1);

  // Write back defaults to avoid weird "OFF" on first run
  saveAlarm();
}

void saveAlarm() {
  EEPROM.update(EE_ALARM_H, alarmHour);
  EEPROM.update(EE_ALARM_M, alarmMin);
  EEPROM.update(EE_ALARM_EN, alarmEnabled ? 1 : 0);
}

void buzzerOff() {
  if (USE_TONE_BUZZER) {
    noTone(PIN_BUZZER);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
  beepOn = false;
}

void buzzerOn() {
  if (USE_TONE_BUZZER) {
    tone(PIN_BUZZER, 2000);
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
  }
}

void toggleAlarmEnabled() {
  alarmEnabled = !alarmEnabled;
  saveAlarm();
}

// Encoder ISR: trigger on CLK rising edge, debounce with micros()
void onClkRise() {
  unsigned long now = micros();
  if (now - lastEncUs < 1500) return; // debounce (tune 1000–3000)
  lastEncUs = now;

  // Direction: read DT when CLK rises
  if (digitalRead(PIN_DT) == HIGH) encoderDelta--;
  else encoderDelta++;
}

void setup() {
  // Rotary pins
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT,  INPUT_PULLUP);
  pinMode(PIN_SW,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClkRise, RISING);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Starting...");

  // RTC
  Wire.begin(); // Mega uses SDA=20, SCL=21 automatically
  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC NOT FOUND");
    while (1) {}
  }

  loadAlarm();

  delay(700);
  lcd.clear();
}

void loop() {
  DateTime now = rtc.now();
  unsigned long ms = millis();

  // ---------- Button processing (short press vs long press) ----------
  bool btnRead = digitalRead(PIN_SW); // LOW when pressed

  // debounce edges
  if (btnRead != lastBtnRead && (ms - lastBtnDebounce) > 30) {
    lastBtnDebounce = ms;
    lastBtnRead = btnRead;

    if (btnRead == LOW) { // pressed down
      btnDown = true;
      btnDownMs = ms;
    } else { // released
      if (btnDown) {
        unsigned long held = ms - btnDownMs;
        btnDown = false;

        if (held >= 800) {
          // Long press: toggle alarm ON/OFF (only from main screen)
          if (state == SHOW_CLOCK) toggleAlarmEnabled();
        } else {
          // Short press:
          if (state == SHOW_CLOCK) {
            state = SET_HOUR;
          } else if (state == SET_HOUR) {
            state = SET_MIN;
          } else if (state == SET_MIN) {
            saveAlarm();

            // Prevent immediate trigger in the current minute
            DateTime n = rtc.now();
            lastTriggeredMinuteKey = (long)n.unixtime() / 60;

            state = SHOW_CLOCK;
          } else if (state == RINGING) {
            state = SHOW_CLOCK;
            buzzerOff();

            // also guard against re-triggering in same minute
            lastTriggeredMinuteKey = (long)now.unixtime() / 60;
          }
        }
      }
    }
  }

  // ---------- Encoder handling ----------
  // Ignore stray ticks unless editing
  if (state == SHOW_CLOCK || state == RINGING) {
    encoderDelta = 0;
  }

  if (encoderDelta != 0) {
    int d;
    noInterrupts();
    d = encoderDelta;
    encoderDelta = 0;
    interrupts();

    if (state == SET_HOUR) {
      alarmHour = (alarmHour + d) % 24;
      if (alarmHour < 0) alarmHour += 24;
    } else if (state == SET_MIN) {
      alarmMin = (alarmMin + d) % 60;
      if (alarmMin < 0) alarmMin += 60;
    }
  }

  // ---------- Alarm trigger (ONLY at second 00, NOT while setting) ----------
  if (state == SHOW_CLOCK && alarmEnabled) {
    long minuteKey = (long)now.unixtime() / 60; // changes every minute

    if (now.hour() == alarmHour &&
        now.minute() == alarmMin &&
        now.second() == 0 &&
        minuteKey != lastTriggeredMinuteKey) {

      lastTriggeredMinuteKey = minuteKey;
      state = RINGING;
      lastBeepToggle = ms;
      beepOn = false;
      buzzerOff();
    }
  }

  // ---------- Alarm sound pattern ----------
  if (state == RINGING) {
    if (ms - lastBeepToggle >= 250) {
      lastBeepToggle = ms;
      beepOn = !beepOn;
      if (beepOn) buzzerOn();
      else buzzerOff();
    }
  } else {
    buzzerOff();
  }

  // ---------- LCD draw ----------
  if (ms - lastDraw > 250) {
    lastDraw = ms;

    // Line 1: time + ON/OFF
    lcd.setCursor(0, 0);
    char l1[17];
    snprintf(l1, sizeof(l1), "%02d:%02d:%02d  %s",
             now.hour(), now.minute(), now.second(),
             alarmEnabled ? "ON " : "OFF");
    lcd.print(l1);

    // Line 2: state text
    lcd.setCursor(0, 1);
    if (state == SHOW_CLOCK) {
      char l2[17];
      snprintf(l2, sizeof(l2), "Alarm %02d:%02d       ", alarmHour, alarmMin);
      lcd.print(l2);
    } else if (state == SET_HOUR) {
      char l2[17];
      snprintf(l2, sizeof(l2), "Set HOUR: %02d       ", alarmHour);
      lcd.print(l2);
    } else if (state == SET_MIN) {
      char l2[17];
      snprintf(l2, sizeof(l2), "Set MIN:  %02d       ", alarmMin);
      lcd.print(l2);
    } else { // RINGING
      lcd.print("WAKE UP! Press   ");
    }
  }
}
