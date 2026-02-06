#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

// ---------- LCD ----------
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ---------- RTC ----------
RTC_DS1307 rtc;

// ---------- Rotary ----------
const int PIN_CLK = 18;
const int PIN_DT  = 19;
const int PIN_SW  = 6;

// ---------- Buzzer ----------
const int PIN_BUZZER = 8;
// If you can, prefer active buzzer mode for robustness during motor PWM:
const bool USE_TONE_BUZZER = false;

// ---------- L293D Motor Pins ----------
const int ENA = 9;
const int IN1 = 22;
const int IN2 = 23;
const int ENB = 10;
const int IN3 = 24;
const int IN4 = 25;

// ---------- EEPROM ----------
const int EE_ALARM_H  = 0;
const int EE_ALARM_M  = 1;
const int EE_ALARM_EN = 2;

// ---------- UI state ----------
enum UiState { SHOW_CLOCK, SET_HOUR, SET_MIN, RINGING };
UiState state = SHOW_CLOCK;

// ---------- Encoder ----------
volatile int encoderDelta = 0;
volatile unsigned long lastEncUs = 0;

// ---------- Button debounce ----------
bool stableBtn = HIGH;
bool lastStableBtn = HIGH;
bool rawBtnLast = HIGH;
unsigned long rawChangedAt = 0;
const unsigned long BTN_STABLE_MS = 80;

bool btnDown = false;
unsigned long btnDownMs = 0;

// ---------- Display timing ----------
unsigned long lastDraw = 0;

// ---------- Alarm ----------
int  alarmHour = 7;
int  alarmMin  = 30;
bool alarmEnabled = true;
long lastTriggeredMinuteKey = -1;

// ---------- Buzzer pattern ----------
unsigned long lastBeepToggle = 0;
bool beepOn = false;

// ---------- Movement pattern ----------
unsigned long moveUntil = 0;
unsigned long ignoreButtonUntil = 0;

// ---------- Helpers ----------
void saveAlarm();

void loadAlarm() {
  int h = EEPROM.read(EE_ALARM_H);
  int m = EEPROM.read(EE_ALARM_M);
  int e = EEPROM.read(EE_ALARM_EN);

  if (h < 0 || h > 23) h = 7;
  if (m < 0 || m > 59) m = 30;
  if (e != 0 && e != 1) e = 1;

  alarmHour = h;
  alarmMin = m;
  alarmEnabled = (e == 1);

  saveAlarm();
}

void saveAlarm() {
  EEPROM.update(EE_ALARM_H, alarmHour);
  EEPROM.update(EE_ALARM_M, alarmMin);
  EEPROM.update(EE_ALARM_EN, alarmEnabled ? 1 : 0);
}

void buzzerOff() {
  if (USE_TONE_BUZZER) noTone(PIN_BUZZER);
  else digitalWrite(PIN_BUZZER, LOW);
  beepOn = false;
}

void buzzerOn() {
  if (USE_TONE_BUZZER) tone(PIN_BUZZER, 2000);
  else digitalWrite(PIN_BUZZER, HIGH);
}

void toggleAlarmEnabled() {
  alarmEnabled = !alarmEnabled;
  saveAlarm();
}

// ---- Motor control ----
void motorsStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motorLeft(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  analogWrite(ENA, speed);
}

void motorRight(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  analogWrite(ENB, speed);
}

void driveLR(int left, int right) {
  motorLeft(abs(left), left >= 0);
  motorRight(abs(right), right >= 0);
}

void pickNextMove() {
  const int base = 255;  // you said 255 made it faster
  const int turn = 230;

  int choice = random(0, 4);
  unsigned long duration;

  if (choice == 0) {
    driveLR(base, base);
    duration = random(600, 1400);
  } else if (choice == 1) {
    driveLR(-base, -base);
    duration = random(250, 650);
  } else if (choice == 2) {
    driveLR(-turn, turn);
    duration = random(250, 750);
  } else {
    driveLR(turn, -turn);
    duration = random(250, 750);
  }

  moveUntil = millis() + duration;
}

// ---- Encoder ISR ----
void onClkRise() {
  unsigned long now = micros();
  if (now - lastEncUs < 1500) return;
  lastEncUs = now;

  if (digitalRead(PIN_DT) == HIGH) encoderDelta--;
  else encoderDelta++;
}

// ---- Button debounce ----
void updateButton(unsigned long ms) {
  bool raw = digitalRead(PIN_SW);

  if (raw != rawBtnLast) {
    rawBtnLast = raw;
    rawChangedAt = ms;
  }
  if ((ms - rawChangedAt) >= BTN_STABLE_MS) {
    stableBtn = raw;
  }
}

// ---- RTC safe read + recovery ----
bool safeRtcNow(DateTime &out) {
  // Try once
  out = rtc.now();

  // Basic sanity: DS1307 should never return year < 2000 in your setup
  if (out.year() < 2000 || out.year() > 2099) {
    // Attempt I2C recovery
    Wire.begin();
    rtc.begin();
    delay(5);
    out = rtc.now();
    if (out.year() < 2000 || out.year() > 2099) return false;
  }
  return true;
}

void lcdRecover() {
  lcd.begin(16, 2);
  lcd.clear();
}

void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT,  INPUT_PULLUP);
  pinMode(PIN_SW,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClkRise, RISING);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motorsStop();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Starting...");

  Wire.begin();
  // Prevent infinite I2C lockups:
  Wire.setWireTimeout(25000, true); // 25ms timeout, auto reset

  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC NOT FOUND");
    while (1) {}
  }

  loadAlarm();
  randomSeed(analogRead(A0));

  delay(700);
  lcd.clear();

  unsigned long ms = millis();
  rawBtnLast = digitalRead(PIN_SW);
  rawChangedAt = ms;
  stableBtn = rawBtnLast;
  lastStableBtn = stableBtn;
}

void loop() {
  unsigned long ms = millis();

  DateTime now;
  if (!safeRtcNow(now)) {
    // If RTC read fails, stop everything and recover LCD
    motorsStop();
    buzzerOff();
    lcdRecover();
    lcd.setCursor(0, 0);
    lcd.print("RTC BUS ERROR   ");
    lcd.setCursor(0, 1);
    lcd.print("Check SDA/SCL   ");
    delay(200);
    return;
  }

  updateButton(ms);

  bool buttonEventsAllowed = (ms >= ignoreButtonUntil);

  if (buttonEventsAllowed && stableBtn != lastStableBtn) {
    lastStableBtn = stableBtn;

    if (stableBtn == LOW) {
      btnDown = true;
      btnDownMs = ms;
    } else {
      if (btnDown) {
        unsigned long held = ms - btnDownMs;
        btnDown = false;

        if (held >= 800) {
          if (state == SHOW_CLOCK) toggleAlarmEnabled();
        } else {
          if (state == SHOW_CLOCK) state = SET_HOUR;
          else if (state == SET_HOUR) state = SET_MIN;
          else if (state == SET_MIN) {
            saveAlarm();
            lastTriggeredMinuteKey = (long)now.unixtime() / 60;
            state = SHOW_CLOCK;
          } else if (state == RINGING) {
            state = SHOW_CLOCK;
            motorsStop();
            buzzerOff();
            lastTriggeredMinuteKey = (long)now.unixtime() / 60;
          }
        }
      }
    }
  }

  if (state == SHOW_CLOCK || state == RINGING) encoderDelta = 0;

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

  // Trigger at :00 only, only on main screen
  if (state == SHOW_CLOCK && alarmEnabled) {
    long minuteKey = (long)now.unixtime() / 60;
    if (now.hour() == alarmHour &&
        now.minute() == alarmMin &&
        now.second() == 0 &&
        minuteKey != lastTriggeredMinuteKey) {
      lastTriggeredMinuteKey = minuteKey;
      state = RINGING;

      lastBeepToggle = ms;
      beepOn = false;
      buzzerOff();

      moveUntil = 0;
      ignoreButtonUntil = ms + 2000;
      btnDown = false;
    }
  }

  if (state == RINGING) {
    // Beep (keep it simple during motor PWM)
    if (ms - lastBeepToggle >= 250) {
      lastBeepToggle = ms;
      beepOn = !beepOn;
      if (beepOn) buzzerOn();
      else buzzerOff();
    }

    if (ms > moveUntil) pickNextMove();
  } else {
    motorsStop();
    buzzerOff();
  }

  if (ms - lastDraw > 250) {
    lastDraw = ms;

    lcd.setCursor(0, 0);
    char l1[17];
    snprintf(l1, sizeof(l1), "%02d:%02d:%02d  %s",
             now.hour(), now.minute(), now.second(),
             alarmEnabled ? "ON " : "OFF");
    lcd.print(l1);

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
    } else {
      lcd.print("RUNNING! Press   ");
    }
  }
}
