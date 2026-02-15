#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- RTC ----------------
RTC_DS3231 rtc;

// ---------------- Pins (YOUR mapping) ----------------
const int PWMA = 9;
const int AIN1 = 5;
const int AIN2 = 6;

const int PWMB = 10;
const int BIN1 = 7;
const int BIN2 = 8;

const int STBY = 11;

// Buzzer
const int PIN_BUZZER = A0;
const bool ACTIVE_BUZZER = true; // true=active buzzer, false=passive buzzer (tone)

// Encoder
const int PIN_CLK = 2;  // interrupt
const int PIN_DT  = 3;
const int PIN_SW  = 4;

// Ultrasonic (HC-SR04)
const int US_TRIG = 12;
const int US_ECHO = A1;

// ---------------- EEPROM addresses ----------------
const int EE_H  = 0;
const int EE_M  = 1;
const int EE_EN = 2;

// ---------------- UI state ----------------
enum UiState { SHOW_CLOCK, SET_HOUR, SET_MIN, RINGING };
UiState state = SHOW_CLOCK;

// ---------------- Encoder ISR ----------------
volatile int encoderDelta = 0;
volatile unsigned long lastEncUs = 0;

// ---------------- Alarm ----------------
int  alarmHour = 7;
int  alarmMin  = 30;
bool alarmEnabled = true;
long lastTriggeredMinuteKey = -1;

// ---------------- Timers ----------------
unsigned long lastDraw = 0;
unsigned long moveUntil = 0;

// Ultrasonic timing + value
unsigned long lastDistMs = 0;
float lastDistCm = -1;

// ---------------- Button handling ----------------
bool btnDown = false;
unsigned long btnDownMs = 0;
bool lastRawBtn = HIGH;
unsigned long lastRawChange = 0;
bool stableBtn = HIGH;
const unsigned long DEBOUNCE_MS = 40;

// ---------------- Improved Alarm Sound Pattern ----------------
// Pattern: beep 120ms, gap 80ms, beep 120ms, pause 500ms, repeat.
const unsigned long BEEP_ON_MS    = 120;
const unsigned long BEEP_GAP_MS   = 80;
const unsigned long BEEP_ON2_MS   = 120;
const unsigned long BEEP_PAUSE_MS = 500;

unsigned long beepPhaseStarted = 0;
int beepPhase = 0; // 0=on1,1=gap,2=on2,3=pause
bool beepOn = false;

void buzzerOff() {
  if (ACTIVE_BUZZER) digitalWrite(PIN_BUZZER, LOW);
  else noTone(PIN_BUZZER);
  beepOn = false;
}

void buzzerOn() {
  if (ACTIVE_BUZZER) digitalWrite(PIN_BUZZER, HIGH);
  else tone(PIN_BUZZER, 2200);
  beepOn = true;
}

void resetBeepPattern() {
  beepPhase = 0;
  beepPhaseStarted = millis();
  buzzerOn();
}

void updateBeepPattern(unsigned long ms) {
  switch (beepPhase) {
    case 0:
      if (ms - beepPhaseStarted >= BEEP_ON_MS) { buzzerOff(); beepPhase = 1; beepPhaseStarted = ms; }
      break;
    case 1:
      if (ms - beepPhaseStarted >= BEEP_GAP_MS) { buzzerOn();  beepPhase = 2; beepPhaseStarted = ms; }
      break;
    case 2:
      if (ms - beepPhaseStarted >= BEEP_ON2_MS) { buzzerOff(); beepPhase = 3; beepPhaseStarted = ms; }
      break;
    case 3:
      if (ms - beepPhaseStarted >= BEEP_PAUSE_MS) { beepPhase = 0; beepPhaseStarted = ms; buzzerOn(); }
      break;
  }
}

// ---------------- Helpers ----------------
void saveAlarm() {
  EEPROM.update(EE_H, alarmHour);
  EEPROM.update(EE_M, alarmMin);
  EEPROM.update(EE_EN, alarmEnabled ? 1 : 0);
}

void loadAlarm() {
  int h = EEPROM.read(EE_H);
  int m = EEPROM.read(EE_M);
  int e = EEPROM.read(EE_EN);

  if (h < 0 || h > 23) h = 7;
  if (m < 0 || m > 59) m = 30;
  if (e != 0 && e != 1) e = 1;

  alarmHour = h;
  alarmMin = m;
  alarmEnabled = (e == 1);
  saveAlarm();
}

void motorsStop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if (speed < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW); }
  analogWrite(PWMA, abs(speed));
}

void setMotorB(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if (speed < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW); }
  analogWrite(PWMB, abs(speed));
}

void drive(int left, int right) {
  setMotorA(left);
  setMotorB(right);
}

void pickNextMove() {
  const int base = 220;
  const int turn = 200;

  int choice = random(0, 4);
  unsigned long duration;

  if (choice == 0) {          // forward
    drive(base, base);
    duration = random(700, 1600);
  } else if (choice == 1) {   // reverse short
    drive(-base, -base);
    duration = random(250, 650);
  } else if (choice == 2) {   // spin left
    drive(-turn, turn);
    duration = random(250, 800);
  } else {                    // spin right
    drive(turn, -turn);
    duration = random(250, 800);
  }

  moveUntil = millis() + duration;
}

// ---- Ultrasonic ----
float readDistanceCM() {
  // Trigger pulse
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  // Measure echo (timeout 25ms ~ 4m)
  unsigned long duration = pulseIn(US_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1;           // no echo
  return duration / 58.0f;                // cm
}

// Obstacle avoidance: reverse then turn
void avoidObstacle() {
  const int base = 220;
  const int turn = 200;

  // reverse briefly
  drive(-base, -base);
  delay(180); // short + effective

  // random turn
  if (random(0, 2) == 0) drive(-turn, turn);
  else                   drive(turn, -turn);
  delay(260);

  // after this, next loop will pick movement again
  moveUntil = millis(); // force immediate re-pick
}

void startRinging(DateTime now) {
  state = RINGING;
  buzzerOff();
  resetBeepPattern();
  moveUntil = 0;
  lastTriggeredMinuteKey = (long)now.unixtime() / 60;
}

void stopRinging() {
  state = SHOW_CLOCK;
  buzzerOff();
  motorsStop();
}

// ---------------- Encoder ISR ----------------
void onClkRise() {
  unsigned long now = micros();
  if (now - lastEncUs < 1500) return;
  lastEncUs = now;

  if (digitalRead(PIN_DT) == HIGH) encoderDelta--;
  else encoderDelta++;
}

// ---------------- Button debounce + press type ----------------
void updateButton(unsigned long ms) {
  bool raw = digitalRead(PIN_SW);

  if (raw != lastRawBtn) {
    lastRawBtn = raw;
    lastRawChange = ms;
  }
  if (ms - lastRawChange >= DEBOUNCE_MS) {
    stableBtn = raw;
  }
}

// ---------------- Pretty OLED helpers ----------------
void print2(int v) {
  if (v < 10) display.print('0');
  display.print(v);
}

const char* stateLabel() {
  if (state == SHOW_CLOCK) return "READY";
  if (state == SET_HOUR)   return "SET H";
  if (state == SET_MIN)    return "SET M";
  return "RING!";
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(25000, true);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1) {}
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // RTC
  if (!rtc.begin()) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("RTC NOT FOUND");
    display.display();
    while (1) {}
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Encoder
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT,  INPUT_PULLUP);
  pinMode(PIN_SW,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClkRise, RISING);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  // TB6612
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH);
  motorsStop();

  // Ultrasonic
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  loadAlarm();

  // Use a pin that is NOT A1 (since A1 is used for echo)
  randomSeed(analogRead(A2));
}

// ---------------- Loop ----------------
void loop() {
  unsigned long ms = millis();
  updateButton(ms);

  DateTime now = rtc.now();

  // Update distance occasionally (non-spam)
  if (ms - lastDistMs >= 120) {
    lastDistMs = ms;
    float d = readDistanceCM();
    if (d > 0) lastDistCm = d;
  }

  // -------- handle encoder changes ----------
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

  // -------- button short vs long press ----------
  if (stableBtn == LOW && !btnDown) { // press started
    btnDown = true;
    btnDownMs = ms;
  }
  if (stableBtn == HIGH && btnDown) { // released
    unsigned long held = ms - btnDownMs;
    btnDown = false;

    if (held >= 1000) {
      if (state == SHOW_CLOCK) startRinging(now);
      else if (state == RINGING) stopRinging();
    } else {
      if (state == SHOW_CLOCK) state = SET_HOUR;
      else if (state == SET_HOUR) state = SET_MIN;
      else if (state == SET_MIN) {
        saveAlarm();
        lastTriggeredMinuteKey = (long)now.unixtime() / 60;
        state = SHOW_CLOCK;
      } else if (state == RINGING) {
        stopRinging();
      }
    }
  }

  // -------- alarm trigger at :00 only ----------
  if (state == SHOW_CLOCK && alarmEnabled) {
    long minuteKey = (long)now.unixtime() / 60;
    if (now.hour() == alarmHour &&
        now.minute() == alarmMin &&
        now.second() == 0 &&
        minuteKey != lastTriggeredMinuteKey) {
      startRinging(now);
    }
  }

  // -------- ringing behavior + obstacle avoidance ----------
  if (state == RINGING) {
    updateBeepPattern(ms);

    // If obstacle close, avoid immediately
    const float STOP_CM = 20.0; // tweak: 15-30cm
    if (lastDistCm > 0 && lastDistCm < STOP_CM) {
      avoidObstacle();
    } else {
      if (ms > moveUntil) pickNextMove();
    }
  } else {
    buzzerOff();
    motorsStop();
  }

  // -------- OLED draw (pretty layout + distance) ----------
  if (ms - lastDraw >= 200) {
    lastDraw = ms;
    display.clearDisplay();

    display.setTextSize(2);
    display.setCursor(0, 0);
    print2(now.hour());
    display.print(':');
    print2(now.minute());

    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("S:");
    print2(now.second());
    display.print(" A:");
    print2(alarmHour);
    display.print(':');
    print2(alarmMin);

    // show distance on the right
    display.setCursor(86, 20);
    display.print("D:");
    if (lastDistCm > 0) {
      int d = (int)(lastDistCm + 0.5);
      display.print(d);
      display.print("cm");
    } else {
      display.print("--");
    }

    display.display();
  }
}
