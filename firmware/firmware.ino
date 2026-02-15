#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= RTC =================
RTC_DS3231 rtc;

// ================= Pins (YOUR mapping) =================
// TB6612
const int PWMA = 9;
const int AIN1 = 5;
const int AIN2 = 6;

const int PWMB = 10;
const int BIN1 = 7;
const int BIN2 = 8;

const int STBY = 11;

// Buzzer
const int PIN_BUZZER = A0;
const bool ACTIVE_BUZZER = true; // true=active buzzer, false=passive (tone)

// Encoder
const int PIN_CLK = 2;  // interrupt-capable on Nano
const int PIN_DT  = 3;
const int PIN_SW  = 4;

// Ultrasonic (HC-SR04)
const int US_TRIG = 12;
const int US_ECHO = A1;

// ================= EEPROM addresses =================
const int EE_H  = 0;
const int EE_M  = 1;
const int EE_EN = 2;

// ================= UI state =================
enum UiState { SHOW_CLOCK, SET_HOUR, SET_MIN, RINGING };
UiState state = SHOW_CLOCK;

// ================= Encoder ISR =================
volatile int encoderDelta = 0;
volatile unsigned long lastEncUs = 0;

// ================= Alarm =================
int  alarmHour = 7;
int  alarmMin  = 30;
bool alarmEnabled = true;
long lastTriggeredMinuteKey = -1;

// ================= Timers =================
unsigned long lastDraw = 0;

// Ultrasonic timing + value
unsigned long lastDistMs = 0;
float lastDistCm = -1;

// ================= Button handling =================
bool btnDown = false;
unsigned long btnDownMs = 0;
bool lastRawBtn = HIGH;
unsigned long lastRawChange = 0;
bool stableBtn = HIGH;
const unsigned long DEBOUNCE_MS = 60;   // a bit stronger
unsigned long ignoreButtonUntil = 0;    // ignore button right after ringing starts

// ================= Simple Beep (consistent) =================
unsigned long lastBeepToggle = 0;
bool beepState = false;
const unsigned long BEEP_INTERVAL_MS = 300; // 300ms ON, 300ms OFF

// ================= Movement tuning =================
const int BASE_SPEED = 170;   // try 150–200
const int TURN_SPEED = 150;   // try 130–180
const float STOP_CM = 20.0f;  // obstacle threshold

// ================= Movement state (NON-BLOCKING) =================
enum MoveMode { MOVE_IDLE, MOVE_RANDOM, MOVE_AVOID_REV, MOVE_AVOID_TURN };
MoveMode moveMode = MOVE_IDLE;
unsigned long moveUntil = 0;

// ================= Bell Icon =================
void drawBellIcon(int x, int y) {
  display.drawCircle(x + 5, y + 3, 3, SSD1306_WHITE);
  display.drawLine(x + 2, y + 4, x + 2, y + 7, SSD1306_WHITE);
  display.drawLine(x + 8, y + 4, x + 8, y + 7, SSD1306_WHITE);
  display.drawLine(x + 2, y + 7, x + 8, y + 7, SSD1306_WHITE);
  display.fillCircle(x + 5, y + 9, 1, SSD1306_WHITE);
  display.fillCircle(x + 5, y + 0, 1, SSD1306_WHITE);
}

// ================= Buzzer =================
void buzzerOff() {
  if (ACTIVE_BUZZER) digitalWrite(PIN_BUZZER, LOW);
  else noTone(PIN_BUZZER);
}
void buzzerOn() {
  if (ACTIVE_BUZZER) digitalWrite(PIN_BUZZER, HIGH);
  else tone(PIN_BUZZER, 2200);
}
void resetSimpleBeep(unsigned long ms) {
  lastBeepToggle = ms;
  beepState = false;
  buzzerOff();
}
void updateSimpleBeep(unsigned long ms) {
  if (ms - lastBeepToggle >= BEEP_INTERVAL_MS) {
    lastBeepToggle = ms;
    beepState = !beepState;
    if (beepState) buzzerOn();
    else buzzerOff();
  }
}

// ================= EEPROM helpers =================
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

// ================= Motors =================
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

// Pick next random move (NO delay)
void pickNextMove(unsigned long ms) {
  int choice = random(0, 4);
  unsigned long dur;

  if (choice == 0) {          // forward
    drive(BASE_SPEED, BASE_SPEED);
    dur = random(800, 1700);
  } else if (choice == 1) {   // reverse short
    drive(-BASE_SPEED, -BASE_SPEED);
    dur = random(300, 750);
  } else if (choice == 2) {   // spin left
    drive(-TURN_SPEED, TURN_SPEED);
    dur = random(250, 900);
  } else {                    // spin right
    drive(TURN_SPEED, -TURN_SPEED);
    dur = random(250, 900);
  }

  moveUntil = ms + dur;
  moveMode = MOVE_RANDOM;
}

// Ultrasonic
float readDistanceCM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1;
  return duration / 58.0f;
}

// Start avoid sequence (NON-blocking)
void startAvoid(unsigned long ms) {
  // reverse for 180ms
  drive(-BASE_SPEED, -BASE_SPEED);
  moveMode = MOVE_AVOID_REV;
  moveUntil = ms + 180;
}

// Continue avoid sequence (NON-blocking)
void updateAvoid(unsigned long ms) {
  if (moveMode == MOVE_AVOID_REV && ms >= moveUntil) {
    // then turn for 280ms
    if (random(0, 2) == 0) drive(-TURN_SPEED, TURN_SPEED);
    else                   drive(TURN_SPEED, -TURN_SPEED);

    moveMode = MOVE_AVOID_TURN;
    moveUntil = ms + 280;
  } else if (moveMode == MOVE_AVOID_TURN && ms >= moveUntil) {
    // finished avoid, immediately pick new move
    pickNextMove(ms);
  }
}

// RTC safe read + recovery (like your old code)
bool safeRtcNow(DateTime &out) {
  out = rtc.now();
  if (out.year() < 2000 || out.year() > 2099) {
    Wire.begin();
    rtc.begin();
    delay(5);
    out = rtc.now();
    if (out.year() < 2000 || out.year() > 2099) return false;
  }
  return true;
}

// OLED recover
void oledRecover() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

// ================= Encoder ISR =================
void onClkRise() {
  unsigned long now = micros();
  if (now - lastEncUs < 1500) return;
  lastEncUs = now;
  if (digitalRead(PIN_DT) == HIGH) encoderDelta--;
  else encoderDelta++;
}

// Button debounce
void updateButton(unsigned long ms) {
  bool raw = digitalRead(PIN_SW);
  if (raw != lastRawBtn) {
    lastRawBtn = raw;
    lastRawChange = ms;
  }
  if (ms - lastRawChange >= DEBOUNCE_MS) stableBtn = raw;
}

// OLED helper
void print2(int v) {
  if (v < 10) display.print('0');
  display.print(v);
}

// Alarm control
void startRinging(DateTime now, unsigned long ms) {
  state = RINGING;
  resetSimpleBeep(ms);
  moveMode = MOVE_IDLE;
  moveUntil = 0;
  lastTriggeredMinuteKey = (long)now.unixtime() / 60;

  // ignore button for 2s (vibration/noise)
  ignoreButtonUntil = ms + 2000;
  btnDown = false;
}

void stopRinging() {
  state = SHOW_CLOCK;
  buzzerOff();
  motorsStop();
  DateTime n = rtc.now();
  lastTriggeredMinuteKey = (long)n.unixtime() / 60;
}

// ================= Setup =================
void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setWireTimeout(25000, true); // 25ms (your old code comment was wrong; value is ms)

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1) {}
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
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

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
  randomSeed(analogRead(A2));
}

// ================= Loop =================
void loop() {
  unsigned long ms = millis();
  updateButton(ms);

  // Safe RTC read
  DateTime now;
  if (!safeRtcNow(now)) {
    motorsStop();
    buzzerOff();
    oledRecover();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("RTC BUS ERROR");
    display.setCursor(0, 10);
    display.println("Check SDA/SCL");
    display.display();
    delay(200);
    return;
  }

  // Ultrasonic read (not too often)
  if (ms - lastDistMs >= 140) {
    lastDistMs = ms;
    float d = readDistanceCM();
    if (d > 0) lastDistCm = d;
  }

  // Encoder: only in setting states
  if (state == SHOW_CLOCK || state == RINGING) {
    noInterrupts(); encoderDelta = 0; interrupts();
  }

  if (encoderDelta != 0) {
    int d;
    noInterrupts(); d = encoderDelta; encoderDelta = 0; interrupts();

    if (state == SET_HOUR) {
      alarmHour = (alarmHour + d) % 24;
      if (alarmHour < 0) alarmHour += 24;
    } else if (state == SET_MIN) {
      alarmMin = (alarmMin + d) % 60;
      if (alarmMin < 0) alarmMin += 60;
    }
  }

  // ================= BUTTON LOGIC =================
  bool buttonEventsAllowed = (ms >= ignoreButtonUntil);

  if (buttonEventsAllowed) {
    if (stableBtn == LOW && !btnDown) { btnDown = true; btnDownMs = ms; }

    if (stableBtn == HIGH && btnDown) {
      unsigned long held = ms - btnDownMs;
      btnDown = false;

      // SHORT press: stop alarm immediately when ringing
      if (state == RINGING) {
        stopRinging();
        return;
      }

      // Long press: toggle alarm ON/OFF (only on clock screen)
      if (held >= 1000) {
        if (state == SHOW_CLOCK) {
          alarmEnabled = !alarmEnabled;
          saveAlarm();
        }
      } else {
        // Short press: enter/advance setting
        if (state == SHOW_CLOCK) state = SET_HOUR;
        else if (state == SET_HOUR) state = SET_MIN;
        else if (state == SET_MIN) {
          saveAlarm();
          lastTriggeredMinuteKey = (long)now.unixtime() / 60;
          state = SHOW_CLOCK;
        }
      }
    }
  }

  // Alarm trigger at :00 only
  if (state == SHOW_CLOCK && alarmEnabled) {
    long minuteKey = (long)now.unixtime() / 60;
    if (now.hour() == alarmHour && now.minute() == alarmMin &&
        now.second() == 0 && minuteKey != lastTriggeredMinuteKey) {
      startRinging(now, ms);
    }
  }

  // Ringing behavior (non-blocking)
  if (state == RINGING) {
    updateSimpleBeep(ms);

    // obstacle avoidance state machine
    if (moveMode == MOVE_AVOID_REV || moveMode == MOVE_AVOID_TURN) {
      updateAvoid(ms);
    } else {
      if (lastDistCm > 0 && lastDistCm < STOP_CM) {
        startAvoid(ms);
      } else {
        if (ms >= moveUntil) pickNextMove(ms);
      }
    }
  } else {
    buzzerOff();
    motorsStop();
    moveMode = MOVE_IDLE;
  }

  // OLED draw
  if (ms - lastDraw >= 200) {
    lastDraw = ms;
    display.clearDisplay();

    display.setTextSize(2);
    display.setCursor(0, 0);
    print2(now.hour()); display.print(':');
    print2(now.minute()); display.print(':');
    print2(now.second());

    if (alarmEnabled) drawBellIcon(118, 2);

    display.setTextSize(1);
    display.setCursor(0, 16);
    display.print("Alarm: ");
    print2(alarmHour); display.print(':'); print2(alarmMin);

    const int HOUR_X = 42;
    const int MIN_X  = 60;
    if (state == SET_HOUR) { display.setCursor(HOUR_X, 24); display.print("^^"); }
    else if (state == SET_MIN) { display.setCursor(MIN_X, 24); display.print("^^"); }

    display.display();
  }
}
