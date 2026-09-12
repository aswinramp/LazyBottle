/*
  =========================================================
   Wi-Fi Water Bottle Delivery Robot  (v2 — audio + reminder loop)
  =========================================================
  Board: ESP32
  Driver: TB6612FNG (dual DC motor driver)
  Audio:  DF Mini Player (MP3 module) -> speaker
  Sensor: IR obstacle/proximity sensor (digital out)

  HOW IT WORKS
  ------------
  1. ESP32 connects to your phone's Wi-Fi hotspot (as a station).
  2. Every cycle it takes a step (drive forward), then compares the
     new Wi-Fi signal strength (RSSI) to the previous reading.
     - If the signal got stronger, it keeps going the same way.
     - If it got weaker (or stayed flat), it arc-turns and tries
       again next cycle, sweeping a wider arc each miss.
  3. When it's close enough (RSSI above threshold) it stops,
     plays track 001 ("I'm here, drink some water!") and waits.
  4. IR sensor goes LOW when you lift the bottle out -> plays
     track 002 ("bottle taken").
  5. When you put the bottle back, IR goes HIGH again -> plays
     track 003 ("bottle back, thanks!").
  6. The robot then idles for TRIP_INTERVAL_MS (10 seconds) and
     automatically starts searching for you again, so it nags you
     to drink water on a repeating cycle.

  WIRING (as you specified)
  --------------------------------------------
  TB6612FNG:
    AIN1  -> GPIO 26
    AIN2  -> GPIO 27
    PWMA  -> GPIO 25
    BIN1  -> GPIO 14
    BIN2  -> GPIO 12
    PWMB  -> GPIO 13
    VM    -> Motor battery +
    VCC   -> ESP32 3.3V (logic)
    GND   -> Common ground with ESP32 AND motor battery

    NOTE ON STBY: you didn't list an STBY pin. The TB6612FNG will
    NOT move at all unless STBY is held HIGH. This sketch assumes
    you've wired STBY directly to 3.3V (or 5V) on the driver board
    itself, so the code never has to toggle it. If you actually do
    have STBY on a spare ESP32 pin, tell me which one and I'll add
    it back into the code.

  DF Mini Player (as you wired it):
    Module RX -> ESP32 GPIO 19  (ESP32 TX2 -> module RX)
    Module TX -> ESP32 GPIO 18  (module TX -> ESP32 RX2)
    SPK1 / SPK2 -> speaker terminals
    VCC -> 5V, GND -> common ground

    RESISTOR NOTE: put it on the Module TX -> ESP32 GPIO18 line, not
    the other one. The DFPlayer's TX pin can output close to its 5V
    supply level, and ESP32 RX pins are NOT 5V-tolerant. A small
    voltage divider (e.g. 1k in series, then 2k from that node to
    GND) on this line protects GPIO18. The ESP32 -> module RX line
    (GPIO19) is fine as-is; 3.3V registers as HIGH on the module.

  IR sensor:
    OUT -> ESP32 GPIO 4
    Bottle present  = LOW
    Bottle removed  = HIGH   (inverted per your latest testing)

  SD CARD (for DF Mini Player):
    Put three audio files in the ROOT of the SD card, named:
      0001.mp3   (or 001.mp3, depending on your module's firmware)
      0002.mp3
      0003.mp3
    Track 1 = arrival / "come drink water"
    Track 2 = "bottle taken"
    Track 3 = "bottle back"

  LIBRARY REQUIRED:
    DFRobotDFPlayerMini  (install via Arduino Library Manager,
    search "DFRobotDFPlayerMini" by DFRobot)
  =========================================================
*/

#include <WiFi.h>
#include <DFRobotDFPlayerMini.h>

// ============ USER CONFIG ============

// --- Wi-Fi credentials (your phone's hotspot) ---
const char* WIFI_SSID     = "700";
const char* WIFI_PASSWORD = "12345678";

// --- Motor driver pins (TB6612FNG) ---
const int PIN_AIN1 = 26;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;   // Left motor PWM
const int PIN_BIN1 = 14;
const int PIN_BIN2 = 12;
const int PIN_PWMB = 13;   // Right motor PWM

// --- DF Mini Player serial pins (ESP32 hardware UART2) ---
// Module RX is wired to ESP32 pin 19, so ESP32 pin 19 must act as TX.
// Module TX is wired to ESP32 pin 18, so ESP32 pin 18 must act as RX.
const int PIN_DFPLAYER_RX = 18;   // ESP32 RX2  <- module TX
const int PIN_DFPLAYER_TX = 19;   // ESP32 TX2  -> module RX

// --- Sensor pin ---
const int PIN_IR_SENSOR = 4;   // Digital IR output

// --- IR sensor logic ---
// Inverted per your latest wiring/testing: bottle present -> LOW,
// bottle removed -> HIGH.
const bool IR_LOW_MEANS_BOTTLE_PRESENT = true;

// --- Audio track numbers on the SD card ---
const int TRACK_ARRIVED         = 1;  // "I'm here, drink water"
const int TRACK_BOTTLE_TAKEN    = 2;  // "bottle taken"
const int TRACK_BOTTLE_RETURNED = 3;  // "bottle back"
const int DFPLAYER_VOLUME       = 22; // 0-30

// --- Motor speed settings (0-255 PWM) ---
const int DRIVE_SPEED = 170;   // Forward driving speed
const int TURN_SPEED  = 150;   // Pivot turning speed

// --- Timing settings (milliseconds) ---
const int MOVE_TURN_TIME_MS   = 260;   // How long to pivot when turning to search
const int DRIVE_TIME_MS       = 500;   // How long to drive forward each cycle
const int RSSI_SAMPLES        = 5;     // Number of RSSI readings averaged per measurement
const int RSSI_SAMPLE_GAP_MS  = 30;    // Delay between samples

// --- Arrival / stopping threshold ---
// RSSI closer to 0 = stronger signal. -50 or higher is "very close".
// Tune this based on real testing with your phone + robot.
const int ARRIVAL_RSSI_THRESHOLD = -50;

// --- Reminder loop timing ---
// After a full deliver -> taken -> returned cycle finishes, wait this
// long, then go find the user again. You asked for 10 seconds; bump
// this up (e.g. 600000UL for 10 minutes) once you've tested the flow,
// or the robot will nag you almost constantly.
const unsigned long TRIP_INTERVAL_MS = 10000UL;

// --- Manual hardware test mode ---
// Set this to true, upload, open Serial Monitor at 115200 baud, and
// send single characters to drive the robot manually:
//   f = forward, b = backward, l = arc left, r = arc right, s = stop
// CONFIRM these actually match real-world directions BEFORE trusting
// the automatic Wi-Fi search below - if "l" turns the chassis right
// in real life, the search algorithm will never work correctly no
// matter how good the RSSI logic is. Set back to false for normal use.
const bool MANUAL_TEST_MODE = false;

// ============ END USER CONFIG ============


// Robot states
enum RobotState {
  SEARCHING_AND_MOVING,
  ARRIVED_WAITING_FOR_PICKUP,
  WAITING_FOR_BOTTLE_RETURN,
  COOLDOWN_BEFORE_NEXT_TRIP
};

RobotState currentState = SEARCHING_AND_MOVING;
unsigned long cooldownStartMs = 0;

// DF Mini Player - uses ESP32 hardware UART2
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini myDFPlayer;
bool dfPlayerReady = false;

// ---------------- Motor low-level control ----------------

void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // PWM setup using ESP32 LEDC (works on all common ESP32 Arduino core versions)
  ledcAttach(PIN_PWMA, 5000, 8);  // 5kHz, 8-bit resolution
  ledcAttach(PIN_PWMB, 5000, 8);
}

// dir: 1 = forward, -1 = backward, 0 = stop (coast)
void setLeftMotor(int dir, int speed) {
  if (dir > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (dir < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }
  ledcWrite(PIN_PWMA, dir == 0 ? 0 : speed);
}

void setRightMotor(int dir, int speed) {
  if (dir > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else if (dir < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
  }
  ledcWrite(PIN_PWMB, dir == 0 ? 0 : speed);
}

void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

void driveForward(int speed, int durationMs) {
  setLeftMotor(1, speed);
  setRightMotor(1, speed);
  delay(durationMs);
  stopMotors();
}

// TURNING: this chassis has only 2 driven wheels at the rear plus a
// passive wheel/skid up front. A true in-place pivot (one wheel
// forward, other backward) makes the front wheel scrub the ground,
// which makes the turn angle inconsistent run to run. An ARC TURN
// (stop one wheel, drive only the other) is far more predictable on
// this kind of chassis, which matters a lot for the hill-climbing
// search below - it needs turns to behave the same way every time.

// Stops the left wheel, drives the right wheel -> chassis arcs left.
void turnLeft(int speed, int durationMs) {
  setLeftMotor(0, 0);
  setRightMotor(1, speed);
  delay(durationMs);
  stopMotors();
}

// Stops the right wheel, drives the left wheel -> chassis arcs right.
void turnRight(int speed, int durationMs) {
  setRightMotor(0, 0);
  setLeftMotor(1, speed);
  delay(durationMs);
  stopMotors();
}

// ---------------- Wi-Fi / RSSI ----------------

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi hotspot: ");
  Serial.println(WIFI_SSID);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    // If it takes too long, keep retrying rather than freezing forever
    if (millis() - startAttempt > 15000) {
      Serial.println("\nStill trying... check phone hotspot is on.");
      startAttempt = millis();
    }
  }
  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Returns an averaged RSSI reading (in dBm, negative number, closer to 0 = stronger)
int getAverageRSSI() {
  long total = 0;
  int validSamples = 0;

  for (int i = 0; i < RSSI_SAMPLES; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      total += WiFi.RSSI();
      validSamples++;
    }
    delay(RSSI_SAMPLE_GAP_MS);
  }

  if (validSamples == 0) return -100; // treat "no signal" as very weak
  return (int)(total / validSamples);
}

void ensureWifiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi dropped, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
      delay(200);
    }
  }
}

// ---------------- IR sensor ----------------

void irInit() {
  pinMode(PIN_IR_SENSOR, INPUT);
}

bool isBottlePresent() {
  int reading = digitalRead(PIN_IR_SENSOR);
  if (IR_LOW_MEANS_BOTTLE_PRESENT) {
    return reading == LOW;
  } else {
    return reading == HIGH;
  }
}

// ---------------- DF Mini Player audio ----------------

void dfPlayerInit() {
  dfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
  Serial.println("Initializing DF Mini Player...");

  // The module needs a moment after power-up to mount its SD card
  // before it will respond. Give it time, and retry once if the
  // first attempt fails, instead of giving up immediately.
  delay(1500);

  dfPlayerReady = myDFPlayer.begin(dfSerial);
  if (!dfPlayerReady) {
    Serial.println("DF Mini Player not detected - retrying once...");
    delay(1000);
    dfPlayerReady = myDFPlayer.begin(dfSerial);
  }

  if (dfPlayerReady) {
    myDFPlayer.volume(DFPLAYER_VOLUME);
    Serial.println("DF Mini Player ready.");
  } else {
    Serial.println("DF Mini Player NOT detected - check wiring/SD card.");
  }
}

void playTrack(int trackNum) {
  if (!dfPlayerReady) {
    Serial.println("DF Mini Player not ready, skipping audio.");
    return;
  }
  myDFPlayer.play(trackNum);
}

// ---------------- Core search-and-move logic ----------------
// Strategy: HILL CLIMBING instead of pivot-scanning.
// Pivoting a few degrees in place barely changes RSSI (the signal
// difference is often smaller than normal Wi-Fi noise), so trying to
// "sense" direction from a quick left/right peek is unreliable and
// can send the robot the wrong way. Instead: take a step, see if the
// signal actually got better. If yes, keep going that way. If not,
// turn (a bit further each miss) and try again.

const int RSSI_UNSET = -200; // sentinel meaning "no previous reading yet"
int lastRSSI = RSSI_UNSET;
int worseStreak = 0;

void resetSearch() {
  lastRSSI = RSSI_UNSET;
  worseStreak = 0;
}

void searchAndMoveOnce() {
  ensureWifiConnected();

  int currentRSSI = getAverageRSSI();
  char prevStr[8];
  if (lastRSSI == RSSI_UNSET) {
    snprintf(prevStr, sizeof(prevStr), "none");
  } else {
    snprintf(prevStr, sizeof(prevStr), "%d", lastRSSI);
  }
  Serial.printf("RSSI now: %d  (previous: %s)\n", currentRSSI, prevStr);

  // 1. Check if we've arrived
  if (currentRSSI >= ARRIVAL_RSSI_THRESHOLD) {
    Serial.println("Signal strong enough - arrived. Waiting for pickup.");
    stopMotors();
    playTrack(TRACK_ARRIVED);
    currentState = ARRIVED_WAITING_FOR_PICKUP;
    return;
  }

  // 2. First move after starting/resetting: no history yet, just step forward
  if (lastRSSI == RSSI_UNSET) {
    Serial.println("First reading - stepping forward to establish a baseline.");
    driveForward(DRIVE_SPEED, DRIVE_TIME_MS);
  }
  // 3. Signal improved since last step -> keep going the same way
  else if (currentRSSI > lastRSSI) {
    Serial.println("Signal improved - continuing straight.");
    worseStreak = 0;
    driveForward(DRIVE_SPEED, DRIVE_TIME_MS);
  }
  // 4. Signal got weaker (or flat) -> turn and try a different heading.
  //    Alternate left/right, widening the turn each consecutive miss,
  //    so it sweeps a wider arc instead of getting stuck.
  else {
    worseStreak++;
    int turnTime = MOVE_TURN_TIME_MS * ((worseStreak + 1) / 2);
    if (worseStreak % 2 == 1) {
      Serial.println("Signal dropped - arcing LEFT to search.");
      turnLeft(TURN_SPEED, turnTime);
    } else {
      Serial.println("Signal dropped - arcing RIGHT to search.");
      turnRight(TURN_SPEED, turnTime);
    }
    driveForward(DRIVE_SPEED, DRIVE_TIME_MS);
  }

  lastRSSI = currentRSSI;
}

// ---------------- Arduino setup / loop ----------------

void setup() {
  Serial.begin(115200);
  delay(300);

  motorsInit();
  irInit();
  dfPlayerInit();
  wifiInit();
  resetSearch();

  Serial.println("Wi-Fi Water Bottle Delivery Robot ready.");
  if (MANUAL_TEST_MODE) {
    Serial.println("MANUAL TEST MODE - send f/b/l/r/s to drive.");
  }
}

void runManualTestMode() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': Serial.println("forward");   setLeftMotor(1, DRIVE_SPEED); setRightMotor(1, DRIVE_SPEED); break;
      case 'b': Serial.println("backward");  setLeftMotor(-1, DRIVE_SPEED); setRightMotor(-1, DRIVE_SPEED); break;
      case 'l': Serial.println("arc left");  setLeftMotor(0, 0); setRightMotor(1, TURN_SPEED); break;
      case 'r': Serial.println("arc right"); setRightMotor(0, 0); setLeftMotor(1, TURN_SPEED); break;
      case 's': Serial.println("stop");      stopMotors(); break;
      default: break;
    }
  }
}

void loop() {
  if (MANUAL_TEST_MODE) {
    runManualTestMode();
    return;
  }
  switch (currentState) {

    case SEARCHING_AND_MOVING:
      searchAndMoveOnce();
      break;

    case ARRIVED_WAITING_FOR_PICKUP:
      stopMotors();
      if (!isBottlePresent()) {
        Serial.println("Bottle taken!");
        playTrack(TRACK_BOTTLE_TAKEN);
        currentState = WAITING_FOR_BOTTLE_RETURN;
      }
      delay(150); // idle-poll the IR sensor
      break;

    case WAITING_FOR_BOTTLE_RETURN:
      stopMotors();
      if (isBottlePresent()) {
        Serial.println("Bottle placed back!");
        playTrack(TRACK_BOTTLE_RETURNED);
        cooldownStartMs = millis();
        currentState = COOLDOWN_BEFORE_NEXT_TRIP;
      }
      delay(150); // idle-poll the IR sensor
      break;

    case COOLDOWN_BEFORE_NEXT_TRIP:
      stopMotors();
      if (millis() - cooldownStartMs >= TRIP_INTERVAL_MS) {
        Serial.println("Time to remind you to drink water again - heading out.");
        resetSearch();
        currentState = SEARCHING_AND_MOVING;
      }
      delay(100);
      break;
  }
}
