// ============================================================
//  Line Following Robot — Line Maze Solver (Left-Hand Rule)
// ============================================================

#include <Adafruit_NeoPixel.h>

#define LED_PIN  7
#define NUM_LEDS 4

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Motor Pins ---
const int MOTOR_A_1 = 11;  // Left  backward
const int MOTOR_A_2 = 10;  // Left  forward
const int MOTOR_B_1 = 9;   // Right forward
const int MOTOR_B_2 = 5;   // Right backward (⚠️ not PWM — rewire to pin 3 or 5 if possible)

// --- Sensors ---
const int SENSOR_COUNT = 8;
const int SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_WEIGHTS[8] = { 7,  5,  3,  1, -1, -3, -5, -7};
const int THRESHOLD = 950;

// --- PD Gains ---
const float Kp = 20.0;
const float Kd = 12.0;

// --- Speeds ---
const int SPEED_BASE  = 255;   // slower = more control at turns
const int SPEED_MIN = 160;
const int SPEED_PIVOT = 180;   // slightly slower pivot, less overshoot

// --- Timing ---
const unsigned long CROSSING_MS = 140;  // back down from 300 — less overshoot
const unsigned long BLIND_MS = 100;  // start checking for line sooner after pivot
const unsigned long TURN_180_MS = 600;   // Timed blind phase for u-turns only
const unsigned long COOLDOWN_MS = 250;  // shorter cooldown at lower speed
const unsigned long TIMEOUT_MS = 5000;  // Max spin before giving up
const unsigned long IGNORE_FINISH_MS = 3000;  // Don't detect finish in first 3s
const unsigned long FINISH_CONFIRM_MS = 100;   // FIX 3: raised from 30ms to reduce false triggers
const unsigned long DEBUG_INTERVAL = 50;    // FIX 5: only print every 50ms

// --- States ---
enum State { FOLLOWING, CROSSING, TURNING, SEARCH, COOLDOWN, RECOVER, FINISH, BACKUP, DONE };
State state = FOLLOWING;

// --- Turn direction: -1 = left, 0 = straight, 1 = right, 2 = u-turn ---
int pendingTurn = 0;

// --- Tracking ---
float lastError = 0;
int lastDirection = 0;
unsigned long stateStart = 0;
unsigned long raceStart = 0;
unsigned long finishTimer = 0;
unsigned long lastDebug = 0;  // FIX 5

// ============================================================
//  FIX 1 — Forward declarations so LED functions can be called
//  from motor helpers defined above them
// ============================================================
void showForwardLights();
void showBrakeLights();
void showLeftLights();
void showRightLights();

// ============================================================
//  Motor Helpers
// ============================================================

void stopMotors() {
  analogWrite(MOTOR_A_1, 0); analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0); analogWrite(MOTOR_B_2, 0);
  showBrakeLights();
}

// Replace the entire setMotor function with this:
void setMotor(int fwd, int bwd, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(fwd, speed);
    analogWrite(bwd, 0);
  } else {
    analogWrite(fwd, 0);
    analogWrite(bwd, abs(speed));
  }
}

void driveForward(int spd = SPEED_BASE) {
  setMotor(MOTOR_A_2, MOTOR_A_1, spd);
  setMotor(MOTOR_B_1, MOTOR_B_2, spd);
  showForwardLights();
}

void driveReverse(int spd = SPEED_BASE) {
  setMotor(MOTOR_A_2, MOTOR_A_1, -spd);
  setMotor(MOTOR_B_1, MOTOR_B_2, -spd);
  showBrakeLights();
}

void pivotLeft(int spd = SPEED_PIVOT) {
  setMotor(MOTOR_A_2, MOTOR_A_1, -spd);
  setMotor(MOTOR_B_1, MOTOR_B_2,  spd);
  showLeftLights();
}

void pivotRight(int spd = SPEED_PIVOT) {
  setMotor(MOTOR_A_2, MOTOR_A_1,  spd);
  setMotor(MOTOR_B_1, MOTOR_B_2, -spd);
  showRightLights();
}

// ============================================================
//  PD Line Following
// ============================================================

void drive(float pos) {
  int base = map((int)(abs(pos) * 10), 0, 70, SPEED_BASE, SPEED_MIN);
  float corr = Kp * pos + Kd * (pos - lastError);
  lastError  = pos;
  if (pos < 0) showLeftLights();
  else if (pos > 0) showRightLights();
  else showForwardLights();
  setMotor(MOTOR_A_2, MOTOR_A_1, base - (int)corr);
  setMotor(MOTOR_B_1, MOTOR_B_2, base + (int)corr);
}

// ============================================================
//  Sensors
// ============================================================

int readSensors(int vals[], long &wsum) {
  wsum = 0;
  int active = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    vals[i] = analogRead(SENSOR_PINS[i]);
    int on = (vals[i] > THRESHOLD) ? 1 : 0;
    wsum += on * SENSOR_WEIGHTS[i];
    active += on;
  }
  return active;
}

// ============================================================
//  Junction Detection — left-hand rule
//  Sensor layout (A0 = leftmost, A7 = rightmost):
//    Left  zone: sensors 5–7 (indices 5,6,7)  — FIX 2: corrected comment
//    Centre zone: sensors 3–4 (indices 3,4)
//    Right zone: sensors 0–2 (indices 0,1,2)
//  Returns: -1 = left, 0 = straight, 1 = right, 2 = u-turn, -99 = not a junction
// ============================================================

// int chooseTurn(int vals[]) {
//   int L = 0, C = 0, R = 0;
//   for (int i = 0; i < SENSOR_COUNT; i++) {
//     int on = (vals[i] > THRESHOLD) ? 1 : 0;
//     if (i <= 2) L += on;      
//     if (i == 3 || i == 4) C += on; 
//     if (i >= 5) R += on;      
//   }

//   bool canLeft = (L >= 2);
//   bool canRight = (R >= 2);
//   // Only say "Straight" if the center is VERY solid and there are NO turns
//   bool canStraight = (vals[3] > THRESHOLD && vals[4] > THRESHOLD);

//   // --- NEW PRIORITY: Any Turn > Straight ---
//   if (canLeft) return -1;   // Left-hand rule priority
//   if (canRight) return 1;    // If no left, take the right immediately
//   if (canStraight) return 0; // Only go straight if no side turns exist
  
//   return -99; 
// }


int chooseTurn(int vals[]) {
  // 1. Detect presence in each zone
  bool L = (vals[5] > THRESHOLD || vals[6] > THRESHOLD || vals[7] > THRESHOLD);
  bool C = (vals[3] > THRESHOLD || vals[4] > THRESHOLD);
  bool R = (vals[0] > THRESHOLD || vals[1] > THRESHOLD || vals[2] > THRESHOLD);
  
  // 2. Combine into a single "Junction ID"
  // If L=1, C=1, R=0 -> JunctionID = 6 (4 + 2)
  int junctionID = (L * 4) + (C * 2) + (R * 1);

  // 3. Map IDs to Actions (Left-Hand Rule)
  switch (junctionID) {
    case 7: return -1; // Full Cross (+): Take Left
    case 6: return -1; // Left T-Junction (L + S): Take Left
    case 5: return -1; // Left/Right only (L + R): Take Left
    case 4: return -1; // 90 deg Left: Take Left
    
    case 3: return 0;  // Right T-Junction (S + R): Take Straight
    case 2: return 0;  // Straight line only: Take Straight
    
    case 1: return 1;  // 90 deg Right: Take Right
    
    default: return -99; // No line or Dead End
  }
}
// ============================================================
//  Debug — FIX 5: called on a 50ms interval, not every tick
// ============================================================

void debugPrint(int vals[]) {
  String s = "";
  switch (state) {
    case FOLLOWING: s = "FOLLOWING"; break;
    case CROSSING:  s = "CROSSING";  break;
    case TURNING:   s = "TURNING";   break;
    case SEARCH:    s = "SEARCH";    break;
    case COOLDOWN:  s = "COOLDOWN";  break;
    case RECOVER:   s = "RECOVER";   break;
    case FINISH:    s = "FINISH";    break;
    case BACKUP:    s = "BACKUP";    break;
    case DONE:      s = "DONE";      break;
  }
  String line = String(millis()) + "," + s;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    line += "," + String(vals[i]);
  }
  Serial.println(line);
}

// ============================================================
//  Setup
// ============================================================

void setup() {
  pinMode(MOTOR_A_1, OUTPUT); pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT); pinMode(MOTOR_B_2, OUTPUT);
  stopMotors();
  Serial.begin(9600);
  pixels.begin();
  pixels.clear();
  pixels.show();
  raceStart = millis();
  Serial.println("Ready");
}

// ============================================================
//  Main Loop
// ============================================================

void loop() {
  int  vals[SENSOR_COUNT];
  long wsum   = 0;
  int  active = readSensors(vals, wsum);
  unsigned long now = millis();

  // FIX 5: throttle debug output so it doesn't slow the loop
  if (now - lastDebug >= DEBUG_INTERVAL) {
    lastDebug = now;
    debugPrint(vals);
  }

  switch (state) {

    // ── FOLLOWING ──────────────────────────────────────────
    case FOLLOWING: {

      // Finish line — 7+ sensors active, past safety window
      // FIX 3: confirmation window raised to 100ms to avoid false triggers at wide junctions
      if (active >= 7 && now - raceStart > IGNORE_FINISH_MS) {
        if (finishTimer == 0) {
          finishTimer = now;
        } else if (now - finishTimer >= FINISH_CONFIRM_MS) {
          Serial.println("Finish detected");
          state       = FINISH;
          stateStart  = now;
          finishTimer = 0;
        }
        driveForward();
        break;
      } else if (active < 7) {
        finishTimer = 0;
      }

      // No line — enter recover
      if (active == 0) {
        state      = RECOVER;
        stateStart = now;
        break;
      }

      // Junction check
      int turn = chooseTurn(vals);
      if (turn != -99 && turn != 0) {
        pendingTurn = turn;
        state       = CROSSING;
        stateStart  = now;
        Serial.print("Junction → turn: "); Serial.println(turn);
        break;
      }

      // Normal PD following (straight at junction falls through here too)
      float pos = (float)wsum / active;
      lastDirection = (pos < 0) ? -1 : 1;
      drive(pos);
      break;
    }

    // ── CROSSING — creep forward to center wheels ───────────
    case CROSSING:
      driveForward();
      if (now - stateStart >= CROSSING_MS) {
        state      = TURNING;
        stateStart = now;
      }
      break;

    // ── TURNING — blind pivot phase ─────────────────────────
    case TURNING: {
      unsigned long blindTime = (pendingTurn == 2) ? TURN_180_MS : BLIND_MS;
      if (pendingTurn == -1) pivotLeft();
      else                   pivotRight();
      if (now - stateStart >= blindTime) {
        state      = SEARCH;
        stateStart = now;
      }
      break;
    }

    // ── SEARCH — spin until centre sensors find line ─────────
    case SEARCH: {
      if (pendingTurn == -1) pivotLeft();
      else pivotRight();

      int hits = 0;
      if (analogRead(SENSOR_PINS[3]) > THRESHOLD) hits++;
      if (analogRead(SENSOR_PINS[4]) > THRESHOLD) hits++;

      if (hits >= 1) {
        Serial.println("Turn done");
        lastError  = 0;
        state = COOLDOWN;
        stateStart = now;
        break;
      }

      // Timeout — switch to u-turn, or give up if already trying
      if (now - stateStart > TIMEOUT_MS) {
        if (pendingTurn != 2) {
          Serial.println("Timeout → u-turn");
          pendingTurn = 2;
          state       = TURNING;
          stateStart  = now;
        } else {
          Serial.println("Completely lost — stopping");
          stopMotors();
          state      = COOLDOWN;
          stateStart = now;
        }
      }
      break;
    }

    // ── COOLDOWN — drive clear before checking junctions again ─
    case COOLDOWN:
      if (active > 0) {
        float pos = (float)wsum / active;
        drive(pos);
      } else {
        driveForward();
      }
      if (now - stateStart >= COOLDOWN_MS) {
        state = FOLLOWING;
        Serial.println("Cooldown done");
      }
      break;

    // ── RECOVER — spin toward last known side until line found ─
    case RECOVER:
      if (lastError <= 0) pivotLeft();
      else                pivotRight();
      if (active > 0) {
        Serial.println("Line recovered");
        state      = COOLDOWN;
        stateStart = now;
      }
      if (now - stateStart > TIMEOUT_MS) {
        pendingTurn = 2;
        state       = TURNING;
        stateStart  = now;
      }
      break;

    // ── FINISH — drive across the finish box ────────────────
    case FINISH:
      driveForward();
      if (active <= 2) {
        stopMotors();
        state      = BACKUP;
        stateStart = now;
        Serial.println("Crossed finish — backing up");
      }
      break;

    // ── BACKUP — reverse to centre, pause, done ─────────────
    // FIX 4 note: Motor B reverse is full power due to pin 4 not being PWM.
    // Robot may veer slightly — rewiring pin 4 to pin 3 or 5 fixes this properly.
    case BACKUP:
      if (now - stateStart < 600) {
        driveReverse((int)(SPEED_BASE * 0.8));
      } else if (now - stateStart < 1600) {
        stopMotors();
      } else if (now - stateStart < 3000) {
        driveReverse((int)(SPEED_BASE * 0.8));
      } else {
        state = DONE;
      }
      break;

    // ── DONE ────────────────────────────────────────────────
    case DONE:
      stopMotors();
      pixels.fill(pixels.Color(0, 255, 0));
      pixels.show();
      break;
  }
}

// ============================================================
//  LED Helpers — FIX 1: defined here, declared above motor helpers
// ============================================================

void showForwardLights() {
  pixels.clear();
  pixels.setPixelColor(2, pixels.Color(100, 100, 100));
  pixels.setPixelColor(3, pixels.Color(100, 100, 100));
  pixels.show();
}

void showBrakeLights() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.setPixelColor(1, pixels.Color(100, 0, 0));
  pixels.show();
}

void showLeftLights() {
  pixels.clear();
  pixels.setPixelColor(3, pixels.Color(65, 40, 0));
  pixels.show();
}

void showRightLights() {
  pixels.clear();
  pixels.setPixelColor(2, pixels.Color(65, 40, 0));
  pixels.show();
}