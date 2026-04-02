// ============================================================
//  Line Following Robot
// ============================================================

// --- Motor Pins ---
const int MOTOR_A_1 = 11;  // Left  backward
const int MOTOR_A_2 = 10;  // Left  forward
const int MOTOR_B_1 = 9;   // Right forward
const int MOTOR_B_2 = 4;   // Right backward

// --- Sensor Config ---
const int SENSOR_COUNT = 8;
const int SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
const int SENSOR_THRESHOLD = 950;

// --- PD Tuning ---
const float Kp = 20.0;
const float Kd = 12.0;

// --- Speed Config ---
const int SPEED_STRAIGHT = 255;
const int SPEED_CURVE_MIN = 80;
const int SPEED_SEARCH = 150;
const int SPEED_PIVOT = 180;

// --- Timing ---
const unsigned long FINISH_CONFIRM_MS = 100;
const unsigned long LINE_SEARCH_MS = 300;

// --- State ---
float lastError = 0;
float savedLastError = 0;

bool  finishDetected = false;
bool  lineLost = false;

unsigned long finishTimer = 0;
unsigned long lineLostTimer = 0;

// ============================================================

void setup() {
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  stopMotors();
  Serial.begin(9600);
}

// ============================================================

void setMotor(int pin_fwd, int pin_bwd, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(pin_fwd, speed);
    analogWrite(pin_bwd, 0);
  } else {
    analogWrite(pin_fwd, 0);
    analogWrite(pin_bwd, abs(speed));
  }
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
}

void pivotToward(float direction) {
  int s;
  if (direction > 0) {
    s = SPEED_PIVOT;
  } 
  else {
    s = -SPEED_PIVOT;
  }
  setMotor(MOTOR_A_2, MOTOR_A_1,  s);
  setMotor(MOTOR_B_1, MOTOR_B_2, -s);
}

// ============================================================

void loop() {
  // --- Read sensors ---
  long weightedSum = 0;
  long activeCount = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int active;
    if (analogRead(SENSOR_PINS[i]) > SENSOR_THRESHOLD) {
      active = 1;
    } else {
      active = 0;
    }
    weightedSum += active * SENSOR_WEIGHTS[i];
    activeCount += active;
  }

  // --- Finish line: all sensors active for >FINISH_CONFIRM_MS ---
  if (activeCount == SENSOR_COUNT) {
    if (!finishDetected) {
      finishDetected = true;
      finishTimer = millis();
    }
    if (millis() - finishTimer > FINISH_CONFIRM_MS) {
      stopMotors();
      Serial.println("Finish line detected");
      while (true);
    }
  } 
  else {
    finishDetected = false;
  }

  // --- Determine position and baseSpeed ---
  float position;
  int   baseSpeed;

  if (activeCount > 0) {
    position = (float)weightedSum / activeCount;
    baseSpeed = map((int)(abs(position) * 10), 0, 70, SPEED_STRAIGHT, SPEED_CURVE_MIN);

    lineLost  = false;
    lineLostTimer = 0;
    lastError = position;

  } 
  else {
    if (!lineLost) {
      lineLost = true;
      lineLostTimer = millis();
      savedLastError = lastError;
    }

    if (millis() - lineLostTimer < LINE_SEARCH_MS) {
      // Phase 1: continue current arc, line probably just mid-turn
      position = savedLastError;
      baseSpeed = SPEED_SEARCH;
    } 
    else {
      // Phase 2: truly lost — pivot hard toward last known side
      pivotToward(savedLastError);
      return;
    }
  }

  // --- PD correction ---
  float correction = Kp * position;
  if (abs(position) <= 4.0) {
    correction += Kd * (position - lastError);  // Add D term on gentle curves only
  }
  lastError = position;  // Fixed: was missing from previous version

  // --- Drive ---
  setMotor(MOTOR_A_2, MOTOR_A_1, baseSpeed - correction);  // Left
  setMotor(MOTOR_B_1, MOTOR_B_2, baseSpeed + correction);  // Right

  Serial.print("Error: ");
  Serial.println(position);
}