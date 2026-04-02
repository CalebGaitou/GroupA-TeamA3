/**
 * @file RobotControl.ino
 * @brief Line-following robot with gripper and NeoPixel status indicators.
 * Adheres to Makerguides C++ Style Guide naming and formatting.
 */

#include <Adafruit_NeoPixel.h>

// --- Configuration Constants (UPPER_CASE) ---
const uint8_t PIXEL_PIN = 7;
const uint8_t PIXEL_COUNT = 4;
const uint8_t SENSOR_COUNT = 8;

// NeoPixel Instance
Adafruit_NeoPixel ledStrip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED Indices
const int FRONT_LEFT  = 0;
const int FRONT_RIGHT = 1;
const int REAR_RIGHT  = 2;
const int REAR_LEFT   = 3;

// Motor Pin Definitions
const int MOTOR_A_1_PIN = 11; // Left backward
const int MOTOR_A_2_PIN = 10; // Left forward
const int MOTOR_B_1_PIN = 9;  // Right forward
const int MOTOR_B_2_PIN = 6;  // Right backward

// Sensors & Actuators
const int TRIGGER_PIN = 4;
const int ECHO_PIN = 13;
const int SERVO_PIN = 12;

// Gripper Pulse Widths
const int GRIPPER_OPEN = 1700;
const int GRIPPER_CLOSE = 1000;

// Sensor Configuration
const int SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_WEIGHTS[SENSOR_COUNT] = {-7, -5, -3, -1, 1, 3, 5, 7};
const int LINE_THRESHOLD = 800;

// PID and Speed Settings
const float KP_GAIN = 20.0;
const float KD_GAIN = 12.0;
const int SPEED_BASE = 200;
const int SPEED_MIN = 160;
const int SPEED_PIVOT = 180;

// Timing and Thresholds (ms)
const unsigned long CONE_DRIVE_DURATION = 1100;
const unsigned long ENTRY_TURN_DURATION = 400; 
const unsigned long CROSSING_DURATION = 150;
const unsigned long BLIND_TURN_DURATION = 300;
const unsigned long U_TURN_DURATION = 600;
const unsigned long COOLDOWN_DURATION = 250;
const unsigned long IGNORE_FINISH_DURATION = 3000;
const unsigned long FINISH_CONFIRM_THRESHOLD = 100;
const unsigned long BACKUP_DURATION = 600;

const int BACKUP_SPEED = -200;
const int STOP_THRESHOLD = 2;
const int WAIT_TO_START_DELAY = 4000;
const int OBSTACLE_THRESHOLD_CM = 20;

/** @brief Enum for the Robot State Machine */
enum RobotState {
    WAIT_FOR_START,
    CONE_STATE,
    CONE_GRAB,
    ON_COURSE,
    FOLLOWING,
    CROSSING,
    TURNING,
    SEARCH,
    COOLDOWN,
    RECOVER,
    FINISH,
    BACKUP,
    DONE
};

// Global State Variables 
RobotState currentRobotState = WAIT_FOR_START;
RobotState lastDebugState = DONE; 
int pendingTurnDirection = 0;
float previousError = 0.0;
bool isGripperClosed = false;

unsigned long stateStartTime = 0;
unsigned long raceStartTime = 0;
unsigned long finishDetectionTime = 0;
unsigned long lastServoPulse = 0;

// ============================================================
// Helper Functions
// ============================================================

/**
 * @brief Logs state transitions to the Serial monitor.
 */
void debugState(String stateName) {
    if (currentRobotState != lastDebugState) {
        Serial.print("STATE CHANGE: ");
        Serial.println(stateName);
        lastDebugState = currentRobotState;
    }
}

/**
 * @brief Non-blocking pulse management for the gripper servo.
 */
void refreshServo() {
    if (millis() - lastServoPulse >= 20) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(isGripperClosed ? GRIPPER_CLOSE : GRIPPER_OPEN);
        digitalWrite(SERVO_PIN, LOW);
        lastServoPulse = millis();
    }
}

/**
 * @brief Measures distance using the ultrasonic sensor.
 * @return Distance in cm, or 999 if no object detected.
 */
long getDistanceCm() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); 
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

/**
 * @brief Controls specific motor pins based on speed.
 */
void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        analogWrite(forwardPin, speed);
        analogWrite(backwardPin, 0);
    } else {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, abs(speed));
    }
}

void stopMotors() {
    analogWrite(MOTOR_A_1_PIN, 0);
    analogWrite(MOTOR_A_2_PIN, 0);
    analogWrite(MOTOR_B_1_PIN, 0);
    analogWrite(MOTOR_B_2_PIN, 0);
}

void driveForward(int speed = SPEED_BASE) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, speed);
}

void pivotLeft(int speed = SPEED_PIVOT) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, -speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, speed);
}

void pivotRight(int speed = SPEED_PIVOT) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, -speed);
}

/**
 * @brief Updates NeoPixel colors based on current robot state.
 */
void updateLEDs() {
    ledStrip.clear();

    uint32_t green  = ledStrip.Color(255, 0, 0);   
    uint32_t red = ledStrip.Color(0, 255, 0);  
    uint32_t blue = ledStrip.Color(0, 0, 255);   
    uint32_t amber  = ledStrip.Color(120, 255, 0); 
    uint32_t purple = ledStrip.Color(0, 150, 255); 

    switch (currentRobotState) {
        case FOLLOWING:
            ledStrip.setPixelColor(FRONT_RIGHT, amber);
            ledStrip.setPixelColor(FRONT_LEFT, amber);
            break;

        case TURNING:
        case SEARCH:
        case RECOVER:
            if (pendingTurnDirection == -1) {
                ledStrip.setPixelColor(FRONT_LEFT, green);
                ledStrip.setPixelColor(REAR_LEFT, green);
            } else {
                ledStrip.setPixelColor(FRONT_RIGHT, green);
                ledStrip.setPixelColor(REAR_RIGHT, green);
            }
            break;

        case WAIT_FOR_START:
        case FINISH:
        case BACKUP:
            ledStrip.setPixelColor(FRONT_LEFT, red);
            ledStrip.setPixelColor(FRONT_RIGHT, red);
            break;

        case CONE_STATE:
            for (int i = 0; i < PIXEL_COUNT; i++) ledStrip.setPixelColor(i, blue);
            break;

        case DONE:
            for (int i = 0; i < PIXEL_COUNT; i++) ledStrip.setPixelColor(i, purple);
            break;
            
        default:
            break;
    }
    ledStrip.show();
}

// Setup & Loop
void setup() {
  pinMode(MOTOR_A_1_PIN, OUTPUT);
  pinMode(MOTOR_A_2_PIN, OUTPUT);
  pinMode(MOTOR_B_1_PIN, OUTPUT);
  pinMode(MOTOR_B_2_PIN, OUTPUT);

  analogwrite(MOTOR_A_1_PIN, LOW);
  analogwrite(MOTOR_A_2_PIN, LOW);
  analogwrite(MOTOR_B_1_PIN, LOw);
  analogwrite(MOTOR_B_2_PIN, LOw);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  
  analogwrite(TRIGGER_PIN, LOW);
  analogwrite(ECHO_PIN, LOW);
  analogwrite(SERVO_PIN, LOW);

  stopMotors();
  isGripperClosed = false; 

  ledStrip.begin();
  ledStrip.setBrightness(50);
  ledStrip.show();

  Serial.begin(9600);
  Serial.println("--- ROBOT INITIALIZED ---");
}

void loop() {
  refreshServo(); 
  updateLEDs();
  
  int sensorValues[SENSOR_COUNT];
  long weightedSum = 0;
  int activeSensorCount = 0;
  unsigned long currentTime = millis();

  // Data acquisition
  if (currentRobotState >= FOLLOWING) {
      for (int i = 0; i < SENSOR_COUNT; i++) {
          sensorValues[i] = analogRead(SENSOR_PINS[i]);
          if (sensorValues[i] > LINE_THRESHOLD) {
              weightedSum += (long)SENSOR_WEIGHTS[i];
              activeSensorCount++;
          }
      }
  }

  // State machine logic
  switch (currentRobotState) {
      
      case WAIT_FOR_START:
          debugState("WAIT_FOR_START");
          stopMotors();
          if (getDistanceCm() < OBSTACLE_THRESHOLD_CM) {
              delay(WAIT_TO_START_DELAY);
              stateStartTime = millis(); 
              currentRobotState = CONE_STATE;
          }
          break;

      case CONE_STATE:
          debugState("CONE_STATE");
          driveForward(SPEED_BASE);
          if (currentTime - stateStartTime >= CONE_DRIVE_DURATION) {
              stopMotors();
              stateStartTime = currentTime;
              currentRobotState = CONE_GRAB;
          }
          break;

      case CONE_GRAB:
          debugState("CONE_GRAB");
          isGripperClosed = true; 
          if (currentTime - stateStartTime >= 1000) { 
              stateStartTime = currentTime;
              currentRobotState = ON_COURSE;
          }
          break;

      case ON_COURSE:
          debugState("ON_COURSE");
          pivotLeft(SPEED_PIVOT);
          if (currentTime - stateStartTime >= ENTRY_TURN_DURATION) {
              raceStartTime = millis(); 
              currentRobotState = FOLLOWING;
          }
          break;

      case FOLLOWING:
          debugState("FOLLOWING");
          
          // Finish detection
          if (activeSensorCount >= 7 && (currentTime - raceStartTime > IGNORE_FINISH_DURATION)) {
              if (finishDetectionTime == 0) {
                  finishDetectionTime = currentTime;
              } else if (currentTime - finishDetectionTime >= FINISH_CONFIRM_THRESHOLD) {
                  currentRobotState = FINISH;
                  stateStartTime = currentTime;
              }
              driveForward();
              break;
          } else {
              finishDetectionTime = 0;
          }

          if (activeSensorCount == 0) {
              currentRobotState = RECOVER;
              stateStartTime = currentTime;
              break;
          }

          // Junction Detection
          {
              // A6 and A7 are your far-left sensors
              bool hasLeft = (sensorValues[6] > LINE_THRESHOLD || sensorValues[7] > LINE_THRESHOLD);
              
              if (hasLeft) { 
                  // We found the turn! Stop the PD controller immediately.
                  stopMotors(); 
                  pendingTurnDirection = -1; // Force Left Pivot
                  currentRobotState = CROSSING; 
                  stateStartTime = millis(); 
                  return; // Exit loop to start the turn sequence
              }
              
              // Only check for Right if we didn't find a Left
              bool hasRight = (sensorValues[0] > LINE_THRESHOLD || sensorValues[1] > LINE_THRESHOLD);
              if (hasRight) {
                  pendingTurnDirection = 1; 
                  currentRobotState = CROSSING; 
                  stateStartTime = millis(); 
                  return; 
              }
          }

          // PD Driving logic
          {
              float currentPos = (float)weightedSum / activeSensorCount;
              int targetSpeed = map((int)(abs(currentPos) * 10), 0, 70, SPEED_BASE, SPEED_MIN);
              float correction = (KP_GAIN * currentPos) + (KD_GAIN * (currentPos - previousError));
              previousError = currentPos;
              setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, targetSpeed - (int)correction);
              setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, targetSpeed + (int)correction);
          }
          break;

      case CROSSING:
          debugState("CROSSING");
          driveForward();
          if (currentTime - stateStartTime >= CROSSING_DURATION) { 
              currentRobotState = TURNING; 
              stateStartTime = currentTime; 
          }
          break;

      case TURNING:
          debugState("TURNING");
          if (pendingTurnDirection == -1) pivotLeft(); else pivotRight();
          if (currentTime - stateStartTime >= BLIND_TURN_DURATION) {
              currentRobotState = SEARCH; 
              stateStartTime = currentTime;
          }
          break;

      case SEARCH:
          debugState("SEARCH");
          if (pendingTurnDirection == -1) pivotLeft(); else pivotRight();
          if (analogRead(SENSOR_PINS[3]) > LINE_THRESHOLD || analogRead(SENSOR_PINS[4]) > LINE_THRESHOLD) {
              previousError = 0; 
              currentRobotState = COOLDOWN; 
              stateStartTime = currentTime;
          }
          break;

      case COOLDOWN:
          debugState("COOLDOWN");
          driveForward();
          if (currentTime - stateStartTime >= COOLDOWN_DURATION) {
              currentRobotState = FOLLOWING;
          }
          break;

      case RECOVER:
          debugState("RECOVER");
          if (previousError <= 0) pivotLeft(); else pivotRight();
          if (activeSensorCount > 0) currentRobotState = FOLLOWING;
          break;

      case FINISH:
          debugState("FINISH");
          driveForward();
          if (activeSensorCount <= STOP_THRESHOLD) { 
              stopMotors(); 
              currentRobotState = BACKUP; 
              stateStartTime = currentTime; 
          }
          break;

      case BACKUP:
          debugState("BACKUP");
          if (currentTime - stateStartTime < BACKUP_DURATION) {
              setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, BACKUP_SPEED); 
              setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, BACKUP_SPEED);
          } else { 
              stopMotors(); 
              currentRobotState = DONE; 
          }
          break;

      case DONE:
          debugState("DONE");
          stopMotors();
          break;
          
      default:
          break;
  }
}