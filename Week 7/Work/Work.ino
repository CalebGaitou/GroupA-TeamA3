/*
 * Line Following Robot — Line Maze Solver (Left-Hand Rule)
 * This program controls a robot to solve a maze using 8 IR sensors
 * and a PD control algorithm.
 */

#include <Adafruit_NeoPixel.h>

// --- Configuration Constants ---
const int LED_PIN = 3;
const int NUM_LEDS = 4;

// --- Motor Pin Definitions ---
const int MOTOR_A_1_PIN = 11; // Left backward
const int MOTOR_A_2_PIN = 10; // Left forward
const int MOTOR_B_1_PIN = 9;  // Right forward
const int MOTOR_B_2_PIN = 5;  // Right backward

const int TRIGGERPIN = 12;
const int ECHOPIN = 13;


// --- Sensor Configuration ---
const int SENSOR_COUNT = 8;
const int SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_WEIGHTS[SENSOR_COUNT] = {-7, -5, -3, -1, 1, 3, 5, 7};
const int LINE_THRESHOLD = 950;

// --- PD Control Gains ---
const float KP_GAIN = 20.0;
const float KD_GAIN = 12.0;

// --- Speed Settings ---
const int SPEED_BASE = 255;
const int SPEED_MIN = 160;
const int SPEED_PIVOT = 180;

// --- Timing and Thresholds (ms) ---
const unsigned long CROSSING_DURATION = 100;
const unsigned long BLIND_TURN_DURATION = 80;
const unsigned long U_TURN_DURATION = 600;
const unsigned long COOLDOWN_DURATION = 250;
const unsigned long TIMEOUT_DURATION = 3000;
const unsigned long IGNORE_FINISH_DURATION = 3000;
const unsigned long FINISH_CONFIRM_THRESHOLD = 100;
const unsigned long DEBUG_INTERVAL = 50;

// --- State Machine ---
enum RobotState {
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

RobotState currentRobotState = FOLLOWING;
int pendingTurnDirection = 0;
int lastTurnDirection = 0;
float previousError = 0;

unsigned long stateStartTime = 0;
unsigned long raceStartTime = 0;
unsigned long finishDetectionTime = 0;
unsigned long lastDebugTime = 0;

Adafruit_NeoPixel statusPixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
// LED Status Helpers
// ============================================================

void showForwardLights() {
    statusPixels.clear();
    statusPixels.setPixelColor(2, statusPixels.Color(100, 100, 100));
    statusPixels.setPixelColor(3, statusPixels.Color(100, 100, 100));
    statusPixels.show();
}

void showBrakeLights() {
    statusPixels.clear();
    statusPixels.setPixelColor(0, statusPixels.Color(100, 0, 0));
    statusPixels.setPixelColor(1, statusPixels.Color(100, 0, 0));
    statusPixels.show();
}

void showLeftTurnLights() {
    statusPixels.clear();
    statusPixels.setPixelColor(3, statusPixels.Color(65, 40, 0));
    statusPixels.show();
}

void showRightTurnLights() {
    statusPixels.clear();
    statusPixels.setPixelColor(2, statusPixels.Color(65, 40, 0));
    statusPixels.show();
}

// ============================================================
// Motor Control Functions
// ============================================================

void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        analogWrite(forwardPin, speed);
        analogWrite(backwardPin, 0);
    } else {
        analogWrite(forwardPin, 0);
        // Hardware hack for Pin 5 if PWM is unavailable
        if (backwardPin == 5) {
            digitalWrite(5, HIGH);
        } else {
            analogWrite(backwardPin, abs(speed));
        }
    }
}

void stopMotors() {
    analogWrite(MOTOR_A_1_PIN, 0);
    analogWrite(MOTOR_A_2_PIN, 0);
    analogWrite(MOTOR_B_1_PIN, 0);
    analogWrite(MOTOR_B_2_PIN, 0);
    showBrakeLights();
}

void driveForward(int speed = SPEED_BASE) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, speed);
    showForwardLights();
}

void driveReverse(int speed = SPEED_BASE) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, -speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, -speed);
    showBrakeLights();
}

void pivotLeft(int speed = SPEED_PIVOT) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, -speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, speed);
    showLeftTurnLights();
}

void pivotRight(int speed = SPEED_PIVOT) {
    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, speed);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, -speed);
    showRightTurnLights();
}

// ============================================================
// Sensor and Control Logic
// ============================================================

int readSensorArray(int sensorValues[], long &weightedSum) {
    weightedSum = 0;
    int activeCount = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensorValues[i] = analogRead(SENSOR_PINS[i]);
        if (sensorValues[i] > LINE_THRESHOLD) {
            weightedSum += (long)SENSOR_WEIGHTS[i];
            activeCount++;
        }
    }
    return activeCount;
}

int determineJunctionTurn(int sensorValues[]) {
    bool hasLeft = (sensorValues[5] > LINE_THRESHOLD || sensorValues[6] > LINE_THRESHOLD || sensorValues[7] > LINE_THRESHOLD);
    bool hasCenter = (sensorValues[3] > LINE_THRESHOLD || sensorValues[4] > LINE_THRESHOLD);
    bool hasRight = (sensorValues[0] > LINE_THRESHOLD || sensorValues[1] > LINE_THRESHOLD || sensorValues[2] > LINE_THRESHOLD);

    if (hasLeft) {
        return -1; // Prioritize Left (Left-Hand Rule)
    }
    if (hasCenter && !hasRight) {
        return 0; // Go Straight
    }
    if (hasRight) {
        return 1; // Turn Right
    }
    if (!hasLeft && !hasCenter && !hasRight) {
        return 2; // Dead end / U-turn
    }
    return -99;
}

void executePDDrive(float currentPosition) {
    int targetBaseSpeed = map((int)(abs(currentPosition) * 10), 0, 70, SPEED_BASE, SPEED_MIN);
    float correction = (KP_GAIN * currentPosition) + (KD_GAIN * (currentPosition - previousError));
    previousError = currentPosition;

    if (currentPosition < 0) {
        showLeftTurnLights();
    } else if (currentPosition > 0) {
        showRightTurnLights();
    } else {
        showForwardLights();
    }

    setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, targetBaseSpeed - (int)correction);
    setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, targetBaseSpeed + (int)correction);
}

// ============================================================
// Core Arduino Functions
// ============================================================

void setup() {
    pinMode(MOTOR_A_1_PIN, OUTPUT);
    pinMode(MOTOR_A_2_PIN, OUTPUT);
    pinMode(MOTOR_B_1_PIN, OUTPUT);
    pinMode(MOTOR_B_2_PIN, OUTPUT);

    stopMotors();
    Serial.begin(9600);
    statusPixels.begin();
    statusPixels.clear();
    statusPixels.show();
    
    raceStartTime = millis();
}

void loop() {
    int sensorValues[SENSOR_COUNT];
    long weightedSum = 0;
    int activeSensorCount = readSensorArray(sensorValues, weightedSum);
    unsigned long currentTime = millis();

    // Throttled Debug Output
    if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
        lastDebugTime = currentTime;
        Serial.print("State: ");
        Serial.println(currentRobotState);
    }

    switch (currentRobotState) {
        case FOLLOWING: {
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

            int junctionDecision = determineJunctionTurn(sensorValues);
            if (junctionDecision != -99 && junctionDecision != 0) {
                pendingTurnDirection = junctionDecision;
                currentRobotState = CROSSING;
                stateStartTime = currentTime;
                break;
            }

            executePDDrive((float)weightedSum / activeSensorCount);
            break;
        }

        case CROSSING: {
            driveForward();
            if (currentTime - stateStartTime >= CROSSING_DURATION) {
                currentRobotState = TURNING;
                stateStartTime = currentTime;
            }
            break;
        }

        case TURNING: {
            if (pendingTurnDirection == -1) {
                pivotLeft();
            } else {
                pivotRight();
            }

            unsigned long turnPhaseLimit = (pendingTurnDirection == 2) ? U_TURN_DURATION : BLIND_TURN_DURATION;
            if (currentTime - stateStartTime >= turnPhaseLimit) {
                currentRobotState = SEARCH;
                stateStartTime = currentTime;
            }
            break;
        }

        case SEARCH: {
            if (pendingTurnDirection == -1) {
                pivotLeft();
            } else {
                pivotRight();
            }

            if (analogRead(SENSOR_PINS[3]) > LINE_THRESHOLD || analogRead(SENSOR_PINS[4]) > LINE_THRESHOLD) {
                previousError = 0;
                currentRobotState = COOLDOWN;
                stateStartTime = currentTime;
                break;
            }

            if (currentTime - stateStartTime > TIMEOUT_DURATION) {
                pendingTurnDirection = 2; // Force U-turn on failure
                currentRobotState = TURNING;
                stateStartTime = currentTime;
            }
            break;
        }

        case COOLDOWN: {
            if (activeSensorCount > 0) {
                executePDDrive((float)weightedSum / activeSensorCount);
            } else {
                driveForward();
            }
            if (currentTime - stateStartTime >= COOLDOWN_DURATION) {
                currentRobotState = FOLLOWING;
            }
            break;
        }

        case RECOVER: {
            if (previousError <= 0) {
                pivotLeft();
            } else {
                pivotRight();
            }
            if (activeSensorCount > 0) {
                currentRobotState = COOLDOWN;
                stateStartTime = currentTime;
            }
            break;
        }

        case FINISH: {
            driveForward();
            if (activeSensorCount <= 2) {
                stopMotors();
                currentRobotState = BACKUP;
                stateStartTime = currentTime;
            }
            break;
        }

        case BACKUP: {
            if (currentTime - stateStartTime < 600) {
                driveReverse(200);
            } else if (currentTime - stateStartTime < 1600) {
                stopMotors();
            } else {
                currentRobotState = DONE;
            }
            break;
        }

        case DONE: {
            stopMotors();
            statusPixels.fill(statusPixels.Color(0, 255, 0));
            statusPixels.show();
            break;
        }
    }
}