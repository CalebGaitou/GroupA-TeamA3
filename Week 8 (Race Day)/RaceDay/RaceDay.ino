// --- Motor Pin Definitions ---
const int MOTOR_A_1_PIN = 11; // Left backward
const int MOTOR_A_2_PIN = 10; // Left forward
const int MOTOR_B_1_PIN = 9;  // Right forward
const int MOTOR_B_2_PIN = 6;  // Right backward

// --- Ultrasonic & Servo Pins ---
const int TRIGGER_PIN = 4;
const int ECHO_PIN = 13;
const int SERVO_PIN = 12; 

// --- Gripper Pulse Widths ---
const int GRIPPER_OPEN = 1700;
const int GRIPPER_CLOSE = 1000;

// --- Sensor Configuration ---
const int SENSOR_COUNT = 8;
const int SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_WEIGHTS[SENSOR_COUNT] = {-7, -5, -3, -1, 1, 3, 5, 7};
const int LINE_THRESHOLD = 800;

// --- PD Control Gains ---
const float KP_GAIN = 20.0;
const float KD_GAIN = 12.0;

// --- Speed Settings ---
const int SPEED_BASE = 200;
const int SPEED_MIN = 160;
const int SPEED_PIVOT = 180;

// --- Timing and Thresholds (ms) ---
const unsigned long CONE_DRIVE_DURATION = 1100;
const unsigned long ENTRY_TURN_DURATION = 400; 
const unsigned long CROSSING_DURATION = 50;
const unsigned long BLIND_TURN_DURATION = 80;
const unsigned long U_TURN_DURATION = 600;
const unsigned long COOLDOWN_DURATION = 250;
const unsigned long TIMEOUT_DURATION = 3000;
const unsigned long IGNORE_FINISH_DURATION = 3000;
const unsigned long FINISH_CONFIRM_THRESHOLD = 100;

// --- Backup & Finish Settings ---
const unsigned long BACKUP_DURATION = 600;  // Time spent reversing after finish
const int BACKUP_SPEED = -200;              // Speed for the backup maneuver
const int STOP_THRESHOLD = 2;               // Max sensors active to consider "stopped"

const int WAIT_TO_START = 4000;
const int OBSTACLE_THRESHOLD_CM = 20;

// --- State Machine ---
enum RobotState {
    WAIT_FOR_START,
    CONE_STATE,
    CONE_GRAB,
    ON_COURSE,
    FOLLOWING,
    LEFT_DECISION,
    CROSSING,
    TURNING,
    SEARCH,
    COOLDOWN,
    RECOVER,
    FINISH,
    BACKUP,
    DONE
};

RobotState currentRobotState = WAIT_FOR_START;
RobotState lastDebugState = DONE; 

int pendingTurnDirection = 0;
float previousError = 0;
bool isGripperClosed = false;

unsigned long stateStartTime = 0;
unsigned long raceStartTime = 0;
unsigned long finishDetectionTime = 0;
unsigned long lastServoPulse = 0;

// ============================================================
// Helpers
// ============================================================

void debugState(String stateName) {
    if (currentRobotState != lastDebugState) {
        Serial.print("STATE CHANGE: ");
        Serial.println(stateName);
        lastDebugState = currentRobotState;
    }
}

void refreshServo() {
    if (millis() - lastServoPulse >= 20) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(isGripperClosed ? GRIPPER_CLOSE : GRIPPER_OPEN);
        digitalWrite(SERVO_PIN, LOW);
        lastServoPulse = millis();
    }
}

long getDistanceCM() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); 
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        analogWrite(forwardPin, speed);
        analogWrite(backwardPin, 0);
    } else {
        analogWrite(forwardPin, 0);
        if (backwardPin == 5) digitalWrite(5, HIGH);
        else analogWrite(backwardPin, abs(speed));
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

// ============================================================
// Setup & Loop
// ============================================================

void setup() {
  pinMode(MOTOR_A_1_PIN, OUTPUT);
  pinMode(MOTOR_A_2_PIN, OUTPUT);
  pinMode(MOTOR_B_1_PIN, OUTPUT);
  pinMode(MOTOR_B_2_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);

  stopMotors();
  isGripperClosed = false; 
  Serial.begin(9600);
  
  Serial.println("--- ROBOT INITIALIZED ---");
}

void loop() {
    refreshServo(); 
    
    int sensorValues[SENSOR_COUNT];
    long weightedSum = 0;
    int activeSensorCount = 0;
    unsigned long currentTime = millis();

    if (currentRobotState >= FOLLOWING) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            sensorValues[i] = analogRead(SENSOR_PINS[i]);
            if (sensorValues[i] > LINE_THRESHOLD) {
                weightedSum += (long)SENSOR_WEIGHTS[i];
                activeSensorCount++;
            }
        }
    }

    switch (currentRobotState) {
        
        case WAIT_FOR_START:
            debugState("WAIT_FOR_START");
            stopMotors();
            isGripperClosed = false;
            if (getDistanceCM() < OBSTACLE_THRESHOLD_CM) {
                Serial.println("Trigger Detected! Starting Delay...");
                delay(WAIT_TO_START);
                stateStartTime = millis(); 
                currentRobotState = CONE_STATE;
            }
            break;

        case CONE_STATE:
            debugState("CONE_STATE (Driving to Cone)");
            driveForward(SPEED_BASE);
            if (currentTime - stateStartTime >= CONE_DRIVE_DURATION) {
                stopMotors();
                stateStartTime = currentTime;
                currentRobotState = CONE_GRAB;
            }
            break;

        case CONE_GRAB:
            debugState("CONE_GRAB (Closing Gripper)");
            stopMotors();
            isGripperClosed = true; 
            if (currentTime - stateStartTime >= 1000) { 
                stateStartTime = currentTime;
                currentRobotState = ON_COURSE;
            }
            break;

        case ON_COURSE:
            debugState("ON_COURSE (Pivoting to Course)");
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
                    Serial.println("FINISH LINE POTENTIAL");
                    finishDetectionTime = currentTime;
                }
                else if (currentTime - finishDetectionTime >= FINISH_CONFIRM_THRESHOLD) {
                    currentRobotState = FINISH;
                    stateStartTime = currentTime;
                }
                driveForward();
                break;
            } else finishDetectionTime = 0;

            if (activeSensorCount == 0) {
                currentRobotState = RECOVER;
                stateStartTime = currentTime;
                break;
            }

            // Junction Detection (Priority Left-Hand Rule)
            {
                bool hasLeft   = (sensorValues[6] > LINE_THRESHOLD || sensorValues[7] > LINE_THRESHOLD);
                bool hasCenter = (sensorValues[3] > LINE_THRESHOLD || sensorValues[4] > LINE_THRESHOLD);
                bool hasRight  = (sensorValues[0] > LINE_THRESHOLD || sensorValues[1] > LINE_THRESHOLD);

                if (hasLeft) { 
                    stopMotors(); 
                    Serial.println("!!! LEFT DETECTED - BREAKING OUT !!!");
                    pendingTurnDirection = -1; 
                    currentRobotState = CROSSING; 
                    stateStartTime = millis(); 
                    return; 
                } 
                else if (hasCenter) {
                    // Continue to PD logic
                } 
                else if (hasRight) {
                    Serial.println("Junction: Right only");
                    pendingTurnDirection = 1; 
                    currentRobotState = CROSSING; 
                    stateStartTime = millis(); 
                    return; 
                }
            }

            // PD Drive logic
            {
                float currentPos = (float)weightedSum / activeSensorCount;
                int targetBaseSpeed = map((int)(abs(currentPos) * 10), 0, 70, SPEED_BASE, SPEED_MIN);
                float correction = (KP_GAIN * currentPos) + (KD_GAIN * (currentPos - previousError));
                previousError = currentPos;
                setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, targetBaseSpeed - (int)correction);
                setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, targetBaseSpeed + (int)correction);
            }
            break;

        case CROSSING:
            debugState("CROSSING (Aligning Pivot)");
            driveForward();
            if (currentTime - stateStartTime >= CROSSING_DURATION) { currentRobotState = TURNING; stateStartTime = currentTime; }
            break;

        case TURNING:
            debugState("TURNING (Blind Pivot)");
            if (pendingTurnDirection == -1) pivotLeft(); else pivotRight();
            if (currentTime - stateStartTime >= (pendingTurnDirection == 2 ? U_TURN_DURATION : BLIND_TURN_DURATION)) {
                currentRobotState = SEARCH; stateStartTime = currentTime;
            }
            break;

        case SEARCH:
            debugState("SEARCH (Finding Line)");
            if (pendingTurnDirection == -1) pivotLeft(); else pivotRight();
            if (analogRead(SENSOR_PINS[3]) > LINE_THRESHOLD || analogRead(SENSOR_PINS[4]) > LINE_THRESHOLD) {
                Serial.println("LINE FOUND");
                previousError = 0; currentRobotState = COOLDOWN; stateStartTime = currentTime;
            }
            break;

        case COOLDOWN:
            debugState("COOLDOWN");
            driveForward();
            if (currentTime - stateStartTime >= COOLDOWN_DURATION) currentRobotState = FOLLOWING;
            break;

        case RECOVER:
            debugState("RECOVER (Lost Line)");
            if (previousError <= 0) pivotLeft(); else pivotRight();
            if (activeSensorCount > 0) currentRobotState = FOLLOWING;
            break;

        case FINISH:
            debugState("FINISH (Slowing Down)");
            driveForward();
            // If we only see 2 or fewer sensors, we've crossed the thick bar
            if (activeSensorCount <= STOP_THRESHOLD) { 
                stopMotors(); 
                currentRobotState = BACKUP; 
                stateStartTime = currentTime; 
            }
            break;

        case BACKUP:
            debugState("BACKUP (Final Position)");
            if (currentTime - stateStartTime < BACKUP_DURATION) {
                setMotorSpeed(MOTOR_A_2_PIN, MOTOR_A_1_PIN, BACKUP_SPEED); 
                setMotorSpeed(MOTOR_B_1_PIN, MOTOR_B_2_PIN, BACKUP_SPEED);
            } 
            else { 
                stopMotors(); 
                currentRobotState = DONE; 
            }
            break;

        case DONE:
            debugState("DONE");
            stopMotors();
            break;
    }
}