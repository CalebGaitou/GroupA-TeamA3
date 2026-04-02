const int MOTOR_A_1 = 11; //motor A = left wheel
const int MOTOR_A_2 = 10;
const int MOTOR_B_1 = 9; //motor B = right wheel
const int MOTOR_B_2 = 4;

const int MOTOR_ASPEED = 236;
const int MOTOR_BSPEED = 255;
const int PAUSE = 1000;

const int ROTATIONSENSOR_1 = 3;
const int ROTATIONSENSOR_2 = 2;

const int SERVO_PIN = 6;
const int OPEN_CLUS  = 1800;
const int CLOSE_CLUS = 1000;
const int CLAWSTIME = 800;

int servoTargetUs = OPEN_CLUS;
long lastServoFrameMs = 0;

void setup() {
  pinMode(SERVO_PIN, OUTPUT);

  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  pinMode(ROTATIONSENSOR_1, INPUT);
  pinMode(ROTATIONSENSOR_2, INPUT);
  
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
  digitalWrite(ROTATIONSENSOR_1, LOW);
  digitalWrite(ROTATIONSENSOR_2, LOW);
  digitalWrite(SERVO_PIN, LOW);

  delay(PAUSE);
  servoTargetUs = OPEN_CLUS;
  claws(CLAWSTIME);
  servoTargetUs = CLOSE_CLUS;
  claws(CLAWSTIME);
  servoTargetUs = OPEN_CLUS;
  claws(1000);
  moveForward();
  delay(PAUSE);
  stopMotors();
  delay(PAUSE);
  servoTargetUs = OPEN_CLUS;
  claws(CLAWSTIME);
  servoTargetUs = CLOSE_CLUS;
  claws(1000);
  moveForward();
  delay(PAUSE);
  stopMotors();
  delay(PAUSE);
  servoTargetUs = OPEN_CLUS; 
  claws(CLAWSTIME);
}

void loop() {
  servoUpdate();
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, 0);
}

void moveForward() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, MOTOR_ASPEED);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, MOTOR_BSPEED);
}

void servoUpdate() {
  long now = millis();
  if (now - lastServoFrameMs >= 20) {
    lastServoFrameMs = now;
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(servoTargetUs);
    digitalWrite(SERVO_PIN, LOW);
  }
}

void claws(long ms) {
  long start = millis();
  while (millis() - start < ms) {
    servoUpdate(); // keep servo active
  }
}
