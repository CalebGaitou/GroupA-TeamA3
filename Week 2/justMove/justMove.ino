//Creating constante for the motor pins
const int MOTOR_A_1 = 11; //motor A = left wheel
const int MOTOR_A_2 = 10;
const int MOTOR_B_1 = 9; //motor B = right wheel
const int MOTOR_B_2 = 4;

const int ROTATIONSENSOR_1 = 3;
const int ROTATIONSENSOR_2 = 2;

//Creating a speed constant
const int MOTOR_ASPEED = 236;
const int MOTOR_BSPEED = 255;

int oneMeterDistance = 4000;
int stopTime = 400;
int turningDurationRight = 1200;
int turningDurationLeft = 1200;

void setup() {
  delay(3000);
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
}

void loop() {
  moveForward();
  delay(oneMeterDistance);
  stopMotors();
  delay(stopTime);
  moveBack();
  delay(oneMeterDistance);
  stopMotors();
  delay(stopTime);
  turnLeft();
  delay(turningDurationLeft);
  stopMotors();
  delay(stopTime);
  turnRight();
  delay(turningDurationRight);
  stopMotors();
  delay(stopTime);
}

void moveForward() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, MOTOR_ASPEED);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, MOTOR_BSPEED);
}

void moveBack() {
  analogWrite(MOTOR_A_1, MOTOR_ASPEED);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_2, MOTOR_BSPEED);
  analogWrite(MOTOR_B_1, 0);
}

void turnRight() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, MOTOR_ASPEED);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, 0);
}

void turnLeft() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, MOTOR_BSPEED);
}

void stopMotors() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, 0);
}
