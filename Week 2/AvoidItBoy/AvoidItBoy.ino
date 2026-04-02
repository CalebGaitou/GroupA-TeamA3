//Creating constante for the motor pins
const int MOTOR_A_1 = 11; //motor A = left wheel
const int MOTOR_A_2 = 10;
const int MOTOR_B_1 = 9; //motor B = right wheel
const int MOTOR_B_2 = 4;

const int TRIG_PIN = 5;
const int ECHO_PIN = 6;

const int PASSDURATION = 1500;
const int MIDPASSDURATION = 800;
const int MOVEBACK = 400;
const int STOPTIME = 1000;
const int TURNDURATIONRIGHT = 1000;
const int TURNDURATIONLEFT = 825;
const int SAFEDISTANCE = 12;
int distance;

const int ROTATIONSENSOR_1 = 3;
const int ROTATIONSENSOR_2 = 2;

//Creating a speed constant
const int MOTOR_ASPEED = 236;
const int MOTOR_BSPEED = 255;

void setup() {
  delay(3000);
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  pinMode(ROTATIONSENSOR_1, INPUT);
  pinMode(ROTATIONSENSOR_2, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
  digitalWrite(ROTATIONSENSOR_1, LOW);
  digitalWrite(ROTATIONSENSOR_2, LOW);
  Serial.begin(9600);
}

void loop() {
  delay(100);
  
  distance = getDistance();
  
  Serial.print("distance: ");
  Serial.println(distance);
  
  if(distance > 0 && distance < SAFEDISTANCE) {
    avoidAction();
  }
  else {
    moveForwards();
  }
} 

int getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 10000); // timeout 1ms

  if (duration == 0) {
    return -1; // no echo
  }

  int distance = duration * 0.034 / 2;
  return distance;
}

void avoidAction() {
  stopMotorsQick();
  delay(STOPTIME);
  moveBackQuick();
  delay(MOVEBACK);
  stopMotorsQick();
  turnRight();
  delay(TURNDURATIONRIGHT);
  stopMotorsQick();
  moveForwards();
  delay(MIDPASSDURATION);
  stopMotorsQick();
  turnLeft();
  delay(TURNDURATIONLEFT);
  stopMotorsQick();
  moveForwards();
  delay(PASSDURATION);
  stopMotorsQick();
  turnLeft();
  delay(TURNDURATIONLEFT);
  stopMotorsQick();
  moveForwards();
  delay(MIDPASSDURATION);
  stopMotorsQick();
  turnRight();
  delay(TURNDURATIONRIGHT);
}

void moveForwards() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, MOTOR_ASPEED);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, MOTOR_BSPEED);
}

void moveBackQuick() {
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

void stopMotorsQick() {
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_2, 0);
  analogWrite(MOTOR_B_1, 0);
}
