const int ledRed = 13;
const int ledYellow = 12;
const int ledGreen = 11;
const int button = 7;
int buttonState = 0; //was the button pressed in the past (ever?)

void setup() {
  pinMode (ledRed, OUTPUT);
  pinMode (ledYellow, OUTPUT);
  pinMode (ledGreen, OUTPUT);
  pinMode (button, INPUT);
}

void loop() {
  buttonState = digitalRead(button);

if(buttonState == HIGH){
  digitalWrite(ledRed, LOW);
  digitalWrite(ledYellow, HIGH);
  digitalWrite(ledGreen, HIGH);
}

if(buttonState == LOW){
  buttonState == true;
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  delay(1000);
  digitalWrite(ledGreen, LOW);
  delay(3000);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledYellow, HIGH);
  delay(1000);
  digitalWrite(ledYellow, LOW);
  delay(1000);
}
}
