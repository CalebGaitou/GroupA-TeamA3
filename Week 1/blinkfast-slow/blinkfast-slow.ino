const int ledRed = 13;
const int button1 = 7;
const int button2 = 6;
int button1State = 0; //was the button pressed in the past (ever?)
int button2State = 0;
int delayBlink = 1000; // delay in ms for the blink routine

void setup() {
  pinMode (ledRed, OUTPUT);
  pinMode (button1, INPUT);
  pinMode (button2, INPUT);
  digitalWrite (ledRed, LOW);

}

void loop() {
  button1State = digitalRead(button1);
  button2State = digitalRead(button2);

  //if(button1State == HIGH && button2State == HIGH){
  //  delayBlink = 1000;
  //}

  if (button1State == LOW){
    delayBlink = 500; 
  }

  if (button2State == LOW){
    delayBlink = 2000;
  }
 blinkLed();
}

void blinkLed() {
  digitalWrite(ledRed, LOW);
  delay(delayBlink);
  digitalWrite(ledRed, HIGH);
  delay(delayBlink);
}
}
