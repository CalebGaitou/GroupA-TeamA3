
int ledRed = 13;
int ledYellow = 12;
int ledGreen = 11;

void setup() {
  pinMode(ledRed, OUTPUT);// put your setup code here, to run once:
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(ledGreen, LOW);
  delay(4000);
  digitalWrite(ledGreen, HIGH);
  delay(100);

  digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledYellow, HIGH);
  delay(100);
  
  digitalWrite(ledRed, LOW);
  delay(3000);
  digitalWrite(ledRed, HIGH);
  delay(100);
};
