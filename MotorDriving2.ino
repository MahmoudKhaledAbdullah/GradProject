#define PWM 5  // Motor Driver PWM Pin
#define IN1 22  // Motor Driver Direction Pin

void setup() {
  // Pins Modes Determination
  pinMode(PWM,OUTPUT);
  pinMode(IN,OUTPUT);
}

void loop(){
  analogWrite(PWM,250);
  digitalWrite(IN1,HIGH);
  delay(2000);
  analogWrite(PWM,0);
  digitalWrite(IN1,LOW);
  delay(500);
  analogWrite(PWM,250);
  digitalWrite(IN1,LOW);
  delay(2000);
}