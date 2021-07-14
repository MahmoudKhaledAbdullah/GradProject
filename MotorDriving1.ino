#define PWM 5  // Motor Driver PWM Pin
#define IN1 22  // Motor Driver Direction Pin 1
#define IN2 23  // Motor Driver Direction Pin 2

void setup() {
  // Pins Modes Determination
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
}

void loop(){
  analogWrite(PWM,250);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  delay(2000);
  analogWrite(PWM,0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  delay(500);
  analogWrite(PWM,250);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  delay(2000);
}