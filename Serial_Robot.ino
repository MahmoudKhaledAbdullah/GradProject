// Testing Motors Encoders For The CPR

// Loop Timing
unsigned long currentMillis ;
unsigned long previousMillis ;

int dist = 0;  // Variable For Travelled Distance 
int dir1 = 0;  // Right Motors Direction
int dir2 = 0;  // Left Motors Direction

int Speed = 100; // Robot Speed (PWM)

// Encoder Pins
#define encoderR1 19
#define encoderR2 18
#define encoderL1 21
#define encoderL2 20

// Motor Speed Parameters
int PWM_R = 0;
int PWM_L = 0;
int Stop = 0;
volatile long encoderRpos = 0;
volatile long encoderLpos = 0;

// Motors Driving Pins
#define PWMR1 4
#define INR11 52
#define INR12 53
#define PWML1 5
#define INL11 22
#define INL12 23
#define PWMR2 6
#define INR21 10
#define INR22 11
#define PWML2 7
#define INL21 12
#define INL22 13

int PWM_val11 = 0;
int PWM_val21 = 0;
int PWM_val12 = 0;
int PWM_val22 = 0;

void setup() {
  pinMode(encoderR1,INPUT_PULLUP);
  pinMode(encoderR2,INPUT_PULLUP);
  pinMode(encoderL1,INPUT_PULLUP);
  pinMode(encoderL2,INPUT_PULLUP);

  pinMode(PWMR1,OUTPUT);
  pinMode(INR11,OUTPUT);
  pinMode(INR12,OUTPUT);
  pinMode(PWML1,OUTPUT);
  pinMode(INL11,OUTPUT);
  pinMode(INL12,OUTPUT);
  pinMode(PWMR2,OUTPUT);
  pinMode(INR21,OUTPUT);
  pinMode(INR22,OUTPUT);
  pinMode(PWML2,OUTPUT);
  pinMode(INL21,OUTPUT);
  pinMode(INL22,OUTPUT);
  

  attachInterrupt(digitalPinToInterrupt(encoderR1), doEncoderR1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL1), doEncoderL1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderR2), doEncoderR2, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL2), doEncoderL2, CHANGE);

  Serial.begin(9600);
}

void loop(){

  // Starting A Loop For Serial Monitoring And Motor Starting & Stopping
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10 ){
    previousMillis = currentMillis;
    if (Serial.available()>0) {                       // manual control of wheesls via terminal
      char c =Serial.read();
      if (c == 'w') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 1900;
        dir1 = 1;
        dir2 = 1;
      }
      if (c == 's') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 1900;
        dir1 = -1;
        dir2 = -1;
      }
      if (c == 'a') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 750;
        dir1 = 1;
        dir2 = -1;
      }
      if (c == 'd') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 750;
        dir1 = -1;
        dir2 = 1;
      }
      if (c == 'q') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 750*2;
        dir1 = 1;
        dir2 = -1;
      }
      if (c == 'e') {
        Stop = 0;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 750*2;
        dir1 = -1;
        dir2 = 1;
      }
      if (c == 'x') {
        Stop = 1;
        encoderRpos = 0;
        encoderLpos = 0;
        dist = 0;
        dir1 = 0;
        dir2 = 0;
      }
      if (c == 'm') {
        Speed += 5;
      }
      if (c == 'n') {
        Speed -= 5;
      }
      }
    Serial.print(PWM_val12);
    Serial.print(" , ");
    Serial.print(PWM_val22);
    Serial.print(" , ");
    Serial.print(encoderLpos);
    Serial.print(" , ");
    Serial.println(encoderRpos);
  }
  if ((abs(encoderLpos) >= dist) && (abs(encoderRpos) >= dist)){  
  // Calling setMotor Function To Set Motor
    PWM_val11 = 0;
    PWM_val21 = 0;
    setMotor(0, PWM_val11, int(PWM_val11 * 105 / 117), PWMR1, PWMR2, INR11, INR12, INR21, INR22, Stop);
    setMotor(0, PWM_val21, PWM_val21 * 2, PWML1, PWML2, INL11, INL12, INL21, INL22, Stop);
  }else{
    PWM_val11 = Speed;
    PWM_val21 = Speed;
    setMotor(dir1, PWM_val11, int(PWM_val11 * 105 / 117), PWMR1, PWMR2, INR11, INR12, INR21, INR22, Stop);
    setMotor(dir2, PWM_val21, PWM_val21 * 2, PWML1, PWML2, INL11, INL12, INL21, INL22, Stop);
  }
}

// Encoder Incremental Function For First Input Pin
void doEncoderR1(){

  if(digitalRead(encoderR2) == LOW){
    if(digitalRead(encoderR1) == HIGH){
      encoderRpos ++ ;
    }
    else{
      encoderRpos -- ;
    }
  }
  else{
    if(digitalRead(encoderR1) == HIGH){
      encoderRpos -- ;
    }
    else{
      encoderRpos ++ ;
    }
  }
}


void doEncoderR2(){

  if(digitalRead(encoderR1) == LOW){
    if(digitalRead(encoderR2) == HIGH){
      encoderRpos -- ;
    }
    else{
      encoderRpos ++ ;
    }
  }
  else{
    if(digitalRead(encoderR2) == HIGH){
      encoderRpos ++ ;
    }
    else{
      encoderRpos -- ;
    }
  }
}


void doEncoderL1(){

  if(digitalRead(encoderL2) == LOW){
    if(digitalRead(encoderL1) == HIGH){
      encoderLpos ++ ;
    }
    else{
      encoderLpos -- ;
    }
  }
  else{
    if(digitalRead(encoderL1) == HIGH){
      encoderLpos -- ;
    }
    else{
      encoderLpos ++ ;
    }
  }
}


void doEncoderL2(){

  if(digitalRead(encoderL1) == LOW){
    if(digitalRead(encoderL2) == HIGH){
      encoderLpos -- ;
    }
    else{
      encoderLpos ++ ;
    }
  }
  else{
    if(digitalRead(encoderL2) == HIGH){
      encoderLpos ++ ;
    }
    else{
      encoderLpos -- ;
    }
  }
}

// Motor Speed & Direction Setting Function
void setMotor(int dir, int pwmVal1, int pwmVal2, int pwm1, int pwm2, int inf1, int inf2, int inb1, int inb2, int St){
  if(pwmVal1 > 255){
    pwmVal1 == 255 ;
  }
  if(pwmVal2 > 255){
    pwmVal2 == 255 ;
  }
  analogWrite(pwm1,pwmVal1);
  analogWrite(pwm2,pwmVal2);
  if((dir == 1) && (St == 0)){
    digitalWrite(inf1,HIGH);
    digitalWrite(inf2,LOW);
    digitalWrite(inb1,HIGH);
    digitalWrite(inb2,LOW);
  }
  else if(dir == -1){
    digitalWrite(inf1,LOW);
    digitalWrite(inf2,HIGH);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,HIGH);
  }
  else{
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    digitalWrite(inf1,LOW);
    digitalWrite(inf2,LOW);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,LOW);
  } 
}
