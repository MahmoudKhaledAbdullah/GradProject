// Manual Motors Speed Matching

unsigned long currentMillis;
unsigned long previousMillis;

#define encoder1 19  // Encoder Output Pin 1
#define encoder2 18  // Encoder Output Pin 2
#define PWMF 4  // Front Motor Driver PWM Pin
#define INF1 52  // Front Motor Driver Direction Pin 1
#define INF2 53  // Front Motor Driver Direction Pin 2
#define PWMB 6  // Front Motor Driver PWM Pin
#define INB1 10  // Front Motor Driver Direction Pin 1
#define INB2 11  // Front Motor Driver Direction Pin 2
int Stop = 0;  // Variable For Stop Condition 
int pwmReq1 = 150;  // Front Motor Used Speed (PWM)
int pwmReq2 = pwmReq1 - 68;  // Front Motor Used Speed (PWM)


volatile long encoderPos = 0;  // Variable For Encoder Readings

void setup() {
  
  // Pins Modes Determination
  pinMode(encoder1,INPUT_PULLUP);  
  pinMode(encoder2,INPUT_PULLUP);  
  pinMode(PWMF,OUTPUT);
  pinMode(INF1,OUTPUT);
  pinMode(INF2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(INB1,OUTPUT);
  pinMode(INB2,OUTPUT);

  // Attaching Interupts For Encoder Input Pins
  attachInterrupt(digitalPinToInterrupt(encoder1), doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1), doEncoder2, CHANGE);

  // Starting The Serial For Testing
  Serial.begin(9600);
  
}

void loop(){

  // Starting A Loop For Serial Monitoring And Motor Starting & Stopping
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10 ){
    previousMillis = currentMillis;
    Serial.print(abs(encoderPos));
    Serial.print(" , ");
    Serial.println(currentMillis);
  }
  if (currentMillis >= 10000){  
  //  Actual Number Of Encoder Counts Per  10 Revolutions "After Testing"
    Stop = 1;  // Stopping The Motor On Reaching 10 Revolutions
    // Calling setMotor Function To Set Motor 
    setMotor(-1, pwmReq1, PWMF, INF1, INF2, Stop);
    setMotor(-1, pwmReq2, PWMB, INB1, INB2, Stop);
  }else{
    setMotor(-1, pwmReq1, PWMF, INF1, INF2, Stop);
    setMotor(-1, pwmReq2, PWMB, INB1, INB2, Stop);
  }
}

// Encoder Incremental Function For First Input Pin
void doEncoder1(){
  if(digitalRead(encoder2) == LOW){
    if(digitalRead(encoder1) == HIGH){
      encoderPos ++ ;
    }
    else{
      encoderPos -- ;
    }
  }
  else{
    if(digitalRead(encoder1) == HIGH){
      encoderPos -- ;
    }
    else{
      encoderPos ++ ;
    }
  }
}

// Encoder Incremental Function For First Input Pin
void doEncoder2(){
  if(digitalRead(encoder1) == LOW){
    if(digitalRead(encoder2) == HIGH){
      encoderPos -- ;
    }
    else{
      encoderPos ++ ;
    }
  }
  else{
    if(digitalRead(encoder2) == HIGH){
      encoderPos ++ ;
    }
    else{
      encoderPos -- ;
    }
  }
}

// Motor Speed & Direction Setting Function
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2, int St){
  if(pwmVal > 255){
    pwmVal == 255 ;
  }
  analogWrite(pwm,pwmVal);
  if((dir == 1) && (St == 0)){
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
  }else if((dir == -1) && (St == 0)){
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
  }else{
      analogWrite(pwm,0);
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
  }
  
}
