// Testing Motors Encoders For The CPR

unsigned long currentMillis;
unsigned long previousMillis;

#define encoder1 20  // Encoder Output Pin 1
#define encoder2 21  // Encoder Output Pin 2
#define PWM 5  // Motor Driver PWM Pin
#define IN1 22  // Motor Driver Direction Pin 1
#define IN2 23  // Motor Driver Direction Pin 2
int Stop = 0;  // Variable For Stop Condition 
int pwmReq = 255;  // Motor Used Speed (PWM)


volatile long encoderPos = 0;  // Variable For Encoder Readings

void setup() {
  
  // Pins Modes Determination
  pinMode(encoder1,INPUT_PULLUP);  
  pinMode(encoder2,INPUT_PULLUP);  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

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
    Serial.println(abs(encoderPos));
  }
  if (abs(encoderPos) >= 3580){  
  //  Actual Number Of Encoder Counts Per  5 Revolutions "After Testing"
    Stop = 1;  // Stopping The Motor On Reaching 10 Revolutions
    // Calling setMotor Function To Set Motor 
    setMotor(1, pwmReq, PWM, IN1, IN2, Stop);
  }else{
    setMotor(1, pwmReq, PWM, IN1, IN2, Stop);
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
