unsigned long currentMillis;
unsigned long previousMillis;

#define encoderR1 19
#define encoderR2 18
#define encoderL1 21
#define encoderL2 20

volatile long encoderRpos = 0;
volatile long encoderLpos = 0;

void setup() {

  pinMode(encoderR1,INPUT_PULLUP);
  pinMode(encoderR2,INPUT_PULLUP);
  pinMode(encoderL1,INPUT_PULLUP);
  pinMode(encoderL2,INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderR1), doEncoderR1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL1), doEncoderL1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderR2), doEncoderR2, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoderL2), doEncoderL2, CHANGE);

  Serial.begin(9600);
  
}

void loop(){
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10 ){
    previousMillis = currentMillis;
    Serial.print(encoderRpos);
    Serial.print(" , ");
    Serial.println(encoderLpos);
  }
}

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
