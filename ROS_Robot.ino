#include <util/atomic.h>        // For The ATOMIC_BLOCK Macro

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Point32.h>        // For Sending Encoder msg
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

#define L 0.38        // Distance Between Left & Right Wheels
#define R 0.06        // Wheel Radius
#define pi 3.1415926        // PI Constant
#define LOOPTIME 100        // PID Loop Time
#define N 358        // Encoder Resolution (Pulses Per Revolution "PPR")
#define gear_ratio 1       // Motor Gear Ratio
#define MAX_RPM 50        // Motor's Maximum RPM
#define Kp 0.5        // P constant
#define Ki 0.0        // I constant
#define Kd 0.0        // D constant

float x=0.0;
float z=0.0;
float VR,VL;

unsigned long currentmillis ;
unsigned long previousmillis ;

#define encoderR1 19
#define encoderR2 18

#define encoderL1 21
#define encoderL2 20

double rpm_reqR = 0;
double rpm_reqL = 0;
double rpm_actR = 0;
double rpm_actL = 0;
// double rpm_reqR_smoothed = 0;
// double rpm_reqL_smoothed = 0;
double RtickOld=0.0;
double RtickNew=0.0;
double LtickOld=0.0;
double LtickNew=0.0;
int PWM_R = 0;
int PWM_L = 0;

int Stop = 0;

volatile long encoderRpos = 0;
volatile long encoderLpos = 0;


unsigned long lastMilli = 0;       // loop timing
unsigned long lastMillipub = 0;

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

int PWM_val1 = 0;
int PWM_val2 = 0;

float u = 0;
int e = 0;

long prevT = 0;
float eprev1 = 0;
float eintegral1 = 0;
float dedt1 = 0;
float eprev2 = 0;
float eintegral2 = 0;
float dedt2 = 0;


geometry_msgs::Point32 Point_msg;

geometry_msgs::Vector3Stamped speed_msg;

ros::Publisher enc_pub("/encoder", &Point_msg);
ros::Publisher speed_pub("/speed", &speed_msg);

ros::NodeHandle nh;



}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velcallback);



void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(enc_pub);
  nh.advertise(speed_pub);
  PWM_val1 = 0;
  PWM_val2 = 0;
  rpm_reqR = 0;
  rpm_reqL = 0;
  rpm_actR = 0;
  rpm_actL = 0;

  RtickOld=0.0;
  RtickNew=0.0;
  LtickOld=0.0;
  LtickNew=0.0;

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

  Serial.begin(57600);

}

void loop() {

  nh.spinOnce();
  unsigned long time = millis();

  if(time-lastMilli>= LOOPTIME){
    Actual_rpm(time-lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_reqR, rpm_actR);
    PWM_val2 = updatePid(2, PWM_val2, rpm_reqL, rpm_actL);

    if(rpm_reqR > 0){
      Stop = 0;
      setMotor(1, PWM_val1, PWMR1, PWMR2, INR11, INR12, INR21, INR22, Stop);
    }
    else if(rpm_reqR == 0){
      Stop = 1;
      setMotor(0, PWM_val1, PWMR1, PWMR2, INR11, INR12, INR21, INR22, Stop);
    }
    else{
      Stop = 0;
      setMotor(-1, abs(PWM_val1), PWMR1, PWMR2, INR11, INR12, INR21, INR22, Stop);
    }

    if(rpm_reqL > 0 ){
      Stop = 0;
      setMotor(1, PWM_val2, PWML1, PWML2, INL11, INL12, INL21, INL22, Stop);
    }
    else if(rpm_reqL == 0){
      Stop = 1;
      setMotor(0, PWM_val2, PWML1, PWML2, INL11, INL12, INL21, INL22, Stop);
    }
    else{
      Stop = 0;
      setMotor(-1, abs(PWM_val2), PWML1, PWML2, INL11, INL12, INL21, INL22, Stop);
    }

    publishSpeed(time-lastMilli);
    lastMilli = time;
  }
  Serial.print(rpm_reqR);
  Serial.print(" , ");
  Serial.println(rpm_reqL);
  Point_msg.x=encoderRpos;
  Point_msg.y=encoderLpos;
  enc_pub.publish(&Point_msg);

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


void setMotor(int dir, int pwmVal, int pwm1, int pwm2, int inf1, int inf2, int inb1, int inb2, int St){
  if(pwmVal > 255){
    pwmVal == 255 ;
  }
  analogWrite(pwm1,pwmVal);
  analogWrite(pwm2,pwmVal);
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


void publishSpeed(double time){
    speed_msg.header.stamp = nh.now();      //timestamp for odometry data
    speed_msg.vector.x = rpm_actL;    //left wheel speed
    speed_msg.vector.y = rpm_actR;   //right wheel speed
    speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
    speed_pub.publish(&speed_msg);
    nh.spinOnce();
}

void Actual_rpm(unsigned long time){
  RtickNew = encoderRpos;
  LtickNew = encoderLpos;
  rpm_actR = double((RtickNew-RtickOld)*1000)/double(time*N*gear_ratio*2*pi*R);
  rpm_actL = double((LtickNew-LtickOld)*1000)/double(time*N*gear_ratio*2*pi*R);
  RtickOld = RtickNew;
  LtickOld = LtickNew;
}


int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}
