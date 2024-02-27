#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CW 15
#define CCW 14

//Finding the milliseconds of the minimum cycle for PWM
const float PWMHZ = 10.0;
const float PWM_CYCLE = 1.0 / PWMHZ;
const float PWM_MILLIS = 1000.0 * PWM_CYCLE;
const float PWM_DEADZONE = 1.0 / PWMHZ;
//const float PWM_DEADZONE = 0.075;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

imu::Vector<3> orientation;
imu::Vector<3> gyro;

void imuStuff(){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

//PID Variables
float oPIDError;
float oLastTarget = 0.0;
float op = 0.0;
float okp = 0.5;
float oi = 0.0;
float oki = 0.0;
float od = 0.0;
float okd = 0.01;
float oPIDOutput;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.0085;
float vi = 0.0;
float vki = 0.0;
float vd = 0.0;
float vkd = 0.00;
float vPIDOutput;

bool CWOn;
bool CCWOn;
float PWMInput = 0.0;

class Timer{
  private:
    long startTime;
  public:
    Timer(){
      startTime = (long)millis();
    }
    void reset(){
      startTime = (long)millis();
    }
    long getTime(){
      return (long)millis() - startTime;
    }
};

class CountdownTimer{
  private:
    long length;
    Timer internal;
  public:
    CountdownTimer(){
      internal = Timer();
      length = 0;
    }
    CountdownTimer(long time){
      internal = Timer();
      length = time;
    }
    void reset(long newTime){
      length = newTime;
      internal.reset();
    }
    void reset(){
      //length does not change
      internal.reset();
    }
    bool isDone(){
      return internal.getTime() > length;
    }
    long getTimeLeft(){
      long timeLeft = length - internal.getTime();
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
};

Timer oPIDTimer;
Timer vPIDTimer;
Timer telem;
Timer timer;

CountdownTimer CWCountdown;
CountdownTimer CCWCountdown;
CountdownTimer PWMCountdown;

float oPID(float target){
  imuStuff();
  float current = orientation.x();

  //Some sort of normalization to find error
  //oPIDError = target - current
  oPIDError = int(target - (360 - current) + 540) % 360 - 180;

  //If in dead zone do nothing
  if(abs(oPIDError) < 5){
    return 0;
  }

  od = (oPIDError - op) / oPIDTimer.getTime();
  oi = oi + (oPIDError * oPIDTimer.getTime());
  op = oPIDError;

  oPIDTimer.reset();

  if(target != oLastTarget){
    oi = 0;
  }
  if(oki * oi > .2){
    oi = .2 / oki;
  }else if(oki * oi < -.2){
    oi = (-1) * (.2 / oki);
  }
  oPIDOutput = (okp * op) + (oki * oi) + (okd * od);
  oLastTarget = target;

  return (-1) * oPIDOutput;
}

float vPID(float target){
  imuStuff();
  float current = gyro.z();
  vPIDError = (target - current);
  if(abs(vPIDError) < 3){
    return 0;
  }

  vd = (vPIDError - vp) / vPIDTimer.getTime();
  vi = vi + (vPIDError * vPIDTimer.getTime());
  vp = vPIDError;
  
  vPIDTimer.reset();

  if(target != vLastTarget){
    vi = 0;
  }

  if(vki * vi > .05){
    vi = 0;
  }else if(vki * vi < -.05){
    vi = 0;
  }
  vLastTarget = target;
  vPIDOutput = (vkp * vp) + (vki * vi) + (vkd * vd);

  return vPIDOutput;
}

void PWM(float percent){
  /*
    3 countdown timers
    PWMCountdown: The period of the PWM cycle (time on + time off)
    CWCountdown: The time that the CW solenoid should be open
    CCWCountdown: The time that the CCW solenoid should be open

    PWM_MILLIS: Time in milliseconds for minimum actuation of solenoids
    percent = onPercent, if positive set CW, if negative set CCW
    if abs(percent) > .5, PWM_MILLIS is off time
    if abs(percent) <= .5, PWM_MILLIS is on time
  */
  if(PWMCountdown.isDone()){
    PWMInput = percent;
    float onPercent = abs(percent);
    float offPercent = 1 - onPercent;
    long onTime = 0;
    long offTime = 0;
    //Set the smaller to PWM_MILLIS
    //Set the larger to PWM_MILLIS times the ratio between the larger / the smaller

    //If in deadzone, set time to PWM millis
    //Means it wont be tie up time
    if(onPercent < PWM_DEADZONE){
      onTime = -1;
      offTime = PWM_MILLIS;
    }else if(onPercent >= (1 - PWM_DEADZONE)){
      //If the onPercent is greater than the deadzone, have the thing be open for the min time then come back and check
      onTime = PWM_MILLIS;
      offTime = -1;
    }else if(onPercent >= offPercent){
      onTime = (onPercent / offPercent) * PWM_MILLIS;
      offTime = PWM_MILLIS;
    }else if(offPercent > onPercent){
      onTime = PWM_MILLIS;
      offTime = (offPercent / onPercent) * PWM_MILLIS;
    }

    if(percent > 0){
      PWMCountdown.reset(onTime + offTime);
      CWCountdown.reset(onTime);
      
    }else if(percent < 0){
      PWMCountdown.reset(onTime + offTime);
      CCWCountdown.reset(onTime);
    }else{
      PWMCountdown.reset(onTime + offTime);
      CWCountdown.reset(-1);
      CCWCountdown.reset(-1);
    }

  }
  
  if(!CWCountdown.isDone()){
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, LOW);
    CWOn = true;
    CCWOn = false;
  }else if(!CCWCountdown.isDone()){
    digitalWrite(CW, LOW);
    digitalWrite(CCW, HIGH);
    CWOn = false;
    CCWOn = true;
  }else{
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
    CWOn = false;
    CCWOn = false;
  }
}

void setup() {
  // put your setup code here, to run once:

  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  Serial.begin(9600);
  digitalWrite(23, HIGH);
  while(1){
    if(bno.begin()){
      break;
    }
  }

  imuStuff();
  gyro.toDegrees();

  oPIDTimer.reset();
  vPIDTimer.reset();
  timer.reset();
  telem.reset();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(telem.getTime() > 50){
    imuStuff();
    Serial.print("Orientation X: ");
    Serial.print(orientation.x());
    Serial.print(", Gyro Z: ");
    Serial.print(gyro.z());
    Serial.print(", CW On: ");
    Serial.print(CWOn);
    Serial.print(", CCW On: ");
    Serial.print(CCWOn);
    Serial.print(", PWM Input: ");
    Serial.print(PWMInput);
    Serial.print(", oPID Output: ");
    Serial.print(oPID(0));
    Serial.print(", oPID Error: ");
    Serial.print(oPIDError);
    Serial.print(", vPID Output: ");
    Serial.println(vPID(oPID(0)));
    telem.reset();
  }
  
  //PWM(vPID(oPID(0)));
  //PWM(0);
  

  if(timer.getTime() < 10000){
    PWM(vPID(oPID(0)));
  }else if(timer.getTime() < 20000){
    PWM(vPID(oPID(90)));
  }else if(timer.getTime() < 30000){
    PWM(vPID(oPID(180)));
  }else if(timer.getTime() < 40000){
    PWM(vPID(oPID(270)));
  }else{
    timer.reset();
  }

  //digitalWrite(22, HIGH);
  
  /*
  if(timer.getTime() < 1000){
    PWM(.5);
  }else if(timer.getTime() < 2000){
    PWM(.25);
  }else if(timer.getTime() < 3000){
    PWM(-.5);
  }else if(timer.getTime() < 4000){
    PWM(-.25);
  }else{
    PWM(0);
  }
  */
}