#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CW 7
#define CCW 9

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
float okp = 0.2;
float oi = 0.0;
float oki = 0.0;
float od = 0.0;
float okd = 0.0;
float oPIDOutput;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.05;
float vi = 0.0;
float vki = 0.0;
float vd = 0.0;
float vkd = 0.00;
float vPIDOutput;

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
}

class CountdownTimer: public Timer{
  private:
    long startTime;
    long initTime;
    long targetTime;
  public:
    CountdownTimer(){
      startTime = (long)millis();
      initTime = 0;
      targetTime = (long)millis();
    }
    CountdownTimer(long time){
      startTime = (long)millis();
      initTime = time;
      targetTime = startTime + time;
    }
    void reset(long newTime){
      startTime = (long)millis();
      initTime = newTime;
      targetTime = startTime + newTime;
    }
    bool isDone(){
      if((targetTime - (long)millis()) <= 0){
        return true;
      }else{
        return false;
      }
    }
    long getTime(){
      long timeLeft = targetTime - (long)millis();
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
}

Timer oPIDTimer;
Timer vPIDTimer;

CountdownTimer CWCountdown;
CountdownTimer CCWCountdown;
CountdownTimer PWMCountdown;

float oPID(float target){
  imuStuff();
  float current = orientation.x;

  //Some sort of normalization to find error
  oPIDError = target - current;

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

  return oPIDOutput;
}

float vPID(float target){
  imuStuff();
  float current = gyro.z();
  vPIDError = (target - current);
  if(abs(vPIDError < 2)){
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
  if(PWMCountdown.isDone()){

  }else if(!CWCountdown.isDone()){
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, LOW);
  }else if(!CCWCountdown.isDone()){
    digitalWrite(CW, LOW);
    digitalWrite(CCW, HIGH);
  }else{
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}