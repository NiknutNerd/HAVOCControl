#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CW 7
#define CCW 9

//Finding the milliseconds of the minimum cycle for PWM
const float PWMHZ = 10.0;
const float PWM_CYCLE = 1.0 / PWMHZ;
const float PWM_MILLIS = 1000.0 * PWM_CYCLE;
const float PWM_DEADZONE = 1.0 / PWMHZ;

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

class Stopwatch{
  private:
    long startTime;
  public:
    Stopwatch(){
      startTime = (long)millis();
    }
    void reset(){
      startTime = (long)millis();
    }
    long getTime(){
      return (long)millis() - startTime;
    }
};

class Timer{
  private:
    Stopwatch internalStopwatch;
    long delay;
  public:
    Timer(){
      internalStopwatch = Stopwatch();
      delay = 0;
    }
    Timer(long time){
      internalStopwatch = Stopwatch();
      delay = time;
    }
    void reset(long newTime){
      delay = newTime;
      internalStopwatch.reset();
    }
    long getTime(){
      return internalStopwatch.getTime();
    }
    bool isDone(){
      return getTime() > delay;
    }
    long getTimeLeft(){
      if (isDone()){
        return -1;
      }
      return delay - internalStopwatch.getTime();
    }
};

Stopwatch oPIDTimer;
Stopwatch vPIDTimer;

Timer CWCountdown;
Timer CCWCountdown;
Timer PWMCountdown;

float oPID(float target){
  imuStuff();
  float current = orientation.x();

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
    float onPercent = abs(percent);
    float offPercent = 1 - onPercent;
    long onTime = 0;
    long offTime = 0;
    //Set the smaller to PWM_MILLIS
    //Set the larger to PWM_MILLIS times the ratio between the larger / the smaller
    if(abs(percent) < PWM_DEADZONE){
      onPercent = 0;
      offPercent = 0;
    }else if(abs(percent) > 1 - PWM_DEADZONE){
      onPercent = 1 - PWM_DEADZONE;
      offPercent = PWM_DEADZONE;
    }

    if(onPercent >= .5){
      onTime = (onPercent / offPercent) * PWM_MILLIS;
      offTime = PWM_MILLIS;
    }else if(offPercent > .5){
      onTime = PWM_MILLIS;
      offTime = (offPercent / onPercent) * PWM_MILLIS;
    }
    if(percent > 0){
      CWCountdown.reset(onTime);
      PWMCountdown.reset(onTime + offTime);
    }else if(percent < 0){
      CCWCountdown.reset(onTime);
      PWMCountdown.reset(onTime + offTime);
    }

  }
  
  if(!CWCountdown.isDone()){
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
