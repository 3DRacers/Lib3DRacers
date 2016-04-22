///////////////////////////////////////////////////
// 3DRacers Robot
// Need a common Ultrasonic Ping Sensor (model US-100 suggested)
//
// This robot sense the object in front of the car, if:
// - too near: the car stops
// - < 30cm: the car goes backward and turn right
// - When the obstacle is far the car go forward


#include <3DRacers_Vendors.h>
#include <3DRacers.h>


#include <NewPing.h>    //Ultrasonic Ping sensor library
#include "Timer.h"      //Lib to do an action every N milliseconds

//Connect RAW, GND, TRIGGER to pin 3, ECHO to pin 2 on the Ultrasonic sensor
#define TRIGGER_PIN  3
#define ECHO_PIN     2
#define MAX_DISTANCE 300

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Timer t;
unsigned int distanceCm = 0;

ThreeDRacers racer;

void setup() {
  Serial.begin(115200);
  racer.Begin(Serial1, Serial);
  racer.OnDriveCommand(OnDriveCommand);
  racer.OnConfigCommand(OnServoConfigCommand);

  t.every(1000, doSonarRead);
}

bool carDirection = true;

void loop() {
  //Standard 3DRacers functions:
  racer.Process();

  //Our custom functions for the robot:
  t.update();

  if(distanceCm < 10) {
    racer.MotorStop();
  }
  else {
    carDirection = true;
    if(distanceCm < 30) {
      carDirection = false;
    }
    
    if(carDirection) {
      racer.LedColor(0, 100, 0); 
      racer.MotorMoveForward(100);
      racer.SetServo(0);
    }
    else {
      racer.LedColor(100, 0, 0); 
      racer.MotorMoveBackwards(100);
      racer.SetServo(60); //Turn right
    }
  }
  
}

void doSonarRead() {
  unsigned int cm = sonar.ping() / US_ROUNDTRIP_CM; //  manda un ping e conta il tempo di risposta (uS).
  if(cm > 0) {
    Serial.println(cm);
    distanceCm = cm;

  }
}

void OnDriveCommand(DriveCommand& cmd, CarInfo& car)
{
//  Serial.print(cmd.steerAngle);
//  Serial.print(" ");
//  Serial.println(car.batteryLevel);
  
  //Motors:
  int motorOutput = cmd.throttle;
  //Serial.println(motorOutput);
  if (cmd.brake) {
    racer.MotorBrake();
  }
  else if (cmd.reverse) {
    racer.MotorMoveBackwards(motorOutput);
  }
  else if (cmd.throttle == 0) {
    racer.MotorStop();
  }
  else {
    racer.MotorMoveForward(motorOutput);
  }
  //Steer:
  racer.SetServo(cmd.steerAngle);

  if(cmd.changeColor) {
     racer.LedColor(cmd.red, cmd.green, cmd.blue); 
  }
}

void OnServoConfigCommand(ConfigCommand& cmd, CarInfo& car)
{
   //Steer calibration:
  if (cmd.flags.steerCenterChanged == true) {
    racer.SetServoCenter(cmd.steerCenter);
  }
  if (cmd.flags.steerMaxChanged == true) {
    racer.SetServoMaxAngle(cmd.steerMax);
  }
}
