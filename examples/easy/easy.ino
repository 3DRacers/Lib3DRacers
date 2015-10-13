
#include <3DRacers_Vendors.h>
#include <3DRacers.h>

ThreeDRacers racer;

void setup() {
  Serial.begin(115200);
  racer.Begin(Serial1, Serial);
  racer.OnDriveCommand(OnDriveCommand);
  racer.OnConfigCommand(OnServoConfigCommand);
}

void loop() {
  racer.Process();
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