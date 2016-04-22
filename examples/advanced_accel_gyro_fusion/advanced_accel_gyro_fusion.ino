///////////////////////////////////////////////////
// 3DRacers Accelerometer/Gyro advanced
// Need an Accelerometer Shield
//
// This example initialize the Accelerometer Shield with 
// Sensor fusion: it programs the 6050 MPU with a code from InvensSense
// that's continuosly read the accelerometer and gyro data
// to determine the absoulte orientation of the car by returning
// a quaternion.
// Other than that the raw accel/gyro data is available.


#include <3DRacers_Vendors.h>
#include <3DRacers.h>


//Include MPU6050 library:
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //NB: fixed version of this file with the constant SERIAL_DEBUG ranamed
#include "Wire.h"
MPU6050 mpu;

//Include our custom accelerometer code:
#include "mpu6050.h"

ThreeDRacers racer;

//Define a custom packet to be sent to the iOS/Android app with the acceleration values:
#include "AccelCommand.h"
AccelCommand accelDataPacket;

//Count the process() calls:
int tick = 0;

//Setup 3DRacers library:
void setup() {
  Serial.begin(115200);
  racer.Begin(Serial1, Serial);
  racer.OnDriveCommand(OnDriveCommand);
  racer.OnConfigCommand(OnServoConfigCommand);
}

void loop() {

  //Run standard 3DRacers code:
  racer.Process();

  //Read data from accelerometer:
  if(mpu_readAccelData(accelDataPacket.teapotPacket)) {

    //1 out of 5 times send back the accelerometer data: (to avoid flooding the app)
    if (tick % 5 == 0) {
      sendData();

      //Send the data also via Serial, to be used with the Processing example:
      mpu_printTeapotValue(accelDataPacket.teapotPacket);
    }
  }

  ++tick;
}

//Function to send the AccelCommand back to the app:
void sendData()
{
  if (!racer.wireless.isConnected) return;
  
  accelDataPacket.packetCount++;
  racer.Send((void *)&accelDataPacket, sizeof(accelDataPacket));
}

//////////////////////////////////////////////////////
// The rest of the code is equal the the "easy" example of 3DRacers:

//Manage the driving packets from the App, like a standard RC car:
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

  if (cmd.changeColor) {
    racer.LedColor(cmd.red, cmd.green, cmd.blue);
  }
}

//Manage the standard config commands for calibration:
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
