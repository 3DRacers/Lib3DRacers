#include <Arduino.h>

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/* ================================================================================================ *
  | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
  |                                                                                                  |
  | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
  |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
  |                                                                                                  |
  | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
  |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
   ================================================================================================ */
uint8_t teapotPre[2] = { '$', 0x02 };
uint8_t teapotPost[2] = { '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void mpu_initAccel() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)

  //while (!Serial); // wait for Leonardo enumeration, others continue immediately
  //Serial.println(F("Ready.."));

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  mpu.initialize();

  // verify connection
  Serial.println(mpu.testConnection() ? F("OK") : F("KO"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }


}

bool mpu_readAccelData(uint8_t* teapotPacket) {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    mpu_initAccel();
    return false;
  }
  
  // wait for MPU interrupt or extra packet(s) available
  while (!digitalRead(3) && fifoCount < packetSize) {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[0] = fifoBuffer[0];
    teapotPacket[1] = fifoBuffer[1];
    teapotPacket[2] = fifoBuffer[4];
    teapotPacket[3] = fifoBuffer[5];
    teapotPacket[4] = fifoBuffer[8];
    teapotPacket[5] = fifoBuffer[9];
    teapotPacket[6] = fifoBuffer[12];
    teapotPacket[7] = fifoBuffer[13];
      
    return true;
  }

  return false;
}

void mpu_printTeapotValue(uint8_t* teapotPacket) {
  if (Serial) {
    Serial.write(teapotPre, 2);
    Serial.write(teapotPacket, 10);
    Serial.write(teapotPost, 2);
  }
}



