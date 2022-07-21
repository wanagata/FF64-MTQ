#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void receive_data()
{
  char myFirstCharacter = Serial1.read();
  float mySecondCharacter = Serial1.parseFloat();
  float myThirdCharacter = Serial1.parseFloat();
  float myfourthCharacter = Serial1.parseFloat();
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

  if (myFirstCharacter == 'M')
  {
    int Motornumber = mySecondCharacter;
    int Direction = myThirdCharacter;
    float SPEED = myfourthCharacter;

    if (Direction == 87)
    {
      char MTS1[100];
      sprintf(MTS1, "Run Moter %d in CCW Direction SPEED %.2f", Motornumber, SPEED);
      Serial.print(MTS1);
    }
    else if (Direction == 67)
    {
      char MTS2[100];
      sprintf(MTS2, "Run Moter %d in CW Direction SPEED %.2f", Motornumber, SPEED);
      Serial.print(MTS2);
    }
  }
  if (myFirstCharacter == 'J')
  {
    int Motornumber = mySecondCharacter;
    char MTS3[100];
    sprintf(MTS3, "Stop Motor %d ", Motornumber);
    Serial.print(MTS3);
  }

  else if (myFirstCharacter == 'I')
  {
    int IMUSTATE = mySecondCharacter; // 0 stop; 1 run

    while (IMUSTATE == 5)
    {
      if (Serial1.available())

      char myFirstCharacter = Serial1.read();
      float mySecondCharacter = Serial1.parseFloat();
      float myThirdCharacter = Serial1.parseFloat();
      float myfourthCharacter = Serial1.parseFloat();

      int IMUSTATE = mySecondCharacter; // 0 stop; 1 run

      // Serial.println("IMU run!!");

      // Serial.println("IMU Streamimg...");

      //        if (Serial1.available())
      //        {
      //
      //          float mySecondCharacterr = Serial1.parseFloat();
      //                IMUSTATE = mySecondCharacter;
      //                  Serial.println(mySecondCharacter);
      //        }

      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      Serial1.print("X: ");
      Serial1.print(euler.x());
      Serial1.print(" Y: ");
      Serial1.print(euler.y());
      Serial1.print(" Z: ");
      Serial1.print(euler.z());
      Serial1.print("\t\t");

      /*
        // Quaternion data
        imu::Quaternion quat = bno.getQuat();
        Serial.print("qW: ");
        Serial.print(quat.w(), 4);
        Serial.print(" qX: ");
        Serial.print(quat.x(), 4);
        Serial.print(" qY: ");
        Serial.print(quat.y(), 4);
        Serial.print(" qZ: ");
        Serial.print(quat.z(), 4);
        Serial.print("\t\t");
      */

      /* Display calibration status for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      Serial.print("CALIBRATION: Sys=");
      Serial.print(system, DEC);
      Serial.print(" Gyro=");
      Serial.print(gyro, DEC);
      Serial.print(" Accel=");
      Serial.print(accel, DEC);
      Serial.print(" Mag=");
      Serial.println(mag, DEC);

      Serial1.print("CALIBRATION: Sys=");
      Serial1.print(system, DEC);
      Serial1.print(" Gyro=");
      Serial1.print(gyro, DEC);
      Serial1.print(" Accel=");
      Serial1.print(accel, DEC);
      Serial1.print(" Mag=");
      Serial1.println(mag, DEC);

      delay(1000);

      if (IMUSTATE == 6)
      {
        Serial.print("IMU STOP !!");
        Serial1.print("IMU STOP !!");
        break;
      }
    }
  }

  else if (myFirstCharacter == 'P')
  {
    float Yawp = mySecondCharacter;  // Get Yaw
    float Pitchp = myThirdCharacter; // Get Pitch
    float Rollp = myfourthCharacter; // Get Roll

    char ypr2[80];
    sprintf(ypr2, "Yaw, Pitch, Roll =%.2f, %.2f, %.2f", Yawp, Pitchp, Rollp);

    Serial.print(ypr2);
  }
  else if (myFirstCharacter == 'S')
  {

    float Yaws = mySecondCharacter;  // Get Yaw
    float Pitchs = myThirdCharacter; // Get Pitch
    float Rolls = myfourthCharacter; // Get Roll

    char ypr1[80];
    sprintf(ypr1, "Yaw, Pitch, Roll =%.2f, %.2f, %.2f", Yaws, Pitchs, Rolls);
    Serial.print(ypr1);
  }

  else if (myFirstCharacter == 'O')
  {

    float KP = mySecondCharacter; // Get kP
    float KI = myThirdCharacter;
    float KD = myfourthCharacter;
    char kpp[50];
    sprintf(kpp, "kp = %.2f, ki = %.2f, kd = %.2f", KP, KI, KD);
    Serial.print(kpp);
  }
}
