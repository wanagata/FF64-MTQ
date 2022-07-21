#include "src/Bdot.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "src/FastPID.h"
#define sampling_rate 40 // millis

#define BNO055_SAMPLERATE_DELAY_MS (100)
typedef enum
{
  MOTOR_CH1 = 1,
  MOTOR_CH2 = 2,
  MOTOR_CH3 = 3,

} motor_ch_t;

BDOT bdot_controller(sampling_rate);

float Kp = 10, Ki = 0, Kd = 0, Hz = 1 / sampling_rate;
int output_bits = 11;
bool output_signed = true;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{

  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  Serial.println("This program demonstrate how CONTROLLER class is working.");

  update_sensors();

}
Vector<3> getPIDcmd(Vector<3> error)
{
  error.toDegrees();
  Vector<3> rt;
  for (int i = 0; i < 3; i++)
  {
    rt[i] = myPID.step(error[i], 0);
  }
  return rt;
}

void loop()
{

  if (!timeloop())
    return;
  update_sensors();
  Vector<3> error_vec = bdot_controller.runbdot();

  Vector<3> cmd = getPIDcmd(error_vec);
  Serial.print(",Motor 1:");
  Serial.print(error_vec[0]);
  Serial.print(",Motor 2:");
  Serial.print(error_vec[1]);
  Serial.print(",Motor 3:");
  Serial.print(error_vec[2]);
  Serial.println("");
}

void update_sensors()
{
  // Quaternion q_update = bno.getQuat();
  Vector<3> rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  bdot_controller.update(0, rates, mag);
}

bool timeloop()
{
  static uint16_t last_time = 0;
  uint16_t now_time = millis();
  uint16_t dif_time = now_time - last_time;
  if (dif_time <= bdot_controller.get_sample_rate())
    return 0; // not in loop
  last_time = now_time;
  return 1;
}
