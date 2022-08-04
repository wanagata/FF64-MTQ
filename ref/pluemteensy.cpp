#define F_SIZE 4
#include "src/CONTROLLER.h"
#ifndef ATTITUDE_h
#include "src/utility/Attitude.h"
#endif
#include "src/FastPID.h"
#include "src/driver.h"

// Create a union to easily convert float to byte
typedef union
{
  float number;
  uint8_t bytes[F_SIZE];
} FLOATUNION_t;

typedef union
{
  int16_t number;
  uint8_t bytes[2];
} INT16UNION_t;

// Receive 14 parameters: [qref0 qref1 qref2 qref3 q0 q1 q2 q3 w1 w2 w3 Mb1 Mb2 Mb3]
FLOATUNION_t t[14];

// Send 12 parameters: [PWMx PWMy PWMz r_err p_err y_err rpm1 rpm2 rpm3 A1 A2 A3]
FLOATUNION_t r[12];
int arr = 0;
bool iscomplete = 0;

// Motor connections
#define m1_dirPin 6
#define m1_enPin  3
#define m1_speedP 28
#define m1_speedM 31

#define m2_dirPin 7
#define m2_enPin  4
#define m2_speedP 25
#define m2_speedM 30

#define m3_dirPin 8
#define m3_enPin  5
#define m3_speedP 24
#define m3_speedM 29

// Declare interrupt functions
void interrupt_fnc1();
void interrupt_fnc2();
void interrupt_fnc3();
volatile uint16_t freq[3];

typedef struct {
  int number;
  int dir_pin;
  int en_pin;
  int pwm_pin;
  int freq_pin;
  volatile float freq;
} motor_pin;

motor_pin motor[3] = {
  {0, m1_dirPin, m1_enPin, m1_speedP, m1_speedM, 0},
  {1, m2_dirPin, m2_enPin, m2_speedP, m2_speedM, 0},
  {2, m3_dirPin, m3_enPin, m3_speedP, m3_speedM, 0}
};
uint32_t st, rt;

int f1 = 0;
int f2 = 0;
int f3 = 0;

// Declare rpm
int32_t rpm1;
int32_t rpm2;
int32_t rpm3;

// Magnetic torquers parameters
double L = 50E-3;
double rd = 2E-3;
double N = 480.0;
double Nd = 4*(log(L/rd)-1)/(pow(L/rd,2)-(4*log(L/rd))); // Demagnetization factor
double mur = 1.00002;
double a = PI*pow(rd,2)*N*(1+((mur-1)/(1+(mur-1)*Nd)));

// Declare current
Vector<3> Amp;

#define sampling_rate 50 // millis
CONTROLLER ctrlor(sampling_rate);
//float Kp = 0.6, Ki = 0.01, Kd = 0.84*4, Hz = 1 / sampling_rate;
float Kp = 4, Ki = 0.01, Kd = 0.4, Hz = 1 / sampling_rate;
int output_bits = 8;
bool output_signed = true;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
Vector<3> error_vec, cmd;
Vector<3> aux;
Vector<3> getPIDcmd(Vector<3> error);



void setup()
{
  //analogWriteResolution(15);
  // put your setup code here, to run once:
  intial_motor(&motor[0]);
  intial_motor(&motor[1]);
  intial_motor(&motor[2]);

  attachInterrupt(digitalPinToInterrupt(m1_speedM), interrupt_fnc1 , CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2_speedM), interrupt_fnc2 , CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3_speedM), interrupt_fnc3 , CHANGE);

  Serial.begin(250000);
  while (!Serial) {

  }
  
  st = millis();
}

uint32_t freq2rpm(const int freq) {
  return freq * 15*20;
}

void loop()
{
  if (updateSensor())
  {
    // ref
    Quaternion ref(t[0].number, t[1].number, t[2].number, t[3].number);
    ref.normalize();
    Attitude ref_att(1, ref, Vector<3>{0, 0, 0});
    ctrlor.setpointing(ref_att);

    Vector<3> gyro(t[8].number, t[9].number, t[10].number);
    Vector<3> mag(t[11].number, t[12].number, t[13].number);
    Vector<3> Bdot = gyro.cross(mag);
    Vector<3> kb(25000, 25000, 25000);

    aux[0] = -kb[0]*Bdot[0];
    aux[1] = -kb[1]*Bdot[1];
    aux[2] = -kb[2]*Bdot[2];

    Amp[0] = aux[0]/a;
    Amp[1] = aux[1]/a;
    Amp[2] = aux[2]/a;

    // sensor
    Quaternion sensor(t[4].number, t[5].number, t[6].number, t[7].number);
    sensor.normalize();
    //ctrlor.update_att(1, sensor, Vector<3>{t[8].number, t[9].number, t[10].number});
    ctrlor.update_att(1, sensor, gyro);

    error_vec = ctrlor.runpointing();
    error_vec.toDegrees();
    cmd = getPIDcmd(error_vec);

    // cmd 0,1,2 = y,p,r
    run_motor(&motor[0], cmd[2]);
    run_motor(&motor[1], cmd[1]);
    run_motor(&motor[2], cmd[0]);
    rt = millis();
    
    r[0].number = cmd[2];
    r[1].number = cmd[1];
    r[2].number = cmd[0];
    r[3].number = error_vec[2];
    r[4].number = error_vec[1];
    r[5].number = error_vec[0];
    r[6].number = -rpm1;
    r[7].number = -rpm2;
    r[8].number = -rpm3;
    r[9].number = Amp[0];
    r[10].number = Amp[1];
    r[11].number = Amp[2];
    sendFloat(r, 12);
  }

  if (millis() - st >= 50)
  {
    noInterrupts();
    f1 = freq[0];
    f2 = freq[1];
    f3 = freq[2];
    freq[0] = 0;
    freq[1] = 0;
    freq[2] = 0;
    interrupts();

    rpm1 = int32_t(freq2rpm(f1));
    rpm2 = int32_t(freq2rpm(f2));
    rpm3 = int32_t(freq2rpm(f3));

    if(cmd[2] < 0)
    {
      rpm1 = -int32_t(freq2rpm(f1));
    }

    if(cmd[1] < 0)
    {
      rpm2 = -int32_t(freq2rpm(f2));
    }

    if(cmd[0] < 0)
    {
      rpm3 = -int32_t(freq2rpm(f3));
    }
    
    st = millis();
  }
}

void intial_motor(motor_pin *_motor) {
  pinMode(_motor->dir_pin, OUTPUT);
  pinMode(_motor->en_pin, OUTPUT);
  pinMode(_motor->pwm_pin, OUTPUT);
  pinMode(_motor->freq_pin, INPUT_PULLUP);
  run_motor(_motor, 0);
}

void run_motor(motor_pin *_motor, int pwm) {
  digitalWrite(_motor->en_pin, 0);
  if (pwm == 0) {
    return;
  }

  if (pwm > 240)
  {
    pwm = 240;
  }
  
  bool dir = (pwm > 0) ? 0 : 1;
  int ab_pwm = (!dir) ? pwm : -pwm;
  digitalWrite(_motor->en_pin, 0);
  digitalWrite(_motor->dir_pin, dir);
  analogWrite(_motor->pwm_pin, ab_pwm);
  //digitalWrite(_motor->en_pin, 1);
  if (pwm > 25 && pwm > 0)
  {
    digitalWrite(_motor->en_pin, 1);
  }

  if (pwm < -25 && pwm < 0)
  {
    digitalWrite(_motor->en_pin, 1);
  }
}

void interrupt_fnc1()
{
  if (digitalRead(motor[0].freq_pin) == 0) {
    freq[0]++;
  }
}

void interrupt_fnc2()
{
  if (digitalRead(motor[1].freq_pin) == 0) {
    freq[1]++;
  }
}

void interrupt_fnc3()
{
  if (digitalRead(motor[2].freq_pin) == 0) {
    freq[2]++;
  }
}  

Vector<3> getPIDcmd(Vector<3> error)
{
  // error.toDegrees();
  Vector<3> rt;
  for (int i = 0; i < 3; i++)
  {
    rt[i] = myPID.step(error[i], 0);
  }
  return rt;
}

bool updateSensor()
{
  if (Serial.available() > 0)
  {
    uint8_t temp = Serial.read();
    if (temp == 'S')
    { // for heading message
      for (int i = 0; i < 14; i++)
      {
        t[i].number = getFloat();
      }
      return 1;
    }
  }
  return 0;
}

void sendInt16(INT16UNION_t *data, int nInt)
{
  Serial.write('A');
  for (int index = 0; index < nInt; index++)
  {
    for (int i = 0; i < 2; i++)
    {
      Serial.write(data[index].bytes[i]);
    }
  }
  Serial.print('\n');
}

void sendFloat(FLOATUNION_t *data, int nInt)
{
  Serial.write('A');
  for (int index = 0; index < nInt; index++)
  {
    for (int i = 0; i < 4; i++)
    {
      Serial.write(data[index].bytes[i]);
    }
  }
  Serial.print('\n');
}

float getFloat()
{
  FLOATUNION_t t;
  for (int i = 0; i < 4; i++)
  {
    t.bytes[i] = Serial.read();
  }
  return t.number;
}