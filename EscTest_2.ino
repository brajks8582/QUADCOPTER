#include <Servo.h>
#include<SoftwareSerial.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#include <PID_v1.h>


SoftwareSerial mySerial(0, 1); // RX, TX

Servo ESC_1; // front left
Servo ESC_2; //front right
Servo ESC_3; // back left
Servo ESC_4; //back right


float esc_f_l, esc_f_r, esc_b_l, esc_b_r;


double throttle = 1000; //initial value of throttle to the motors


//Define Variables we'll be connecting to
double Setpoint_r, Input_r, Output_r;

//Specify the links and initial tuning parameters
double Kp_r = 2, Ki_r = 5, Kd_r = 1;
PID myPID_r(&Input_r, &Output_r, &Setpoint_r, Kp_r, Ki_r, Kd_r, DIRECT);

double Setpoint_p, Input_p, Output_p;

//Specify the links and initial tuning parameters
double Kp_p = 2, Ki_p = 5, Kd_p = 1;
PID myPID_p(&Input_p, &Output_p, &Setpoint_p, Kp_p, Ki_p, Kd_p, DIRECT);


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  Input_r = kalAngleY;
  Setpoint_r = 0;

  Input_p = kalAngleX;
  Setpoint_p = 0;

  //turn the PID on
  myPID_r.SetMode(AUTOMATIC);

  //turn the PID on
  myPID_p.SetMode(AUTOMATIC);

  mySerial.begin(9600);
  ESC_1.attach(3, 1000, 2000);
  ESC_2.attach(6, 1000, 2000);
  ESC_3.attach(9, 1000, 2000);
  ESC_4.attach(10, 1000, 2000); // (pin, min pulse width, max pulse width in milliseconds)

  ESC_1.writeMicroseconds(1000);
  ESC_2.writeMicroseconds(1000);
  ESC_3.writeMicroseconds(1000);
  ESC_4.writeMicroseconds(1000);
  //  delay(2000);

}
char z;
void loop() {
  if(Serial.available() > 0 )
  {
    z = Serial.read();
    if(z == 'a'){
        ESC_1.writeMicroseconds(1000);
  ESC_2.writeMicroseconds(1000);
  ESC_3.writeMicroseconds(1000);
  ESC_4.writeMicroseconds(1000);
  while(z != 'b'){
    Serial.println(" press b to continue");
    z= Serial.read();
  }
    }
    if(z == 'c'){
      throttle = throttle + 100;
    }
    if(z == 'd')
    {
      throttle = throttle - 100;
    }
    if( z == 'e'){
      throttle = throttle + 50;
      
    }
    if(z == 'f'){
      throttle = throttle - 50;
    }
  }
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  kalAngleX = kalAngleX - 0.09;
  kalAngleY = kalAngleY + 2.35;
  Serial.print("\tpitch = ");
  Serial.print(kalAngleX); 
  Serial.print("\troll = ");
  Serial.print(kalAngleY); 
  Input_r = kalAngleY;
  myPID_r.Compute();


  Input_p = kalAngleX;
  myPID_p.Compute();

  esc_f_l = throttle + Output_r + Output_p;
  esc_b_l = throttle + Output_r - Output_p;
  esc_f_r = throttle - Output_r + Output_p;
  esc_b_r = throttle - Output_r - Output_p;

  
    Serial.print("\tfront left = "); Serial.print(esc_f_l); Serial.print("\tfront right = "); Serial.print(esc_f_r); Serial.print("\tback left = "); Serial.print(esc_b_l); Serial.print("\tback right = "); Serial.print(esc_b_r);
     Serial.print("\t pitch gain = "); Serial.print(Output_p); Serial.print("\troll gain = "); Serial.println(Output_r);
    ESC_1.writeMicroseconds(esc_f_l);
    ESC_2.writeMicroseconds(esc_f_r);
    ESC_3.writeMicroseconds(esc_b_l);
    ESC_4.writeMicroseconds(esc_b_r);

  }
