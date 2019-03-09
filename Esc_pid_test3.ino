#include <Servo.h>
#include<Wire.h>
#include<SoftwareSerial.h> 


SoftwareSerial mySerial(0, 1); // RX, TX

Servo ESC_1; // front left
Servo ESC_2; //front right
Servo ESC_3; // back left
Servo ESC_4; //back right


int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime_r, time_r, timePrev_r;
float elapsedTime_p, time_p, timePrev_p;
int i;
float rad_to_deg = 180/3.141592654;

float PID_r, pwmLeft, pwmRight, error_r, previous_error_r;
float pid_p_r=0;
float pid_i_r=0;
float pid_d_r=0;


float PID_p, pwmBack, pwmFront, error_p, previous_error_p;
float pid_p_p=0;
float pid_i_p=0;
float pid_d_p=0;



/////////////////PID CONSTANTS/////////////////
double kp_r=3.6;//3.55
double ki_r=3.277;//0.003
double kd_r=0.99;//2.05

double kp_p=3.6;//3.55
double ki_p=3.277;//0.003
double kd_p=0.99;//2.05
///////////////////////////////////////////////

double throttle=1000; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

double total_error_r,last_error_r,total_error_p,last_error_p;





void setup() {
  // put your setup code here, to run once:

    Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  mySerial.begin(9600);
  ESC_1.attach(5,1000,2000); 
   ESC_2.attach(6,1000,2000);
  ESC_3.attach(9,1000,2000);
  ESC_4.attach(10,1000,2000);// (pin, min pulse width, max pulse width in milliseconds) 

  time_r = millis(); //Start counting time in milliseconds
  time_p = millis();
  ESC_1.writeMicroseconds(1000); 
  ESC_2.writeMicroseconds(1000);
  ESC_3.writeMicroseconds(1000); 
  ESC_4.writeMicroseconds(1000);
  delay(2000);

}

char z;

void loop() {
  // put your main code here, to run repeatedly:

  
     if(mySerial.available() >0 ){
      delay(100);
      z = mySerial.read();
       mySerial.println(z);
          if(z == 'a'){
             throttle = throttle + 100;
          }
          else if(z == 'b'){
             throttle = throttle - 100;
          }
          if(z == 'c'){
             throttle = throttle + 50;
          }
          else if(z == 'd'){
             throttle = throttle + 25;
          }
          if(z == 'e'){
             throttle = 1000;
          }
     }     
    timePrev_r = time_r;  // the previous time is stored before the actual time read
    time_r = millis();  // actual time read
    elapsedTime_r = (time_r - timePrev_r) / 1000;

    timePrev_p = time_p;  // the previous time is stored before the actual time read
    time_p = millis();  // actual time read
    elapsedTime_p = (time_p - timePrev_p) / 1000;

    Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 


     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

    /*---X---*/
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

    Wire.beginTransmission(0x68);
     Wire.write(0x43); //Gyro data first adress
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); //Just 4 registers
   
     Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();

    /*---X---*/
    Gyro_angle[0] = Gyr_rawX/131.0; 
    /*---Y---*/
    Gyro_angle[1] = Gyr_rawY/131.0;


    /*---X axis angle---*/ //pitch angle
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime_p) + 0.02*Acceleration_angle[0];
    /*---Y axis angle---*///roll angle
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime_r) + 0.02*Acceleration_angle[1];


    // new total angle
   Total_angle[1] = Total_angle[1] + 0.032994;
   Total_angle[0] = Total_angle[0] + 0.014066;
   
     /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
      Serial.print("Pitch angle = ");
      Serial.print(Total_angle[0]);

      Serial.print("roll angle = ");
      Serial.print(Total_angle[1]);

      Serial.print("  throttle is ");
     Serial.println(throttle);
     error_r = Total_angle[1] - desired_angle;
     error_p = Total_angle[0] - desired_angle;

    
    if(throttle == 1000)
    {
    ESC_1.writeMicroseconds(1000);
    ESC_3.writeMicroseconds(1000);
    ESC_4.writeMicroseconds(1000);
    ESC_2.writeMicroseconds(1000);
    }
    else
    {

    if(fabs(error_r) > fabs(error_p))
    {      
      pid_p_r = kp_r*error_r;
      total_error_r = total_error_r + error_r;
      pid_i_r = ki_r * elapsedTime_r*total_error_r;
      pid_d_r = kd_r*((error_r - previous_error_r)/elapsedTime_r);

      /*The final PID values is the sum of each of this 3 parts*/
      PID_r = pid_p_r + pid_i_r + pid_d_r;
      if(PID_r < -1000)
      {
      PID_r=-1000;
      }
      if(PID_r > 1000)
      {
        PID_r=1000;
       }

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
      pwmLeft = throttle + PID_r;
      pwmRight = throttle - PID_r;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
      if(pwmRight < 1000)
      {
        pwmRight= 1000; 
      } 
      if(pwmRight > 2000)
      {
        pwmRight=2000;
      }
//Left
      if(pwmLeft < 1000)
      {
        pwmLeft= 1000;
      }
      if(pwmLeft > 2000)
      {
        pwmLeft=2000;
      }

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
      ESC_1.writeMicroseconds(pwmLeft);
      ESC_3.writeMicroseconds(pwmLeft);
      ESC_4.writeMicroseconds(pwmRight);
      ESC_2.writeMicroseconds(pwmRight);

      

        pid_p_p = kp_p*error_p;

        pid_d_p = kd_p*((error_p - previous_error_p)/elapsedTime_p);

        total_error_r = total_error_r + error_r;
        pid_i_r = ki_r * elapsedTime_r*total_error_r;

/*The final PID values is the sum of each of this 3 parts*/
        PID_p = pid_p_p + pid_i_p + pid_d_p;


        if(PID_p < -1000)
        {
           PID_p=-1000;
        }
        if(PID_p > 1000)
        {
          PID_p=1000;
        }

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
        pwmFront = throttle - PID_p;
        pwmBack = throttle + PID_p;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
        if(pwmFront < 1000)
        {
          pwmFront= 1000;
        }
        if(pwmFront > 2000)
        {
          pwmFront=2000;
        }
//Left
        if(pwmBack < 1000)
        {
          pwmBack= 1000;
        }
        if(pwmBack > 2000)
        {
          pwmBack=2000;
        }

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
        ESC_1.writeMicroseconds(pwmFront);
        ESC_3.writeMicroseconds(pwmBack);
        ESC_4.writeMicroseconds(pwmBack);
        ESC_2.writeMicroseconds(pwmFront);
      }
       //Remember to store the previous error.

 //Remember to store the previous error.
      
      else
      {

        pid_p_p = kp_p*error_p;

        pid_d_p = kd_p*((error_p - previous_error_p)/elapsedTime_p);

        total_error_r = total_error_r + error_r;
        pid_i_r = ki_r * elapsedTime_r*total_error_r;

/*The final PID values is the sum of each of this 3 parts*/
        PID_p = pid_p_p + pid_i_p + pid_d_p;


        if(PID_p < -1000)
        {
           PID_p=-1000;
        }
        if(PID_p > 1000)
        {
          PID_p=1000;
        }

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
        pwmFront = throttle - PID_p;
        pwmBack = throttle + PID_p;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
        if(pwmFront < 1000)
        {
          pwmFront= 1000;
        }
        if(pwmFront > 2000)
        {
          pwmFront=2000;
        }
//Left
        if(pwmBack < 1000)
        {
          pwmBack= 1000;
        }
        if(pwmBack > 2000)
        {
          pwmBack=2000;
        }

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
        ESC_1.writeMicroseconds(pwmFront);
        ESC_3.writeMicroseconds(pwmBack);
        ESC_4.writeMicroseconds(pwmBack);
        ESC_2.writeMicroseconds(pwmFront);
        
      pid_p_r = kp_r*error_r;
      total_error_r = total_error_r + error_r;
      pid_i_r = ki_r * elapsedTime_r*total_error_r;
      pid_d_r = kd_r*((error_r - previous_error_r)/elapsedTime_r);

      /*The final PID values is the sum of each of this 3 parts*/
      PID_r = pid_p_r + pid_i_r + pid_d_r;
      if(PID_r < -1000)
      {
      PID_r=-1000;
      }
      if(PID_r > 1000)
      {
        PID_r=1000;
       }

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
      pwmLeft = throttle + PID_r;
      pwmRight = throttle - PID_r;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
      if(pwmRight < 1000)
      {
        pwmRight= 1000; 
      } 
      if(pwmRight > 2000)
      {
        pwmRight=2000;
      }
//Left
      if(pwmLeft < 1000)
      {
        pwmLeft= 1000;
      }
      if(pwmLeft > 2000)
      {
        pwmLeft=2000;
      }

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
      ESC_1.writeMicroseconds(pwmLeft);
      ESC_3.writeMicroseconds(pwmLeft);
      ESC_4.writeMicroseconds(pwmRight);
      ESC_2.writeMicroseconds(pwmRight);
      }
    }   //Remember to store the previous error.
    previous_error_p = error_p; //Remember to store the previous error.
    previous_error_r = error_r;      
}//end of loop void
  


