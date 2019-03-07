#include <Servo.h>
Servo ESC_1;  
Servo ESC_2;
Servo ESC_3;
Servo ESC_4;    
static int potValue;  
 int e;
void setup() {
  Serial.begin(9600);
  ESC_1.attach(5,1000,2000); 
   ESC_2.attach(6,1000,2000);
  ESC_3.attach(9,1000,2000);
  ESC_4.attach(10,1000,2000);// (pin, min pulse width, max pulse width in milliseconds) 
}
void loop() {

  potValue = analogRead(A0);
  potValue = map(potValue, 0, 1023, 0, 180);
 ESC_1.write(potValue);
  ESC_2.write(potValue);
  ESC_3.write(potValue);
  ESC_4.write(potValue);
  Serial.println(potValue);
  delay(3000);
  if(Serial.available() >0)
  {
    while(1)
    {
      if(Serial.available() > 0)
      {
        char z;
        int a=0;
          z = Serial.read();
           
          switch(z)
          {
               case 'a':
                        potValue = potValue + 10;
                        Serial.println("a is excuted");
                        break;

              case 'b':
                         potValue = potValue - 20;
                         Serial.println("b is excuted");
                         break;
          
                
               case 'c':
                        potValue = potValue + 20;
                        Serial.println("c is excuted");
                        break;

                
               case 'd': 
                        potValue = potValue + 5;
                        Serial.println("d  is excuted");
                        break;
           
              default:
                        exit(0);
                          a=1;
                         Serial.println("default excuted");
          }
          e = potValue + 8;
          ESC_1.write(potValue);
          ESC_2.write(e);
          ESC_3.write(potValue);
          ESC_4.write(potValue);
          Serial.println(potValue);
        }
      }
  }
  else
  {
    potValue = 60;   
    e = potValue + 8;
    ESC_1.write(potValue);
    ESC_2.write(e);
    ESC_3.write(potValue);
    ESC_4.write(potValue);
    Serial.println(potValue);
    delay(100);
  }    
}
