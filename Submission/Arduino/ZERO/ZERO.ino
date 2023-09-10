#include <Servo.h>

Servo myservo1;  
Servo myservo2;
Servo myservo3;
Servo myservo4;

int pos1 = 0;    
int pos2 = 120       ;  //90 is ZERO   125
int pos3 = 102-90   ; //102 is ZER0
int pos4 = 130      ;               // 130

void setup() {
  myservo1.attach(9);  
  myservo2.attach(6); 
  myservo3.attach(5); 
  myservo4.attach(3); 
}

void loop() {
    myservo1.write(pos1);
    myservo2.write(pos2);
    myservo3.write(pos3);
    myservo4.write(pos4);          
    delay(50);                     
 
}
