// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
 ///
 // auta mporeis na alakseis
 // kai meta upload
int start = 45; // a value between 0 and 180
int end_pos = 165; // must be greater than start
int speed_milli = 3;
int pause  = 2  * 1000;
int pause_front  = 0.8  * 1000;

// ---------

 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{ 
  for(pos = start ; pos < end_pos; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(speed_milli);                       // waits 15ms for the servo to reach the position 
  } 
  delay(pause_front);
  for(pos = end_pos; pos>=start +1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(speed_milli);                       // waits 15ms for the servo to reach the position 
  } 
  delay(pause);
} 
