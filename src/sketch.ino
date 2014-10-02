// Sweep
// by BARRAGAN <http://barraganstudio.com>
// This example code is in the public domain.

#include <Arduino.h>
#include <Servo.h>

 ///
 // auta mporeis na alakseis
 // kai meta upload
int start = 45; // a value between 0 and 180
int end_pos = 165; // must be greater than start
int speed_milli = 3;
int pause  = 2  * 1000;
int pause_front  = 0.8  * 1000;


#define echoPin    10 // Echo Pin
#define trigPin    11 // Trigger Pin
#define SERVO_PIN  9
#define BUTTON_PIN 8

// ---------


Servo servo;


int pos = 0;

void setup()
{
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  servo.attach(SERVO_PIN);
}


void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Kissing...");
        kiss();
        Serial.println("Kissing done.");
    }
}

void kiss() {
    for(pos = start ; pos < end_pos; pos += 1) {
        servo.write(pos);
        delay(speed_milli);
    }
    delay(pause_front);
    for(pos = end_pos; pos>=start +1; pos-=1) {
        servo.write(pos);
        delay(speed_milli);
    }
    delay(pause);
}
