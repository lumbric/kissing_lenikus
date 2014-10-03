// Sweep
// by BARRAGAN <http://barraganstudio.com>
// This example code is in the public domain.

#include <Arduino.h>
#include <Servo.h>

///
// auta mporeis na alakseis
// kai meta upload
int SERVO_POS_BACK    = 65;          // a value between 0 and 180
int SERVO_POS_FRONT   = 165;         // must be greater than SERVO_POS_BACK
int SERVO_DELAY       = 3;           // increase to make servo slower, in millisecs
int KISS_LENGTH       = 0.8 * 1000;  // delay front, in millisecs
int PAUSE_BETWEEN     = 2 * 1000;    // pause between two kisses
int NUMBER_KISSES     = 1;

// Ultra sound distance sensor
int MAX_DISTANCE         = 300;      // sensore cannot measure more
int MIN_DISTANCE         = 0;        // sensor cannot measure closer distances
int NUM_STABLE           = 10;       // will measure often too increase stability
int MIN_KISSING_DISTANCE = 50;       // in cm

#define ECHO_PIN    10 // Echo Pin
#define TRIG_PIN    11 // Trigger Pin
#define SERVO_PIN   9

#define BUTTON_PIN  8
#define POTI_PIN    A0

// ---------


Servo servo;


int pos = 0;

void setup() {
    Serial.begin (9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    servo.attach(SERVO_PIN);
}


void loop() {
    if (digitalRead(BUTTON_PIN) == LOW)  {
        kiss();
    }
    else {
        int distance = get_distance_stable();
        Serial.print("Distance ");
        Serial.println(distance);
        if (distance <= MIN_KISSING_DISTANCE && distance > 0) {
            kiss();
        }
    }

    // Debug mode
    // hold button until kissed
    if (digitalRead(BUTTON_PIN) == LOW) {
        while (true) {
            int pos = map(analogRead(POTI_PIN) , 0, 1023, 0, 180);
            servo.write(pos);
            Serial.print("Servo position: ");
            Serial.println(pos);
            delay(1000);
        }
    }
}

void kiss() {
    Serial.println("Kissing...");
    for (int i=0; i < NUMBER_KISSES; i++) {
        for(pos = SERVO_POS_BACK ; pos < SERVO_POS_FRONT; pos += 1) {
            servo.write(pos);
            delay(SERVO_DELAY);
        }
        delay(KISS_LENGTH);
        for(pos = SERVO_POS_FRONT; pos>=SERVO_POS_BACK +1; pos-=1) {
            servo.write(pos);
            delay(SERVO_DELAY);
        }
        delay(PAUSE_BETWEEN);
    }
    Serial.println("Kissing done.");
}

void turn(int pos, int speed_delay) {

}

int get_distance_stable() {
    // delay to avoid interference with servo
    delay(50);

    int max_dist = 0;
    for (int i = 0; i < NUM_STABLE; i++) {
        int distance = get_distance();
        if (distance > max_dist)
            max_dist = distance;
    }
    return max_dist;
}


/*
 HC-SR04 Ping distance sensor:
 VCC to arduino 5v 
 GND to arduino GND
 Echo to Arduino pin 7 
 Trig to Arduino pin 8

 This sketch originates from Virtualmix: http://goo.gl/kJ8Gl
 Has been modified by Winkle ink here: http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html
 And modified further by ScottC here: http://arduinobasics.blogspot.com/
 on 10 Nov 2012.
 */
int get_distance() {
    long duration, distance;

    /* The following TRIG_PIN/ECHO_PIN cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate the distance (in cm) based on the speed of sound.
    distance = duration/58.2;

    if (distance >= MAX_DISTANCE || distance <= MIN_DISTANCE) {
        return -1;
    }
    else {
        return distance;
    }

    //Delay 50ms before next reading.
    delay(50);
}
