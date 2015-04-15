// Kissing Lenikus
//
//  Based on:   (based on? well not much left of original code...)
//    Sweep
//    by BARRAGAN <http://barraganstudio.com>
//    This example code is in the public domain.

#include <Arduino.h>
#include <Servo.h>

// Auta mporeis na alakseis,  kai meta patas upload
int SERVO_POS_BACK             = 15;          // a value between 0 and 180
int SERVO_POS_FRONT            = 75;          // must be greater than SERVO_POS_BACK
int SERVO_DELAY                = 3;           // increase to make servo slower, in ms
int KISS_LENGTH                = 0.8 * 1000;  // delay front, in millisecs
unsigned long PAUSE_BETWEEN    = 17 * 1000UL; // pause between two kisses
int NUMBER_KISSES              = 3;           // used only for manual trigger via button
unsigned long SERVO_IDLE_MS    = 7000L;       // servo goes idle after X seconds
                                              // this might increase lifetime
                                              // of servo, since we cut VCC
                                              // using a powerful MOSFET

// Ultra sound distance sensor
int MAX_DISTANCE         = 300;      // sensore cannot measure more
int MIN_DISTANCE         = 0;        // sensor cannot measure closer distances
int DIST_TIMEOUT_MS      = 20;       // timeout for ultrasound ping
int NUM_STABLE           = 2;       // will measure often too increase stability
int MIN_KISSING_DISTANCE = 226;       // in cm
int MAX_KISSING_DISTANCE = 229;       // in cm


#define ECHO_PIN    10 // Echo Pin
#define TRIG_PIN    11 // Trigger Pin
#define SERVO_PIN   9
#define SERVO_OFF   12
#define LED_PIN     13  // lights up when kissing

#define BUTTON_PIN  8
#define POTI_PIN    A0

// ---------------------------------------------------------------------------


Servo servo;
int pos = 0;
unsigned long last_kiss = 0;

void setup() {
    Serial.begin (9600);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    servo.attach(SERVO_PIN);
    servo.write(SERVO_POS_BACK);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(SERVO_OFF, OUTPUT);
    digitalWrite(SERVO_OFF, LOW);
}


void loop() {
    if (digitalRead(BUTTON_PIN) == LOW)  {
        Serial.println("Kiss triggered by button");
        for (int i = 0; i < NUMBER_KISSES; i++)
            kiss();
    }
    else {
        int distance = get_distance_stable();
        if (distance < MIN_KISSING_DISTANCE || distance > MAX_KISSING_DISTANCE) {
            Serial.println("Kiss triggered by distance");
            kiss();
        }
    }

    // FIXME need to detect overflow of last_kiss
    if (millis() - last_kiss > SERVO_IDLE_MS) {
        digitalWrite(SERVO_OFF, LOW);
        Serial.println("Switching off servo --> IDLE mode");
    }
}

void kiss() {
    digitalWrite(SERVO_OFF, HIGH);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Kissing...");
    for(pos = SERVO_POS_BACK ; pos < SERVO_POS_FRONT; pos += 1) {
        servo.write(pos);
        delay(SERVO_DELAY);
    }
    delay(KISS_LENGTH);
    for(pos = SERVO_POS_FRONT; pos >= SERVO_POS_BACK + 1; pos-=1) {
        servo.write(pos);
        delay(SERVO_DELAY);
    }
    Serial.print("Sleeping PAUSE_BETWEEN=");
    Serial.println(PAUSE_BETWEEN);
    delay(PAUSE_BETWEEN);

    digitalWrite(LED_PIN, LOW);
    last_kiss = millis();
    Serial.println("Kissing done.");
}

int get_distance_stable() {
    for (int i = 0; i < NUM_STABLE; i++) {
        int distance = get_distance();
        if (distance > 0)
            return distance;

        delay(50);
    }

    // still no valid distance, assuming out of range
    return -1;
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
    /* The following TRIG_PIN/ECHO_PIN cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, DIST_TIMEOUT_MS * 1000UL);

    // Calculate the distance (in cm) based on the speed of sound.
    int distance = (float)duration/58.2;

    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance >= MAX_DISTANCE || distance <= MIN_DISTANCE) {
        return -1;
    }
    else {
        return distance;
    }
}
