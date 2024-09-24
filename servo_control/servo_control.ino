#include <ESP32Servo.h>
 
Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
 
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin1 = 12;
int servoPin2 = 13;
int servoPin3 = 14;
 
void setup() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo1.setPeriodHertz(100);    // standard 50 hz servo
	myservo2.setPeriodHertz(100);    // standard 50 hz servo
	myservo3.setPeriodHertz(100);    // standard 50 hz servo
	myservo1.attach(servoPin1, 500, 2400); // attaches the servo on pin 13 to the servo object
	myservo2.attach(servoPin2, 500, 2400); // attaches the servo on pin 13 to the servo object
	myservo3.attach(servoPin3, 600, 2800); // attaches the servo on pin 13 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
}
 
void loop() {
 
	for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo1.write(pos);    // tell servo to go to position in variable 'pos'
		myservo2.write(pos);    // tell servo to go to position in variable 'pos'
		myservo3.write(pos);    // tell servo to go to position in variable 'pos'
		delay(5);             // waits 15ms for the servo to reach the position
	}
	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		myservo1.write(pos);    // tell servo to go to position in variable 'pos'
		myservo2.write(pos);    // tell servo to go to position in variable 'pos'
		myservo3.write(pos);    // tell servo to go to position in variable 'pos'
		delay(5);             // waits 15ms for the servo to reach the position
	}
}