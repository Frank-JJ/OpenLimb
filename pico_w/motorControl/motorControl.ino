/*
Project for controlling servo using Arduino board for Bio inspired robotics project

Using board [Arduino UNO R3]
With shield [TA0039 Sensor Expansion Shield V5.0] (https://makerhero.com/img/files/download/TA0039-Datasheet.pdf)
To control servo [Fitec FS90 9G Mini Servo] (https://www.addicore.com/products/feetech-fitec-fs90-9g-mini-servo-with-accessories)

Servo has cables:
Brown    = Ground
Red      = Power
Yellow   = Signal

Servo can have pins >= 2, since 0 and 1 are interfered with by Serial connection.
*/

#include <Servo.h>
#define NUM_SERVOS 3

Servo servo[NUM_SERVOS]; // create servo object to control servos
uint8_t servoPins[NUM_SERVOS] = {19,20,21}; // Servo blue TG9E, Gaffa black, Black without gaffa
uint16_t servoMins[NUM_SERVOS] = {550,550,750}; // Servo blue TG9E, Gaffa black, Black without gaffa
uint16_t servoMaxs[NUM_SERVOS] = {2450,2300,2500}; // Servo blue TG9E, Gaffa black, Black without gaffa
uint8_t servoVals[NUM_SERVOS] = {0,0,0}; // Initializes to zero {Servo blue TG9E, Gaffa black, Black without gaffa}

void setup() {
    Serial.begin(115200, SERIAL_8N1); // Begin serial connection with 115200 Baud Rate, 8 data bits, no parity, one stop bit
    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
        servo[i].write(servoVals[i]); // Initial servo angle
        servo[i].attach(servoPins[i], servoMins[i], servoMaxs[i]);  // Servo pin
    }
    
    Serial.println("ManualControl started!");
}

void loop() {
    
    // Check for data in serial connection buffer
    if (Serial.available() >= NUM_SERVOS) {
        uint32_t c = Serial.readBytes(servoVals, NUM_SERVOS);  // Get NUM_SERVOS bytes from serial buffer
        
        // Handle data and send response back through serial connection
        Serial.print("Recieved: ");
        for (size_t i = 0; i < NUM_SERVOS; i++)
        {
            Serial.print(servoVals[i]);
            Serial.print(" | ");
        }

        Serial.print("Setting angle: ");
        for (size_t i = 0; i < NUM_SERVOS; i++)
        {
            Serial.print(servoVals[i]);
            Serial.print(" | ");
            servo[i].write(servoVals[i]); // Set servo angles
        }
        Serial.println("");
    }
}