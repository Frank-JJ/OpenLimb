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

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#pragma once

#include <Servo.h>

#include <array>
#include <chrono>


using namespace std::chrono;


class MotorControl
{
  public:
    MotorControl();
    ~MotorControl();
    void setup();
    void loop(std::array<uint8_t, 3> servoVals);

  private:
    static const int NUM_SERVOS = 3;

    Servo servo[NUM_SERVOS]; // create servo object to control servos
    uint8_t servoPins[NUM_SERVOS] = {19,20,21}; // Servo blue TG9E, Gaffa black, Black without gaffa
    uint16_t servoMins[NUM_SERVOS] = {544,544,544}; // Servo blue TG9E, Gaffa black, Black without gaffa
    uint16_t servoMaxs[NUM_SERVOS] = {2400,2400,2400}; // Servo blue TG9E, Gaffa black, Black without gaffa
    std::array<uint8_t, NUM_SERVOS> servoVals = {0,0,0}; // Initializes to zero {Servo blue TG9E, Gaffa black, Black without gaffa}
};

#endif