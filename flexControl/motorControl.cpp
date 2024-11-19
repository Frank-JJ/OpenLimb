#include "motorControl.h"

MotorControl::MotorControl()
{

}

MotorControl::~MotorControl()
{

}

void MotorControl::setup() {
	for (size_t i = 0; i < NUM_SERVOS; i++)
	{
		servo[i].attach(servoPins[i], servoMins[i], servoMaxs[i]);  // Servo pin
		servo[i].write(servoVals[i]); // Initial servo angle
	}

	Serial.println("MotorControl started!");
}

void MotorControl::loop(std::array<uint8_t, NUM_SERVOS> servoVals) {
  bool same = true;
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    if (this->servoVals[i] != servoVals[i])
      same = false;
  }
  if (!same){
    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
      Serial.printf("servo%i:%i\n",i,servoVals[i]);
      servo[i].write(servoVals[i]); // Set servo angles
    }
    Serial.println("");
    this->servoVals = servoVals;
  }
}