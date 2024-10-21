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
		servo[i].write(initServoVals[i]); // Initial servo angle
		servo[i].attach(servoPins[i], servoMins[i], servoMaxs[i]);  // Servo pin
	}

	Serial.println("MotorControl started!");
  last_call = steady_clock::now();
}

void MotorControl::loop(std::array<uint8_t, 3> servoVals) {
  auto now = steady_clock::now();
  microseconds deltaT = duration_cast<microseconds>(now - last_call);
  if (deltaT >= commandTime){
    for (size_t i = 0; i < NUM_SERVOS; i++)
    {
      Serial.printf("servo%i:%i\n",i,servoVals[i]);
      servo[i].write(servoVals[i]); // Set servo angles
    }
    Serial.println("");
    last_call = now;
  }
}