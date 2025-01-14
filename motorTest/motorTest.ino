#include "bluetoothHandler.h"
#include "gaitMachine.h"
#include "motorControl.h"

BluetoothHandler bh;
GaitMachine gm;
MotorControl mc;

void setup() 
{
  Serial.begin();
  delay(3000);

  bh.setup();
  gm.setup();
  mc.setup();
}


void loop() 
{
  bluetoothHandler::BluetoothOutput btOut = bh.loop();
  std::array<uint8_t, 3> motorOutput = gm.loop(btOut);
  mc.loop(motorOutput);
}
