#include "bluetoothHandler.h"

bluetoothHandler bh;

void setup() 
{
  Serial.begin();
  delay(3000);

  bh.setup();
}


void loop() 
{
  auto [direction, rotation] = bh.loop();
}
