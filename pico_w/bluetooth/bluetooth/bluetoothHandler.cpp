#include "SerialUSB.h"
#include "bluetoothHandler.h"

bluetoothHandler::bluetoothHandler()
{

}

bluetoothHandler::~bluetoothHandler()
{

}

// Joystick can get reports of 4 analog axes, 1 d-pad bitfield, and up to 32 buttons
// Axes and hats that aren't reported by the joystick are read as 0
void bluetoothHandler::joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons) 
{
  // (void) cbdata;
  // const char *hats[16] = { "U", "UR", "R", "DR", "D", "DL", "L", "UL", "", "", "", "", "", "", "", "." };
  // Serial.printf("Joystick: (%4d, %4d) (%4d, %4d), Hat: %-2s, Buttons:", x, y, z, rz, hats[hat & 15]);
  // for (int i = 0; i < 32; i++) {
  //   Serial.printf(" %c", (buttons & 1 << i) ? '*' : '.');
  // }
  // Serial.println();

  // bluetoothHandler* thisObject = (reinterpret_cast<bluetoothHandler*>(cbdata));

  if (x > 256*0.9) 
  {
    moveTowards = Right;
  }
  else if (x < 256*0.1) 
  {
    moveTowards = Left;
  }
  else if (y < 256*0.1) 
  {
    moveTowards = Forwards;
  }
  else if (y > 256*0.9) 
  {
    moveTowards = Backwards;
  }
  else
  {
    moveTowards = None;
  }
  // Serial.printf("moveTowards: %i\n", moveTowards);

  if (z > 256*0.9) 
  {
    rotateTowards = Right;
  }
  else if (z < 256*0.1) 
  {
    rotateTowards = Left;
  }
  else
  {
    rotateTowards = None;
  }
}

void bluetoothHandler::setup()
{
  Serial.printf("Starting HID master, put your device in pairing mode now.\n");

  hid.onJoystick(joy);

  hid.begin();

  hid.connectJoystick();
}

std::tuple<direction, direction> bluetoothHandler::loop()
{
  if (BOOTSEL) 
  {
    while (BOOTSEL) 
    {
      delay(1);
    }
    hid.disconnect();
    hid.clearPairing();
    Serial.printf("Restarting HID master, put your device in pairing mode now.\n");
    hid.connectAny();
  }
  if (hid.connected() && !hid_connected)
  {
    Serial.printf("hid connected: yes\n");
    hid_connected = true;
  }
  else if (!hid.connected() && hid_connected)
  {
    Serial.printf("hid connected: no\n");
    hid_connected = false;
  }
  if (hid.running() && !hid_running)
  {
    Serial.printf("hid running: yes\n");
    hid_running = true;
  }
  else if (!hid.running() && hid_running)
  {
    Serial.printf("hid running: no\n");
    hid_running = false;
  }

  if (moveTowards != None)
  {
    Serial.printf("moveTowards: %i\n", moveTowards);
  }
  if (rotateTowards != None)
  {
    Serial.printf("rotateTowards: %i\n", rotateTowards);
  }

  return {moveTowards, rotateTowards};
}