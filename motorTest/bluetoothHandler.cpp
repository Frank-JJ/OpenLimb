#include "bluetoothHandler.h"

using namespace bluetoothHandler;


BluetoothHandler::BluetoothHandler()
{

}

BluetoothHandler::~BluetoothHandler()
{

}

// Joystick can get reports of 4 analog axes, 1 d-pad bitfield, and up to 32 buttons
// Axes and hats that aren't reported by the joystick are read as 0
void BluetoothHandler::joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons) 
{
  // (void) cbdata;
  // const char *hats[16] = { "U", "UR", "R", "DR", "D", "DL", "L", "UL", "", "", "", "", "", "", "", "." };
  // Serial.printf("Joystick: (%4d, %4d) (%4d, %4d), Hat: %-2s, Buttons:", x, y, z, rz, hats[hat & 15]);
  // for (int i = 0; i < 32; i++) {
  //   Serial.printf(" %c", (buttons & 1 << i) ? '*' : '.');
  // }
  // Serial.println();

  // Get arrow buttons output
  auto arrows = hat & 15;
  if (arrows == 0)
  {
    arrowButtons = ArrowButtons::Forwards;
  }
  else if(arrows == 4)
  {
    arrowButtons = ArrowButtons::Backwards;
  }
  else if(arrows == 6)
  {
    arrowButtons = ArrowButtons::Left;
  }
  else if(arrows == 2)
  {
    arrowButtons = ArrowButtons::Right;
  }
  else
  {
    arrowButtons = ArrowButtons::None;
  }

  // Get left joystick up-down output
  leftJoystickUpDown = 1 - ((float)y / 256);

  
  // Get right joystick up-down output
  rightJoystickUpDown = 1 - ((float)rz / 256);
  
  
  // Get right joystick left-right output
  rightJoystickLeftRight = (float)z / 256;
  
  if (buttons & 1 << 12)
  {
    bluetoothHandler::buttons = TheFourButtonsOnTheFrontOfTheController::None;
  }
  else if (buttons & 1 << 3)
  {
    bluetoothHandler::buttons = TheFourButtonsOnTheFrontOfTheController::X;
  }
  else if (buttons & 1 << 1)
  {
    bluetoothHandler::buttons = TheFourButtonsOnTheFrontOfTheController::B;
  }
  else if (buttons & 1 << 4)
  {
    bluetoothHandler::buttons = TheFourButtonsOnTheFrontOfTheController::Y;
  }
  else if (buttons & 1 << 0)
  {
    bluetoothHandler::buttons = TheFourButtonsOnTheFrontOfTheController::A;
  }
  // Serial.printf("arrowButtons: %i | leftJoystickUpDown: %f | rightJoystickUpDown: %f | rightJoystickLeftRight: %f | bluetoothHandler: %i\n", arrowButtons, leftJoystickUpDown, rightJoystickUpDown, rightJoystickLeftRight, bluetoothHandler::buttons);
}

void BluetoothHandler::setup()
{
  Serial.printf("Starting HID master, put your device in pairing mode now.\n");

  hid.onJoystick(joy);

  hid.begin();

  hid.connectJoystick();
}

BluetoothOutput BluetoothHandler::loop()
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
  // if (movement != None)
  //   Serial.printf("movement:%i\n", movement);
  // Serial.printf("arrowButtons: %i | leftJoystickUpDown: %f | rightJoystickUpDown: %f | rightJoystickLeftRight: %f | bluetoothHandler: %i\n", arrowButtons, leftJoystickUpDown, rightJoystickUpDown, rightJoystickLeftRight, bluetoothHandler::buttons);
  // return {arrowButtons, leftJoystickUpDown, rightJoystickUpDown, rightJoystickLeftRight, bluetoothHandler::buttons};
  return {arrowButtons,
    leftJoystickUpDown,
    leftJoystickLeftRight,
    rightJoystickUpDown,
    rightJoystickLeftRight,
    buttons};
}