#ifndef BLUETOOTHHANDLER_H
#define BLUETOOTHHANDLER_H

#pragma once

#include <BluetoothHIDMaster.h>
#include <tuple>


namespace bluetoothHandler{
  enum class ArrowButtons{None, Forwards, Backwards, Left, Right};
  enum class TheFourButtonsOnTheFrontOfTheController{None, X, Y, B, A};
  
  static ArrowButtons arrowButtons;
  static float leftJoystickUpDown = 0;
  static float leftJoystickLeftRight = 0;
  static float rightJoystickUpDown = 0;
  static float rightJoystickLeftRight = 0;
  static TheFourButtonsOnTheFrontOfTheController buttons;
  
  struct BluetoothOutput{
    ArrowButtons arrowButtons;
    float leftJoystickUpDown;
    float leftJoystickLeftRight;
    float rightJoystickUpDown;
    float rightJoystickLeftRight;
    TheFourButtonsOnTheFrontOfTheController buttons;
  };
};

class BluetoothHandler
{
  public:
    BluetoothHandler();
    ~BluetoothHandler();
    static void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons);
    void setup();
    bluetoothHandler::BluetoothOutput loop();

  private:
    BluetoothHIDMaster hid;
    bool hid_connected = true;
    bool hid_running = true;
};

#endif