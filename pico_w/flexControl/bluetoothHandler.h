#ifndef BLUETOOTHHANDLER_H
#define BLUETOOTHHANDLER_H

#pragma once

#include <BluetoothHIDMaster.h>
#include <tuple>


namespace bluetoothHandler{
  enum class DirectionGait{None, Forwards, Backwards, Left, Right};
  enum class LeftJoystick{None, Forwards, Backwards, Left, Right};
  enum class RightJoystick{None, Forwards, Backwards, Left, Right};
  enum class TheFourButtonsOnTheFrontOfTheController{None, X, Y, B, A};
  
  static DirectionGait directionGait;
  static LeftJoystick leftJoystick;
  static RightJoystick rightJoystick;
  static TheFourButtonsOnTheFrontOfTheController buttonsGait;
};

class BluetoothHandler
{
  public:
    BluetoothHandler();
    ~BluetoothHandler();
    static void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons);
    void setup();
    loop();

  private:
    BluetoothHIDMaster hid;
    bool hid_connected = true;
    bool hid_running = true;
};

#endif