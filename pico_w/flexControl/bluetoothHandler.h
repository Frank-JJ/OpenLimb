#ifndef BLUETOOTHHANDLER_H
#define BLUETOOTHHANDLER_H

#pragma once

#include <BluetoothHIDMaster.h>
#include <tuple>


namespace bluetoothHandler{
  enum direction{None, Forwards, Backwards, Left, Right, RotateLeft, RotateRight};
  
  static direction movement;
};

class BluetoothHandler
{
  public:
    BluetoothHandler();
    ~BluetoothHandler();
    static void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons);
    void setup();
    bluetoothHandler::direction loop();

  private:
    BluetoothHIDMaster hid;
    bool hid_connected = true;
    bool hid_running = true;
};

#endif