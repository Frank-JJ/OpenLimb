#ifndef BLUETOOTHHANDLER_H
#define BLUETOOTHHANDLER_H

#pragma once

#include <BluetoothHIDMaster.h>
#include <tuple>

enum direction{None, Forwards, Backwards, Left, Right};

namespace bluetoothHandler{
  static direction moveTowards;
  static direction rotateTowards;
};

class bluetoothHandler
{
  public:
    bluetoothHandler();
    ~bluetoothHandler();
    static void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons);
    void setup();
    std::tuple<direction, direction> loop();

  private:
    BluetoothHIDMaster hid;
    bool hid_connected = true;
    bool hid_running = true;
};

#endif