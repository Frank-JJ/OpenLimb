#ifndef GAITMACHINE_H
#define GAITMACHINE_H

#pragma once

#include <vector>
#include <chrono>

#include <array>

#include "bluetoothHandler.h"

namespace gaits{
  enum Motors {Right=0, Tail=1, Left=2};

  struct motorCMD{
    Motors motorID;
    float amount;
    float start;
    float duration;
  };

  typedef std::vector<motorCMD> Gait;


  struct GaitStruct{
    std::vector<float> initialMotorPositions = {0,0,0};
    Gait gait;
    float gait_time = 2;   //Duration of the gait's repeating pattern (seconds)
    float gait_amp = 1;
  };

  typedef std::vector<GaitStruct> GaitVector;

  struct GaitSelectionInfo{
    std::vector<float> motor_init = {0,0,0};
    Gait gait;
    float gait_time = 2;
    float gait_amp = 1;
    float direction = 0;
    float gait_time_modifier = 1;
  };

  enum class BodySide {Left=0, Right=1, Center=2};
  enum class ServoSide {Left=0, Right=1};

  struct MotorConfig{
    BodySide bodySide;
    ServoSide servoSide;
  };

  typedef std::vector<MotorConfig> MotorConfigVector;  
}

using namespace gaits;
using namespace std::chrono;

class GaitMachine
{
  public:
    GaitMachine();
    ~GaitMachine();
    void setup();
    std::array<uint8_t, 3> loop(bluetoothHandler::BluetoothOutput btIn);

  private:
    int mapToMotorValue(float M_pos);
    void gaitControl(Gait gait, std::chrono::microseconds deltaT = std::chrono::microseconds(0));
    GaitSelectionInfo selectGait(bluetoothHandler::BluetoothOutput btIn);
    
    std::array<int8_t, 3> running_commands = {-1,-1,-1};

    std::array<uint8_t, 3> program_end_motorpos = {0,0,0};
    std::array<float, 3> M_pos = {0,0,0};
    std::array<float, 3> prev_M_pos = {0,0,0};

    std::vector<float> MOTOR_INIT = {0,0,0};
    float GAIT_T = 2;   // Duration of the gait's repeating pattern (seconds).
    float GAIT_AMP = 1;   // Max degree of amplitude.
    float MOTOR_MAX_VAL = 180; // Max degree of servos allowed.
    float MODE_DIR = 0; // Mode direction of gait. Goes from 0 to 1, where 0.5 is the center.
    float directionCenter = 0.5;
    float GAIT_T_AMP = 1; // Amplitude of gait duration. As in what to multiply GAIT_T with to get the final gait duration.
    
    float tick_frq = 200; // Hz
    microseconds tick_Time = duration_cast<microseconds>(std::chrono::duration<float>(1/tick_frq));

    std::array<uint8_t, 3> motor_pos_array;

    std::chrono::time_point<std::chrono::steady_clock> tick_start;
    std::chrono::time_point<std::chrono::steady_clock> last_call;

    
};

#endif