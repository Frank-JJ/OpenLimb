#ifndef GAITMACHINE_H
#define GAITMACHINE_H

#pragma once

#include <vector>
#include <chrono>

#include <array>

#include "bluetoothHandler.h"

namespace gaits{
  enum Motors {Left=1, Right=2, Tail=3};

  struct motorCMD{
    int motorID;
    float amount;
    float start;
    float duration;
  };

  typedef std::vector<motorCMD> Gait;

  struct GaitStruct{
    Gait gait;
    float gait_time = 2;   //Duration of the gait's repeating pattern (seconds)
    float gait_amp = 1;
  };

  typedef std::vector<GaitStruct> GaitVector;

  struct GaitSelectionInfo{
    Gait gait;
    float gait_time = 2;
    float gait_amp = 1;
    float direction = 0;
  };
}

using namespace gaits;
using namespace std::chrono;

class GaitMachine
{
  public:
    GaitMachine();
    ~GaitMachine();
    void setup();
    std::array<uint8_t, 3> loop();

  private:
    int mapToMotorValue(float M_pos);
    void gaitControl(Gait gait, std::chrono::microseconds deltaT = std::chrono::microseconds(0));
    GaitSelectionInfo selectGait();
    
    std::array<int8_t, 3> running_commands = {-1,-1,-1};

    std::array<uint8_t, 3> program_end_motorpos = {0,0,0};
    std::array<float, 3> M_pos = {0,0,0};
    std::array<float, 3> prev_M_pos = {0,0,0};

    float GAIT_T = 2;   //Duration of the gait's repeating pattern (seconds)
    float GAIT_AMP = 1;   //Max degree of amplitude
    float MOTOR_MAX_VAL = 90; //Max degree of servos allowed
    float MODE_DIR = 0; //Mode direction of gait - for three legs it is 
    
    float tick_frq = 100; //Hz
    microseconds tick_Time = duration_cast<microseconds>(std::chrono::duration<float>(1/tick_frq));

    std::array<uint8_t, 3> motor_pos_array;

    std::chrono::time_point<std::chrono::steady_clock> tick_start;
    std::chrono::time_point<std::chrono::steady_clock> last_call;

    
};

#endif