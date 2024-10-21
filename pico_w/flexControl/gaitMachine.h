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

  typedef std::vector<Gait> GaitVector;
}

using namespace gaits;
using namespace std::chrono;

class GaitMachine
{
  public:
    GaitMachine();
    ~GaitMachine();
    void setup();
    std::array<uint8_t, 3> loop(bluetoothHandler::direction movement);

  private:
    int mapToMotorValue(float M_pos);
    void gaitControl(Gait gait, std::chrono::microseconds deltaT = std::chrono::microseconds(0));
    Gait selectGait(bluetoothHandler::direction movement);
    
    std::array<int8_t, 3> running_commands = {-1,-1,-1};

    std::array<uint8_t, 3> program_end_motorpos = {0,0,0};
    std::array<float, 3> M_pos = {0,0,0};
    std::array<float, 3> prev_M_pos = {0,0,0};

    const uint8_t GAIT_T = 2;   //Duration of the gait's repeating pattern (seconds)
    const uint8_t GAIT_AMP = 1;  //Amplitude of the gait (0-1)
    const uint8_t MOTOR_MAX_VAL = 90; //Max degree of servos allowed
    float tick_frq = 100; //Hz
    microseconds tick_Time = duration_cast<microseconds>(std::chrono::duration<float>(1/tick_frq));

    std::array<uint8_t, 3> motor_pos_array;

    std::chrono::time_point<std::chrono::steady_clock> tick_start;
    std::chrono::time_point<std::chrono::steady_clock> last_call;

    
};

#endif