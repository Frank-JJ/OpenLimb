#ifndef GAITMACHINE_H
#define GAITMACHINE_H

#pragma once

#include <vector>
#include <chrono>

#include <array>

#include "bluetoothHandler.h"

namespace gaits{
  // Defines order of limbs and thus what order their actuator values should be in when sent to motorControl
  enum Motors {Right=0, Tail=1, Left=2};

  // Defines a single movement command for a motor
  struct motorCMD{
    Motors motorID; // What motor id corresponding to one of the limbs defined in motors should be commanded
    float amount; // What percentage of the gait amplitude should it be commanded to
    float start; // When during a gait duration should the command start as a percentage
    float duration; // How long should the command take as a percentage of the gait duration
  };

  // The patter for a gait is a vector of motor commands
  typedef std::vector<motorCMD> Gait;

  // A gait has a duration and amplitude, as well as initial motor values.
  // These parameters are defined with a gait using this struct
  struct GaitStruct{
    std::vector<float> initialMotorPositions = {0,0,0}; // Here we define the initial motor values
    Gait gait; // Here the gait is inserted
    float gait_time = 2; // Duration of the gait's repeating pattern (seconds)
    float gait_amp = 1; // Amplitude of the gait. A percentage of MOTOR_MAX_VAL
  };

  // Gaits can be stored in a vector for easy indexin
  typedef std::vector<GaitStruct> GaitVector;

  // This struct defines the parameters used to define the currently running gait, based on additional parameters
  // These parameters are set based on either a defined GaitStruct or modified from joystick input
  struct GaitSelectionInfo{
    std::vector<float> motor_init = {0,0,0}; // These are the initial values as from GaitStruct
    Gait gait; // This is the gait pattern
    float gait_time = 2; // This is the gait time from a GaitStruct
    float gait_amp = 1; // This is the gait amplitude as from GaitStruct or from the joystick left stick FORWARDS/BACKWARDS
    float direction = 0; // This is the gait direction, a modifier to the left and right motors as defined by the MotorConfigVector. It is set by the joystick right stick LEFT/RIGHT. In the center both are activated at 100%, when turned all the way to the LEFT or RIGHT, the opposite side is set to 0%.
    float gait_time_modifier = 1; // This is a multiplier for the gait duration. This is set by joystick right stick FORWARDS/BACKWARDS. When the stick is FORWARDS the value becomes 0, and thus duration becomes zero. When BACKWARDS the value becomes 2, and duration is doubled.
  };

  // Each limb can be defined as being on the left, center, or right side of the robot body
  enum class BodySide {Left=0, Right=1, Center=2};

  // Each servo can be defined as being on the left or right side of the servo mount for a limb, when looking from the servo mount with the limb foot poiinting away from the viewer
  enum class ServoSide {Left=0, Right=1};

  // The limb and servo sides are encapsulated in a MotorConfig struct, and further in a vector of these
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
    std::array<float, 3> prev_M_pos = {0,0,0}; // When running a gait, we do linear interpolation in each motor command. For this we use the previous commands amount value stored in this variable. If there is no previous command we use the initial motor values
    std::array<float, 3> M_pos = {0,0,0};  // We store the current linearly interpolated motor values here

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