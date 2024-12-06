#include "SerialUSB.h"
#include "gaitMachine.h"

using namespace gaits;
using namespace bluetoothHandler;

GaitStruct MAX_MOTOR = {
  .initialMotorPositions = {1,1,1},
  .gait={
    {Motors::Tail, 1, 0, 1},
    {Motors::Left, 1, 0, 1},
    {Motors::Right, 1, 0, 1}
  },
  .gait_time=5,
  .gait_amp=1
};


GaitStruct MIN_MOTOR = {
  .gait={
    {Motors::Tail, 0, 0, 1},
    {Motors::Left, 0, 0, 1},
    {Motors::Right, 0, 0, 1}
  },
  .gait_time=5,
  .gait_amp=1
};

GaitStruct Crawler = {
  .gait={
      {Motors::Tail, 0.8, 0, 0.1},
      {Motors::Left, 0.8, 0.1, 0.35},
      {Motors::Tail, 0, 0.1, 0.35},
      {Motors::Left, 0, 0.45, 0.05},
      {Motors::Tail, 0.8, 0.5, 0.1},
      {Motors::Right, 0.8, 0.6, 0.35},
      {Motors::Tail, 0, 0.6, 0.35},
      {Motors::Right, 0, 0.95, 0.05}
  },
  .gait_time=2,
  .gait_amp=1
};

GaitStruct TailPush = {
  .gait={
      {Motors::Tail, 0.8, 0, 0.1},
      {Motors::Tail, 0, 0.1, 0.35},
      {Motors::Tail, 0.8, 0.5, 0.1},
      {Motors::Tail, 0, 0.6, 0.35}
  },
  .gait_time=2,
  .gait_amp=1
};

GaitStruct Worm = {
  .gait={
    {Motors::Tail,  0.68,  0,    0.2},
    {Motors::Left,  0.9,  0,    0.5},
    {Motors::Right, 0.9,  0,    0.5},
    {Motors::Tail,  0.2,  0.5,  0.2},
    {Motors::Left,  0,    0.8,  0.05},
    {Motors::Right, 0,    0.8,  0.05},
    {Motors::Tail,  0,    0.8,  0.2}
  },
  .gait_time=1,
  .gait_amp=1
};

GaitStruct Jump = {
  .gait={
    {Motors::Tail, 1.0, 0, 0.001},
    {Motors::Tail, 0.0, 0.4, 0.1}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct WormR = {
  .gait={
    {Motors::Tail,  0.68,  0,    0.2},
    {Motors::Left,  0.5,  0,    0.5},
    {Motors::Right, 0.9,  0,    0.5},
    {Motors::Tail,  0.2,  0.5,  0.2},
    {Motors::Left,  0,    0.8,  0.05},
    {Motors::Right, 0,    0.8,  0.05},
    {Motors::Tail,  0,    0.8,  0.2}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct PushLegs = {
  .gait={
    {Motors::Left,  1,  0,    0.5},
    {Motors::Right, 1,  0,    0.5},
    {Motors::Left,  0,  0.5,  1},
    {Motors::Right, 0,  0.5,  1},
  },
  .gait_time=5,
  .gait_amp=1
};

GaitStruct PushTail = {
  .gait={
    {Motors::Tail,  1,  0,    0.5},
    {Motors::Tail,  0,  0.5,  1},
  },
  .gait_time=5,
  .gait_amp=1
};

GaitStruct UP_DOWN3 = {
  .initialMotorPositions = {0,0,0},
  .gait={
    {Motors::Tail, 0.8, 0, 0.8},
    {Motors::Tail, 0, 0.8, 1}
  },
  .gait_time=1,
  .gait_amp=1
};
GaitStruct UP_DOWN2 = {
  .initialMotorPositions = {0,0,0},
  .gait={
    {Motors::Right, 0.8, 0, 0.8},
    {Motors::Right, 0, 0.8, 1}
  },
  .gait_time=1,
  .gait_amp=1
};
GaitStruct UP_DOWN1 = {
  .initialMotorPositions = {0,0,0},
  .gait={
    {Motors::Left, 0.8, 0, 0.8},
    {Motors::Left, 0, 0.8, 1}
  },
  .gait_time=1,
  .gait_amp=1
};
GaitStruct BackJump = {
  .initialMotorPositions = {0,0,0},
  .gait={
    {Motors::Left, 1, 0, 0.5},
    {Motors::Right, 1, 0, 0.5},
    {Motors::Left, 0, 0.5, 0.5},
    {Motors::Right, 0, 0.5, 0.5}
  },
  .gait_time=0.5,
  .gait_amp=1
};
GaitStruct BackCrawl = {
  .gait={
      {Motors::Left, 1, 0, 0.25},
      {Motors::Left, 0, 0.25, 0.25},
      {Motors::Right, 1, 0.5, 0.25},
      {Motors::Right, 0, 0.75, 0.25}
  },
  .gait_time=1,
  .gait_amp=1
};

GaitVector movementGaits = {Crawler, TailPush, BackJump, BackCrawl};

MotorConfigVector motorConfigVector = {
  {BodySide::Left, ServoSide::Left},
  {BodySide::Center, ServoSide::Left},
  {BodySide::Right, ServoSide::Left}
};

GaitMachine::GaitMachine()
{

}

GaitMachine::~GaitMachine()
{

}

int GaitMachine::mapToMotorValue(float M_pos){
  return (int)min(MOTOR_MAX_VAL,(M_pos * GAIT_AMP * MOTOR_MAX_VAL));
}

void GaitMachine::gaitControl(Gait gait, std::chrono::microseconds deltaT){
  static int M1_mapped = 0, M2_mapped = 0, M3_mapped = 0;
  static float T_in_gait = 0.0, T_elapsed = 0.0;

  //Incrementing the timing of the current gait
  float deltaT_sec = deltaT.count() * 0.000001;
  T_in_gait += deltaT_sec;
  T_elapsed += deltaT_sec;

  
  if(T_in_gait >= GAIT_T*GAIT_T_AMP){ 
    //Wrap around if the timing is exceeding the gait period
    T_in_gait -= GAIT_T*GAIT_T_AMP;
  }

  prev_M_pos[0] = MOTOR_INIT[0];
  prev_M_pos[1] = MOTOR_INIT[1];
  prev_M_pos[2] = MOTOR_INIT[2];
  M_pos[0] = MOTOR_INIT[0];
  M_pos[1] = MOTOR_INIT[1];
  M_pos[2] = MOTOR_INIT[2];
  for(const auto& cmd : gait){
    float cmd_start_time = cmd.start * GAIT_T*GAIT_T_AMP, cmd_end_time = (cmd.start + cmd.duration) * GAIT_T*GAIT_T_AMP;
    if (T_in_gait < cmd_start_time || T_in_gait >= cmd_end_time){
      prev_M_pos[cmd.motorID] = cmd.amount;
    }
    if(T_in_gait >= cmd_start_time && T_in_gait < cmd_end_time){
      float elapsed_time_in_cmd = (T_in_gait - cmd_start_time) / (cmd_end_time - cmd_start_time);
      
      M_pos[cmd.motorID] = prev_M_pos[cmd.motorID] + (cmd.amount - prev_M_pos[cmd.motorID]) * elapsed_time_in_cmd;
    }
  }

  for (int i = 0; i < motorConfigVector.size(); i++)
  {
    if (motorConfigVector[i].servoSide == ServoSide::Right)
    {
      M_pos[i] = 1 - M_pos[i];
      // Serial.printf("ServoSide::Right");
    }
    if (motorConfigVector[i].bodySide == BodySide::Left)
    {
      M_pos[i] = M_pos[i] * min(1, 2 - MODE_DIR*2);
      // Serial.printf(" BodySide::Left");
    }
    else if(motorConfigVector[i].bodySide == BodySide::Right)
    {
      M_pos[i] = M_pos[i] * min(1, MODE_DIR*2);
      // Serial.printf(" BodySide::Right");
    }
    Serial.printf("\n");
  }

  M1_mapped = mapToMotorValue(M_pos[0]);
  M2_mapped = mapToMotorValue(M_pos[1]);
  M3_mapped = mapToMotorValue(M_pos[2]);
  motor_pos_array = {(uint8_t)M1_mapped, (uint8_t)M2_mapped, (uint8_t)M3_mapped};
}

void GaitMachine::setup(){
  Serial.println("GaitMachine started!");
  tick_start = steady_clock::now();
  last_call = tick_start;
}

GaitSelectionInfo GaitMachine::selectGait(BluetoothOutput btIn)
{
  // Serial.printf("arrowButtons: %i | leftJoystickUpDown: %f | rightJoystickUpDown: %f | rightJoystickLeftRight: %f | bluetoothHandler: %i\n", btIn.arrowButtons, btIn.leftJoystickUpDown, btIn.rightJoystickUpDown, btIn.rightJoystickLeftRight, btIn.buttons);
  if (btIn.arrowButtons != ArrowButtons::None)
  {
    // Serial.printf("arrowButtons: %i\n", static_cast<int>(btIn.arrowButtons));
    auto selectedGait = movementGaits[static_cast<int>(btIn.arrowButtons)-1];
    return {selectedGait.initialMotorPositions, selectedGait.gait, selectedGait.gait_time, selectedGait.gait_amp, directionCenter, 1};
  }

  if (btIn.buttons != TheFourButtonsOnTheFrontOfTheController::None)
  {
    auto selectedGait = movementGaits[static_cast<int>(btIn.buttons)-1];
    // Serial.printf("btIn.buttons: %i\n", static_cast<int>(btIn.buttons)-1);
    return {selectedGait.initialMotorPositions, selectedGait.gait, selectedGait.gait_time, btIn.leftJoystickUpDown*2, btIn.rightJoystickLeftRight, (1-btIn.rightJoystickUpDown)*2};
  }
  return {{0,0,0}, movementGaits[0].gait, 0, 0, 0.5, 0};
}

std::array<uint8_t, 3> GaitMachine::loop(BluetoothOutput btIn){
  auto [motor_init, gaitSelected, gait_time, gait_amp, direction, gait_time_modifier] = selectGait(btIn);
  // Serial.printf("gaitSelected: %f, gait_time: %f, gait_amp: %f, direction: %f, gait_time_modifier: %f\n", gaitSelected, gait_time, gait_amp, direction, gait_time_modifier);
  MOTOR_INIT = motor_init;
  GAIT_T = gait_time;
  GAIT_T_AMP = gait_time_modifier;
  GAIT_AMP = gait_amp;
  MODE_DIR = direction;

  if (gaitSelected.size() != 0)
  {

    auto now = steady_clock::now();
    
    if(tick_start == last_call){
      last_call = steady_clock::now();
      gaitControl(gaitSelected);
    }
    else{
      microseconds deltaT = duration_cast<microseconds>(now - last_call);
      if (deltaT >= tick_Time) 
      {
        gaitControl(gaitSelected, deltaT);
        last_call = now;
      }
    }
  }
  return motor_pos_array;
}