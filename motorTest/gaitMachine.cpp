#include "SerialUSB.h"
#include "gaitMachine.h"

using namespace gaits;
using namespace bluetoothHandler;

GaitStruct Test_Gait = {
  .initialMotorPositions = {0,0,0},
  .gait={
    {Motors::Tail, 1, 0, 0.5},
  },
  .gait_time=0.5,
  .gait_amp=1
};

GaitVector movementGaits = {Test_Gait};

MotorConfigVector motorConfigVector = {
  {BodySide::Left, ServoSide::Left},
  {BodySide::Right, ServoSide::Left},
  {BodySide::Center, ServoSide::Left}
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

  
  if(T_in_gait >= (GAIT_T + GAIT_T_AMP)){ 
    //Wrap around if the timing is exceeding the gait period
    T_in_gait -= (GAIT_T + GAIT_T_AMP);
  }

  prev_M_pos[0] = MOTOR_INIT[0];
  prev_M_pos[1] = MOTOR_INIT[1];
  prev_M_pos[2] = MOTOR_INIT[2];
  M_pos[0] = MOTOR_INIT[0];
  M_pos[1] = MOTOR_INIT[1];
  M_pos[2] = MOTOR_INIT[2];
  for(const auto& cmd : gait){
    float cmd_start_time = cmd.start * (GAIT_T + GAIT_T_AMP), cmd_end_time = (cmd.start + cmd.duration) * (GAIT_T + GAIT_T_AMP);
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
  // Serial.printf("arrowButtons: %i\n", static_cast<int>(btIn.arrowButtons));
  auto selectedGait = movementGaits[0];
  float gait_time_modifier = GAIT_T_AMP;
  if (btIn.arrowButtons == ArrowButtons::Forwards && lastArrowButton == ArrowButtons::None)
  {
    gait_time_modifier -= gait_t_amp_stepsize;
  }
  else if (btIn.arrowButtons == ArrowButtons::Backwards && lastArrowButton == ArrowButtons::None)
  {
    gait_time_modifier += gait_t_amp_stepsize;
  }
  if ((GAIT_T + gait_time_modifier) < 0)
  {
    gait_time_modifier = GAIT_T;
  }
  lastArrowButton = btIn.arrowButtons;
  Serial.printf("gait_time_modifier:%f",gait_time_modifier);
  return {selectedGait.initialMotorPositions, selectedGait.gait, selectedGait.gait_time, selectedGait.gait_amp, directionCenter, gait_time_modifier};
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