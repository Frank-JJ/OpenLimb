#include "SerialUSB.h"
#include "gaitMachine.h"

using namespace gaits;

GaitStruct Crawler = {
  .gait={
    {Tail, 0.7, 0, 0.05},
    {Left, 0.9, 0.05, 0.35},
    {Tail, 0, 0.1, 0.3},
    {Left, 0, 0.45, 0.05},
    {Tail, 0.7, 0.5, 0.05},
    {Right, 0.9, 0.55, 0.35},
    {Tail, 0, 0.6, 0.3},
    {Right, 0, 0.95, 0.05}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct Worm = {
  .gait={
    {Tail,  0.68,  0,    0.2},
    {Left,  0.9,  0,    0.5},
    {Right, 0.9,  0,    0.5},
    {Tail,  0.2,  0.5,  0.2},
    {Left,  0,    0.8,  0.05},
    {Right, 0,    0.8,  0.05},
    {Tail,  0,    0.8,  0.2}
  },
  .gait_time=1,
  .gait_amp=1
};

GaitStruct Jump = {
  .gait={
    {Tail, 1.0, 0, 0.001},
    {Tail, 0.0, 0.4, 0.1}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct WormR = {
  .gait={
    {Tail,  0.68,  0,    0.2},
    {Left,  0.5,  0,    0.5},
    {Right, 0.9,  0,    0.5},
    {Tail,  0.2,  0.5,  0.2},
    {Left,  0,    0.8,  0.05},
    {Right, 0,    0.8,  0.05},
    {Tail,  0,    0.8,  0.2}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct WormL = {
  .gait={
    {Tail,  0.68,  0,    0.2},
    {Left,  0.9,  0,    0.5},
    {Right, 0.5,  0,    0.5},
    {Tail,  0.2,  0.5,  0.2},
    {Left,  0,    0.8,  0.05},
    {Right, 0,    0.8,  0.05},
    {Tail,  0,    0.8,  0.2}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitStruct Lesgo = {
  .gait={
    {Tail,  0.9,  0,    0.05},
    {Right,  1.0,  0,    0.05},
    {Left,  1.0,  0,    0.05},
    {Tail,  0,    0.05,  0.95}
  },
  .gait_time=2,
  .gait_amp=0.5
};

GaitVector movementGaits = {Crawler, Worm, Jump, WormR, WormL, Lesgo};

GaitMachine::GaitMachine()
{

}

GaitMachine::~GaitMachine()
{

}

int GaitMachine::mapToMotorValue(float M_pos){
  return (int)(M_pos * GAIT_AMP * MOTOR_MAX_VAL);
}

void GaitMachine::gaitControl(Gait gait, std::chrono::microseconds deltaT){
  static int M1_mapped = 0, M2_mapped = 0, M3_mapped = 0;
  static float T_in_gait = 0.0, T_elapsed = 0.0;

  //Incrementing the timing of the current gait
  float deltaT_sec = deltaT.count() * 0.000001;
  T_in_gait += deltaT_sec;
  T_elapsed += deltaT_sec;

  
  if(T_in_gait >= GAIT_T){ 
    //Wrap around if the timing is exceeding the gait period
    T_in_gait -= GAIT_T;
  }

  prev_M_pos[0] = 0;
  prev_M_pos[1] = 0;
  prev_M_pos[2] = 0;
  for(const auto& cmd : gait){
    float cmd_start_time = cmd.start * GAIT_T, cmd_end_time = (cmd.start + cmd.duration) * GAIT_T;
    if (T_in_gait < cmd_start_time || T_in_gait >= cmd_end_time){
      prev_M_pos[cmd.motorID-1] = cmd.amount;
    }
    if(T_in_gait >= cmd_start_time && T_in_gait < cmd_end_time){
      float elapsed_time_in_cmd = (T_in_gait - cmd_start_time) / (cmd_end_time - cmd_start_time);
      
      M_pos[cmd.motorID-1] = prev_M_pos[cmd.motorID-1] + (cmd.amount - prev_M_pos[cmd.motorID-1]) * elapsed_time_in_cmd;
    }
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

GaitStruct GaitMachine::selectGait()
{
  using namespace bluetoothHandler;
  if (movement == direction::None)
    return {};
  else
    return movementGaits[movement-1]; // None is the 0'th value, so the size of movement is 6, which is 1 larger than movementGaits
}

std::array<uint8_t, 3> GaitMachine::loop(){
  auto [gaitSelected, gait_time, gait_amp, direction] = selectGait();
  GAIT_T = gait_time;
  GAIT_AMP = gait_amp;

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