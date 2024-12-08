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

// Here we define the list of four gaits the joystick can change between
// In order they are FORWARDS, BACKWARDS, LEFT, RIGHT on the physical joystick
// As defined in bluetoothHandler.h: ArrowButtons{None, Forwards, Backwards, Left, Right};
GaitVector movementGaits = {Crawler, TailPush, BackJump, BackCrawl};

// Here we define the motor and limb configurations
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

// Here we map the interpolated motor values from the gait to actual servo values
int GaitMachine::mapToMotorValue(float M_pos){
  return (int)min(MOTOR_MAX_VAL,(M_pos * GAIT_AMP * MOTOR_MAX_VAL));
}

// Here we interpolate between motor commands to the the motor values for a gait over time
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

  // Store initial motor values
  prev_M_pos[0] = MOTOR_INIT[0];
  prev_M_pos[1] = MOTOR_INIT[1];
  prev_M_pos[2] = MOTOR_INIT[2];
  M_pos[0] = MOTOR_INIT[0];
  M_pos[1] = MOTOR_INIT[1];
  M_pos[2] = MOTOR_INIT[2];

  // Go each motor command in the gait
  for(const auto& cmd : gait){
    // Get the start and end times of the motor command
    float cmd_start_time = cmd.start * GAIT_T*GAIT_T_AMP, cmd_end_time = (cmd.start + cmd.duration) * GAIT_T*GAIT_T_AMP;
    // Check if we are currently outside these timepoints, and if so, save the motor command amount as the previous motor command
    // Since a gait is defined from start to end, this successfully saves the value of the previous motor command
    if (T_in_gait < cmd_start_time || T_in_gait >= cmd_end_time){
      prev_M_pos[cmd.motorID] = cmd.amount;
    }
    // If we are instead inside the motor command start and end times we should interpolate between the previous motor command and the current commands end value
    if(T_in_gait >= cmd_start_time && T_in_gait < cmd_end_time){
      // First we get the current time elapsed in the motor command
      float elapsed_time_in_cmd = (T_in_gait - cmd_start_time) / (cmd_end_time - cmd_start_time);
      // Then a linear interpolation is made to get what the motor value should be for the current time
      M_pos[cmd.motorID] = prev_M_pos[cmd.motorID] + (cmd.amount - prev_M_pos[cmd.motorID]) * elapsed_time_in_cmd;
    }
  }

  // The resulting interpolated values must be modified based on joystick input and to take the servo placement on the limb into consideration
  for (int i = 0; i < motorConfigVector.size(); i++)
  {
    // If a servo is placed on the right side, it will rotate from 100%->0% instead of the normal 0%->100%
    if (motorConfigVector[i].servoSide == ServoSide::Right)
    {
      M_pos[i] = 1 - M_pos[i];
      // Serial.printf("ServoSide::Right");
    }
    // The right joystick stick modifies servos placed on limbs on the left and right side of the body
    if (motorConfigVector[i].bodySide == BodySide::Left)
    {
      M_pos[i] = M_pos[i] * min(1, 2 - MODE_DIR*2); // If a limb is on the left side, the output value should be decreased as the stick is moved to the right
      // Serial.printf(" BodySide::Left");
    }
    else if(motorConfigVector[i].bodySide == BodySide::Right)
    {
      M_pos[i] = M_pos[i] * min(1, MODE_DIR*2); // If a limb is on the right side, the output value should be decreased as the stick is moved to the left
      // Serial.printf(" BodySide::Right");
    }
    Serial.printf("\n");
  }

  // Finally map the modified interpolated motor commands to the currect motor max and min values
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

// Here we select and modify the current gait
GaitSelectionInfo GaitMachine::selectGait(BluetoothOutput btIn)
{
  // Serial.printf("arrowButtons: %i | leftJoystickUpDown: %f | rightJoystickUpDown: %f | rightJoystickLeftRight: %f | bluetoothHandler: %i\n", btIn.arrowButtons, btIn.leftJoystickUpDown, btIn.rightJoystickUpDown, btIn.rightJoystickLeftRight, btIn.buttons);
  // If an arrow button on the joystick has been pressed, a gait should be selected, and its parameters set to its defined values from its GaitStruct
  if (btIn.arrowButtons != ArrowButtons::None)
  {
    // Serial.printf("arrowButtons: %i\n", static_cast<int>(btIn.arrowButtons));
    auto selectedGait = movementGaits[static_cast<int>(btIn.arrowButtons)-1];
    return {selectedGait.initialMotorPositions, selectedGait.gait, selectedGait.gait_time, selectedGait.gait_amp, directionCenter, 1};
  }

  // If on of the X, B, Y, A buttons has been pressed, we should select the gait as with the arrow buttons, but modify its parameters based on the stick inputs
  if (btIn.buttons != TheFourButtonsOnTheFrontOfTheController::None)
  {
    auto selectedGait = movementGaits[static_cast<int>(btIn.buttons)-1];
    // Serial.printf("btIn.buttons: %i\n", static_cast<int>(btIn.buttons)-1);
    // The left stick UP/DOWN controls the gait amplitude as a percentage of the MOTOR_MAX_VAL, from 0% (DOWN) to 200% (UP)
    // The right stick UP/DOWN controls the gait duration as a multiplier of the gaits standard duration, from 0% (DOWN) to 200% (UP)
    // The right stick LEFT/RIGHT controls the gait direction, gradually turning off the left limbs when turned to the RIGHT and the right limbs when turned to the LEFT 
    return {selectedGait.initialMotorPositions, selectedGait.gait, selectedGait.gait_time, btIn.leftJoystickUpDown*2, btIn.rightJoystickLeftRight, (1-btIn.rightJoystickUpDown)*2};
  }

  // If no button is pressed, the motors are set to 0%
  return {{0,0,0}, movementGaits[0].gait, 0, 0, 0.5, 0};
}

// This runs the actual gait machine code
std::array<uint8_t, 3> GaitMachine::loop(BluetoothOutput btIn){
  // First select and modify a gait
  auto [motor_init, gaitSelected, gait_time, gait_amp, direction, gait_time_modifier] = selectGait(btIn);
  // Serial.printf("gaitSelected: %f, gait_time: %f, gait_amp: %f, direction: %f, gait_time_modifier: %f\n", gaitSelected, gait_time, gait_amp, direction, gait_time_modifier);
  MOTOR_INIT = motor_init;
  GAIT_T = gait_time;
  GAIT_T_AMP = gait_time_modifier;
  GAIT_AMP = gait_amp;
  MODE_DIR = direction;

  // If a gait has been selected then run it
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

  // Output the resulting motor values
  return motor_pos_array;
}