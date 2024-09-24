/*
Project file for controlling the gait-algorithm of the Y-robot for Bio inspired robotics project

Using board [Arduino UNO R3]
With shield [TA0039 Sensor Expansion Shield V5.0] (https://makerhero.com/img/files/download/TA0039-Datasheet.pdf)
To control servo [Fitec FS90 9G Mini Servo] (https://www.addicore.com/products/feetech-fitec-fs90-9g-mini-servo-with-accessories)

Servo has cables:
Brown    = Ground
Red      = Power
Yellow   = Signal

Servo can have pins >= 2, since 0 and 1 are interfered with by Serial connection.

This file takes a gait description vector and creates a series of motor inputs for the arduino
*/

// #include <matplot/matplot.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>



//UART Connection to Arduino UNO
struct termios tty;
const char* portName = "/dev/ttyACM0";
int serialPort;

struct motorCMD{
  int motorID;
  float amount;
  float start;
  float duration;
};

// struct gaitStruct
// {
//   std::vector<motorCMD> commands;
//   float gaitTime;
//   float gaitAmplitude;
//   float motorMaxValue;
//   float tickFrequency;
// };

int8_t running_commands[3] = {-1,-1,-1};

enum Motors {Left=1, Right=2, Tail=3};
uint8_t program_end_motorpos[3] = {0,0,0};
float M_pos[3] = {0,0,0};
float prev_M_pos[3] = {0,0,0};

void exiting() {
  std::cout << "Exiting";
  write(serialPort, program_end_motorpos, sizeof(program_end_motorpos));   
}

// Goes backwards with frictionpads in direction of tail to two front legs
// On white table in the back of the lab, with friction pads with infill form, it moves towards the tail and slightly to the right
std::vector<motorCMD> Gait_1 = {motorCMD{Left,  0.9,  0.0,  0.3}, 
                                motorCMD{Right, 0.9,  0.0,  0.3},
                                motorCMD{Tail,  0.9,  0.3,  0.2},
                                motorCMD{Left,  0.5,  0.3,  0.2}, 
                                motorCMD{Right, 0.5,  0.3,  0.2},
                                motorCMD{Left,  0.0,  0.5,  0.3},
                                motorCMD{Right, 0.0,  0.5,  0.3},
                                motorCMD{Tail,  0.0,  0.8,  0.2}};

// Follows the connecting wires between arduino and servos. 
// Using GAIT_T 2, GAIT_AMP 1, and MOTOR_MAX_VAL 90 can move up ramp and on flat surface 
std::vector<motorCMD> Gait_2 = {{Left,  0.5,  0.0,  0.5},
                                {Right, 0.5,  0.0,  0.5},
                                {Tail,  0.5,  0.0,  0.5},
                                {Left,  0.0,  0.5,  0.5},
                                {Right, 0.0,  0.5,  0.5},
                                {Tail,  0.0,  0.7,  0.3}
                                };


// Jumps forward (from tail towards legs)
// Works well with GAIT_T 1, GAIT_AMP 1, MOTOR_MAX_VAL 90, tick_frq = 50
std::vector<motorCMD> Gait_3 = {{Tail,  0.9,  0.0,  0.1},
                                {Tail,  0.0,  0.4,  0.1},
                                };
std::vector<motorCMD> Gait_4 = {{Tail,  0.9,  0.0,  0.4},
                                {Tail,  0.0,  0.4,  0.1},
                                };


std::vector<motorCMD> Gait_5 = {{Left,  1.0,  0.0,  0.1},
                                {Right, 1.0,  0.0,  0.1},
                                {Left,  0.0,  0.1,  0.9},
                                {Right, 0.0,  0.1,  0.9},
                                };

// GAIT_T 2, GAIT_AMP 1, MOTOR_MAX_VAL 90, tick_frq = 100
std::vector<motorCMD> Gait_6 = {{Right, 1.0 , 0.0, 0.5},
                                {Left,  1.0, 0.0, 0.5},
                                {Tail, 1.0, 0.0, 0.5},
                                {Right, 0.0, 0.55, 0.2},
                                {Left, 0.0, 0.55, 0.2},
                                {Tail, 0.0, 0.55, 0.45}
                                };

std::vector<motorCMD> Gait_7 = {motorCMD{Left,  0.9,  0.0,  0.3}, 
                                motorCMD{Right, 0.9,  0.0,  0.3},
                                motorCMD{Tail,  0.9,  0.3,  0.2},
                                motorCMD{Left,  0.0,  0.5,  0.3},
                                motorCMD{Right, 0.0,  0.5,  0.3},
                                motorCMD{Tail,  0.0,  0.8,  0.2}};

std::vector<motorCMD> Jump = {
  {Tail, 1.0, 0, 0.001},
  {Tail, 0.0, 0.4, 0.1}
};

// Tested with:
// #define GAIT_T 0.5   //Duration of the gait's repeating pattern (seconds)
// #define GAIT_AMP 1  //Amplitude of the gait (0-1)
// #define MOTOR_MAX_VAL 90 //Max degree of servos allowed
// bool online = true;
// float tick_frq = 100; //Hz
std::vector<motorCMD> Worm = {
  {Tail,  0.68,  0,    0.2},
  {Left,  0.9,  0,    0.5},
  {Right, 0.9,  0,    0.5},
  {Tail,  0.2,  0.5,  0.2},
  {Left,  0,    0.8,  0.05},
  {Right, 0,    0.8,  0.05},
  {Tail,  0,    0.8,  0.2},
};
std::vector<motorCMD> WormR = {
  {Tail,  0.68,  0,    0.2},
  {Left,  0.5,  0,    0.5},
  {Right, 0.9,  0,    0.5},
  {Tail,  0.2,  0.5,  0.2},
  {Left,  0,    0.8,  0.05},
  {Right, 0,    0.8,  0.05},
  {Tail,  0,    0.8,  0.2},
};
std::vector<motorCMD> WormL = {
  {Tail,  0.68,  0,    0.2},
  {Left,  0.9,  0,    0.5},
  {Right, 0.5,  0,    0.5},
  {Tail,  0.2,  0.5,  0.2},
  {Left,  0,    0.8,  0.05},
  {Right, 0,    0.8,  0.05},
  {Tail,  0,    0.8,  0.2},
};

std::vector<motorCMD> Lesgo = {
  {Tail,  0.9,  0,    0.05},
  {Right,  1.0,  0,    0.05},
  {Left,  1.0,  0,    0.05},
  {Tail,  0,    0.05,  0.95},
};

std::vector<motorCMD> Crawler = {
  {Tail, 0.7, 0, 0.05},
  {Left, 0.9, 0.05, 0.35},
  {Tail, 0, 0.1, 0.3},
  {Left, 0, 0.45, 0.05},
  {Tail, 0.7, 0.5, 0.05},
  {Right, 0.9, 0.55, 0.35},
  {Tail, 0, 0.6, 0.3},
  {Right, 0, 0.95, 0.05}
};

#define SELECTED_GAIT Crawler

#define GAIT_T 2   //Duration of the gait's repeating pattern (seconds)
#define GAIT_AMP 1  //Amplitude of the gait (0-1)
#define MOTOR_MAX_VAL 90 //Max degree of servos allowed
bool online = true;
float tick_frq = 200; //Hz

void testWorkFunction(int max = 40){
  using namespace std::chrono;

  int rand_work = 2 + (std::rand() % ((max + 5) - 2 + 1));

  milliseconds work_time(rand_work);
  std::cout << "Work time: " << work_time.count() << std::endl;
  
  std::this_thread::sleep_for(work_time);
  return;
}

int mapToMotorValue(float M_pos){
  return (int)(M_pos * MOTOR_MAX_VAL);
}

std::vector<float> gaitControl(std::vector<motorCMD> gait, std::chrono::microseconds deltaT = std::chrono::microseconds(0)){
  static int M1_mapped = 0, M2_mapped = 0, M3_mapped = 0;
  static float T_in_gait = 0.0, T_elapsed = 0.0;

  //Incrementing the timing of the current gait
  float deltaT_sec = deltaT.count() * 0.000001;
  T_in_gait += deltaT_sec;
  T_elapsed += deltaT_sec;
  //std::cout << T_in_gait << std::endl;
  
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
      std::cout << "prev: " << prev_M_pos[cmd.motorID-1] << " cur: " << M_pos[cmd.motorID-1] << std::endl;
      prev_M_pos[cmd.motorID-1] = cmd.amount;
      std::cout << prev_M_pos[cmd.motorID-1] << std::endl;
    }
    if(T_in_gait >= cmd_start_time && T_in_gait < cmd_end_time){
      float elapsed_time_in_cmd = (T_in_gait - cmd_start_time) / (cmd_end_time - cmd_start_time);

      if (cmd.motorID > 3)
      {
        std::cerr << "Invalid motor ID" << std::endl;
      }
      
      M_pos[cmd.motorID-1] = prev_M_pos[cmd.motorID-1] + ((cmd.amount * GAIT_AMP) - prev_M_pos[cmd.motorID-1]) * elapsed_time_in_cmd;
      std::cout << "cmd_id: " << cmd.motorID << " gives: " << ((cmd.amount * GAIT_AMP) - prev_M_pos[cmd.motorID-1]) * elapsed_time_in_cmd << std::endl;

      std::cout << "["<< T_elapsed << "]: Motor " << cmd.motorID << " going to pos: " << cmd.amount * GAIT_AMP << " at time [" << T_in_gait << "]" << std::endl;
    }
  }

  M1_mapped = mapToMotorValue(M_pos[0]);
  M2_mapped = mapToMotorValue(M_pos[1]);
  M3_mapped = mapToMotorValue(M_pos[2]);

  uint8_t motor_pos_array[3] = {(uint8_t)M1_mapped, (uint8_t)M2_mapped, (uint8_t)M3_mapped};
  write(serialPort, motor_pos_array, sizeof(motor_pos_array));

  std::cout << "["<< T_elapsed << "]: Motor positions: [" << M_pos[0] << ", " << M_pos[1] << ", " << M_pos[2] << "]" << std::endl;

  return {T_elapsed, M_pos[0], M_pos[1], M_pos[2]};
}


int main(int argc, char* argv[]){

  if(online){
    //Test connection to arduino:
    serialPort = open(portName, O_RDWR | O_NOCTTY);
    if(serialPort == -1){
      std::cerr << "Error opening serial port" << std::endl;
      return 1;
    }

    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(serialPort, &tty) != 0){
      std::cerr << "Cannot get serial attributes!" << std::endl;
      return 1;
    }

    cfsetospeed(&tty, B115200); //Set BAUD-rate
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB; //No parity
    tty.c_cflag &= ~CSTOPB; //1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; //8 data bits
    tcsetattr(serialPort, TCSANOW, &tty);

    uint8_t test_array[3] = {0x0, 0x0, 0x0};
    write(serialPort, test_array, sizeof(test_array));

  }


  using namespace std::chrono;

  //The interval of which the motor commands are calculated (motor ticks):
  float time_ms = 1000/tick_frq;
  milliseconds motor_tick((int)time_ms);
  std::cout << "A motor tick is: " << (int)time_ms << " ms" << std::endl;

  std::vector<std::vector<float>> motor_positions; 
  bool run = true;

  while(run){
    //Motor_tick start time
    auto tick_start = steady_clock::now();
    static auto last_call = tick_start;

    //INSERT CODE TO CALCULATE MOTOR COMMANDS BELOW:
    //testWorkFunction((int)time_ms);
    std::vector<float> new_motor_positions;
    if(tick_start == last_call){
      last_call = steady_clock::now();
      motor_positions.push_back(gaitControl(SELECTED_GAIT));
      std::cout << "Goooooooooooooo" << std::endl;
    }
    else{
      auto now = steady_clock::now();
      microseconds deltaT = duration_cast<microseconds>(now - last_call);
      last_call = now;
      motor_positions.push_back(gaitControl(SELECTED_GAIT, deltaT));
    }

    auto work_complete = steady_clock::now();
    auto work_duration = duration_cast<milliseconds>(work_complete - tick_start);

    auto time_to_sleep = motor_tick - work_duration;
    if(time_to_sleep > milliseconds(0)){
      //Sleep remaining time of tick
      std::this_thread::sleep_for(time_to_sleep);
    }
    else{
      auto time_e = duration_cast<milliseconds>(time_to_sleep);
      std::cout << "Work is taking too long, timing issue of " << time_e.count() << " milliseconds" << std::endl;
    }
    auto tick_end = steady_clock::now();
    auto tick_duration = duration_cast<milliseconds>(tick_end - tick_start);
    //std::cout << "Tick timing: " << tick_duration.count() << std::endl;

    if(!motor_positions.empty()){
      /*
      if(motor_positions.back()[0] >= 25.0){
        run = false;
      }
      else{
        std::cout << "Time has not exceeded 25 seconds: " << motor_positions.back()[0] << std::endl;
      }
      */
    }
    else{
      std::cout << "motor_positions is empty!" << std::endl;
    }
  }


  
  // //Plotting the gait after 25 seconds:
  // std::vector<double> time, M1, M2, M3;
  // for(const auto& command_list : motor_positions){
  //   if(command_list.size() < 4){
  //     std::cerr << "Error: Each entry should have exactly 4 elements" << std::endl;
  //     return 1;
  //   }
  //   time.push_back(command_list[0]);
  //   M1.push_back(command_list[1]);
  //   M2.push_back(command_list[2]);
  //   M3.push_back(command_list[3]);
  // }
  // //Plotting:
  // using namespace matplot;
  // plot(time, M1, "r-")->line_width(2);
  // hold(on);
  // plot(time, M2, "g-")->line_width(2);
  // plot(time, M3, "b-")->line_width(2);
  // hold(off);
  // xlabel("Time");
  // ylabel("Motor pos");
  // legend({"Motor1", "Motor2", "Motor3"});
  // std::string title_str = "Gait Period: " + std::to_string(GAIT_T) + "s, Gait Amplitude Gain: " + std::to_string(GAIT_AMP);
  // title(title_str);
  // show();
  
  // std::atexit(exiting);
  close(serialPort);
  return 0;
}