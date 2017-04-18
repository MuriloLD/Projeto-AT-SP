#ifndef _ATSP_H_
#define _ATSP_H_

/*
*   ATSP Project library.
*/

#include <Arduino.h>
#define ENCODER_USE_INTERRUPTS // Precisa ser definido antes de Encoder.h
#include <Encoder.h> //Docs: https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize
#include <Navigator.h> //Docs: https://github.com/solderspot/NavBot/blob/master/NavBot_v1/docs/Navigator.md
#include <PID_v1.h>    //Documentação: https://github.com/br3ttb/Arduino-PID-Library  & http://playground.arduino.cc/Code/PIDLibrary
#include <Thread.h> //Documentação: https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h> //Documentação: https://github.com/ivanseidel/ArduinoThread

class ATSP {
public:
  ATSP(const int resistor_r1, const int resistor_r2);

  void run();

  /*****************************SETUP********************************/
  void setup(HardwareSerial &esp_serial, uint16_t baud);

  /****************************GETTERS*******************************/

  /****************************SETTERS*******************************/
  void set_motor_pins(const int right_motor_IN1, const int right_motor_IN2,
                      const int left_motor_IN3, const int left_motor_IN4);

  void change_state(int next_state) { _state = next_state; }

  //---------------------------------
private:
  /****************************OBJECTS*******************************/

  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder encoder_right;
  Encoder encoder_left;
  //   avoid using pins with LEDs attached

  double rPID_Setpoint, rPID_Input, rPID_Output, lPID_Setpoint, lPID_Input,
      lPID_Output;
  double Kp = 0.55, Ki = 0.35, Kd = 0.0;
  double pid_max_PWM = 150;

  // Rigth and Left PID object:
  PID PID_right;
  PID PID_left;

  // Navigator and aux vars:
  Navigator navigator;
  double lticks = 0, last_rticks = 0, rticks = 0, last_lticks = 0;
  double t_rticks, t_lticks; // Total ticks

  // Actual State saved here:
  enum State {
    WAITING, // stopAll
    RUNNING, // Automatic pilot
  };
  State _state = WAITING;

  /******************************************************************/

  /*************************CONFIGURATIONS***************************/

  double _CFG_MINBATERRY = 9.5;    // Minimum Battery Voltage
  double _CFG_MAXSPEED = 250.0;    // [mm/s]
  double _CFG_MAXTURNRATE = 150.0; // [theta/s]

  // Battery Monitoring
  int _CFG_RESISTOR_R1;
  int _CFG_RESISTOR_R2;

  // Navigator defines
  double _WHEEL_BASE = nvMM(191);
  double _WHEEL_DIAMETER = nvMM(60);
  double _TICKS_PER_REV = 3591.84;

  /*****************************PINS*********************************/

  // PINs motores:
  // May be inverted in case of wrong direction
  uint8_t _rightmotor_IN1 = 6;
  uint8_t _rightmotor_IN2 = 5;
  uint8_t _leftmotor_IN3 = 9;
  uint8_t _leftmotor_IN4 = 10;

  /**************************METHODS*********************************/

  void state_controller(); // States machine
  void odometry();         // Calculates position, velocity, orientation, etc
  void set_max_pwm();

  // Motors handlers:
  void drive_robot(double v, double w);
  void motor_handler(); // Update new PWM values
};

#endif