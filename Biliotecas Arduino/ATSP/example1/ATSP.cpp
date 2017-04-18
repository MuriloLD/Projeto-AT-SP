/*
*   ATSP Project library.
*/

#include "ATSP.h"

ATSP::ATSP(const int resistor_r1, const int resistor_r2)
    : encoder_right(20, 21), encoder_left(19, 18),
      PID_right(&rPID_Input, &rPID_Output, &rPID_Setpoint, Kp, Ki, Kd, DIRECT),
      PID_left(&lPID_Input, &lPID_Output, &lPID_Setpoint, Kp, Ki, Kd, DIRECT) {
  this->_CFG_RESISTOR_R1 = resistor_r1;
  this->_CFG_RESISTOR_R2 = resistor_r2;
}

void ATSP::set_motor_pins(const int right_motor_IN1, const int right_motor_IN2,
                          const int left_motor_IN1, const int left_motor_IN2) {
  this->_rightmotor_IN1 = 6;
  this->_rightmotor_IN2 = 5;
  this->_leftmotor_IN3 = 9;
  this->_leftmotor_IN4 = 10;
}

void ATSP::setup(HardwareSerial &esp_serial, uint16_t baud) {
  esp_serial.begin(baud);

  // Pinos I/O dos motores:
  pinMode(_rightmotor_IN1, OUTPUT);
  pinMode(_rightmotor_IN2, OUTPUT);
  pinMode(_leftmotor_IN3, OUTPUT);
  pinMode(_leftmotor_IN4, OUTPUT);
  //--------------------------------------

  // Inicia a Odometria e Navegação:
  // set up navigation
  navigator.InitEncoder(_WHEEL_DIAMETER, _WHEEL_BASE, _TICKS_PER_REV);
  navigator.Reset(millis());
  //-------------------------------------------------------------------

  // Inicialize PID:
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(100);

  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(100);
  //------------------------------------------------------------------
}

void ATSP::odometry() {

  // Current encoders counters:
  t_rticks = encoder_right.read();
  t_lticks = encoder_left.read();

  // Counter differencial:
  rticks = t_rticks - last_rticks;
  lticks = t_lticks - last_lticks;

  // Update Navigator parameters:
  navigator.UpdateTicks(lticks, rticks, millis());

  // Update last encoders counters:
  last_rticks = t_rticks;
  last_lticks = t_lticks;
  //------------------------------------------------

  // Update PID inputs:
  rPID_Input = navigator.RightSpeed();
  lPID_Input = navigator.LeftSpeed();
  //-------------------------------------------
}

void ATSP::drive_robot(double v, double w) {
  this->rPID_Setpoint = (v + (w * ATSP::_WHEEL_BASE / 2));
  this->lPID_Setpoint = (v - (w * ATSP::_WHEEL_BASE / 2));
}

void ATSP::motor_handler() {

  // Set motors new values:
  if ((rPID_Output >= 0) && (lPID_Output >= 0)) {
    analogWrite(_rightmotor_IN1, LOW);
    analogWrite(_rightmotor_IN2, rPID_Output);
    analogWrite(_leftmotor_IN3, lPID_Output);
    analogWrite(_leftmotor_IN4, LOW);
  } else if ((rPID_Output <= 0) && (lPID_Output >= 0)) {
    analogWrite(_rightmotor_IN1, abs(rPID_Output));
    analogWrite(_rightmotor_IN2, LOW);
    analogWrite(_leftmotor_IN3, lPID_Output);
    analogWrite(_leftmotor_IN4, LOW);
  } else if ((rPID_Output >= 0) && (lPID_Output <= 0)) {
    analogWrite(_rightmotor_IN1, LOW);
    analogWrite(_rightmotor_IN2, rPID_Output);
    analogWrite(_leftmotor_IN3, LOW);
    analogWrite(_leftmotor_IN4, abs(lPID_Output));
  } else if ((rPID_Output <= 0) && (lPID_Output <= 0)) {
    analogWrite(_rightmotor_IN1, abs(rPID_Output));
    analogWrite(_rightmotor_IN2, LOW);
    analogWrite(_leftmotor_IN3, LOW);
    analogWrite(_leftmotor_IN4, abs(lPID_Output));
  }
}

void ATSP::state_controller() {

  switch (_state) {
  case ATSP::WAITING:

    break;

  case ATSP::RUNNING:
    ATSP::motor_handler(); // Update new PWM values

    break;
  }
}

void ATSP::run() {
  state_controller(); // States machine
  odometry();         // Calculates position, velocity, orientation, etc
}