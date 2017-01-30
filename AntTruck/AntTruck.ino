#include <Servo.h>
#include <PID_v1.h> //Documentação: https://github.com/br3ttb/Arduino-PID-Library  & http://playground.arduino.cc/Code/PIDLibrary
#include <Thread.h> //Documentação: https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h> //Documentação: https://github.com/ivanseidel/ArduinoThread 
#define ENCODER_USE_INTERRUPTS//Precisa ser definido antes de Encoder.h
#include <Encoder.h> //Documentação: https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize
#include <Navigator.h> //Documentação: https://github.com/solderspot/NavBot/blob/master/NavBot_v1/docs/Navigator.md
#include <PulseInZero.h>  //Documentação: https://gist.github.com/mikedotalmond/6044960
#include <PulseInOne.h> //Documentação: https://github.com/mikedotalmond/arduino-pulseInWithoutDelay
#include <math.h>
#include "Serialize.h"


//----------------------------------------------------------------------------//
//**************************** CONFIGURATIONS ********************************//
//----------------------------------------------------------------------------//
  #define CFG_MINBATERRY          9.5 // Minimum Battery Voltage
  #define CFG_MAXSPEED            250.0 // [mm/s]
  #define CFG_MAXTURNRATE         150.0 // [theta/s]
  #define CFG_FORCEFIELD_RADIUS   250 // Minimum US distance [mm]

  // Battery Monitoring
  #define CFG_RESISTOR_R1       18790 
  #define CFG_RESISTOR_R2       12830

  // Navigator defines
  #define WHEEL_BASE      nvMM(191)
  #define WHEEL_DIAMETER  nvMM(60)
  #define TICKS_PER_REV   3591.84

  //Final Position Controller Defines
  #define FPC_CONVERGENCERADIUS 30.0  // Outside circle radius in mm for FPController
  #define FPC_MAXRo   400.0 // Distance in mm when the FPC starts to decrease speed
  #define FPC_Kw      0.3 // Angular Velocity Equation Constant 


  //Debugs:
  #define DEBUG_SETPIDINTERVAL  0     // PWM max
  #define DEBUG_ESP_FEEDBACK    0     // Usage of "Serial.print" 

  //Testes:
  #define TST_MOTORS      0 // Nominal voltage test
  #define TST_COLLECTDATA   0 // Used to collect motors data
  #define TST_ESPsendData   0 // Test sending variables to ESP

  const int TENSAO_NOMINAL_MOTOR = 6; // Motors nominal power suply
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************** OBJECTS DECLARATION *****************************//
//----------------------------------------------------------------------------//
  /* Variables that had to be declared at the top of the code */

  /**** FPC vars *****/
  Navigator navigator;
  nvPose nav_Pose;
  double nav_pos_x, nav_pos_y, nav_heading;
  double nav_velLinear, nav_velAngular;
  double Ro, alpha, betha, gamma;  // angles for navigation kinematics
  /*******************/

  //Support vars for RUNNING state:
  double RUNNING_x, RUNNING_y;
  // TANGENTIAL ESCAPE
  double virtual_x, virtual_y;
  //------------------------------

  // US Distances:
    uint16_t dist_US_Frente=0, dist_US_Direito=0, dist_US_Esquerdo=0;
  //--------------------------------------------------------------

  //Threads:
  ThreadController TC_ESP_SERIAL;
  Thread T_ESP_SERIAL;
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************* PINS, INPUTS, OUTPUTS ****************************//
//----------------------------------------------------------------------------//
  //PINs motores: 
  // May be inverted in case of wrong direction 
  #define motorDireita_IN1  5
  #define motorDireita_IN2  6
  #define motorEsquerda_IN3 10
  #define motorEsquerda_IN4 9
  //---------------------------------

  //PINs ultrassom:
  #define US_Frente_TrigPin 39
  //----------------------------------------------
  #define US_Direito_TrigPin 49
  //----------------------------------------------
  #define US_Esquerdo_TrigPin 29
  //--------US Interrupt
  const int US_EchoPin = 2; //pino de interrupção dos US
  //----// Pins US Servo:
  #define US_servo_TrigPin 4
  #define pin_Servo 8
  const int US_servo_EchoPin = 3; //pino de interrupção dos US servo
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//*********************** PID/FINAL POSITION CONTROLLER ************************
//----------------------------------------------------------------------------//

  // PID Variables:
  double  rPID_Setpoint,rPID_Input,rPID_Output,
  lPID_Setpoint,lPID_Input,lPID_Output;
  double  Kp= 0.55, Ki = 0.006, Kd=0.0;
  // double  Kp= 0.2, Ki = 0.005, Kd=0.0; // Constants for FPC
  //---------------------------------------------//
  double pid_max_PWM = 100; 
  //---------------------------------------------//

  //Objetos PIDs:
  PID PID_right(&rPID_Input, &rPID_Output, &rPID_Setpoint, Kp, Ki, Kd, DIRECT);
  PID PID_left(&lPID_Input, &lPID_Output, &lPID_Setpoint, Kp, Ki, Kd, DIRECT);


  //Methods:
  void set_PID_Intervals(){ //Altera intervalo de saida PID:
    pid_max_PWM = (255*(TENSAO_NOMINAL_MOTOR + 0.7))/calcula_V_Bateria(0);
                                             //+0.7->queda de tensao diodos

    PID_right.SetOutputLimits(-pid_max_PWM, pid_max_PWM);
    PID_left.SetOutputLimits(-pid_max_PWM, pid_max_PWM);
    //Debug:
    #if DEBUG_SETPIDINTERVAL
    Serial.println("*****Debug set_PID_Intervals():");
    Serial.print("Novo limite: ");
    Serial.println(pid_max_PWM);
    Serial.println(" ");
    Serial.println(" ");
    #endif
  }

  /**  Speed Saturation Function **
   ** Returns a Speed Scale for the Speed saturator
   *  Replaces tanh(Ro) **/ 
   double FPC_Saturator(double Ro){
    if (Ro > FPC_MAXRo){
      return 1;
    }
    else{
      return (Ro/FPC_MAXRo);
    }
  }
  //------------------------------------------------------------

  //Final Position Controller (FPC):
  void FPController(double fpos_x, double fpos_y){ //need to be called in every 'RUNNING' loop
    //Vars
    double fv, fw;
    
    //Calc Ro and alpha:
    double delta_y = fpos_y - nav_pos_y;
    double delta_x = fpos_x - nav_pos_x;
    Ro = sqrt( delta_y*delta_y + delta_x*delta_x );
      //x-axe forward , y-axe to the left:
      alpha = ( atan2(delta_y,delta_x) - nav_heading );

    //Calc v and w:
      fv = CFG_MAXSPEED*FPC_Saturator(Ro)*cos(alpha); // CFG_MAXSPEED*tanh(Ro)*cos(alpha)
      fw = FPC_Kw*alpha + fv*sin(alpha)/Ro; // fw = Kw*alpha + CFG_MAXSPEED*(tanh(Ro)/Ro)*sin(alpha)*cos(alpha)
      
      //Update new setpoints:
      driveRobot(fv,fw);
  }

  // PIDs Calculations:
  void computePID(){
    PID_right.Compute();
    PID_left.Compute();
  }
  //-------------------------------------------------------------

  //Thread para alterar os intervalos:
  Thread T_set_PID_Intervals;

  //Thread Controller:
  ThreadController TC_BATERIA;
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------------//
//************************ TANGENCIAL ESCAPE CONTROLLER ************************
//----------------------------------------------------------------------------//
  // Calc virtual target for Tangecial Escape Controler
  void calcVirtualTarget(){
    int8_t sign;  

    //Look for minimum distance:
      if( (dist_US_Frente<dist_US_Direito) && (dist_US_Frente<dist_US_Esquerdo)){
        betha = 0;
        sign = 1;
      }
      else if( (dist_US_Direito<dist_US_Frente) && (dist_US_Direito<dist_US_Esquerdo) ){
        betha = -(M_PI/4);
        sign = -1;   
      }
      else if( (dist_US_Esquerdo<dist_US_Frente) && (dist_US_Esquerdo<dist_US_Direito) ){
        betha = M_PI/4;
        sign = 1;
      }

    
    //Calc rotation angle 
    // gamma = (sign*(abs(betha)-M_PI/2)) - alpha; //FRODO
    gamma = (sign*(M_PI/2)) - (betha - alpha); // FABRICIO

    //Coordinates transformation
    // ->  Robot referential
    double theta = nav_heading; // Readable equations
    double robot_Xd = (cos(theta)*RUNNING_x) + (sin(theta)*RUNNING_y) - (cos(theta)*nav_pos_x)-(sin(theta)*nav_pos_y);
    double robot_Yd = -(sin(theta)*RUNNING_x) + (cos(theta)*RUNNING_y) + (sin(theta)*nav_pos_x)-(cos(theta)*nav_pos_y);

    // Calc Robot Virtual Destination
    // Rotate by the angle 'gamma' calculated:
    double robot_VirX = (cos(gamma)*robot_Xd) + (sin(gamma)*robot_Yd);
    double robot_VirY = -(sin(gamma)*robot_Xd) + (cos(gamma)*robot_Yd);

    //Calc Virtual Target:
    // Robot Reference -> World Reference
    virtual_x = cos(theta)*robot_VirX - sin(theta)*robot_VirY + nav_pos_x;
    virtual_y = sin(theta)*robot_VirX + cos(theta)*robot_VirY + nav_pos_y;
  }


////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************ QUADRATURE ENCODER  *******************************//
//----------------------------------------------------------------------------//
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder enc_Direita(20, 21);
  Encoder enc_Esquerda(19, 18);
  //   avoid using pins with LEDs attached

  ///////Variáveis de posição:
  double encDIR_Position  = 0, encDIR_lastPosition = 0, encDIR_lastTIME=0;
  double encESQ_Position  = 0, encESQ_lastPosition = 0, encESQ_lastTIME=0;
  /////////////////////////////////////////////////////////////////////////

  double calcVel_Direita(bool _debug){
    double _vel, tempoAtual;
    tempoAtual = millis();

  encDIR_Position = enc_Direita.read(); //leitura posição atual

  _vel = ((encDIR_Position - encDIR_lastPosition)/(tempoAtual - encDIR_lastTIME) )* 52.491105; // v = deltaS/deltaT
  encDIR_lastTIME = tempoAtual;
  encDIR_lastPosition = encDIR_Position;

  if(_debug){
    Serial.println("**** Debug Calculo Velocidade Direito *****");
    Serial.print("Tempo entre uma leitura e outra:  ");
    Serial.println(tempoAtual - encDIR_lastTIME);
    Serial.print("encDIR_lastPosition:  ");
    Serial.println(encDIR_lastPosition);
    Serial.print("Velocidade Direito:  ");
    Serial.println(_vel);
    Serial.println("*********************************************");
    Serial.println(" ");
  }

  return _vel;
  }

  double calcVel_Esquerda(bool _debug){
    double _vel, tempoAtual;
    tempoAtual = millis();

  encESQ_Position = enc_Esquerda.read(); //leitura posição atual

  _vel = ((encESQ_Position - encESQ_lastPosition)/(tempoAtual - encESQ_lastTIME) )* 52.491105; // v = deltaS/deltaT
  encESQ_lastTIME = tempoAtual;
  encESQ_lastPosition = encESQ_Position;

  if(_debug){
    Serial.println("**** Debug Calculo Velocidade Esquerdo *****");
    Serial.print("Tempo entre uma leitura e outra:  ");
    Serial.println(tempoAtual - encESQ_lastTIME);
    Serial.print("encESQ_lastPosition:  ");
    Serial.println(encESQ_lastPosition);
    Serial.print("Velocidade Esquerdo:  ");
    Serial.println(_vel);
    Serial.println("*********************************************");
    Serial.println(" ");
  }

  return _vel;
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
************************************ ODOMETRY **********************************
*******************************************************************************/

  double lticks=0, last_rticks=0, rticks=0, last_lticks=0;
  double t_rticks, t_lticks; //Total ticks


  void odometria(bool debug){
    // Current encoders counters:
    t_rticks = enc_Direita.read();
    t_lticks = enc_Esquerda.read();

    //Counter differencial:
    rticks = t_rticks - last_rticks;
    lticks = t_lticks - last_lticks;

    // Update Navigator parameters:
    navigator.UpdateTicks( lticks, rticks, millis() );

    // Update last encoders counters:
    last_rticks = t_rticks;
    last_lticks = t_lticks;
    //------------------------------------------------

    //Update PID inputs:
    rPID_Input = navigator.RightSpeed();
    lPID_Input = navigator.LeftSpeed();
    //-------------------------------------------------

    // Update Atual Position for the FPC: 
    nav_Pose = navigator.Pose();
    nav_pos_x = nav_Pose.position.x;
    nav_pos_y = nav_Pose.position.y;
    nav_heading = nav_Pose.heading;
    nav_velLinear = navigator.Speed(); //Debug only
    nav_velAngular = navigator.TurnRate(); //Debug only
    //-------------------------------------------

    if(debug){
      Serial.print("enc_Direita.read(): " );
      Serial.print(enc_Direita.read());
      Serial.print("\trticks: ");
      Serial.print(rticks);
      Serial.print("\tenc_Esquerda.read(): " );
      Serial.print(enc_Esquerda.read());
      Serial.print("\tlticks: " );
      Serial.println(lticks);
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
******************************** MOVEMENT FUNCTIONS ****************************
*******************************************************************************/

  void driveRobot(double v, double w){ 
    //Saturation:
    // v = (v>CFG_MAXSPEED) ? CFG_MAXSPEED : v;
    // w = (w>CFG_MAXTURNRATE) ? CFG_MAXTURNRATE : w;

    //Inverse calculation of RifhtSpeed and LeftSpeed:
    rPID_Setpoint = (v + (w*WHEEL_BASE/2));
    lPID_Setpoint = (v - (w*WHEEL_BASE/2));
  }

  void motorHandler(){ //Must be called constantly

    //Set motors new values:
    if( (rPID_Output>=0) && (lPID_Output>=0) ){
      analogWrite(motorDireita_IN1, LOW);
      analogWrite(motorDireita_IN2, rPID_Output);
      analogWrite(motorEsquerda_IN3, lPID_Output);
      analogWrite(motorEsquerda_IN4, LOW);
    }
    else if( (rPID_Output<=0) && (lPID_Output>=0) ){
      analogWrite(motorDireita_IN1, abs(rPID_Output));
      analogWrite(motorDireita_IN2, LOW);
      analogWrite(motorEsquerda_IN3, lPID_Output);
      analogWrite(motorEsquerda_IN4, LOW);
    }
    else if( (rPID_Output>=0) && (lPID_Output<=0) ){
      analogWrite(motorDireita_IN1, LOW);
      analogWrite(motorDireita_IN2, rPID_Output);
      analogWrite(motorEsquerda_IN3, LOW);
      analogWrite(motorEsquerda_IN4, abs(lPID_Output));
    }
    else if( (rPID_Output<=0) && (lPID_Output<=0) ){
      analogWrite(motorDireita_IN1, abs(rPID_Output));
      analogWrite(motorDireita_IN2, LOW);
      analogWrite(motorEsquerda_IN3, LOW);
      analogWrite(motorEsquerda_IN4, abs(lPID_Output));
    }
  }

  void stopAll(){  
    analogWrite(motorDireita_IN1, LOW);
    analogWrite(motorDireita_IN2, LOW);
    analogWrite(motorEsquerda_IN3, LOW);
    analogWrite(motorEsquerda_IN4, LOW);    
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
*************************** ULTRASOUNDS VARS AND FUNCTIONS *********************
*******************************************************************************/

  //Constantes de Cálculos:
  const float SpeedOfSound      = 343.2; // ~speed of sound (m/s) in air, at 20°C         
  const float MicrosecondsPerMillimetre   = 1000.0 / SpeedOfSound; // microseconds per millimetre - sound travels 1 mm in ~2.9us
  const float  MicrosecondsToMillimetres  = (1.0 / MicrosecondsPerMillimetre);
  const float  MicrosecondsToMillimetres2 = MicrosecondsToMillimetres / 2.0; // beam travels the distance twice... so halve the time.
  //--------------------------------------------------------------------------

  //Variáveis de controle:
  uint8_t _swch = 1;
  boolean _done = true;
  //----------------------------------------------------------------------------

  //Threads:
  Thread T_USinterrupt_Read;
  //-------------------
  ThreadController TC_US;
  //-------------------------

  void us_Read() {
    if (_done){
      uint8_t trig_pin;

      switch(_swch){
        case 1:
        trig_pin = US_Frente_TrigPin;
        break;
        case 2:
        trig_pin = US_Direito_TrigPin;
        break;
        case 3:
        trig_pin = US_Esquerdo_TrigPin;
        break;
        default:
        _swch=1;
        break;
      }

      //seta o pino TRIG com pulso alto "HIGH" 
      digitalWrite(trig_pin, HIGH);  
      //delay de 10 microssegundos  
      delayMicroseconds(11);  
      //seta o pino TRIG com pulso baixo novamente  
      digitalWrite(trig_pin, LOW);

      //start listening out for the echo pulse on interrupt 0 (pino 2):
      PulseInZero::begin();

      _done = false;
    }
  }

  void us_PulseComplete(unsigned long duration){
    switch(_swch){
      case 1:
      dist_US_Frente = duration * MicrosecondsToMillimetres2;
      _swch = 2;
      break;
      case 2:
      dist_US_Direito = duration * MicrosecondsToMillimetres2;
      _swch = 3;
      break;
      case 3:
      dist_US_Esquerdo = duration * MicrosecondsToMillimetres2;
      _swch = 1;
      break;
    }
    _done = true;
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
********************************* SERVO ULTRASOUND *****************************
*******************************************************************************/
  //Servo
  Servo servoUS;

  //Threads:
  Thread T_USi_Servo_Read;

  //Variaveis:
  uint16_t dist_US_servo=0;
  //-----------------------------------------------------------

  // Funções de leitura do US:

  //Read:
  void us_servoRead() {
  //seta o pino TRIG com pulso alto "HIGH" 
  digitalWrite(US_servo_TrigPin, HIGH);  
  //delay de 10 microssegundos  
  delayMicroseconds(11);  
  //seta o pino TRIG com pulso baixo novamente  
  digitalWrite(US_servo_TrigPin, LOW);

  //start listening out for the echo pulse on interrupt 1 (pino 3):
  PulseInOne::begin();
  }
  //----------------------------------------------

  //Gatilho da interrupção:
  void us_servo_PulseComplete(unsigned long duration){
    dist_US_servo = MicrosecondsToMillimetres2 * duration;
  }
  //

  // Funções de Movimento - Servo:
  void servo_varrer(){
    byte i;

    if (i < 90){
      for(i; i>=180; i++){
        servoUS.write(i);
        delay(50);
      }
    }
    else{      
      for(i; i<=0; i--){
        servoUS.write(i);
        delay(50);
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
********************************* BATTERY MONITORING ***************************
*******************************************************************************/

  //Valor dos resistores (Divisor de tensão):
  const int resistor_R1 = CFG_RESISTOR_R1, resistor_R2 = CFG_RESISTOR_R2; 
  const int pin_AnalogDivisor = A0;
  /////////////////////////////////////////////

  //Encontrar a tensão da bateria:
  double calcula_V_Bateria(bool _debug){
    double tensao_Vo[10], tensao_Vi[10], tensao_Vin=0;

    for(int i=0; i<10; i++){
      tensao_Vo[i] = (analogRead(pin_AnalogDivisor)*5.00)/1023.00;
    tensao_Vi[i] = (tensao_Vo[i]*(resistor_R1+resistor_R2))/resistor_R2; //divisor de tensão.
    tensao_Vin += tensao_Vi[i];
    }

    tensao_Vin = (tensao_Vin / 10.0);

    //Debug:
    if(_debug){
      Serial.println("*****Debug calcula_V_Bateria():");
      Serial.println(">>tensao_Vo: ");
      for(int i=0; i<10; i++){
        Serial.println(tensao_Vo[i]);
      }
      Serial.println(">>tensao_Vi: ");
      for(int i=0; i<10; i++){
        Serial.println(tensao_Vi[i]);
      }
      Serial.print(">>tensao_Vin: ");
      Serial.println(tensao_Vin);
      Serial.println(" ");
      Serial.println(" ");  
    }

    return tensao_Vin - 0.15; //(0.15 margem de erro)
  }

  Thread T_alertaBateria;
  //Tensão abaixo do limite minimo:
  boolean warn=false;
  void alertaBateria(){
    if( calcula_V_Bateria(0) > CFG_MINBATERRY ){
      warn=false;
      }else{
        warn=true;
      }
  //Desabilita Robô caso tensão baixa:
    while (warn){ //loop infinito
      changeState("stopAll");
      stopAll();
      
      servoUS.write(90);
      delay(200);
      servoUS.write(180);
      delay(800);
      servoUS.write(0);
      delay(800);
      servoUS.write(90);

      Serial.println("=======> BATERIA FRACA!! <=========");
      delay(3000);
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
******************************** STATES CONTROLLER *****************************
*******************************************************************************/
  #if TST_COLLECTDATA          //
  uint16_t tm[501], rt[501], lt[501]; // Avoid extra memory usage
  long last, time;       // Only compile if needed
  #endif        //

  //Support vars for DRIVING state:
  uint32_t DRIVING_now;
  int DRIVING_time;
  double DRIVING_v, DRIVING_w;
  //------------------------------


  // Actual State saved here:
  enum State
  {
    INIT,
    COLLECTING_DATA, //Testing motors only 
    RUNNING, // Automatic pilot
    DRIVING, // Manual with time (by ESP/MQTT commands)
    ARROWS, //Using arrows (Keyboard)
    WAITING, //stopAll
    US_OBSTACLE  // Avoid obstacles 
  };
  State state       = INIT;
  //--------------------------------------------------------

  //Receive command from ESP-Serial and change actual state:
  void changeState(String cmd){
    Serial.print(" Command-> ");
    Serial.println(cmd);

    if (cmd.startsWith("driveRobot")){ //DRIVING STATE
      //Parenthesis Indexes 
      int p1 = cmd.indexOf('('); //Parenthesis position
      int comma = cmd.indexOf(','); //Separate linear and angular velocities 
      int colon = cmd.indexOf('/'); //Separate time
      int p2 = cmd.indexOf(')'); //Second Parenthesis position
      //Take the arguments from the cmd string:
      String vlinear,vAngular,time;
      vlinear = cmd.substring(p1+1,comma);
      vAngular = cmd.substring(comma+1,colon);
      time = cmd.substring(colon+1,p2);
      DRIVING_time = (time == "0") ? -1 : 1000*time.toInt(); //IF time==0->DRIVING_time=INFINITY
      //Store Linear and Angular velocities:
      DRIVING_v = (double) vlinear.toInt();
      DRIVING_w = (double) vAngular.toInt();
      //Set new parameters:
      driveRobot(DRIVING_v,nvDegToRad(DRIVING_w));
      DRIVING_now = millis();
      //Change State:
      state = DRIVING; 
      //debug:
      Serial.print("DRIVING_v: ");
      Serial.print(DRIVING_v);
      Serial.print("\tDRIVING_w: ");
      Serial.print(DRIVING_w);
      Serial.print("\tDRIVING_time: ");
      Serial.println(DRIVING_time);
      //------------------------------------
    }else if (cmd.startsWith("goTo")){ //RUNNING STATE
      //Parenthesis Indexes 
      int p1 = cmd.indexOf('('); //Parenthesis position
      int comma = cmd.indexOf(','); //Separate linear and angular velocities 
      int p2 = cmd.indexOf(')'); //Second Parenthesis position
      //Take the arguments from the cmd string:
      String _x, _y;
      _x = cmd.substring(p1+1,comma);
      _y = cmd.substring(comma+1,p2);
      //Stores x,y position:
      RUNNING_x = (double) _x.toInt();
      RUNNING_y = (double) _y.toInt();
      //Change State:
      state = RUNNING; 
      //debug:
      Serial.print("RUNNING_x: ");
      Serial.print(RUNNING_x);
      Serial.print("\tRUNNING_y: ");
      Serial.println(RUNNING_y);
      //--------------------------------------------------
    }else if(cmd == "obstacleDetected"){ //US_OBSTACLE STATE
      state = US_OBSTACLE;      
    }else if(cmd == "goUp"){
      //goUp
      driveRobot(230.0,0);
      state = ARROWS;
    }else if(cmd == "goDown"){
      //goDown
      driveRobot(-230.0,0.0);
      state = ARROWS;
    }else if(cmd == "goRight"){
      //goRight
      driveRobot(0.0,nvDegToRad(-90));
      state = ARROWS;
    }else if(cmd == "goLeft"){
      //goLeft
      driveRobot(0.0,nvDegToRad(90));
      state = ARROWS;
    }else if (cmd == "stopAll"){
      stopAll();
      state = WAITING;
    }else if(cmd == "sendData"){
        T_ESP_SERIAL.enabled = true;
        Serial.print("T_ESP_SERIAL.enabled: ");
        Serial.println(T_ESP_SERIAL.enabled);
    }else if(cmd == "stopData"){
        T_ESP_SERIAL.enabled = false;
        Serial.print("T_ESP_SERIAL.enabled: ");
        Serial.println(T_ESP_SERIAL.enabled);
    }else if(cmd.startsWith("goTo")){
      int i1 = cmd.indexOf('('); //Parentesis position
    }else{
        Serial.println("Invalid State!");
    }
  }
  //------------------------------------------------------

  // Robot States control
  void stateController(){
    switch(state){
      case INIT:
      { 
        #if TST_MOTORS
        analogWrite(motorDireita_IN1, LOW);
        analogWrite(motorDireita_IN2, pid_max_PWM);
        analogWrite(motorEsquerda_IN3, pid_max_PWM);
        analogWrite(motorEsquerda_IN4, LOW);

        #elif TST_COLLECTDATA  //Only compile if needed
        Serial.println("ENTERING COLLECT DATA!");
        delay(1000);
        last = time = millis();

        analogWrite(motorDireita_IN1, LOW);
        analogWrite(motorDireita_IN2, pid_max_PWM);
        analogWrite(motorEsquerda_IN3, pid_max_PWM);
        analogWrite(motorEsquerda_IN4, LOW);

        state = COLLECTING_DATA;
        #endif
        state = WAITING;
        break;
      }

      case COLLECTING_DATA:
      { 
        #if TST_COLLECTDATA //Only compile if needed
        int i=1;

        while ( i<501 ){
          time = millis();
          if ( (time - last) >= 5 ){
            tm[i] = time;
            rt[i] = enc_Direita.read();
            lt[i] = enc_Esquerda.read();
            last = time;
            i++;
          }
          if ( i>400 ){
            analogWrite(motorDireita_IN2, LOW);
            analogWrite(motorDireita_IN1, LOW);
            analogWrite(motorEsquerda_IN3, LOW);
            analogWrite(motorEsquerda_IN4, LOW);

          }
        }
        Serial.println("TEMPO!");

        for (int i=1;i<501;++i){
          Serial.print(tm[i]);
          Serial.print(", ");
          Serial.print(rt[i]);
          Serial.print(", ");
          Serial.println(lt[i]);
        }
        Serial.println("DONE!");
        #endif

        state = WAITING;
        break;
      }

      case RUNNING :
      {

          if( (dist_US_Direito<CFG_FORCEFIELD_RADIUS) ||
              (dist_US_Esquerdo<CFG_FORCEFIELD_RADIUS) ||
              (dist_US_Frente<CFG_FORCEFIELD_RADIUS)       ){ //In case of obstacle inside the virtual force field:
            
            calcVirtualTarget();
            state = US_OBSTACLE; 
          }
          // else if(dist_US_servo<CFG_FORCEFIELD_RADIUS){ // Obstacle found behind the robot
          //     stopAll(); //################################################################################################################################################################################################################################################################
          // } 
          else {

            FPController(RUNNING_x,RUNNING_y);
            if ( Ro < nvMM(FPC_CONVERGENCERADIUS) ){
              state = WAITING;  
            }
          }
            motorHandler(); //Update new PWM values

        break;

      }

      case DRIVING:
      {
        if (millis() - DRIVING_now >= DRIVING_time){
         stopAll();
         state = WAITING;
         }else{
           motorHandler();
         }
        //Debug:
        //  Serial.print("millis() - DRIVING_now: ");
        //  Serial.println((millis() - DRIVING_now));
        //----------------------------------------
        break;
      }
      case ARROWS:
      {
        motorHandler();

        break;
      }

      case US_OBSTACLE:
      {

        FPController(virtual_x,virtual_y);
        motorHandler();

        if(dist_US_Frente>CFG_FORCEFIELD_RADIUS &&
            dist_US_Esquerdo>CFG_FORCEFIELD_RADIUS &&
            dist_US_Direito>CFG_FORCEFIELD_RADIUS){
              
              state = RUNNING;
        }
        break;
      }

      case WAITING :
      {
        //Do nothing ?
        stopAll();

        //Reset PID Parameters:
        rPID_Setpoint = 0;
        rPID_Output = 0;
        lPID_Setpoint = 0;
        lPID_Output = 0;  
        break;
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
***************************** ESP8266 VARS AND FUNCTIONS ***********************
*******************************************************************************/

  // Serializer: Split variables in bytes:
  Serializer se;
  //Receive command from ESP:
  String command = ""; // Received from Serial3 (ESP8266)
  String str_Monitor = ""; // Received from Serial (Typing)
  bool cmd = false;

  void ESP8266_feedback(){
    if (Serial3.available()) {
      char inB1 = Serial3.read(); // Read incoming char
      //-------------------------------------------------------
        #if DEBUG_ESP_FEEDBACK
          Serial.print(inB1); //Debug
        #endif
      //-------------------------------------------------------
      //Receive command from Serial3/ESP
      if (inB1=='#') {
        cmd = true;
      }
      else if (inB1==';'){
        cmd = false;
        changeState(command);
        command = "";
      } else if(cmd){
        command += inB1;
      }
    }
  }

  // Size of vars in sequence:
  char key[]={'[','4','4','4','4','4','4','4','4','4','4','2','2','2','2','4','4',']'}; 
  char bgn_Str = '>';
  char end_Str = '!';
  // >>[4444444444222244]12341234123412341234123412341234123412341212121244444444!! //78 bytes

  void sendToESP(){ //----------------> WATCH OUT: Serializer doesn't support Double/Float
    #if TST_ESPsendData

      //Test vars:
        int32_t var1 = 111,
                var2 = -222,
                var3 = 333,
                var4 = -444,
                var5 = 555,
                var6 = -666,
                var7 = 777,
                var8 = -888,
                var9 = 999,
                var10 = 1010,
                var15 = 1515,
                var16 = 1616;
        uint16_t  var11 = 11011,
                var12 = 12012,
                var13 = 13013,
                var14 = 14014;
      //-------------------------------


      //Add begin marker (2):
      se.add(&bgn_Str, sizeof(bgn_Str));
      se.add(&bgn_Str, sizeof(bgn_Str));

      for (int i=0; i < sizeof(key); ++i){
        se.add(&key[i],sizeof(key[i])); //==> Always check vars sizes!
      }
      se.add(&var1,sizeof(var1));
      se.add(&var2,sizeof(var2));
      se.add(&var3,sizeof(var3));
      se.add(&var4,sizeof(var4));
      se.add(&var5,sizeof(var5));
      se.add(&var6,sizeof(var6));
      se.add(&var7,sizeof(var7));
      se.add(&var8,sizeof(var8));
      se.add(&var9,sizeof(var9));
      se.add(&var10,sizeof(var10));
      se.add(&var11,sizeof(var11));
      se.add(&var12,sizeof(var12));
      se.add(&var13,sizeof(var13));
      se.add(&var14,sizeof(var14));
      se.add(&var14,sizeof(var15));
      se.add(&var14,sizeof(var16));
      
      //Add final marker (2):
      se.add( &end_Str, sizeof(end_Str));
      se.add( &end_Str, sizeof(end_Str));
      
      for (int i = 0; i < se.getCount(); ++i)
      {
        Serial3.write( se.getBufferIndex(i) ); 
        // Serial.write( se.getBufferIndex(i) ); 
        Serial3.flush();
      }
      Serial3.flush();

      //-------DEBUG
        // Serial.print('\n');
        // Serial.print("\tse.getCount(): ");
        // Serial.print(se.getCount());
        // Serial.print('\n');
      //------------------------------

      se.clean();
      
    #else

      //Add begin marker (2):
      se.add(&bgn_Str, sizeof(bgn_Str));
      se.add(&bgn_Str, sizeof(bgn_Str));

      //Add key:
      for (int i=0; i < sizeof(key); ++i){
        se.add(&key[i],sizeof(key[i])); //==> Always check vars sizes!
      }
      //Add vars:
        int32_t i_nav_pos_x = (int32_t) nav_pos_x; //casting
      se.add(&i_nav_pos_x,sizeof(i_nav_pos_x)); //double -> int32_t

        int32_t i_nav_pos_y = (int32_t) nav_pos_y; //casting
      se.add(&i_nav_pos_y ,sizeof(i_nav_pos_y)); //double -> int32_t

        int32_t i_nav_velLinear = (int32_t) nav_velLinear; //casting
      se.add(&i_nav_velLinear ,sizeof(i_nav_velLinear)); //double -> int32_t

        int32_t i_nav_velAngular = (int32_t) nav_velAngular; //casting
      se.add(&i_nav_velAngular ,sizeof(i_nav_velAngular)); //double -> int32_t

        int32_t i_rPID_Setpoint = (int32_t) rPID_Setpoint; //casting
      se.add(&i_rPID_Setpoint ,sizeof(i_rPID_Setpoint)); //double -> int32_t

        int32_t i_lPID_Setpoint = (int32_t) lPID_Setpoint; //casting
      se.add(&i_lPID_Setpoint ,sizeof(i_lPID_Setpoint)); //double -> int32_t

        int32_t i_rPID_Input = (int32_t) rPID_Input;  //casting
      se.add(&i_rPID_Input ,sizeof(i_rPID_Input)); //double -> int32_t

        int32_t i_lPID_Input = (int32_t) lPID_Input; //casting
      se.add(&i_lPID_Input ,sizeof(i_lPID_Input)); //double -> int32_t

        int32_t i_rPID_Output = (int32_t) rPID_Output; //casting
      se.add(&i_rPID_Output ,sizeof(i_rPID_Output)); //double -> int32_t

        int32_t i_lPID_Output = (int32_t) lPID_Output; //casting
      se.add(&i_lPID_Output ,sizeof(i_lPID_Output)); //double -> int32_t

      se.add(&(dist_US_Frente) ,sizeof(dist_US_Frente)); //uint16_t
      se.add(&(dist_US_Direito) ,sizeof(dist_US_Direito)); //uint16_t
      se.add(&(dist_US_Esquerdo) ,sizeof(dist_US_Esquerdo)); //uint16_t
      se.add(&(dist_US_servo) ,sizeof(dist_US_servo)); //uint16_t

        int32_t i_vBateria = (int32_t) (calcula_V_Bateria(0)*10);
      se.add(&i_vBateria, sizeof(i_vBateria));

        int32_t i_nav_heading = (int32_t) nvRadToDeg(nav_heading);
      se.add(&i_nav_heading, sizeof(i_nav_heading));
      //End data

      //Add final marker (2):
      se.add( &end_Str, sizeof(end_Str));
      se.add( &end_Str, sizeof(end_Str));

      //Send to ESP by Serial3
      String str = "";
        for (int i = 0; i < se.getCount(); ++i){
          Serial3.write( se.getBufferIndex(i) );     //
          // str = str + se.getBufferIndex(i);      // Qual é melhor?
        Serial3.flush();
        }
      Serial3.flush();
      // Serial.print(str);
        //-------DEBUG
          // Serial.print('\n');
          // Serial.print("--->STR: ");
          // Serial.print(str);
          // Serial.print("\tse.getCount(): ");
          // Serial.print(se.getCount());
          // Serial.print('\n');
        //------------------------------
      se.clean();
      // str = "";
    #endif
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
****************************** DEBUG FUNCTIONS  ********************************
*******************************************************************************/

  void debug_Ultrassom(){
    Serial.print("US Frente (mm): ");
    Serial.print(dist_US_Frente);
    Serial.print("\t US Direito (mm): ");
    Serial.print(dist_US_Direito);
    Serial.print("\tUS Esquerdo (mm): ");
    Serial.print(dist_US_Esquerdo);
    Serial.print("\tUS Servo (mm): ");
    Serial.println(dist_US_servo);
  }
  //----------------------------------------
  //Debug Quad Encoder
  void debug_Encoder(){
    encDIR_Position = enc_Direita.read();
    encESQ_Position = enc_Esquerda.read();

    if((encDIR_Position != encDIR_lastPosition) || (encESQ_Position != encESQ_lastPosition) ){
      Serial.print(">>> DEBUG ENCODER >>> ");
      Serial.print("Contador Direito: ");
      Serial.print(encDIR_Position);
      Serial.print("   Contador Esquerdo: ");
      Serial.println(encESQ_Position);
      Serial.println("");

      encDIR_lastPosition = encDIR_Position;
      encESQ_lastPosition = encESQ_Position;
    }
  }
  //-------------------------------------------------

  // Debug Odometria:
  void debug_Odometria(){

    Serial.print("Heading: ");
    Serial.print( nvRadToDeg(nav_heading) );

    Serial.print("\tPosition: (");
    Serial.print( nav_pos_x );
    Serial.print(", ");
    Serial.print( nav_pos_y );
    Serial.print(")");

    Serial.print("\tSpeed: ");
    Serial.print( nav_velLinear );
    Serial.print("\tRight Speed: ");
    Serial.print( navigator.RightSpeed() );
    Serial.print("\tLeft Speed: ");
    Serial.print( navigator.LeftSpeed() );

    Serial.print("\tTurn Rate: ");
    Serial.println( nav_velAngular );
  }
  //------------------------------------------------

  //Debug PID:
  void debug_PID(){
    Serial.print("RI: ");
    Serial.print(rPID_Input);
    Serial.print("\tRO: ");
    Serial.print(rPID_Output);
    Serial.print("\tRS: ");
    Serial.print(rPID_Setpoint);
    Serial.print("\t|| LI: ");
    Serial.print(lPID_Input);
    Serial.print("\tLO: ");
    Serial.print(lPID_Output);
    Serial.print("\tLS: ");
    Serial.println(lPID_Setpoint);
  }
  //------------------------------------------------

  //Mostra bits:
  void mostraBits(unsigned int i){
  uint16_t x, a=1;//mascara (0000 0001)
  x=i; //conserva valor inicial
  a<<=15; //a=128 (1000 0000|0000 0000)

  for(int j=1; j<=16; j++){
    int ans;
    Serial.print( (i & a) ? '1' : '0');
    i <<= 1;
    if( !(j%8) ) {Serial.print(" ");}
  }
  Serial.print(" = ");
  Serial.println(x);
  }
////////////////////////////////////////////////////////////////////////////////

void setup() {

  //Inicia Serial:
  Serial.begin(115200);
  Serial3.begin(115200);
  //----------------------

  //Pinos I/O dos motores:
  pinMode(motorDireita_IN1, OUTPUT);
  pinMode(motorDireita_IN2, OUTPUT);
  pinMode(motorEsquerda_IN3, OUTPUT);
  pinMode(motorEsquerda_IN4, OUTPUT);
  //--------------------------------------

  //Pinos Ultrassom (Interrupt):
  pinMode(US_Frente_TrigPin, OUTPUT);
  digitalWrite(US_Frente_TrigPin, LOW);
  pinMode(US_Direito_TrigPin, OUTPUT);
  digitalWrite(US_Direito_TrigPin, LOW);
  pinMode(US_Esquerdo_TrigPin, OUTPUT);
  digitalWrite(US_Esquerdo_TrigPin, LOW);
  pinMode(US_servo_TrigPin, OUTPUT);
  digitalWrite(US_servo_TrigPin, LOW);
  //---//
  pinMode(US_EchoPin,INPUT);
  pinMode(US_servo_EchoPin,INPUT);
  //---//
  PulseInOne::setup(us_servo_PulseComplete);
  PulseInZero::setup(us_PulseComplete);
  //----------------------------------------------------


  //Servo:
  servoUS.attach(pin_Servo);
  //-----------------------------

  // Threads' setups:
    //Thread Controllers:
    TC_BATERIA.add(&T_set_PID_Intervals);
    TC_BATERIA.add(&T_alertaBateria);
    //-------------------------------------
    TC_US.add(&T_USinterrupt_Read);
    TC_US.add(&T_USi_Servo_Read);
    //-------------------------------------
    TC_ESP_SERIAL.add(&T_ESP_SERIAL);
    //-------------------------------------

    //Setting time and function to call:
    T_set_PID_Intervals.setInterval(5000); // in milisseconds
    T_set_PID_Intervals.onRun(set_PID_Intervals);

    T_alertaBateria.setInterval(10000); // in milisseconds
    T_alertaBateria.onRun(alertaBateria);
    //--------------------------------------------
    T_USinterrupt_Read.setInterval(10); // in milisseconds
    T_USinterrupt_Read.onRun(us_Read);

    T_USi_Servo_Read.setInterval(50); // in milisseconds
    T_USi_Servo_Read.onRun(us_servoRead);
    //--------------------------------------------
    T_ESP_SERIAL.setInterval(500); // in milisseconds
    T_ESP_SERIAL.onRun(sendToESP);
    T_ESP_SERIAL.enabled = 0; //Doesn't start running before ESP's "sendData" call.
  /****************************************************/

  //Inicia a Odometria e Navegação:
  // set up navigation
  navigator.InitEncoder( WHEEL_DIAMETER, WHEEL_BASE, TICKS_PER_REV );
  navigator.Reset( millis() );
  //-------------------------------------------------------------------

  //Inicialize PID:
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(100);

  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(100);
  //------------------------------------------------------------------

  //Realiza a primeira leitura da bateria:
  Serial.println("##### Iniciando leitura de bateria ########");
  set_PID_Intervals();
  Serial.println(pid_max_PWM);
  Serial.println("###########################################");
  //-------------------------------------------------------------------

}

void loop() {
  TC_BATERIA.run(); // Controls battery monitoring functions
  TC_US.run(); // Controls ultrasound sensors readings
  TC_ESP_SERIAL.run(); // Sends variables buffer to the ESP8266 every 500ms
  computePID(); // Calculates PID's parameters
  odometria(0); // Calculates position, velocity, orientation, etc
//-------------------//
  stateController();    // States machine
//-------------------//
  ESP8266_feedback();   // Receives commands coming from the ESP8266 
//-------------------//

//Debugs:
  // debug_Odometria();
  // debug_PID();
  // debug_Encoder();
  // debug_Ultrassom();
//------------------------    

}
