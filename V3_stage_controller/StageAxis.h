#ifndef StageAxis_h
#define StageAxis_h
#include "Arduino.h"

// define maximum number of steps each stage is allowed to move in one direction at a time
#define MAX_STEPS 16500
#define STEP_DELAY 200

class StageAxis
{
  public:
    void setup_pins(byte STP_pin, byte DIR_pin, byte EN_pin, byte SENS_pin, byte DIAG_pin, byte IND_pin, byte ADDR);
    void change_direction(int new_direction);
    void check_direction(int distance);
    void move_one_step();
    void set_pin(byte value);
    bool is_within_limits();
    void invert_dir(bool invert);
    void invert_prox(bool invert);

    void move_nonblocking();
    void move_blocking(long distance);
    void set_distance(long distance);
    long get_distance();
    void set_step_delay(int stepdelayin);
    int step_delay = 200;
    int axis_direction = 1;
    long steps_taken = 0;
    
  private:
    float accel_time = 0.2;
    float min_speed = (1000000 / step_delay) / 20;
    float curr_speed = min_speed;
    float max_speed = (1000000 / step_delay);
    unsigned long last_step_time;
    unsigned long curr_step_delay = 200;
    
    float accel = max_speed/accel_time;
    long steps_left = 0;
    
    bool _invert_dir = 0;
    bool _invert_prox = 0;
    
    byte _DIR;
    byte _STP;
    byte _EN;
    byte _SENS;
    byte _DIAG;
    byte _IND;
    byte _ADDR;
};

void StageAxis::setup_pins(byte STP_pin, byte DIR_pin, byte EN_pin, byte SENS_pin, byte DIAG_pin,
                           byte IND_pin, byte ADDR) {
  // save pin locations
  _DIR = DIR_pin;
  _STP = STP_pin;
  _EN = EN_pin;
  _SENS = SENS_pin;
  _DIAG = DIAG_pin;
  _ADDR = ADDR;
  // set pins
  pinMode(_DIR, OUTPUT);
  pinMode(_STP, OUTPUT);
  pinMode(_EN, OUTPUT);
  pinMode(_DIAG, INPUT);
  pinMode(_SENS, INPUT);
  digitalWrite(_DIR, LOW);
  digitalWrite(_EN, LOW);
}

void StageAxis::set_step_delay(int stepdelayin) {
   step_delay = stepdelayin;
   
   min_speed = (1000000 / step_delay) / 50;
   curr_speed = min_speed;
   max_speed = (1000000 / step_delay);
   accel = max_speed/accel_time;
//   Serial.print("accel");
//   Serial.println(accel);
}

void StageAxis::change_direction(int new_direction) {
  bool dir;
  if (new_direction == 1) {
    dir = 0;
  }
  if (new_direction == -1) {
    dir = 1;
  }
  
  if (_invert_dir){
    digitalWrite(_DIR, !dir);
  }else
    digitalWrite(_DIR, dir);
  delayMicroseconds(1);
  axis_direction = new_direction;
}

void StageAxis::invert_dir(bool invert) {
  _invert_dir = invert;
}

void StageAxis::invert_prox(bool invert) {
  _invert_prox = invert;
}

void StageAxis::check_direction(int distance) {
  // check direction of movement of x
  if (distance > 0 && axis_direction < 0) {
    //Serial.println("Changing direction to 1");
    change_direction(1);
  }
  else if (distance < 0 && axis_direction > 0) {
    //Serial.println("Changing direction to -1");
    change_direction(-1);
  }
  
#ifdef DEBUGGING_MODE
  Serial.print(distance);
  Serial.print(' ');
  Serial.print(axis_direction);
  Serial.print(' ');
  Serial.println(abs(distance));
#endif
}

void StageAxis::move_one_step() {
  digitalWrite(_STP, HIGH);
  digitalWrite(_STP, LOW);
  delayMicroseconds(step_delay);
}

void StageAxis::set_pin(byte value) {
  digitalWrite(_STP, value);
}

bool StageAxis::is_within_limits() {
  if (_invert_prox)
    return !digitalRead(_SENS);
  else
    return digitalRead(_SENS);
}

void StageAxis::set_distance(long distance) {
  if (distance > 0)
    change_direction(1);
  else
    change_direction(-1);
    
  steps_left = abs(distance);
  last_step_time = micros();
  curr_speed = min_speed;
}

void StageAxis::move_nonblocking() {
  if (steps_left > 0 &&  micros()-last_step_time >= curr_step_delay) {
    digitalWrite(_STP, HIGH);

    float steps_to_stop = (curr_speed*curr_speed)/(2*accel)+2;
    float speed_bump = curr_step_delay*accel/1000000;
    
//    curr_speed = max_speed;
//    Serial.print("NumSteps ");
//    Serial.print(steps_left);
//    Serial.print(" curr_step_delay ");
//    Serial.print(curr_step_delay, 5);
//    Serial.print(" steps_to_stop "); 
//    Serial.println(steps_to_stop, 1);
//    Serial.print(" speed_bump ");
//    Serial.println(speed_bump*1000000, 5);


    
    //DECEL
    if (steps_left <= steps_to_stop) { //decelerate close to the end
      if(curr_speed > (min_speed+speed_bump))
        curr_speed -= speed_bump;
      else
        curr_speed = min_speed;
    }

    //ACCEL
    else {
      if (curr_speed+speed_bump >  max_speed){  //when current speed lower then accelerate
        curr_speed = max_speed;
      }else {
        curr_speed += speed_bump;
      }
    }

//    if (curr_speed < min_speed){
//      curr_speed = min_speed;
//    }
//    if (curr_speed > max_speed){
//      curr_speed = max_speed;
//    }
    
    curr_step_delay = 1000000/curr_speed;
    last_step_time = micros();
    steps_taken += axis_direction;
    digitalWrite(_STP, LOW);
    steps_left--;
  }
}

void StageAxis::move_blocking(long distance) {
  set_distance(distance);
  while(get_distance() > 0){
    move_nonblocking();
    }
}


long StageAxis::get_distance() {
  return steps_left;
}

#endif
