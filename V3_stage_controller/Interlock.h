/* Purpose: A class to control the enclosure interlock.
  Private Variables:
  - _SNR: interlock door's sensor pin
  - _RLY: Pin that control the relay for the spectrometer power

  Public variables:
  - axis_direction: current direction.
  - position: current position.
  - step_delay: the delay between consecutive steps. Control velocity of motor.
  In microseconds.

  Public methods:
  - setup_pins: sets up the pin mode. To be run during setup().
  - change_direction: it changes direction. Accepts 1 or -1 as arguments.
  - move_one_step: Move the motor one step.
*/
#ifndef Interlock_h
#define Interlock_h

#include "Arduino.h"
#define MAX_STRING_SIZE 32


class Interlock
{
  private:
    byte _SNR;
    byte _RLY;
    int _sensor_value;
    bool _power = true;

  public:
    void setup_pins(byte SNR_pin, byte RLY_pin);
    bool get_power_state();
    String get_interlock_state();
    bool restart();
    bool set_power(bool state);
    bool active = false;
    bool interrupted = false;
};


void Interlock::setup_pins(byte SNR_pin, byte RLY_pin) {
  // save pin locations
  _SNR = SNR_pin;
  _RLY = RLY_pin;
  pinMode(_RLY, OUTPUT);
  digitalWrite(_RLY,HIGH);
  pinMode(SNR_pin, INPUT_PULLUP);
  restart();
}

bool Interlock::get_power_state() {
  return _power;
}

bool Interlock::restart() {
  digitalWrite(_RLY, HIGH);
  _power = true;
  active = false;
  interrupted = false;
  return true;
}

bool Interlock::set_power(bool state) {
  if (state) {
    digitalWrite(_RLY, HIGH);
    _power = true;
  }
  else {
    digitalWrite(_RLY, LOW);
    _power = false;
  }
  return true;
}

String Interlock::get_interlock_state() {
  _sensor_value = digitalRead(_SNR);
  if (interrupted) {
    return "INTERRUPTED";
  }
  else {
    if (_sensor_value == 0) {
      return "OPEN";
    }
    else {
      return "CLOSED";
    }
  }
}

#endif
