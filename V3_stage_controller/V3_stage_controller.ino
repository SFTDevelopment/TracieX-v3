#include "Hardware_Rev1.h"
#include "StageAxis.h"
#include "InputCommand.h"
#include "Interlock.h"
#include "RGBLED.h"

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <SoftwareSerial.h>

// define other constants and variables
//#define DEBUGGING_MODE
const char separator[2] = " ";
bool message_result;
bool command_result;
bool interlock_interrupted = false;
bool TestMode = false;
long LastTest = 0;

//Class definitions
InputCommand command = InputCommand();
StageAxis stage_z = StageAxis();
StageAxis stage_y = StageAxis();
StageAxis stage_x = StageAxis();
Interlock interlock = Interlock();
RGBLED ledstrip(LEDPWR_EN, LED_DATA, LEDPWR_OVRCUR_N, 23);

SoftwareSerial TMCSerial(39, 40);
TMC2209Stepper xDriver(&TMCSerial, 0.1, 0);
TMC2209Stepper yDriver(&TMCSerial, 0.1, 1);
TMC2209Stepper zDriver(&TMCSerial, 0.1, 2);
#define XCURR 500
#define YCURR 500
#define ZCURR 500 

////////////////////////////////////////////////////////
////////////////////// MAIN LOOP ///////////////////////
////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);


  ledstrip.begin();
  ledstrip.set_pwr(1);

  //slow fade to on;
  for (int i = 0; i < 255; i++) {
    ledstrip.set_colour(0, i, 0);
    delay(2);
  }

  delay(100);

  // this section here for soft-start of motor. So when power on, there is not a large thud sound.

  TMCSerial.begin(115200);


  xDriver.begin();
  xDriver.rms_current(100); // mA
  xDriver.microsteps(8);

  yDriver.begin();
  yDriver.rms_current(100); // mA
  yDriver.microsteps(8);

  zDriver.begin();
  zDriver.rms_current(100); // mA
  zDriver.microsteps(8);

  delay(100);

  stage_z.setup_pins(TMC3_STEP, TMC3_DIR, TMC3_EN_N, ITL1 , TMC3_DIAG, TMC3_INDEX, TMC3_UART_ADDR);
  stage_z.set_step_delay(200);
  stage_z.invert_dir(0);

  stage_y.setup_pins(TMC2_STEP, TMC2_DIR, TMC2_EN_N, PROX2, TMC2_DIAG, TMC2_INDEX, TMC2_UART_ADDR);
  stage_y.set_step_delay(200);
  stage_y.invert_dir(0);

  stage_x.setup_pins(TMC1_STEP, TMC1_DIR, TMC1_EN_N, PROX1, TMC1_DIAG, TMC1_INDEX, TMC1_UART_ADDR);
  stage_x.set_step_delay(200);
  stage_x.invert_dir(0);

  interlock.setup_pins(ITL2, SPTPWR_EN);
  command._restart();
  attachInterrupt(digitalPinToInterrupt(ITL2), int_interrupt , FALLING);

  analogWriteFrequency(15000);
  analogWriteResolution(10);
  analogWrite(TMCX_VREF, 850) ;

  interlock.set_power(1);

  //set proper current after soft-start
  xDriver.rms_current(XCURR); // mA
  yDriver.rms_current(YCURR); // mA
  zDriver.rms_current(ZCURR); // mA
}

void int_interrupt() {
  if (interlock.active && interlock.interrupted == false) {
    interlock.set_power(0);
    interlock.interrupted = true;
    ledstrip.set_orange();
  }
}

void loop() {
  message_result = command.receive_character();
  if (message_result) {
    command_switch();
  }

  if (TestMode && millis() - LastTest > 500) {
    LastTest = millis();
    Serial.print("Interlock: ");
    Serial.print(interlock.get_interlock_state());
    Serial.print(" 5V: ");

    interlock.set_power(!interlock.get_power_state());
    Serial.print(interlock.get_power_state());

    Serial.print("  XYZ_LIM: ");
    Serial.print(digitalRead(stage_x.is_within_limits()));

    //    Serial.print("  Y_LIM: ");
    Serial.print(digitalRead(stage_y.is_within_limits()));

    //    Serial.print("  Z_LIM: ");
    Serial.println(digitalRead(stage_z.is_within_limits()));
  }
}

bool command_switch() {
  //  Serial.println("Starting command switch");
  if (strcmp(command.method_name, "move") == 0) {
    bool outcome = false;
    if (strcmp(command.argument_1, "xyz") == 0) {
      int distance_x = atof(command.argument_2) * 1.28;
      int distance_y = atof(command.argument_3) * 1.28;
      int distance_z = atof(command.argument_3) * 1.28;
      
      stage_x.set_distance(distance_x);
      stage_y.set_distance(distance_y);
      stage_z.set_distance(distance_z);
      
      while (stage_x.get_distance() > 0 || stage_y.get_distance() > 0 || stage_z.get_distance() > 0) {
        stage_x.move_nonblocking();
        stage_y.move_nonblocking();
        stage_z.move_nonblocking();
      }
      outcome = true;
    }
    else if (strcmp(command.argument_1, "x") == 0) {
      int distance = atof(command.argument_2) * 1.28;
      stage_x.move_blocking(distance);
      outcome = true;
    }
    else if (strcmp(command.argument_1, "y") == 0) {
      int distance = atof(command.argument_2) * 1.28;
      stage_y.move_blocking(distance);
      outcome = true;
    }
    else if (strcmp(command.argument_1, "z") == 0) {
      int distance = atof(command.argument_2) * 1.28;
      stage_z.move_blocking(distance);
      outcome = true;
    }
    else if (strcmp(command.argument_1, "xyz") == 0) {
      //      stage_z.set_step_delay(150);
      //      stage_y.set_step_delay(150);
      //      stage_x.set_step_delay(150);

      int distance = atof(command.argument_2) * 1.28;
      if (distance < 0) { //moving in the negative direction
        stage_x.set_distance(distance);
        stage_y.set_distance(distance);
        stage_z.set_distance(distance);
      } else {
        stage_x.set_distance(distance);
        stage_y.set_distance(distance);
        stage_z.set_distance(distance);
      }

      while (stage_x.get_distance() > 0 || stage_y.get_distance() > 0 || stage_z.get_distance() > 0) {
        stage_x.move_nonblocking();
        stage_y.move_nonblocking();
        stage_z.move_nonblocking();
      }
      if (distance < 0)
      outcome = true;
    }
    print_boolean(outcome);
  }

  else if (strcmp(command.method_name, "raster") == 0) {
    bool outcome = false;
    outcome = spiral_raster(atof(command.argument_1), atof(command.argument_2), atof(command.argument_3), stage_x, stage_y, interlock);
    print_boolean(outcome);
  }
  else if (strcmp(command.method_name, "home") == 0) {
    //    Serial.println(command.argument_1);
    if (strcmp(command.argument_1, "x") == 0) {
      home_axes(15000, 0, 0);
    }
    else if (strcmp(command.argument_1, "y") == 0) {
      home_axes(0, 15000, 0);
    }
    else if (strcmp(command.argument_1, "z") == 0) {
      home_axes(0, 0, 15000);
    }
    else if (strcmp(command.argument_1, "xy") == 0) {
      home_axes(15000, 15000, 0);
    }
    else if (strcmp(command.argument_1, "xyz") == 0) {
      home_axes(15000, 15000, 15000);
    }
  }
  else if (strcmp(command.method_name, "interlock") == 0) {
    if (strcmp(command.argument_1, "state") == 0) {
      String outcome;
      outcome = interlock.get_interlock_state();
      Serial.println(outcome);
    }
    else if (strcmp(command.argument_1, "activate") == 0) {
      interlock.restart();
      interlock.active = true;
      print_boolean(true);
      String outcome;
      ledstrip.set_blue();
      outcome = interlock.get_interlock_state();
      if (outcome == "OPEN" || outcome == "INTERRUPTED") {
        interlock_interrupted = true;
        int_interrupt();
      }
    }
    else if (strcmp(command.argument_1, "deactivate") == 0) {
      interlock.active = false;
      ledstrip.set_green();
      print_boolean(true);
    }
    else if (strcmp(command.argument_1, "isactive") == 0) {
      print_boolean(interlock.active);
    }
    else if (strcmp(command.argument_1, "isinterrupted") == 0) {
      print_boolean(interlock.interrupted);
    }
    else if (strcmp(command.argument_1, "poweron") == 0) {
      print_boolean(interlock.set_power(true));
    }
    else if (strcmp(command.argument_1, "poweroff") == 0) {
      print_boolean(interlock.set_power(false));
    }
    else if (strcmp(command.argument_1, "ispowered") == 0) {
      print_boolean(interlock.get_power_state());
    }
    else if (strcmp(command.argument_1, "restart") == 0) {
      print_boolean(interlock.restart());
      interlock_interrupted = false;
    }
    else if (strcmp(command.argument_1, "interrupted") == 0) {
      print_boolean(interlock_interrupted);
    }
  }
  else if (strcmp(command.method_name, "testmode") == 0) {
    if (strcmp(command.argument_1, "activate") == 0) {
      TestMode = true;
      Serial.println("test mode ON");
    }
    else if (strcmp(command.argument_1, "deactivate") == 0) {
      TestMode = false;
      Serial.println("test mode OFF");
      interlock.set_power(true);
    }
  }
  else if (strcmp(command.method_name, "led") == 0) {
    if (strcmp(command.argument_1, "green") == 0) {
      ledstrip.set_green();
    } else if (strcmp(command.argument_1, "blue") == 0) {
      ledstrip.set_blue();
    } else if (strcmp(command.argument_1, "orange") == 0) {
      ledstrip.set_orange();
    }
    else if (strcmp(command.argument_1, "brightness") == 0) {
      ledstrip.set_brightness(atoi(command.argument_2));
    }
    else {
      byte r = atoi(command.argument_1);
      byte g = atoi(command.argument_2);
      byte b = atoi(command.argument_3);
      ledstrip.set_colour(r, g, b);
    }
    print_boolean(true);
  }
  else if (strcmp(command.method_name, "motor") == 0) {
    if (strcmp(command.argument_1, "current") == 0) {
      uint16_t current = atoi(command.argument_2);
      if (current > 100 && current < 1000) {
        Serial.print("Set motor current to ");
        Serial.println(current);
        xDriver.rms_current(current); // mA
        yDriver.rms_current(current); // mA
      }
    }
  }
  else if (strcmp(command.method_name, "compile") == 0) {
    Serial.print (__DATE__) ;
    Serial.print (" ") ;
    Serial.println (__TIME__) ;
  }

  else if (strcmp(command.method_name, "eeprom") == 0) {
    if (strcmp(command.argument_1, "put") == 0) {
      uint16_t location = atoi(command.argument_2);
      if (location >= 0 && location < 512) {
        bool outcome = writeMemory(location, atoi(command.argument_3));
        print_boolean(outcome);
      }else{
        Serial.println("Exceeded allocated memory position");
      }
    }
    else if (strcmp(command.argument_1, "get") == 0) {
//    readMemory();
    }
  }

  // restart command
  command._restart();
  return (true);
}

////////////////////////////////////////////////////////
//////////////////////  FUNCTIONS //////////////////////
////////////////////////////////////////////////////////

void print_boolean(bool outcome) {
  if (outcome) {
    Serial.println("True");
  }
  else {
    Serial.println("False");
  }
}

bool spiral_raster(int max_radius,
                   double cycle_time,
                   double transition_time,
                   class StageAxis & x_stage,
                   class StageAxis & y_stage,
                   class Interlock & laser_interlock) {

  // declare original variables
  double pi = 3.141592;
  double angle_factor = 1 / cycle_time * 2 * pi;
  double radius_factor = max_radius / (transition_time * angle_factor);
  double delta_t, angle2, start_time, angle, radius, end_time = 0.0, now, interlock_time0;
  bool end_movement = false;
  int current_x = 0;
  int current_y = 0;
  int target_x, target_y, distance_x, distance_y, original_latency;
  String message;
  int i = 0;
  String interlock_state;

  // raster latency is 200, same as normal
  // activate the interlock
  laser_interlock.active = true;
  // reply ready to start
  Serial.println("Starting");
  // start movement
  start_time = millis();
  while (1) {
    // measure current time
    now = millis();
    delta_t = (now - start_time);

    if (interlock.interrupted) {
      end_movement = true;
      end_time = now + transition_time;
      break;
      //Serial.println("interrupted...");
    }

    // if the MCU receives text, prcess it.
    if (Serial.available() > 0) {
      // grab the message
      message = Serial.readStringUntil('\n');
      // if asking to check interlock, return value
      if (message == "interlock state") {
        Serial.println(interlock.get_interlock_state());
        //Serial.println(interlock.interrupted);
      }
      // else, set to terminate raster movement
      else {
        end_movement = true;
        // Serial.println(message);
        // calulate end time
        end_time = now + transition_time;
      }
    }

    if (end_time != 0.0 && now > end_time) {
      break;
    }

    // make the next step
    if (end_movement == false) {
      angle = delta_t * angle_factor;
      radius = min(radius_factor * angle, max_radius);
      target_x = round(radius * cos(angle));
      target_y = round(radius * sin(angle));
    }

    else {
      angle = delta_t * angle_factor;
      angle2 = (end_time - delta_t) / cycle_time * 2 * pi;
      radius = min(radius_factor * angle2, max_radius);
      target_x = round(radius * cos(angle));
      target_y = round(radius * sin(angle));
    }

    // check if need to move to next locations
    distance_x = target_x - current_x;
    distance_y = target_y - current_y;
        
    move_both_axes(distance_x, distance_y, x_stage, y_stage);

    // update position
    current_x = target_x;
    current_y = target_y;
  }

  // check that we end where we started
  distance_x = 0 - current_x;
  distance_y = 0 - current_y;
  
  move_both_axes(distance_x, distance_y, x_stage, y_stage);

  // deactivate interlock
  laser_interlock.active = false;

  // return success
  return (true);
}

bool move_both_axes(int distance_x, int distance_y, class StageAxis & stage_x, class StageAxis & stage_y) {
  // check if changing firection for x
  stage_x.check_direction(distance_x);
  distance_x = abs(distance_x);

  // check if changing direction for y
  stage_y.check_direction(distance_y);
  distance_y = abs(distance_y);

  int longest_distance = max(distance_x, distance_y);
  //Serial.println(longest_distance);
  unsigned long i = 0;
  while (i < longest_distance) {
    if (i < distance_x) {
      stage_x.set_pin(HIGH);
    }
    if (i < distance_y) {
      stage_y.set_pin(HIGH);
    }
    if (i < distance_x) {
      stage_x.set_pin(LOW);
    }
    if (i < distance_y) {
      stage_y.set_pin(LOW);
    }
    delayMicroseconds(STEP_DELAY);
    i += 1;
  }
  return (true);
}

void home_axes(long steps_x, long steps_y, long steps_z) {
  //Homing is done in phases:
  //phase 0 = homing, 1 = wait, 2 = adjust, 3 = done, 128 = error;
  uint8_t x_phase = 3, y_phase = 3, z_phase = 3;

  //variable to store the time at start of wait
  long x_wait, y_wait, z_wait;

  //reset steps taken
  stage_x.steps_taken = 0;
  stage_y.steps_taken = 0;
  stage_z.steps_taken = 0;

  //start moving in negative direction and change to homing phase for selected axes
  if (steps_x > 0) {
    stage_x.set_distance(-steps_x);
    x_phase = 0;
  }
  if (steps_y > 0) {
    stage_y.set_distance(-steps_y);
    y_phase = 0;
  }
  if (steps_z > 0) {
    stage_z.set_distance(-steps_z);
    z_phase = 0;
  }

  while (true) {
    mini_home(stage_x, &x_phase, &x_wait);
    mini_home(stage_y, &y_phase, &y_wait);
    mini_home(stage_z, &z_phase, &z_wait);
    if (x_phase == 128 || y_phase == 128 || z_phase == 128) {
      Serial.println("-1");
      return;
    }
    if (x_phase > 2 && y_phase > 2 && z_phase > 2) {
      break;
    }
  }

  bool firstPrint = 1;
  if (steps_x > 0) {
    Serial.print(abs(stage_x.steps_taken));
    firstPrint = 0;
  }
  if (steps_y > 0) {
    if (firstPrint == 0)
      Serial.print(",");

    Serial.print(abs(stage_y.steps_taken));

  }
  if (steps_z > 0) {
    if (firstPrint == 0)
      Serial.print(",");
      
    Serial.print(abs(stage_z.steps_taken));
  }

  Serial.println();
  //we're done moving
  stage_x.set_distance(0);
  stage_y.set_distance(0);
  stage_z.set_distance(0);
}

void mini_home(class StageAxis & stage, uint8_t* phase, long * wait) {
  //Homing is done in phases:
  //phase 0 = homing, 1 = wait, 2 = adjust, 3 = done, 128 = error;
  if (*phase == 0) {
    if (stage.is_within_limits()) {
      stage.move_nonblocking();
    } else {
      *phase = 1;
      *wait = millis();
    }
    if (stage.get_distance() <= 0) {
      *phase = 128;
      return;
    }
  }
  else if (*phase == 1) {
    if (millis() - *wait > 50) {
      *phase = 2;
      stage.set_distance(100000);
    }
  }
  else if (*phase == 2) {
    if (!stage.is_within_limits()) {
      stage.move_nonblocking();
    } else {
      *phase = 3;
    }
    if (stage.get_distance() <= 0) {
      *phase = 128;
      return;
    }
  }
}
