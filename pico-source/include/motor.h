#ifndef MOTOR_H_
#define MOTOR_H_

#include <memory>
//#include <stdlib>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <QuickPID.h>

class Motor
{
  // Pin Values
  uint pwm_pin;
  uint speed_pin;
  uint direction_pin;
  uint brake_pin;

  // Interrupt variables
  bool first_reading = true;
  uint64_t last_time = 0;
  unsigned long counter = 0;
  float velocity = 0.0;
  bool brake_active = false;

  // Control variables
  int direction = 1;
  float Kp = 0.0001, Ki = 0.0002, Kd = 0;
  float setpoint = 0.0, output_duty_cycle = 0.0;
  float max_duty_cycle = 0.3;
  const uint pwm_count_top = 9803;

  // PID
  std::unique_ptr<QuickPID> motorPID;
  //QuickPID motorPID;

  //QuickPID testPID(setpoint, output_duty_cycle, velocity);

  public:

  void set_velocity(float angular_vel);
  float get_velocity();
  bool get_brake();

  float get_duty_cycle();
  void set_duty_cycle(float dc);
  void set_brake(bool state);
  void set_pins(uint _pwm_pin, uint _speed_pin, uint _direction_pin, uint _brake_pin);

  /*
   * Initializes the GPIO pins for this motor.
   */
  void init_gpio();

  /*
   * Initializes the PID velocity controller for this motor.
   */
  void init_PID();

  /*
   * Computes the next PID controller output.
   */
  void compute_PID();

  /*
   * Interrupt service routine for speed pin.
   */
  void speed_pin_ISR();

  /*
   * Constructor
   */
  Motor(uint _pwm_pin, uint _speed_pin, uint _direction_pin, uint _brake_pin);

  /*
   * Constructor
   */
  Motor();
};



#endif // MOTOR_H_
