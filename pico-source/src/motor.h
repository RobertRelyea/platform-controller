#ifndef MOTOR_H_
#define MOTOR_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

class Motor
{
  // Pin Values
  uint pwm_pin;
  uint speed_pin;
  uint direction_pin;

  // Interrupt variables
  bool first_reading = true;
  volatile unsigned long last_time = 0;
  volatile unsigned long counter = 0;
  volatile double velocity = 0.0;

  // Control variables
  int direction = 1;
  double Kp = 0.1, Ki = 0.2, Kd = 0;
  double setpoint = 0.0, output_duty_cycle = 0.0;
  double max_duty_cycle = 0.2;
  const uint pwm_count_top = 1000;

  public:


  void set_duty_cycle(double dc);

    /*
   * Initializes the GPIO pins for this motor.
   */
  void init_gpio();

  /*
   * Interrupt service routine for speed pin.
   */
  void speed_pin_ISR();

  /*
   * Constructor
   */
  Motor(uint _pwm_pin, uint _speed_pin, uint _direction_pin);
};



#endif // MOTOR_H_
