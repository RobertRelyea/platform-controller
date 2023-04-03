#include "motor.h"

Motor::Motor()
{
  init_PID();
}

Motor::Motor(uint _pwm_pin, uint _speed_pin, uint _direction_pin)
  {
    init_PID();
    set_pins(_pwm_pin, _speed_pin, _direction_pin);
  }

void Motor::set_velocity(float angular_vel)
  {
    setpoint = angular_vel;
  }

double Motor::get_velocity()
  {
    return velocity;
  }

void Motor::set_pins(uint _pwm_pin, uint _speed_pin, uint _direction_pin)
  {
    pwm_pin = _pwm_pin;
    speed_pin = _speed_pin;
    direction_pin = _direction_pin;
  }

void Motor::set_duty_cycle(float dc)
  {
    // Clip duty cycle at max allowed value
    if (dc > max_duty_cycle)
      dc = max_duty_cycle;

    // Set gpio level to appropriate counter value
    pwm_set_gpio_level(pwm_pin, dc * (pwm_count_top + 1));
  }

void Motor::init_gpio()
  {
    // Configure motor pwm pin
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, pwm_count_top);
    pwm_config_set_clkdiv(&cfg, 256);
    pwm_init(pwm_gpio_to_slice_num(pwm_pin), &cfg, true);

    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    // Set PWM duty cycle to zero
    set_duty_cycle(0.0);

    // Configure motor direction pin
  }

void Motor::init_PID()
  {
    motorPID = std::make_unique<QuickPID>(QuickPID(&velocity, &output_duty_cycle, &setpoint,
                                                   Kp, Ki, Kd, motorPID->Action::direct));
    //motorPID->SetTunings(Kp, Ki, Kd);
    //motorPID->SetOutputLimits(-max_duty_cycle, max_duty_cycle);
    motorPID->SetOutputLimits(0, max_duty_cycle);
    motorPID->SetMode(motorPID->Control::automatic);
  }

float Motor::get_duty_cycle()
  {
    return output_duty_cycle;
  }

void Motor::compute_PID()
  {
    motorPID->Compute();
    set_duty_cycle(output_duty_cycle);
  }

void Motor::speed_pin_ISR()
  {
    // We are here because there was a state change on the speed pin.
    // This means the motor has spun 1/90 of a full rotation in either direction.
    if(first_reading)
    {
      last_time = time_us_64();
      first_reading = false;
      return;
    }

    uint64_t new_time = time_us_64();

    double dt = double(new_time - last_time) / 1e6;
    velocity = (4 * direction) / dt;
    counter += int(direction);

    last_time = new_time;

  }
