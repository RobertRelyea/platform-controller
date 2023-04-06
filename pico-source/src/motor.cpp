#include "motor.h"

Motor::Motor()
{
  init_PID();
}

Motor::Motor(uint _pwm_pin, uint _speed_pin, uint _direction_pin, uint _brake_pin)
  {
    init_PID();
    set_pins(_pwm_pin, _speed_pin, _direction_pin, _brake_pin);
  }

void Motor::set_velocity(float angular_vel)
  {
    if (!brake_active)
      setpoint = angular_vel;
    else
      setpoint = 0.0;
  }

float Motor::get_velocity()
  {
    return velocity;
  }

void Motor::set_pins(uint _pwm_pin, uint _speed_pin, uint _direction_pin, uint _brake_pin)
  {
    pwm_pin = _pwm_pin;
    speed_pin = _speed_pin;
    direction_pin = _direction_pin;
    brake_pin = _brake_pin;
  }

void Motor::set_duty_cycle(float dc)
  {

    // Set direction pin according to requested dc
    if (dc < 0){
        direction = -1;
        gpio_put(direction_pin, 1);
        dc = -dc;
    }
    else if (dc > 0) {
      direction = 1;
      gpio_put(direction_pin, 0);
    }
    // A zero DC will not change the direction, must be > or < zero

    //dc = abs(dc);
    // Clip duty cycle at max allowed value
    if (dc > max_duty_cycle)
      dc = max_duty_cycle;

    // Set gpio level to appropriate counter value
    pwm_set_gpio_level(pwm_pin, dc * (pwm_count_top + 1));
  }

void Motor::set_brake(bool state)
  {
    brake_active = state;
    if (brake_active)
      {
        gpio_put(brake_pin, 1);
        setpoint = 0.0;
      }
    else
      gpio_put(brake_pin, 0);
  }

bool Motor::get_brake()
  {
    return brake_active;
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
    gpio_init(direction_pin);
    gpio_set_dir(direction_pin, GPIO_OUT);
    gpio_put(direction_pin, 0);

    // Configure brake pin
    gpio_init(brake_pin);
    gpio_set_dir(brake_pin, GPIO_OUT);
    gpio_put(brake_pin, 0);
  }

void Motor::init_PID()
  {
    motorPID = std::make_unique<QuickPID>(QuickPID(&velocity, &output_duty_cycle, &setpoint,
                                                   Kp, Ki, Kd, motorPID->Action::direct));
    motorPID->SetOutputLimits(-max_duty_cycle, max_duty_cycle);
    motorPID->SetMode(motorPID->Control::automatic);
    motorPID->SetAntiWindupMode(motorPID->iAwMode::iAwOff);
    //motorPID->SetProportionalMode(motorPID->pMode::pOnMeas);
  }

float Motor::get_duty_cycle()
  {
    return output_duty_cycle;
  }

void Motor::compute_PID()
  {
    motorPID->Compute();
    if (setpoint == 0.0)
      set_duty_cycle(0.0);
    else
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
    velocity = float((4 * direction) / dt);
    counter += int(direction);

    last_time = new_time;

  }
