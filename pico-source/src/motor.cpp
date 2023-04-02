#include "motor.h"

Motor::Motor(uint _pwm_pin, uint _speed_pin, uint _direction_pin)
  {
    pwm_pin = _pwm_pin;
    speed_pin = _speed_pin;
    direction_pin = _direction_pin;

    init_gpio();
  }

void Motor::set_duty_cycle(double dc)
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
    pwm_init(pwm_gpio_to_slice_num(pwm_pin), &cfg, true);

    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    // Set PWM duty cycle to zero
    set_duty_cycle(0.0);

    // Configure motor direction pin


    // Configure motor speed pin

  }
