//#include "controller.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "motor.h"

const uint LEFT_MOTOR_PIN = 3;
const uint LEFT_SPEED_PIN = 4;
const uint LEFT_DIR_PIN = 5;
//const uint RIGHT_MOTOR_PIN = ;

Motor left_motor;
Motor right_motor;

static void motor_speed_isr(uint gpio, uint32_t events)
  {
    if (gpio == LEFT_SPEED_PIN)
      left_motor.speed_pin_ISR();
    //else if (gpio == RIGHT_SPEED_PIN)
      //right_motor.speed_pin_ISR();
  }

void print_speed(Motor* motor)
  {
    for (int i = 0; i < 100; i++)
      {
        motor->compute_PID();
        printf("Output duty cycle: %f ", motor->get_duty_cycle());
        printf("Velocity: %f\n", motor->get_velocity());
        sleep_ms(10);      }
  }

int main() {

  left_motor.set_pins(LEFT_MOTOR_PIN, LEFT_SPEED_PIN, LEFT_DIR_PIN);
  left_motor.init_gpio();
  gpio_set_irq_enabled_with_callback(LEFT_SPEED_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true,
                                     &motor_speed_isr);



  stdio_init_all();
  while(true){
    printf("Motor on\n");
    left_motor.set_velocity(1000);
    print_speed(&left_motor);
    printf("Motor off\n");
    left_motor.set_velocity(0);
    print_speed(&left_motor);
  }
}
