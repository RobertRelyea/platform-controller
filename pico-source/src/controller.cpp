//#include "controller.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "motor.h"


const uint LEFT_MOTOR_PIN = 16;
const uint LEFT_SPEED_PIN = 17;
const uint LEFT_DIR_PIN = 18;
const uint LEFT_BRAKE_PIN = 19;

const uint RIGHT_MOTOR_PIN = 15;
const uint RIGHT_SPEED_PIN = 14;
const uint RIGHT_DIR_PIN = 13;
const uint RIGHT_BRAKE_PIN = 12;

Motor left_motor;
Motor right_motor;

static void motor_speed_isr(uint gpio, uint32_t events)
  {
    if (gpio == LEFT_SPEED_PIN)
      left_motor.speed_pin_ISR();
    else if (gpio == RIGHT_SPEED_PIN)
      right_motor.speed_pin_ISR();
  }

bool motor_pid_timer_callback(struct repeating_timer *t)
  {
    left_motor.compute_PID();
    right_motor.compute_PID();
    return true;
  }

void print_speed(Motor* motor)
  {
    printf("Output duty cycle: %f ", motor->get_duty_cycle());
    printf("Velocity: %f\n", motor->get_velocity());
  }



int main() {

  left_motor.set_pins(LEFT_MOTOR_PIN, LEFT_SPEED_PIN, LEFT_DIR_PIN, LEFT_BRAKE_PIN);
  left_motor.init_gpio();
  gpio_set_irq_enabled_with_callback(LEFT_SPEED_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true,
                                     &motor_speed_isr);

  right_motor.set_pins(RIGHT_MOTOR_PIN, RIGHT_SPEED_PIN, RIGHT_DIR_PIN, RIGHT_BRAKE_PIN);
  right_motor.init_gpio();
  gpio_set_irq_enabled_with_callback(RIGHT_SPEED_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true,
                                     &motor_speed_isr);



  stdio_init_all();

  struct repeating_timer timer;
  // Run motor PID at 1000hz
  add_repeating_timer_ms(1, motor_pid_timer_callback, NULL, &timer);

  while(true){
    printf("Motor forward\n");
    left_motor.set_velocity(1500);
    right_motor.set_velocity(1500);
    sleep_ms(1000);
    printf("Motor off\n");
    left_motor.set_velocity(100);
    right_motor.set_velocity(100);
    sleep_ms(1000);
    printf("Motor reverse\n");
    left_motor.set_velocity(-1500);
    right_motor.set_velocity(-1500);
    sleep_ms(1000);
    printf("Motor off\n");
    left_motor.set_velocity(-100);
    right_motor.set_velocity(-100);
    sleep_ms(1000);
    left_motor.set_brake(true);
    right_motor.set_brake(true);
    sleep_ms(1000);
    left_motor.set_brake(false);
    right_motor.set_brake(false);
    sleep_ms(1000);
  }

}
