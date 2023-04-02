//#include "controller.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "motor.h"

const uint LEFT_MOTOR_PIN = 3;
const uint RIGHT_MOTOR_PIN = 4;

int main() {

  Motor left_motor(LEFT_MOTOR_PIN, 4, 5);

  stdio_init_all();
  while(true){
    printf("Motor on\n");
    left_motor.set_duty_cycle(0.01);
    sleep_ms(1000);
    printf("Motor off\n");
    left_motor.set_duty_cycle(0.0);
    sleep_ms(1000);
  }
}
