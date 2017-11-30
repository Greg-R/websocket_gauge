/* MCPWM basic config example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 19   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 18 as PWM0B

//static void mcpwm_example_gpio_initialize()
//{
//    printf("initializing mcpwm gpio...\n");
//    mcpwm_pin_config_t pin_config = {
//        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
//        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
//    };
//    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
//}

//  This is the statically allocated MCPWM config struct.
mcpwm_config_t pwm_config;

void set_mcpwm_config() {
pwm_config.frequency = 300;    //frequency = 1000Hz
pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 50.0%
pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
pwm_config.counter_mode = MCPWM_UP_COUNTER;
pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

mcpwm_pin_config_t pin_config = {
    .mcpwm0a_out_num = GPIO_PWM0A_OUT,
    .mcpwm0b_out_num = GPIO_PWM0B_OUT,
};
mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
}



