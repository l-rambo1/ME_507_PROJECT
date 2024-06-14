/*
 * encoder_driver.c
 *
 *  Created on: June 6, 2024
 *      Author: mitch
 *
 *      Drives servo with remote inputs and returns it to its original position by reading the
 *      total number of degrees traveled when remote input is ceased
 *
 */

#include <encoder_driver2.h>
#include "string.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "stm32f411xe.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_rcc.h"
#include <stdint.h>
#include <stdio.h>

float compare_value;
//int duty;

void set_duty(motor_t * p_mot, int32_t duty)
  {
    // Assign the duty cycle to a field in the structure
    p_mot -> duty = duty;  // set duty value
    compare_value = (duty/1000.0f)-0.1f; // convert time to ms with SS error correction

    if (compare_value < 1.52-0.1 && compare_value > 1.5-0.1)
    {

    }
    else
    {
    __HAL_TIM_SET_COMPARE(p_mot -> timer, p_mot -> channel1, 3780/3 *compare_value); //set servo compare
    }
  }
