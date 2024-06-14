/*
 * encoder_return.c
 *
 *  Created on: Jun 10, 2024
 *      Author: mitch
 */

#include "encoder_return2.h"
#include "string.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "stm32f411xe.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_rcc.h"
#include <stdint.h>
#include <stdio.h>

void encoder_return(servo_t * p_mot, int32_t angle)
  {
	p_mot -> angle = angle;  // set duty value

		if (angle < -20) // if there is a negative position
		{
			__HAL_TIM_SET_COMPARE(p_mot -> timer, p_mot -> channel1, 6400); //set servo compare
		}
		else if (angle > 20) // if there is positive position
		{
			__HAL_TIM_SET_COMPARE(p_mot -> timer, p_mot -> channel1, 3200);
		}
	}








