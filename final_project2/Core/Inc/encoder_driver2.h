/*
 * encoder_driver.h
 *
 *  Created on: Jun 9, 2024
 *      Author: mitch
 */

#ifndef INC_ENCODER_DRIVER2_H_
#define INC_ENCODER_DRIVER2_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

// Motor object data structure
struct {
    int32_t  duty;
    TIM_HandleTypeDef *timer;
    uint32_t channel1;
} typedef motor_t;

// Prototype for motor object "method"
void set_duty(motor_t* p_mot, int32_t duty);

#endif /* INC_ENCODER_DRIVER2_H_ */
