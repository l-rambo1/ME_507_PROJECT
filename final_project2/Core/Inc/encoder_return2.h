/*
 * encoder_retun.h
 *
 *  Created on: Jun 10, 2024
 *      Author: mitch
 */

#ifndef INC_ENCODER_RETURN2_H_
#define INC_ENCODER_RETURN2_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

struct {
    uint32_t angle;
    TIM_HandleTypeDef *timer;
    uint32_t channel1;
} typedef servo_t;


// Prototype for motor object "method"
void encoder_return(servo_t* p_mot, int32_t angle);


#endif /* INC_ENCODER_RETURN2_H_ */
