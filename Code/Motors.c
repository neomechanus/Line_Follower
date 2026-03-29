/*
 * Motors.c
 *
 *  Created on: Mar 3, 2026
 *      Author: maria
 */
#include "main.h"
#include "Motors.h"


#define MAX_SPEED 1000

void set_lmotor_speed(uint16_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    TIM2->CCR1 = speed;

}

void set_rmotor_speed(uint16_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    TIM4->CCR4 = speed;
}
int set_tmotor_speed(uint16_t turbine_speed);


void init_left_driver(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
}

void init_right_driver(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

}
