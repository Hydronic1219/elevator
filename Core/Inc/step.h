/*
 * stepper.h
 *
 *  Created on: Jun 18, 2025
 *      Author: user21
 */

#ifndef INC_STEP_H_
#define INC_STEP_H_

#include "main.h"




#define STEPS_PER				4096		//HALF STEP
#define DIR_CW					0			// 시계
#define DIR_CCW					1			// 반시계

#define IN1_Pin					GPIO_PIN_1
#define IN1_GPIO_Port			GPIOB
#define IN2_Pin					GPIO_PIN_15
#define IN2_GPIO_Port			GPIOB
#define IN3_Pin					GPIO_PIN_14
#define IN3_GPIO_Port			GPIOB
#define IN4_Pin					GPIO_PIN_13
#define IN4_GPIO_Port			GPIOB

void stepMotor(uint8_t step);
void rotateSteps(uint16_t steps, uint8_t direction);
void rotateDegrees(uint16_t degrees, uint8_t direction);










#endif /* INC_STEP_H_ */
