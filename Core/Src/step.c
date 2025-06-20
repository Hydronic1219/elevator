

#include <step.h>

#include "delay_us.h"

static const uint8_t HALF_STEP[8][4]=
{
		{1, 0, 0, 0},
		{1, 1, 0, 0},
		{0, 1, 0, 0},
		{0, 1, 1, 0},
		{0, 0, 1, 0},
		{0, 0, 1, 1},
		{0, 0, 0, 1},
		{1, 0, 0, 1}
};

volatile uint8_t stopFlag = 0;

void stopStepper()
{
	stopFlag = 1;
}

void stepMotor(uint8_t step)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, HALF_STEP[step][0]);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, HALF_STEP[step][1]);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, HALF_STEP[step][2]);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, HALF_STEP[step][3]);


}

//CW = 내려감  CCW = 올라감

void rotateSteps(uint16_t steps, uint8_t direction) // 반시계 위로 시계 아래로
{
	for(uint16_t i = 0; i < steps; i++)
	{
		if (stopFlag) break;
		//회전방향에 따라 스텝패턴
		uint8_t step;
		if(direction == DIR_CW)
		{
			step = i % 8;  //시계

		}
		else
		{
			step = 7 - (i % 8);
		}
		stepMotor(step);

		delay_us(1000);
	}

}
void rotateDegrees(uint16_t degrees, uint8_t direction)
{
	// 각도에 해당하는 스텝수 계산
	uint16_t steps = (uint16_t)((uint32_t)(degrees * STEPS_PER) / 360);

	rotateSteps(steps, direction);

}

