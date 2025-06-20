
#include "fnd.h"


FND_CONTROL fnd[7] =
	{
			{GPIOC, GPIO_PIN_9, 0 , 1},  //7
			{GPIOB, GPIO_PIN_8, 0 , 1},  //6
			{GPIOB, GPIO_PIN_9, 0 , 1},  //4
			{GPIOA, GPIO_PIN_5, 0 , 1},  //2
			{GPIOA, GPIO_PIN_6, 0 , 1},	 //1
			{GPIOA, GPIO_PIN_7, 0 , 1},  //9
			{GPIOC, GPIO_PIN_11, 0 , 1},  //10
	};



void ledOne(uint8_t num)
{
	for(uint8_t i=1; i < num; i++)
	{
		HAL_GPIO_WritePin(fnd[i].stmPort, fnd[i].number, fnd[i].on);
	}
}

void ledTwo()
{
	HAL_GPIO_WritePin(fnd[0].stmPort, fnd[0].number, fnd[0].on);
	HAL_GPIO_WritePin(fnd[1].stmPort, fnd[1].number, fnd[1].on);
	HAL_GPIO_WritePin(fnd[3].stmPort, fnd[3].number, fnd[3].on);
	HAL_GPIO_WritePin(fnd[4].stmPort, fnd[4].number, fnd[4].on);
	HAL_GPIO_WritePin(fnd[6].stmPort, fnd[6].number, fnd[6].on);

}

void ledThree()
{
	HAL_GPIO_WritePin(fnd[0].stmPort, fnd[0].number, fnd[0].on);
	HAL_GPIO_WritePin(fnd[1].stmPort, fnd[1].number, fnd[1].on);
	HAL_GPIO_WritePin(fnd[3].stmPort, fnd[3].number, fnd[3].on);
	HAL_GPIO_WritePin(fnd[2].stmPort, fnd[2].number, fnd[2].on);
	HAL_GPIO_WritePin(fnd[6].stmPort, fnd[6].number, fnd[6].on);

}

void ledOff(uint8_t num)
{
	for(uint8_t i=0; i < num; i++)
	{
		HAL_GPIO_WritePin(fnd[i].stmPort, fnd[i].number, fnd[i].off);
	}
}







