
#ifndef INC_FND_H_
#define INC_FND_H_

#include "main.h"

typedef struct
{
	GPIO_TypeDef *stmPort;
	uint16_t	  number;
	GPIO_PinState on;
	GPIO_PinState off;
}FND_CONTROL;


void ledOne(uint8_t num);
void ledTwo();
void ledThree();
void ledOff(uint8_t num);





#endif /* INC_FND_H_ */
