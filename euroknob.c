#include "ch32v003fun.h"
#include "lib_i2c.h"
#include <stdio.h>

#define AS5600_ADDR      0x36
#define AS5600_RAW_ANGLE 0x0C
#define AS5600_MAGNITUDE 0x1B

int main()
{
	SystemInit();

	if (i2c_init(I2C_CLK_100KHZ) != I2C_OK) printf("Failed to init I2C");
	Delay_Ms(100);

	while(1){
		uint8_t buf[2];
		if (i2c_read(AS5600_ADDR, AS5600_RAW_ANGLE, buf, 2) == I2C_OK)
			printf("Read %04d ", (buf[0]<<8)+ buf[1]);

		if (i2c_read(AS5600_ADDR, AS5600_MAGNITUDE, buf, 2) == I2C_OK)
			printf("/ %04d\n", (buf[0]<<8)+ buf[1]);

		Delay_Ms(200);
	}

/*
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

	GPIOC->CFGLR &= ~((0xf<<(4*6)) | (0xf<<(4*5)) | (0xf<<(4*4)) | (0xf<<(4*3)));
	GPIOC->CFGLR |=((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*6))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3));
*/

}
