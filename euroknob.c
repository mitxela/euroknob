#include "ch32v003fun.h"
#include "lib_i2c.h"
#include <stdio.h>

#define AS5600_ADDR      0x36
#define AS5600_RAW_ANGLE 0x0C
#define AS5600_MAGNITUDE 0x1B

// I2C on PC1,PC2
// ADC PA1
// LEDs on PD6,PD5,PD4,PD3,PD2, PC7,PC6,PC5,PC4,PC3

void draw_bargraph(uint16_t v)
{
	uint8_t outD = 0xFF, outC=0xFF;
	if (v > 205) outD &=~(1<<6);
	if (v > 614) outD &=~(1<<5);
	if (v >1024) outD &=~(1<<4);
	if (v >1433) outD &=~(1<<3);
	if (v >1842) outD &=~(1<<2);
	if (v >2252) outC &=~(1<<7);
	if (v >2662) outC &=~(1<<6);
	if (v >3071) outC &=~(1<<5);
	if (v >3481) outC &=~(1<<4);
	if (v >3890) outC &=~(1<<3);

	GPIOD->OUTDR = outD;
	GPIOC->OUTDR = outC;
	Delay_Us(50);
	GPIOD->OUTDR |= 0xFF;
	GPIOC->OUTDR |= 0xFF;
}

int main()
{
	SystemInit();

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD;

	GPIOC->CFGLR &= ~((0xf<<(4*7)) | (0xf<<(4*6)) | (0xf<<(4*5)) | (0xf<<(4*4)) | (0xf<<(4*3)));
	GPIOC->CFGLR |=((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*7))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*5))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3));

	GPIOD->CFGLR &= ~((0xf<<(4*6)) | (0xf<<(4*5)) | (0xf<<(4*4)) | (0xf<<(4*3)) | (0xf<<(4*2)));
	GPIOD->CFGLR |=((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*5))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3))
				 | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*2));

	GPIOD->OUTDR |= 0xFF;
	GPIOC->OUTDR |= 0xFF;

	if (i2c_init(I2C_CLK_100KHZ) != I2C_OK) printf("Failed to init I2C");
	Delay_Ms(100);

	while(1){
		uint8_t buf[2];
		uint16_t raw_angle=0, magnitude=0;

		if (i2c_read(AS5600_ADDR, AS5600_RAW_ANGLE, buf, 2) == I2C_OK)
			raw_angle = (buf[0]<<8)+ buf[1];

		if (i2c_read(AS5600_ADDR, AS5600_MAGNITUDE, buf, 2) == I2C_OK)
			magnitude = (buf[0]<<8)+ buf[1];

		draw_bargraph(raw_angle);

		//printf("Read %04d / %04d \n", raw_angle, magnitude);

		Delay_Ms(10);
	}

/*
*/

}
