#include "spi.h"

void initSPI2(uint16_t prescaler)
{/// init SPI2 in master mode
	///wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	/// SPI2 Pin configuration
	///-----------------------------------------------------------------
	/// PB13 = SCK
	/// PA14 = MISO
	/// PA15 = MOSI
	///wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 								//
	GPIOB->MODER &= ~((GPIO_MODER_MODER13)|(GPIO_MODER_MODER14)|(GPIO_MODER_MODER15));
	GPIOB->MODER |= (GPIO_MODER_MODER13_1)|(GPIO_MODER_MODER14_1)|(GPIO_MODER_MODER15_1); 
	GPIOB->AFR[1] |= 0x55500000;										//   
	GPIOB->OSPEEDR |= 0xA8000000;										// 
	
	
	GPIOB->MODER &= ~(GPIO_MODER_MODER12);
	GPIOB->MODER |= (GPIO_MODER_MODER12_0); 
	GPIOB->OTYPER &= ~(0x03000000);
	GPIOB->OSPEEDR |= 0x02000000;										// 
	SPI2_CS_HIGH;
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; 								// 	
	SPI2->CR1 = (SPI_CR1_MSTR);											// enable 8 bit data & master mode			
	SPI2->CR1 |= (SPI_CR1_SSI)|(SPI_CR1_SSM);
	
	SPI2->CR1 |= prescaler;		
	SPI2->CR1 |= (SPI_CR1_SPE);											// 
}

uint8_t rxByteSPI2(void)
{///  receive data using SPI2
	uint8_t data;
	
	SPI2->DR = 0x00;						 							// 
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE)); 								// 
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE)); 							// 
	while(SPI2->SR & SPI_I2S_FLAG_BSY); 								//	
	data = SPI2->DR; 													//
	
	return data;
}	

uint8_t txByteSPI2(uint8_t data)
{/// send data using SPI2
	uint8_t tmp;

	SPI2->DR = data; 													//
	while(!(SPI2->SR & SPI_I2S_FLAG_TXE));								// 
	while(!(SPI2->SR & SPI_I2S_FLAG_RXNE)); 							// 
	while(SPI2->SR & SPI_I2S_FLAG_BSY); 								// 
	SPI2->DR; 													// 
		
	return tmp;
}

void txSPI2(uint8_t * data, uint16_t size)
{
	uint16_t k;
	for(k=0;k<size;k++)
	{
		txByteSPI2(data[k]);
	}
}

void rxSPI2(uint8_t * data, uint16_t size)
{
	uint16_t k;
	for(k=0;k<size;k++)
	{
		data[k] = rxByteSPI2();
	}
}
