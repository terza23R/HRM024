#include "stm32f4xx.h"
#include "lcd_driver.h"
#include "delay.h"
#include "usart.h"


uint8_t clc; 	// variable to read SR1 and SR2 registers


void I2C_Conf(void){
RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 

// I2C software reset
I2C1->CR1 |= I2C_CR1_SWRST;
I2C1->CR1 &= ~(I2C_CR1_SWRST);


GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	
GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;     	// Open Drain
GPIOB->OSPEEDR |= 0xA000;
GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0; 	// Pull-up
GPIOB->AFR[0] |= 0x44000000;								// AF4 for I2C1 on pins PB6 and PB7

// Konfiguracija registara za I2C
I2C1->OAR1 |= 0x4000;
I2C1->CR1 &= ~I2C_CR1_PE; 	
I2C1->CR2 = 0x0010; 		// 16MHz tact, can go to 42Mhz
I2C1->CCR = 0x50;			// T=1/F=1/100khz=0.01ms,Ton=0.01ms/2=5000ns because we want the duty cycle to be 0.5
							// Tpclk=1/16Mhz=62.5ns,  CCR=5000ns/62.5=80 or  CCR=16Mhz/2*100khz
I2C1->TRISE = 0x11; 		// PCF8574 requires that the max rise time does not exceed 1000ns,so 1000ns/62.5=16 and we add 1 to ensure TRISE value is at least 1

I2C1->CR1 |= I2C_CR1_PE; 	
}


void I2C_Start(){

	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){} 		

}

void I2C_Write(uint8_t data){
	while(!(I2C1->SR1 & I2C_SR1_TXE)){} 		
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}		

}

void I2C_Send_Addr(uint8_t Addr){
    I2C1->DR = Addr;  						
    while(!(I2C1->SR1 & I2C_SR1_ADDR)){}	
    clc = (I2C1->SR1 | I2C1->SR2);			
}
void I2C_Stop(){
    I2C1->CR1 |= I2C_CR1_STOP;				
	while(I2C1->CR1 & I2C_CR1_STOP){}
}



// RS(Register select),0 - instruction register, 1 - data register , RW(Read/Write), 0 - write, 1 - read, 
// E(Enable), need to pulse the E pin so the command is saved and processed
// B(Backlight), 1 - on 0 - off
void LCD_Write_Cmd8bit(uint8_t DevAddr,uint8_t data){ 
uint8_t data_h,d1,d2;
data_h=(data & 0xF0);
d1=data_h | 0x04; 		// Blacklight_enable,	E enable  D7 D6 D5 D4 B E RW RS 
d2=data_h | 0x00; 		// Same just without the enable

I2C_Start();
I2C_Send_Addr(DevAddr);	

I2C_Write(d1);
I2C_Write(d2);

I2C_Stop();
}

void LCD_Write_Cmd4bit(uint8_t DevAddr,uint8_t data)
{
	uint8_t data_h,data_l,d1,d2,d3,d4;
	data_h=(data & 0xF0);
	data_l=((data<<4) & 0xF0);

	d1=data_h | 0x0C; // 1100
	d2=data_h | 0X08;	

	d3=data_l | 0X0C;
	d4=data_l | 0X08;

	I2C_Start();
	I2C_Send_Addr(DevAddr); 		

	I2C_Write(d1);
	delay_ms(2);
	I2C_Write(d2);


	I2C_Write(d3);
	delay_ms(2);
	I2C_Write(d4);

	I2C_Stop();
}

void LCD_Write_Data(uint8_t DevAddr,uint8_t data)
{
	uint8_t data_h,data_l,d1,d2,d3,d4;
	data_h=(data & 0xF0);
	data_l=((data<<4) & 0xF0);

	d1=data_h | 0X0D;
	d2=data_h | 0X09;	

	d3=data_l | 0X0D;
	d4=data_l | 0X09;

	I2C_Start();
	I2C_Send_Addr(DevAddr); 			

	I2C_Write(d1);
	delay_ms(2);
	I2C_Write(d2);


	I2C_Write(d3);
	delay_ms(2);
	I2C_Write(d4);

	I2C_Stop();
}


void LCD_Init(){
	delay_ms(100);
        LCD_Write_Cmd8bit(LCD_ADDR,0x30);
	delay_ms(5);
	LCD_Write_Cmd8bit(LCD_ADDR,0x30);
	delay_ms(1);
	LCD_Write_Cmd8bit(LCD_ADDR,0x30);
	delay_ms(1);
	LCD_Write_Cmd8bit(LCD_ADDR,0x20);

	delay_ms(1);
	LCD_Write_Cmd4bit(LCD_ADDR,0x28);
	delay_ms(1);
	LCD_Write_Cmd4bit(LCD_ADDR,0x08);
	delay_ms(1);
	LCD_Write_Cmd4bit(LCD_ADDR,0x01);
	delay_ms(1);
	LCD_Write_Cmd4bit(LCD_ADDR,0x06);
	delay_ms(1);
	LCD_Write_Cmd4bit(LCD_ADDR,0x0F);

}


void LCD_Cursor(int r, int c){
    if (r==1){
        c |= 0xC0;
        LCD_Write_Cmd4bit(LCD_ADDR,c);
    }
    else{
        c |= 0x80;
        LCD_Write_Cmd4bit(LCD_ADDR,c);
    }
}

void LCD_Send_String (char *str)
{
	uint8_t cnt=0;
	while (*str)
	{
	if(cnt==16)
	{
	LCD_Cursor(1,0);
	}

	else if(cnt>=32){
	delay_ms(500);
	LCD_Write_Cmd4bit(LCD_ADDR,LCD_CLEAR_DISPLAY);
	LCD_Cursor(0,0);
	cnt=0;
	}

	LCD_Write_Data(LCD_ADDR,*str++);
	cnt++;
	}
}
void LCD_Clear(void)
{
    LCD_Write_Cmd4bit(LCD_ADDR,0x01);
    delay_ms(2);
}

