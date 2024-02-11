#include "stm32f4xx.h"
#include "delay.h"
#include "usart.h"
#include "nrf24l01.h"
#include "lcd_driver.h"

void runSlaveNodeSYS(void);
uint8_t node_type = NRF24L01_NODE_TYPE_RX;

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(GPIO_MODER_MODER6);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR6_0);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~0x00000003;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    NVIC_EnableIRQ(EXTI0_IRQn);
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->IMR = EXTI_IMR_MR0;
    EXTI->EMR &= ~EXTI_EMR_MR0;
    EXTI->RTSR = EXTI_RTSR_TR0;
    EXTI->FTSR = 0x00000000;

    delay_ms(10);

    initUSART2(USART2_BAUDRATE_921600);

    I2C_Conf();
    LCD_Init();
    
    initNRF24L01(node_type);

    while (1)
    {
        if (node_type == NRF24L01_NODE_TYPE_RX)
        {
            runSlaveNodeSYS();
        }
    }

    return 0;
}

void runSlaveNodeSYS(void)
{
    uint8_t res;
    uint8_t nrf_data[NRF24L01_PIPE_LENGTH];
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= 0x55000000;
    GPIOD->OTYPER |= 0x00000000;
    GPIOD->OSPEEDR |= 0xFF000000;
    GPIOD->ODR &= ~0xF000;

    while (node_type == NRF24L01_NODE_TYPE_RX)
    {
        setTxAddrNRF24L01(c_nrf_master_addr);
        res = dataReadyNRF24L01();

        if (res == NRF_DATA_READY)
        {
            rxDataNRF24L01(nrf_data);

            LCD_Clear();
            LCD_Send_String((char *)nrf_data);
        }
    }
}


