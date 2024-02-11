#include <stdint.h>
#include "max.h"
#include "stm32f4xx.h"
#include "i2c.h"
#include "usart.h"
#include "delay.h"
#include "nrf24l01.h"
#include "spi.h"

uint32_t raw_Samples[BUFFER_SIZE];
float    dc_Rem_Samples[BUFFER_SIZE];
float    maf[BUFFER_SIZE];
float    butter[BUFFER_SIZE];

void runMasterNodeSYS(uint8_t avgBPM,int state);

int main(void) {
    initUSART2(USART2_BAUDRATE_921600);

    uint8_t node_type = NRF24L01_NODE_TYPE_TX;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(GPIO_MODER_MODER6);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR6_0);

    delay_ms(10);
    if ((GPIOC->IDR & 0x00000040) == 0x00000000)
    {
        // init as Tx node
        node_type = NRF24L01_NODE_TYPE_TX;
    }

    initNRF24L01(node_type);

    I2C_Conf();
    MAX_Init();
    delay_ms(1500);

    int bpmReadings[4] = {0}; // Array to store BPM readings
    int bpmcnt = 0;
    int state = 2;
    int bpm = 0;
    int averageBPM = 0;




   while(1)
   {
        MAX_GetFifoSample(raw_Samples,BUFFER_SIZE);
        dcRemoval(raw_Samples,dc_Rem_Samples,BUFFER_SIZE);
        movingAverageFilter(dc_Rem_Samples,maf,BUFFER_SIZE);
        bandpassFilter(maf,butter,0.67,8.0);


        if(raw_Samples[10]<100000)
        {   int i;
            for (i = 0; i < 4; i++)
            {
            bpmReadings[i] = 0;
            }
            bpmcnt=0;
            state=-1;

            if (node_type == NRF24L01_NODE_TYPE_TX)
            {
                runMasterNodeSYS(averageBPM,state);
            }
        }
        else
        {
            bpm=calculateBPM(butter,BUFFER_SIZE,100.0);
            int j=0;
            if(bpm!=0 && bpm<=220)
            {
                for (j = 0; j < 3; j++)
                {
                    bpmReadings[j] = bpmReadings[j + 1];
                }
            bpmReadings[3] = bpm;
            bpmcnt++;
            }



            if(bpmcnt>=4)
            {
                state=1;
                averageBPM = (bpmReadings[0] + bpmReadings[1] + bpmReadings[2] + bpmReadings[3]) / 4;
                printUSART2("AVG BPM:%d\n",averageBPM);
                printUSART2("BPM:%d\n",bpm);
                printUSART2("-------------------------\n");
                if (node_type == NRF24L01_NODE_TYPE_TX)
                {
                    runMasterNodeSYS(averageBPM,state);
                }
            }
            else
            {
                state=0;
                if (node_type == NRF24L01_NODE_TYPE_TX)
                {
                    runMasterNodeSYS(averageBPM,state);
                }
            }
        }
    }
    return 0;

}


void runMasterNodeSYS(uint8_t avgBPM,int state)
{
    char nrf_data[NRF24L01_PIPE_LENGTH] = "Vas BPM Je:  ";
    char nrf_data1[NRF24L01_PIPE_LENGTH] = "Ucitavanje...";
    char nrf_greska[NRF24L01_PIPE_LENGTH] ="Postavite prst!";
    uint8_t *p_msg = (uint8_t *)nrf_data;
    uint8_t *p_msg1 = (uint8_t *)nrf_data1;
    uint8_t *p_msg2 =(uint8_t *)nrf_greska;


    p_msg[11] = '0' + (avgBPM / 100);
    p_msg[12] = '0' + ((avgBPM / 10) % 10);
    p_msg[13] = '0' + (avgBPM % 10);

    if (p_msg[11] == '0')
    {
        p_msg[11] = ' ';
    }


	if(state==1)
    {
        txDataNRF24L01((uint8_t *)c_nrf_slave_addr, p_msg);
    }
    else if(state==-1)
    {
        txDataNRF24L01((uint8_t *)c_nrf_slave_addr, p_msg2);
    }
    else if(state==0)
    {
        txDataNRF24L01((uint8_t *)c_nrf_slave_addr, p_msg1);
    }
}


