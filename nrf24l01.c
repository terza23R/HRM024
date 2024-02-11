#include "nrf24l01.h"

volatile uint8_t g_nrf24l01_node_type;
const char c_nrf_master_addr[6] = "MDR01";
const char c_nrf_slave_addr[6] = "SDR00";
volatile uint8_t * g_node_addr;

//volatile uint16_t nrf_ch[NRF24L01_MAX_CHANNEL];
volatile uint8_t nrf_mode;

void initNRF24L01(uint8_t node_type) 
{
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// PB11	- nRF24L01 CE
	// PB12	- nRF24L01 CS
	// PB13	- nRF24L01 CLK
	// PB14	- nRF24L01 MISO
	// PB15	- nRF24L01 MOSI
	//---------------------------------------------------------------------	
	uint8_t reg_val;
	
	g_nrf24l01_node_type = node_type;
	if(g_nrf24l01_node_type == (NRF24L01_NODE_TYPE_RX))
	{
		g_node_addr = (uint8_t *)c_nrf_slave_addr;
	}
	else
	{
		g_node_addr = (uint8_t *)c_nrf_master_addr;
	}
	
	
	initSPI2(SPI_BaudRatePrescaler_32);
	
	// setup CE pin as PB11
	GPIOB->MODER &= ~(GPIO_MODER_MODER11);
	GPIOB->MODER |= (GPIO_MODER_MODER11_0); 
	GPIOB->OTYPER &= ~(0x00C00000);
	GPIOB->OSPEEDR |= 0x00800000;										 
	
	NRF_CE_LOW;
	SPI2_CS_HIGH;
	delay_ms(100);
	
	setRxAddrNRF24L01((uint8_t *)g_node_addr,NRF24L01_RX_ADDR_P1);		// set Rx address 

	conRegNRF24L01(NRF24L01_RF_CH, NRF24L01_ACTIVE_CHANNEL);			// set active channel
	
	conRegNRF24L01(NRF24L01_RX_PW_P0, NRF24L01_PIPE_LENGTH);			// set length of pipe 0
	conRegNRF24L01(NRF24L01_RX_PW_P1, NRF24L01_PIPE_LENGTH);			// set length of pipe 1
	
	setRxModeNRF24L01();
	flushRxNRF24L01();
	
	
	//{
		// uint8_t k;
		//printUSART2("\n");
		//for(k=0;k<30;k++)
			//printUSART2("REG[%xb]: %xb\n",k, getRegNRF24L01(k));
					
		//delay_ms(3000);
	//}
}

void setRxAddrNRF24L01(uint8_t * addr, uint8_t reg)
{
	NRF_CE_LOW;
	setRegNRF24L01(reg, addr, 5);
	NRF_CE_HIGH;
}

void setRegNRF24L01(uint8_t reg, uint8_t * reg_val, uint8_t n_val)
{/// set register "reg" with values specified in the "reg_val" array of "n_val" elements
	uint8_t k;
	reg &= NRF24L01_REGISTER_MASK;										// apply register mask -> clear bits 7:5
	reg |= NRF24L01_W_REGISTER;											// write to W register  -> sets the bit 5
	
	SPI2_CS_LOW;
	txSPI2(&reg, 1);													
	txSPI2(reg_val, n_val); 
	SPI2_CS_HIGH;														
}

void conRegNRF24L01(uint8_t reg, uint8_t reg_val)
{
	uint8_t k;
	reg &= NRF24L01_REGISTER_MASK;										// apply register mask -> clear bits 7:5
	reg |= NRF24L01_W_REGISTER;											// write to W register  -> sets the bit 5
	
	SPI2_CS_LOW;
	txSPI2(&reg, 1);													// transmit bits 
	txSPI2(&reg_val, 1); 
	SPI2_CS_HIGH;	
}

void setRxModeNRF24L01(void)
{/// Rx mode, power up the nRF TRx, CRC One byte!
	nrf_mode = NRF_MODE_RX;
		
	conRegNRF24L01(NRF24L01_CONFIG,((1<<NRF24L01_CONFIG_EN_CRC)|(0<<NRF24L01_CONFIG_CRCO))|((1<<NRF24L01_CONFIG_POWER_UP)|(1<<NRF24L01_CONFIG_PRIM_RX)));
	NRF_CE_HIGH;
	delay_us(130);	
	conRegNRF24L01(NRF24L01_STATUS, (1<<NRF24L01_STATUS_TX_DS)|(1<<NRF24L01_STATUS_MAX_RT));
}

void flushRxNRF24L01(void)
{/// flush Rx pipe
	SPI2_CS_LOW;
	txByteSPI2(NRF24L01_FLUSH_RX);
	SPI2_CS_HIGH;
}

void setTxAddrNRF24L01(uint8_t * addr)
{
	setRegNRF24L01(NRF24L01_RX_ADDR_P0, addr, 5);
	setRegNRF24L01(NRF24L01_TX_ADDR, addr, 5);	
}

uint8_t	txOverNRF24L01(void)
{
	uint8_t status;
	if(nrf_mode == (NRF_MODE_TX))
	{
		status = getRegNRF24L01(NRF24L01_STATUS);
		if(status & ((1 << (NRF24L01_STATUS_TX_DS))|(1<<(NRF24L01_STATUS_MAX_RT))))
		{
			setRxModeNRF24L01();
			status = (NRF_TX_FINISHED); 
		}
		else
		{
			status = (NRF_TX_IN_PROGRESS); 
		}	
	}
	else
	{
		status = (NRF_TX_FINISHED); 
	}
	
	return status;
}

//uint8_t getRxDataNRF24L01(uint8_t * data)
//{
	//uint8_t k,status,reg_val, tmp;
		
	//status = getRegNRF24L01(NRF24L01_STATUS);
	//if(status&(1<<NRF24L01_STATUS_RX_DR))
	//{// data ready	
		
		//clrSS();
		//tmp = (NRF24L01_R_RX_PAYLOAD);
		//txSPI2(&tmp, 1);												// transmit command to read RX payload 
		//rxSPI2(data, (NRF24L01_PIPE_LENGTH));
		//setSS();
		//status = NRF_DATA_READY;
	//}
	//else
	//{
		//status = NRF_DATA_NOT_READY;
	//}
	//reg_val = 0xFF;
	//setRegNRF24L01(NRF24L01_STATUS,&reg_val,1);
		
	//flushRxNRF24L01();	
	
		
	//return status;		
//} 

void onNRF24L01(void)
{
	uint8_t rx = getRegNRF24L01(NRF24L01_CONFIG);
	rx |= (1<<(NRF24L01_CONFIG_POWER_UP));
	setRegNRF24L01(NRF24L01_CONFIG,&rx,1);								// switch on the nRF24L01p chip
	delay_us(130);
}

//void offNRF24L01(void)
//{
	//uint8_t rx = getRegNRF24L01(NRF24L01_CONFIG);
	//rx &= ~(1<<NRF24L01_CONFIG_POWER_UP);					
	//setRegNRF24L01(NRF24L01_CONFIG,&rx,1);								// switch off the nRF24L01p chip
//}

uint8_t getRegNRF24L01(uint8_t reg)
{///
	uint8_t rx;
	reg &= (NRF24L01_REGISTER_MASK);									// applay register mask -> clear bits 7:5
	
	SPI2_CS_LOW;
	txSPI2(&reg, 1);													// transmit 	
	rxSPI2(&rx, 1);														// receive data
	SPI2_CS_HIGH;
	return rx;
}

void flushTxNRF24L01(void)
{// flush Tx pipe
	SPI2_CS_LOW;
	txByteSPI2(NRF24L01_FLUSH_TX);
	SPI2_CS_HIGH;
}

void setTxModeNRF24L01(void)
{/// Rx mode, power up the nRF TRx, CRC One byte!
	nrf_mode = (NRF_MODE_TX);
	conRegNRF24L01(NRF24L01_CONFIG,((1<<(NRF24L01_CONFIG_EN_CRC))|(0<<(NRF24L01_CONFIG_CRCO)))|((1<<(NRF24L01_CONFIG_POWER_UP))|(0<<(NRF24L01_CONFIG_PRIM_RX))));
}

//uint8_t	getModeNRF24L01(void)
//{
	//uint8_t status = NRF_MODE_RX;
	//if(nrf_mode == NRF_MODE_TX)
	//{
		//status = getRegNRF24L01(NRF24L01_STATUS);
		
		//if((status & ((1 << NRF24L01_STATUS_TX_DS)|(1<<NRF24L01_STATUS_MAX_RT)))){
			//setRxModeNRF24L01();
			//status = NRF_MODE_RX; 
		//}
		//else
		//{
			//status = NRF_MODE_TX;
		//}
	//}
	//return status;
//}

void rxDataNRF24L01(uint8_t * data)
{
	uint8_t k, tmp;
	
	SPI2_CS_LOW;
	tmp = (NRF24L01_R_RX_PAYLOAD);
	txSPI2(&tmp , 1);												// transmit command to read RX payload 
	rxSPI2(data, (NRF24L01_PIPE_LENGTH));
	SPI2_CS_HIGH;
	
	conRegNRF24L01(NRF24L01_STATUS,(1<<(NRF24L01_STATUS_RX_DR)));
	//flushRxNRF24L01();
}

uint8_t dataReadyNRF24L01(void)
{
	uint8_t status = getRegNRF24L01(NRF24L01_STATUS);
	
	if (status & (1 << (NRF24L01_STATUS_RX_DR)))
	{
		status = (NRF_DATA_READY);
	}
	else
	{	// get Rx Fifo status
		status = getRegNRF24L01(NRF24L01_FIFO_STATUS);
		status &= (1 << NRF24L01_FIFO_STATUS_RX_EMPTY);
		if(status)
		{
			status = (NRF_DATA_NOT_READY);
		}
		else
		{
			status = (NRF_DATA_READY);
		}
	} 
	return status;
}

uint8_t txDataNRF24L01(uint8_t * daddr, uint8_t * data)
{
	uint8_t res = (NRF24L01_TX_COMPLETED);
	uint32_t timer = getSYSTIM();

	setTxAddrNRF24L01(daddr);	
	txPacketNRF24L01(data);					
	while(txOverNRF24L01() == (NRF_TX_IN_PROGRESS))
	{
		if(chk4TimeoutSYSTIM(timer, NRF24L01_TX_WAIT_PERIOD) == (SYSTIM_TIMEOUT))
		{
			res = (NRF24L01_TX_FAILED);
			break;
		}
	}
	
	return res;
}

void txPacketNRF24L01(uint8_t * data)
{
	uint8_t status, k, tmp;
	uint32_t timer = getSYSTIM();
	
	while(nrf_mode == NRF_MODE_TX)
	{
		status = getRegNRF24L01(NRF24L01_STATUS);
		if(status & ((1 << (NRF24L01_STATUS_TX_DS))|(1<<(NRF24L01_STATUS_MAX_RT))))
		{
			nrf_mode = NRF_MODE_RX; 
			break;
		}
		
		if(chk4TimeoutSYSTIM(timer, 1000) == (SYSTIM_TIMEOUT))
		{
			printUSART2("TIMEOUT\n");
			return;
		}
	}
	
	NRF_CE_LOW;															// clear CE bit
	setTxModeNRF24L01();												// set Tx mode
	
	flushTxNRF24L01();
	
	SPI2_CS_LOW;														// clear SS bit
	tmp = NRF24L01_W_TX_PAYLOAD;
	txSPI2(&tmp , 1);													// write cmd to write payload
	txSPI2(data, (NRF24L01_PIPE_LENGTH));
	SPI2_CS_HIGH;
	NRF_CE_HIGH;
	delay_us(130);														// delay >10us
}

//void updateAddrNRF24L01(uint8_t * addr_old, uint8_t * addr_new)
//{
	//uint8_t buff[NRF24L01_PIPE_LENGTH];
	//uint8_t k, kmax, n;
	
	//for(k=0;k<4;k++)
	//{
		//buff[k] = addr_old[k];
	//}
	//buff[k] = 'U';
	//k++;
	//kmax = k + 4;
	//for(n=0;k<kmax;k++,n++)
	//{
		//buff[k] = addr_new[n];
	//}
	//buff[k] = ';';
	//k++;
	//buff[k] = '\0';
	//sendNRF24L01(NRF24L01_REMOTE_ADDRESS, buff);
//}

//void setAlarmNRF24L01(uint8_t * addr, uint8_t state)
//{
	//uint8_t buff[NRF24L01_PIPE_LENGTH];
	//uint8_t k, kmax, n;
	
	//for(k=0;k<4;k++)
	//{
		//buff[k] = addr[k];
	//}
	//buff[k] = 'A';
	//k++;
	//buff[k] = state;
	//k++;
	//buff[k] = ';';
	//k++;
	//buff[k] = '\0';
	
	//sendNRF24L01(NRF24L01_REMOTE_ADDRESS, buff);
//}

//void factoryResetRemoteNRF24L01(uint8_t * addr)
//{
	//uint8_t buff[NRF24L01_PIPE_LENGTH];
	//uint8_t k, kmax, n;
	
	//for(k=0;k<4;k++)
	//{
		//buff[k] = addr[k];
	//}
	//buff[k] = 'F';
	//k++;
	//buff[k] = ';';
	//k++;
	//buff[k] = '\0';
	
	//sendNRF24L01(NRF24L01_REMOTE_ADDRESS, buff);
//}

//void resetRemoteNRF24L01(uint8_t * addr)
//{
	//uint8_t buff[NRF24L01_PIPE_LENGTH];
	//uint8_t k, kmax, n;
	
	//for(k=0;k<4;k++)
	//{
		//buff[k] = addr[k];
	//}
	//buff[k] = 'R';
	//k++;
	//buff[k] = ';';
	//k++;
	//buff[k] = '\0';
	
	//sendNRF24L01(NRF24L01_REMOTE_ADDRESS, buff);
//}
