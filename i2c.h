#ifndef __I2C_H_
#define __I2C_H_


void I2C_Conf(void);
void I2C_Start();
void I2C_Send8bit(uint8_t data);
void I2C_Send_Addr(uint8_t Addr);
void I2C_Stop();
uint8_t I2C_Receive8bit(uint8_t ack);

#endif
