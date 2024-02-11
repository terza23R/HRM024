# HRM024
My colleagues and I were tasked with creating a device using the STM32F407 microcontroller. 

Our idea was to implement the mentioned microcontroller as a device for monitoring heart rate, i.e., a Heart Rate Monitor (HRM). In addition to the ability to measure pulse, the device transmits data, i.e., the measured values, to a remote device (another STM32F407 microcontroller). The MAX30102 was used to determine the heart rate, and the NRF24L01 was used for data transmission between devices. Additionally, a 1602 LCD I2C display was used to show the received values. 

The entire code was written in the C programming language.
