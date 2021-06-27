//
// Desc: Driver of SDS011-sensor version 2.18 for STM32
// Auth: Nguyen Trong Anh , Email: Nguyenhoanganh.hust@gmail.com
//
#include "SDS011.h"

UART_HandleTypeDef uartSDS;
// Handle Uart

static uint8_t SLEEPCMD[19] = {
	0xAA,	// head
	0xB4,	// command id
	0x06,	// data byte 1
	0x01,	// data byte 2 (set mode)
	0x00,	// data byte 3 (sleep)
	0x00,	// data byte 4
	0x00,	// data byte 5
	0x00,	// data byte 6
	0x00,	// data byte 7
	0x00,	// data byte 8
	0x00,	// data byte 9
	0x00,	// data byte 10
	0x00,	// data byte 11
	0x00,	// data byte 12
	0x00,	// data byte 13
	0xFF,	// data byte 14 (device id byte 1)
	0xFF,	// data byte 15 (device id byte 2)
	0x05,	// checksum
	0xAB	// tail
};

static uint8_t WAKEUPCMD[19] = {
	0xAA,	// head
	0xB4,	// command id
	0x06,	// data byte 1
	0x01,	// data byte 2 (set mode)
	0x01,	// data byte 3 (sleep)
	0x00,	// data byte 4
	0x00,	// data byte 5
	0x00,	// data byte 6
	0x00,	// data byte 7
	0x00,	// data byte 8
	0x00,	// data byte 9
	0x00,	// data byte 10
	0x00,	// data byte 11
	0x00,	// data byte 12
	0x00,	// data byte 13
	0xFF,	// data byte 14 (device id byte 1)
	0xFF,	// data byte 15 (device id byte 2)
	0x06,	// checksum
	0xAB	// tail
};

void SDS011_SET_UART(UART_HandleTypeDef *huartSDS) 
{
	uartSDS = *huartSDS;
}

int SDS011_READ(float *p25, float *p10)
{
	uint8_t buffer = 0;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is;
	int checksum_ok = 0;
	int error = 1;
	
  do {
		HAL_UART_Receive(&uartSDS, &buffer, 1, 0x0FFF);
		value = (int)buffer;
		
		switch(len) {
			case (0): if (value != 0xAA) len = -1; break;
			case (1): if (value != 0xC0) len = -1; break;
			case (2): pm25_serial = value; checksum_is = value; break;
			case (3): pm25_serial += (value << 8); checksum_is += value; break;
			case (4): pm10_serial = value; checksum_is += value; break;
			case (5): pm10_serial += (value << 8); checksum_is += value; break;
			case (6): checksum_is += value; break;
			case (7): checksum_is += value; break;
			case (8): if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
			case (9): if (value != 171) { len = -1; }; break;
		}
		len++;
			
		if (len == 10 && checksum_ok == 1) {
			*p10 = (float)pm10_serial/10.0;
			*p25 = (float)pm25_serial/10.0;
			len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
			error = 0;
		}
	} while (error);	
	
	return error;
}

void SDS011_SLEEP(void)
{
	for (uint8_t i = 0; i < 19; i++) {
		HAL_UART_Transmit(&uartSDS, &SLEEPCMD[i], 1, 0x00FF);
	}	
	__HAL_UART_FLUSH_DRREGISTER(&uartSDS);
}

void SDS011_WAKEUP(void)
{
	for (uint8_t i = 0; i < 19; i++) {
		HAL_UART_Transmit(&uartSDS, &WAKEUPCMD[i], 1, 0x00FF);
	}	
	__HAL_UART_FLUSH_DRREGISTER(&uartSDS);
}



