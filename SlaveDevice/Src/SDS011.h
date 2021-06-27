//
// Desc: Driver of SDS011-sensor version 2.18 for STM32
// Auth: Nguyen Trong Anh , Email: Nguyenhoanganh.hust@gmail.com
//

#include "stm32f1xx_hal.h"

void SDS011_SET_UART(UART_HandleTypeDef *huartSDS);
int SDS011_READ(float *p25, float *p10);
void SDS011_SLEEP(void);
void SDS011_WAKEUP(void);
