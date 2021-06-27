/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "bme280_defs.h"
#include "SDS011.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ESP_CMD_INDEX UINT8_C(4)
#define LCD_CMD_INDEX UINT8_C(2)
#define RTC_DATA_NUM_FORM UINT8_C(2)
#define RTC_DATA_FIELD UINT8_C(7)
#define RTC_DATA_STATUS_BYTE UINT8_C(0)
#define SDS_FRAME (9)
#define LCD_LEVEL_DATA_INDEX (6)
#define DATA_AQI_INDEX (0)
#define ESP_DATA_FIELD (6)
#define ESP_DATA_NUM_FORM (4)
#define MAX_COUNTER_VARIABLES (10)
#define FLAG_RAISING (1)
#define FLAG_FALLING (0)
#define TIME_FORMAT_SECOND (0)
#define TIME_FORMAT_MINUTE (1)
#define TIME_FORMAT_HOUR (2)
#define TIME_FORMAT_DAY (3)
#define FLAG_RAISE (1)
#define FLAG_DEFAULT (0)
#define MEASURE_COUNT (20)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hua;
struct bme280_dev dev;
struct bme280_data cdata;
uint32_t countloop = 0;
uint8_t ib = 0;
uint8_t CNT_Measure = 0;
float pm25 = 0, pm10 = 0;
uint8_t level = 0;
uint32_t req_time = 0;
uint16_t SDS_OFFSET = 100;
uint32_t TIME_UNIT[] = {1, 60, 3600, 86400};
uint16_t PM_MAX1 = 0, PM_MIN1 = 500; 
uint16_t PM_MAX2 = 0, PM_MIN2 = 500; 
uint8_t COMMAND_ID = 0;

//Interrupt UART Variables 
uint8_t Rx_DataReg1;
uint8_t Rx_DataReg2;
uint8_t Rx_DataReg3;
volatile uint8_t index1 = 0;
volatile uint8_t index2 = 0;
volatile uint8_t index3 = 0;

//Buffers 
uint8_t buffer1[100];
char buffer2[100];
char buffer3[100];
char message1[100]; 
char message2[100];
char rtc_data[100];
char esp_data[100];
char wifi_ssid[100];
char wifi_pass[100];
uint8_t sds_data[10];
uint16_t data_outdoor[10];

//Flags
uint8_t ESP_BUSY = 1;
uint8_t ESP_ISCONNECTED = 0;
uint8_t FLAG_SDS = 1;
uint8_t FLAG_RTC = 1;
uint8_t FLAG_SENSORS_READ = 1;
uint8_t FLAG_REQUEST_DATA_STM = 1;
uint8_t FLAG_ACTIVE_SENSORS = 1;
uint16_t checksum = 0;
uint8_t checksumFlag = 0;

//Couter variables
uint32_t counter[MAX_COUNTER_VARIABLES] = {0};
uint8_t hcnt[MAX_COUNTER_VARIABLES] = {0}; //Handle for counter registration
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// I2C delay/read/write for driver BME

int fputc(int c, FILE *f)
{
	HAL_UART_Transmit(&hua, (uint8_t*)&c, 1, 0x00FF);
	return 0;
}
uint8_t USER_ATOI(char c)
{
	if ((c > 0x2F) && (c < 0x3A)) {
		return (c - 48);
	}
	else {
		return 0;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if (huart->Instance == huart2.Instance) 
	{
		
		if (index2 == 0) memset(&buffer2, 0, 100);
		switch (index2) 
		{
			case 0: 
				if (Rx_DataReg2 == '#') {buffer2[index2++] = Rx_DataReg2; }
				break;
			case 1:
				if (Rx_DataReg2 == 'E') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			case 2:
				if (Rx_DataReg2 == 'S') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			case 3:
				if (Rx_DataReg2 == 'P') {buffer2[index2++] = Rx_DataReg2; }
				else index2 = 0;
				break;
			default:
				if (Rx_DataReg2 != 0x0D) buffer2[index2++] = Rx_DataReg2;
				else 
				{
					// check buffer1[ESP_CMD_INDEX] for commander code:
					// 0 is State ESP in progress (Busy) 
					// 1 is State IsConnected Wifi
					// 2 is RTC data from NTP server
					// 3 is DataMessage from Local Network ESP, then copy to Message1[100]
					switch (buffer2[ESP_CMD_INDEX])
					{
						case '0': ESP_BUSY = 0; 
							break;
						case '1': 
							if (buffer2[ESP_CMD_INDEX + 1] == '1') 
								ESP_ISCONNECTED = 1 ;
							else 
								ESP_ISCONNECTED = 0; 
							break;
						case '2':
							FLAG_RTC = 0;
							for (int i = 0; i < index2 - ESP_CMD_INDEX - 1; i++) 
								rtc_data[i] = buffer2[i + ESP_CMD_INDEX + 1];
							break;
						case '3': 
							for (int i = 0; i < index2 - ESP_CMD_INDEX - 1; i++) 
								message1[i] = buffer2[i + ESP_CMD_INDEX + 1];
							break;
						case '4': //CMD
							COMMAND_ID = buffer2[ESP_CMD_INDEX + 1];
							switch (COMMAND_ID)
							{
								case '0': HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); break;
								case '1': HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); break;
								case '2': HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); break;							
							}
							break;
						case '5':
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
							FLAG_REQUEST_DATA_STM = 0;
					}
					index2 = 0;
				}
		}
		HAL_UART_Receive_IT(&huart2, &Rx_DataReg2, 1);
	}
	
	
	if (huart->Instance == huart1.Instance)
	{
		if (index1 == 0) memset(&buffer1, 0 , 100);
		switch (index1)
		{
			case 0:
				if (Rx_DataReg1 == 0xAA) {buffer1[index1++] = Rx_DataReg1;}
				break;
			case 1: 
				if (Rx_DataReg1 == 0xC0) {buffer1[index1++] = Rx_DataReg1;}
				else index1 = 0;
				break;
			default:
				if (index1  < SDS_FRAME) 
				{
					buffer1[index1] = Rx_DataReg1;
					if (index1 != 8) {checksum += Rx_DataReg1;}
					else if (Rx_DataReg1 == (checksum % 256)) checksumFlag = 1;
					index1++;
				}
				if (index1 == SDS_FRAME && Rx_DataReg1 == 0xAB)
				{
					if (checksumFlag)
					{
						for (int i = 0; i < 4; i++)  // Take 4 byte data of PM2.5 & PM10 
						{
							sds_data[i] = buffer1[i+2];
						}
						FLAG_SDS = 0;
					}
					index1 = 0;
				}
		}			
		HAL_UART_Receive_IT(&huart1, &Rx_DataReg1, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == htim3.Instance)
	{
		for(int i = 0; i < MAX_COUNTER_VARIABLES; i++)
		{
			if(hcnt[i] == 1) counter[i]++;
		}
	}
}
void PARSE_TIME(char time[RTC_DATA_FIELD][RTC_DATA_NUM_FORM], char *data)
{
	uint8_t n = 0;
	for(int i = 1; i < RTC_DATA_FIELD * RTC_DATA_NUM_FORM + 1;)  // Byte 1 of RTC_data for Status
	{
		for(int j = 0; j < RTC_DATA_NUM_FORM; j++)
		{
			time[n][j] = data[i++];
		}
		n++;
	}
}
uint8_t BcdToDec(uint8_t Num) {
	return ((Num / 16) * 10 + (Num % 16)); 
}

uint8_t DecToBcd(uint8_t Num) {
	return ((Num / 10) * 16 + (Num % 10)); 
}


uint16_t USER_ROUND(float Num)
{
	int a = (int)Num;
	if (Num <= 0) return 0;
	if (Num - a >= 0.5){
		return (a + 1);
	}
	else{
		return a;
	}
}



void SEND_DATA_THINGSPEAK(volatile uint8_t *esp_status, volatile uint8_t *flag, uint16_t *data, uint8_t field)
{
	if (*esp_status == 0 && *flag == 0)
	{
		char str[100] = {0};
		char buf[10] = {0};
		for(int i = 0; i < field; i++)
		{
			sprintf(buf, "%d%c", data[i], ';');
			strcat(str, buf);
		}
		hua  = huart2;
		printf("#3DATA;%s\r", str);
//		hua  = huart3;
//		printf("#3DATA;%s\r", str);
		
		*esp_status = 1;
		*flag = 1;
	}
}	

void ESP_RESET()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}
uint16_t getAqi(float C, uint8_t type) 
{
  float I, I_hi, I_low;
  float  C_hi, C_low;
  if (type == 25) {
    if (C > 0 && C < 15.4) {
      C_hi  = 15.4;
      C_low = 0.0;
      I_hi  = 50;
      I_low = 0;
    }
    else if (C < 40.4) {
      C_hi  = 40.4;
      C_low = 15.5;
      I_hi  = 100;
      I_low = 51;
    }
    else if (C < 65.4) {
      C_hi  = 65.4;
      C_low = 40.5;
      I_hi  = 150;
      I_low = 101;
    }
    else if (C < 150.4) {
      C_hi  = 150.4;
      C_low = 65.5;
      I_hi  = 200;
      I_low = 151;
    }
    else if (C < 250.4) {
      C_hi  = 250.4;
      C_low = 150.5;
      I_hi  = 300;
      I_low = 201;
    }
    else if (C < 350.4) {
      C_hi  = 350.4;
      C_low = 250.5;
      I_hi  = 400;
      I_low = 301;
    }
    else if (C < 500.4) {
      C_hi  = 500.4;
      C_low = 350.5;
      I_hi  = 500;
      I_low = 401;
    }
  }
  if (type == 10) {
    if (C > 0 && C < 54) {
      C_hi  = 54;
      C_low = 0.0;
      I_hi  = 50;
      I_low = 0;
    }
    else if (C < 154) {
      C_hi  = 154;
      C_low = 55;
      I_hi  = 100;
      I_low = 51;
    }
    else if (C < 254) {
      C_hi  = 254;
      C_low = 155;
      I_hi  = 150;
      I_low = 101;
    }
    else if (C < 354) {
      C_hi  = 354;
      C_low = 255;
      I_hi  = 200;
      I_low = 151;
    }
    else if (C < 424) {
      C_hi  = 424;
      C_low = 355;
      I_hi  = 300;
      I_low = 201;
    }
    else if (C < 504) {
      C_hi  = 504;
      C_low = 425;
      I_hi  = 400;
      I_low = 301;
    }
    else if (C < 604) {
      C_hi  = 604;
      C_low = 505;
      I_hi  = 500;
      I_low = 401;
    }
    else return 500;
  }

  I = ((I_hi - I_low) / (C_hi - C_low) * (C - C_low)) + I_low;
  return (uint16_t)I;
}
// I2C delay/read/write for driver BME
void USER_DELAY(uint32_t period)
{
 HAL_Delay(period);
}
int8_t USER_READ(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	int8_t rslt = 0;
	status = HAL_I2C_Mem_Read(&hi2c1, (uint8_t)(dev_id<<1), (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 10);
	if (status != HAL_OK) rslt = 1;
	return rslt;
}
int8_t USER_WRITE(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	int8_t rslt = 0;
	status = HAL_I2C_Mem_Write(&hi2c1, (uint8_t)(dev_id<<1), (uint8_t)reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100);
	if (status != HAL_OK) rslt = 1;
	return rslt;
}

int8_t BME280_UINIT(struct bme280_dev *pdev)
{
	int8_t rslt = BME280_OK;
	pdev -> dev_id = BME280_I2C_ADDR_PRIM;
	pdev -> intf = BME280_I2C_INTF;
	pdev -> read = USER_READ;
	pdev -> write = USER_WRITE;
	pdev -> delay_ms = USER_DELAY;

	rslt = bme280_init(pdev);
	return rslt;
}
int8_t BME280_SETTING(struct bme280_dev *pdev)
{
	int8_t rslt;
	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	pdev->settings.osr_h = BME280_OVERSAMPLING_1X;
	pdev->settings.osr_p = BME280_OVERSAMPLING_16X;
	pdev->settings.osr_t = BME280_OVERSAMPLING_2X;
	pdev->settings.filter = BME280_FILTER_COEFF_16;
	pdev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, pdev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, pdev);
	
	return rslt;
}
int8_t BME280_READ_DATA(struct bme280_data *pcomp_data, struct bme280_dev *pdev)
{
	int8_t rslt;
	rslt = bme280_get_sensor_data(BME280_ALL, pcomp_data, pdev);
	return rslt;
}

void FLAG_ROUTINE(uint8_t flag0, uint16_t cnt, uint8_t cycletime)
{
	
}

int8_t BME280_SETTING_FORCE_MODE(struct bme280_dev *dev, struct bme280_data *comp_data, uint32_t *req_delay)
{
	int8_t rslt;
	uint8_t settings_sel;
	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, dev);
	
	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
	 *  and the oversampling configuration. */
	*req_delay = bme280_cal_meas_delay(&dev->settings);
	return rslt;
}
void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
	uint8_t settings_sel;
	uint32_t req_delay;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	bme280_set_sensor_settings(settings_sel, dev);

	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
	 *  and the oversampling configuration. */
	req_delay = bme280_cal_meas_delay(&dev->settings);

	printf("Temperature, Pressure, Humidity\r\n");
	/* Continuously stream sensor data */
	while (1) {
			bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
			/* Wait for the measurement to complete and print data @25Hz */
			dev->delay_ms(req_delay);
			bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
			print_sensor_data(&comp_data);
		HAL_Delay(1000);
	}
}
void LCD_CMD(char *cmd)
{
//	hua = huart3;
//	printf("%s", cmd);
//	printf("%c%c%c", 0xFF, 0xFF, 0xFF);
}

void SDS011_USER_READ(uint8_t *flag, float *pm25, float *pm10)
{
	if (*flag == 0)
	{
		*pm25 = (float)(sds_data[1] << 8);
		*pm25 += (float)sds_data[0];
		*pm25 /= 10.0;
		*pm10 = (float)(sds_data[3] << 8);
		*pm10 += (float)sds_data[2];
		*pm10 /= 10.0;
		*flag = 1;
	}
}
void FLAG_UPDATE_STATUS(uint8_t *flag, uint8_t counter_id, uint8_t type_action, uint8_t time_format, uint16_t period)
{
	/*Type Action: 	0 for FALLING (1 -> 0)
									1 for RASING (0 -> 1)
									
		Time Format:	0 - Second
									1 - Minute
									2 - Hour
									3 - Day
		TIME_UNIT[] = {1, 60, 3600, 86400}
	*/
	if(counter_id < MAX_COUNTER_VARIABLES)
	{
		if(hcnt[counter_id] == 0) hcnt[counter_id] = 1;  // Flag Registration 
		if(counter[counter_id] >= period * TIME_UNIT[time_format])
		{
			if(type_action == FLAG_RAISING) *flag = 1;
			else *flag = 0;
			counter[counter_id] = 0; // Reset counter
		}
	}

}
void READ_SENSORS(uint8_t *flag1, uint8_t *flag2, uint8_t *cnt, struct bme280_dev *pdev, struct bme280_data *pdata, float *pm25, float *pm10, uint8_t *flagsds, uint16_t pulse, uint16_t *stored_data)
{
	if(*flag1 == 0 && *flag2 == 0)
	{
		if (*cnt == 0) 
		{
			SDS011_WAKEUP();
//			hua = huart3;
//			printf("\nWakeup SDS\n");
		}
		bme280_set_sensor_mode(BME280_FORCED_MODE, pdev);
		HAL_Delay(req_time);
//		hua = huart3;
//		printf("\n%d\n", *cnt);
		
		BME280_READ_DATA(pdata, pdev);
		SDS011_USER_READ(flagsds, pm25, pm10);	
		stored_data[0] = USER_ROUND(*pm25);
		stored_data[1] = USER_ROUND(*pm10);
		stored_data[2] = USER_ROUND(pdata->temperature);
		stored_data[3] = USER_ROUND(pdata->humidity);
		stored_data[4] = USER_ROUND((pdata->pressure)/100);
		stored_data[5] = USER_ROUND(ib);
		
//		hua = huart3;
//		printf("BME280 DATA: Temp = %.1f /Humi = %.1f /Press = %.1f\n", pdata->temperature, pdata->humidity, pdata->pressure/100);
//		printf("SDS011 DATA: AQI = %d/ PM2.5 = %.1f /PM10 = %.1f\n\n", getAqi(*pm25, 25), *pm25, *pm10);
		*flag2 = 1;
		if (*cnt == MEASURE_COUNT)
		{
//			hua = huart3;
//			printf("\nSLEEP SDS\n");
			SDS011_SLEEP();
			*cnt = 0;
			*flag1 = 1;
		}
		else *cnt = *cnt + 1; 
	}
}
void RESPONSE_DATA(uint8_t *flag, uint16_t *data)
{
	if (*flag == 0)
	{
		char str[100] = {0};
		char buf[10] = {0};
		for(int i = 0; i < 6; i++)
		{
			sprintf(buf, "%d%c", data[i], ';');
			strcat(str, buf);
		}
		hua  = huart2;
		printf("#DATA;%s\r", str);
//		hua  = huart3;
//		printf("#DATA;%s\r", str);
		
		*flag = 1;
		ib += 10;
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	//---------------------------------------------------------
	countloop++;
	ESP_RESET();
	HAL_Delay(2000);
	SDS011_SET_UART(&huart1);
	HAL_UART_Receive_IT(&huart1, &Rx_DataReg1, 1);
	HAL_UART_Receive_IT(&huart2, &Rx_DataReg2, 1);
//	HAL_UART_Receive_IT(&huart3, &Rx_DataReg3, 1);
	
	// INIT & SETUP ESP 
	hua = huart2;
	printf("import dev\r");
	
	//INIT BME280
//	hua = huart3;
//	printf("Init BME\n");
	BME280_UINIT(&dev);
//	hua = huart3;
//	printf("Init BME OK\n");
//	BME280_SETTING(&dev);
	BME280_SETTING_FORCE_MODE(&dev, &cdata, &req_time);

	
	//Start Timer for OS
	HAL_TIM_Base_Start_IT(&htim3);
	uint16_t slave_data[10] = {0};
	FLAG_SENSORS_READ = 0;
	FLAG_ACTIVE_SENSORS = 0;
	//---------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		// Read data from Sensors
		
		FLAG_UPDATE_STATUS(&FLAG_ACTIVE_SENSORS, 0, FLAG_FALLING, TIME_FORMAT_SECOND, 60); 
		FLAG_UPDATE_STATUS(&FLAG_SENSORS_READ, 1, FLAG_FALLING, TIME_FORMAT_SECOND, 1);
		READ_SENSORS(&FLAG_ACTIVE_SENSORS, &FLAG_SENSORS_READ, &CNT_Measure, &dev, &cdata, &pm25, &pm10, &FLAG_SDS, 80, slave_data);

		//Send Data when ESP request 
		RESPONSE_DATA(&FLAG_REQUEST_DATA_STM, slave_data);
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
