/**
  ******************************************************************************
  * File Name          : MAX31865.h
  * Description        : Header file for MAX31865 RTD converter driver code
  ******************************************************************************
  */

#ifndef __MAX31865_H
#define __MAX31865_H

/* BEGIN Includes */
#include "stm32f3xx_hal.h"

#define TEMPERATURE_SENSOR0 0
#define TEMPERATURE_SENSOR1	1

/* Macros to enable & disable CS pin */
#define CS0_ENABLE		do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); } while(0);
#define CS0_DISABLE		do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); } while(0);
#define CS1_ENABLE		do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); } while(0);
#define CS1_DISABLE		do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); } while(0);

#define TMP_ERROR 2 // Temperature Sensor error 2 degree celsius
#define CONFIG_REG_WRITE_ADDR 	0x80U // Configuration register write address
#define CONFIG_REG_READ_ADDR 		0x00U // Configuration register read address
#define CONFIG_REG_WRITE_VALUE 	0xC2U // VBIAS on, Auto conversion mode, 2-Wire, Fault status clear, 60 Hz filter

/* SPI TIMEOUT Value*/
#define TIMEOUT_VAL 60
/* END Includes */

/* BEGIN Private function prototypes */
void configure_MAX31865(uint8_t Sensor);
void MAX31865_full_read(uint8_t Sensor);
void calculate_temperature(void);
double MAX31865_main(uint8_t Sensor);
/* END Private function prototypes */

#endif
/**********************************END OF FILE***********************************/
