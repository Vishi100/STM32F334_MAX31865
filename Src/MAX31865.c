/**
  ******************************************************************************
  * File Name          : MAX31865.c
  * Description        : Driver code for MAX31865 RTD converter 
  ******************************************************************************
  */

/* BEGIN Includes */
#include "stm32f3xx_hal.h"
#include "MAX31865.h"
/* END Includes */

/* BEGIN Private Variables */
extern SPI_HandleTypeDef hspi1;
struct __attribute__((packed)) var_max31865
{
  uint16_t rtd_res_raw;			// RTD IC raw resistance register
  uint8_t  status;					// RTD status - full status code
  uint8_t  conf_reg;				// Configuration register readout
  uint16_t  HFT_val;				// High fault threshold register readout
  uint16_t  LFT_val;				// Low fault threshold register readout
};

struct var_max31865 rtd_data;
double tmp;
/* END Private Variables */

/* Function to configure MAX31865 */
void configure_MAX31865(uint8_t Sensor)
{
	uint8_t config_reg_write[] = {CONFIG_REG_WRITE_ADDR, CONFIG_REG_WRITE_VALUE};
	
	/* (1) : SPI Transmit, write to config reg on address 0x80 */
	// Step(1): Bring the CS pin low to activate the slave device
	if(Sensor == TEMPERATURE_SENSOR0)
	{
		CS0_ENABLE
	}
	else
	{
		CS1_ENABLE
	}
	HAL_Delay(10); //delay to initially setup the configuration register
	// Step(2): Transmit config reg address  & data
	HAL_SPI_Transmit(&hspi1, &config_reg_write[0], 1, TIMEOUT_VAL);
	HAL_SPI_Transmit(&hspi1, &config_reg_write[1], 1, TIMEOUT_VAL);
	// Step(3): Bring the CS pin high again
	if(Sensor == TEMPERATURE_SENSOR0)
	{
		CS0_DISABLE
	}
	else
	{
		CS1_DISABLE
	}
	// give the sensor time to set up
	HAL_Delay(100);
}

/* Function to unpack and store MAX31865 data */
void MAX31865_full_read(uint8_t Sensor)
{
	uint8_t config_reg_read_addr = CONFIG_REG_READ_ADDR;
	uint8_t read_data[8]; //array to store register contents
	uint8_t i = 0; //loop variable
	
	HAL_Delay(500); //delay for each cycle of read operation
	/* (2) : SPI Receive, read the contents of all registers starting from config reg on address 0x00 */
	// Step(1): Bring the CS pin low to activate the slave device
	if(Sensor == TEMPERATURE_SENSOR0)
	{
		CS0_ENABLE
	}
	else
	{
		CS1_ENABLE
	}
	// Step(2): Transmit config reg address telling IC that we want to 'read' and start at register 0
	HAL_SPI_Transmit(&hspi1, &config_reg_read_addr, 1, TIMEOUT_VAL);
	/* Step (3): Receive the first 8 bits (Config reg data) */
	for(i = 0; i < 8; i++)
	{
		HAL_SPI_Receive(&hspi1, &read_data[i], 1, TIMEOUT_VAL);
	}
	// Step(4): Bring the CS pin high again
	if(Sensor == TEMPERATURE_SENSOR0)
	{
		CS0_DISABLE
	}
	else
	{
		CS1_DISABLE
	}
	/* Step (5): Store the data read from the sensor */
	rtd_data.conf_reg = read_data[0]; //Store config reg data
	rtd_data.rtd_res_raw = ((read_data[1] << 8) | read_data[2]) >> 1; // Store rtd_res_raw
	rtd_data.HFT_val = ((read_data[3] << 8) | read_data[4]) >> 1; // Store HFT_val
	rtd_data.LFT_val = (read_data[5] << 8) | read_data[6]; // Store LFT_val
	rtd_data.status = read_data[7]; //Store fault status reg data	
}

void calculate_temperature(void)
{
	/* calculate RTD temperature (simple calc, +/- 2 deg C from -100C to 100C) */
	/* more accurate curve can be used outside that range */
	tmp = ((double)rtd_data.rtd_res_raw / 32) - 256 - TMP_ERROR;
}

double MAX31865_main(uint8_t Sensor)
{
	configure_MAX31865(Sensor);
	MAX31865_full_read(Sensor);
	calculate_temperature();
	
	return tmp;
}
/**********************************END OF FILE***********************************/
