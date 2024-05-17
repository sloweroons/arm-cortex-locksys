/*DS3231 low level API - Reza Ebrahimi v1.0*/
#include "ds3231.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"

/*function to transmit one byte of data to register_address on ds3231 (device_address: 0X68)*/
void time_i2c_write_single(uint8_t device_address, uint8_t register_address, uint8_t data_byte)
{
	HAL_I2C_Master_Transmit(&hi2c1, device_address, register_address, sizeof(register_address), 1000);
	HAL_I2C_Master_Transmit(&hi2c1, device_address, &data_byte, sizeof(data_byte), 1000);
}

/*function to transmit an array of data to device_address, starting from start_register_address*/
void time_i2c_write_multi(uint8_t device_address, uint8_t start_register_address, uint8_t *data_array, uint8_t data_length)
{
	for(int i = 0; i < data_length; i++) {
		HAL_I2C_Master_Transmit(&hi2c1, device_address, start_register_address+i, sizeof(start_register_address), 1000);
		HAL_I2C_Master_Transmit(&hi2c1, device_address, data_array, sizeof(data_array), 1000);
		data_array += 1;
	}
}

/*function to read one byte of data from register_address on ds3231*/
void time_i2c_read_single(uint8_t device_address, uint8_t register_address, uint8_t data_byte)
{
	HAL_I2C_Master_Transmit(&hi2c1, device_address, register_address, sizeof(register_address), 1000);
	HAL_I2C_Master_Receive(&hi2c1, device_address, &data_byte, sizeof(data_byte), 1000);
}

/*function to read an array of data from device_address*/
void time_i2c_read_multi(uint8_t device_address, uint8_t start_register_address, uint8_t *data_array, uint8_t data_length)
{
	for(int i = 0; i < data_length; i++) {
			HAL_I2C_Master_Transmit(&hi2c1, device_address, start_register_address+i, sizeof(start_register_address), 1000);
			HAL_I2C_Master_Receive(&hi2c1, device_address, data_array, sizeof(data_array), 1000);
			data_array += 1;
	}
}

/*function to initialize I2C peripheral in 100khz or 400khz*/
void ds3231_I2C_init()
{
}
