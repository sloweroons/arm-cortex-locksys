#include "mfrc522.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"

/*
 * mfrc522.c
 *
 *  Created on: May 22, 2024
 *      Author: abel
 */

//#define MFRC522_ADDRESS 0x7e

void MFRC522_init() {

}

void MFRC522_write() {

}

void MFRC522_read() {
	// Transcieve
	uint8_t data[1] = "5";
	HAL_SPI_Transmit(&hspi1, data, sizeof(data), 1000);

	// Receive
	uint8_t receive[1] = "6";

}


