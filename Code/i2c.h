/* 
 * File:   i2c.h
 *  simple i2c driver adapted for ESE 3500 WS3: Serial
 *  Note: this is an adaption of code I wrote previously and modified quickly
 *          to meet the needs of this assignment. If you are looking at this for
 *          your final project, note your project needs and particular i2c.c 
 *          implementation will drive what functions you need, trying to 
 *          recreate these functions may be insufficient, unnecessary, or incorrect
 * 
 * Author: James Steeman
 * 
 * Created on November 17, 2024
 * Updated Feb 2025
 */

#include <stdint.h>

#ifndef I2C_H
#define I2C_H

/* Function Declarations */

void I2C_start();

void I2C_repStart();

void I2C_stop();

void I2C_writeBegin(uint8_t addr);

void I2C_readBegin(uint8_t addr);

void I2C_writeRegister(uint8_t addr, uint8_t data, uint8_t reg);

void I2C_readRegister(uint8_t addr, uint8_t* data_addr, uint8_t reg);

void I2C_writeStream(uint8_t* data, int len);

void I2C_readStream(uint8_t* data_addr, int len);

void I2C_init();

void ERROR();

void I2C_writeCompleteStream(uint8_t *dataArrPtr, uint8_t *addrArrPtr, int len, uint8_t addr);

void I2C_readCompleteStream(uint8_t* dataArrPtr, uint8_t addr, uint8_t reg, int len);

#endif