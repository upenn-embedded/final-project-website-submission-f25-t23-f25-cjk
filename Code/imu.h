/* 
 * File:   imu.h
 *  Supported IMU(s): LSM6DSO
 *  Note: all but IMU_getTemp have been defined
 * 
 * Author: James Steeman
 *
 * Created on February 22, 2025, 2:20 AM
 * Updated Oct. 1, 2025
 */

#include <stdint.h>

#ifndef IMU_H
#define	IMU_H

void IMU_init(uint8_t addr);

void IMU_getAll();

void IMU_getAccX();

void IMU_getAccY();

void IMU_getAccZ();

void IMU_getGyroX();

void IMU_getGyroY();

void IMU_getGyroZ();

// not actually defined
void IMU_getTemp();

int IMU_checkNewData();

#endif	/* LSM6DS0_H */

