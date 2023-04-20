/*
 * bno055_wrapper.h
 *
 *  Created on: Apr 5, 2023
 *      Author: emilycrowl
 */

#ifndef SRC_BNO055_WRAPPER_H_
#define SRC_BNO055_WRAPPER_H_

#include "bno055.h"

#define BNO055_API

struct bno055_accel_t read_raw_accel(void);
struct bno055_mag_t read_raw_mag(void);
struct bno055_gyro_t read_raw_gyro(void);
struct bno055_euler_t read_raw_euler(void);
struct bno055_quaternion_t read_raw_quaternion(void);
struct bno055_linear_accel_t read_raw_lin_accel(void);
struct bno055_gravity_t read_raw_gravity(void);
struct bno055_accel_double_t read_convert_accel(void);
struct bno055_mag_double_t read_convert_mag(void);
struct bno055_gyro_double_t read_convert_gyro(void);
struct bno055_euler_double_t read_convert_euler(void);
struct bno055_linear_accel_double_t read_convert_lin_accel(void);
struct bno055_gravity_double_t read_convert_gravity(void);
void bno055_init_ic2(void);
s32 bno055_deinit(void);

#endif /* SRC_BNO055_WRAPPER_H_ */
