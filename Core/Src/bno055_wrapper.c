#include "bno055.h"
#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdlib.h>
/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*   sensor data using I2C communication
*----------------------------------------------------------------------------*/
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
#define I2C_BUFFER_LEN 8
#define I2C0           5

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart2;

/* Variable used to return value of
* communication routine*/
s32 comres = BNO055_ERROR;

/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;

static struct bno055_t bno055;

static s8 I2C_routine(void);
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BNO055_delay_msek(u32 msek);

struct bno055_accel_t read_raw_accel(void) {
	/*********read raw accel data***********/
    /* structure used to read the accel xyz data */
    struct bno055_accel_t accel_xyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
    comres += bno055_read_accel_xyz(&accel_xyz);

    return accel_xyz;
}

struct bno055_mag_t read_raw_mag() {
    /*********read raw mag data***********/
    /* structure used to read the mag xyz data */
    struct bno055_mag_t mag_xyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
    comres += bno055_read_mag_xyz(&mag_xyz);

    return mag_xyz;
}

struct bno055_gyro_t read_raw_gyro() {
    /***********read raw gyro data***********/
    /* structure used to read the gyro xyz data */
    struct bno055_gyro_t gyro_xyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
    comres += bno055_read_gyro_xyz(&gyro_xyz);

    return gyro_xyz;
}

struct bno055_euler_t read_raw_euler() {
    /*************read raw Euler data************/
    /* structure used to read the euler hrp data */
    struct bno055_euler_t euler_hrp;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
    comres += bno055_read_euler_hrp(&euler_hrp);

    return euler_hrp;
}

struct bno055_quaternion_t read_raw_quaternion() {
    /************read raw quaternion data**************/
    /* structure used to read the quaternion wxyz data */
    struct bno055_quaternion_t quaternion_wxyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);

    return quaternion_wxyz;
}

struct bno055_linear_accel_t read_raw_lin_accel(void) {
    /************read raw linear acceleration data***********/
    /* structure used to read the linear accel xyz data */
    struct bno055_linear_accel_t linear_acce_xyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);

    return linear_acce_xyz;
}

struct bno055_gravity_t read_raw_gravity() {
    /*****************read raw gravity sensor data****************/
    /* structure used to read the gravity xyz data */
    struct bno055_gravity_t gravity_xyz;
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    comres += bno055_read_gravity_xyz(&gravity_xyz);

    return  gravity_xyz;
}

struct bno055_accel_double_t read_convert_accel(void) {
    /*************read accel converted data***************/
    /* structure used to read the accel xyz data output as m/s2*/
    struct bno055_accel_double_t d_accel_xyz;
    comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);

    return d_accel_xyz;
}

struct bno055_mag_double_t read_convert_mag() {
    /******************read mag converted data********************/
    /* structure used to read the mag xyz data output as uT*/
    struct bno055_mag_double_t d_mag_xyz;
    comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);

    return d_mag_xyz;
}

struct bno055_gyro_double_t read_convert_gyro() {
    /*****************read gyro converted data************************/
    /* structure used to read the gyro xyz data output as dps or rps */
    struct bno055_gyro_double_t d_gyro_xyz;
    // comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
    comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

    return d_gyro_xyz;
}

struct bno055_euler_double_t read_convert_euler() {
    /*******************read euler converted data*******************/
    /* structure used to read the euler hrp data output as as degree or radians */
    struct bno055_euler_double_t d_euler_hpr;
    // comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
    comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);

    return d_euler_hpr;
}

struct bno055_linear_accel_double_t read_convert_lin_accel(void) {
    /*********read linear acceleration converted data**********/
    /* structure used to read the linear accel xyz data output as m/s2*/
    struct bno055_linear_accel_double_t d_linear_accel_xyz;
    comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);

    return d_linear_accel_xyz;
}

struct bno055_gravity_double_t read_convert_gravity() {
    /********************Gravity converted data**********************/
    /* structure used to read the gravity xyz data output as m/s2*/
    struct bno055_gravity_double_t d_gravity_xyz;
    comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);

    return d_gravity_xyz;
}

void bno055_init_ic2() {
    I2C_routine();
    comres = bno055_init(&bno055);
    power_mode = BNO055_POWER_MODE_NORMAL;
    comres += bno055_set_power_mode(power_mode);
}

s32 bno055_deinit() {
	power_mode = BNO055_POWER_MODE_SUSPEND;
	comres += bno055_set_power_mode(power_mode);
	return comres;
}

s8 I2C_routine(void) {
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    return BNO055_INIT_VALUE;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    BNO055_iERROR = HAL_I2C_Mem_Write(&hi2c3, dev_addr << 1, reg_addr, 1, reg_data, cnt, 1000);
    return (s8)BNO055_iERROR;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    BNO055_iERROR = HAL_I2C_Mem_Read(&hi2c3, dev_addr << 1, reg_addr, 1, reg_data, cnt, 1000);
    return (s8)BNO055_iERROR;
}

void BNO055_delay_msek(u32 msek) { HAL_Delay(msek); }
