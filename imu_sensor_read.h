#ifndef IMU_SENSOR_READ
#define IMU_SENSOR_READ
#endif



#include "main.h"

//// All constant declarations
// conversion constants 
#define TEMP_C 0.00390625
#define ACCEL_G 0.061
#define GYRO_DPS 0.00875

// device registries
#define ADDR_READ 0b11010101
#define ADDR_WRITE 0b11010100

// registries for enable writing
#define GLOBAL_REG 0x01
#define CTRL_RESET_REG 0x12
#define ACCEL_CTRL_REG 0x10
#define GYRO_CTRL_REG 0x11
#define GYRO_DPS_REG 0x15
#define GYRO_FILTER_REG 0x16
#define ACCEL_FILTER_REG 0x17

// values for enable writing
#define GLOBAL_RESET 0b00000100
#define CONF_RESET 0b00000001
#define GRYO_240HZ 0b00000111
#define GYRO_250DPS 0b00001001
#define GYRO_ENABLE_FILTER 0b00000001
#define ACCEL_240HZ 0b00000111
#define ACCEL_ENABLE_FILTER 0b00000000

// registries to enable interrupts
#define INT1_ENABLE_REG 0x0D
#define INT2_ENABLE_REG 0x0E

// values for enable interrupts
#define INT1_ENABLE 0b01111011
#define INT2_ENABLE 0b11111011

// addresses for gryo values
#define ADDR_GRYO_X1 0x22
#define ADDR_GRYO_X2 0x23
#define ADDR_GRYO_Y1 0x24
#define ADDR_GRYO_Y2 0x25
#define ADDR_GRYO_Z1 0x26
#define ADDR_GRYO_Z2 0x27

// addresses for gryo values
#define ADDR_ACCEL_X1 0x28
#define ADDR_ACCEL_X2 0x29
#define ADDR_ACCEL_Y1 0x2A
#define ADDR_ACCEL_Y2 0x2B
#define ADDR_ACCEL_Z1 0x2C
#define ADDR_ACCEL_Z2 0x2D

// addresses for temperature sensor values
#define ADDR_TEMP1 0x20
#define ADDR_TEMP2 0x21

//// IMU sensor headers

// sets the configuration registries for the IMU. do not alter
void imuSetConfigRegs(I2C_HandleTypeDef *hi2c1);

// functions for each of the gyroscope axes
float readGyroX(I2C_HandleTypeDef *hi2c1);
float readGyroY(I2C_HandleTypeDef *hi2c1);
float readGyroZ(I2C_HandleTypeDef *hi2c1);

// functions for each of the accelerometer axes
float readAccelX(I2C_HandleTypeDef *hi2c1);
float readAccelY(I2C_HandleTypeDef *hi2c1);
float readAccelZ(I2C_HandleTypeDef *hi2c1);

// function to read the imu temp sensor
float readTempIMU(I2C_HandleTypeDef *hi2c1);