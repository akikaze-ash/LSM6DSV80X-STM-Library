# if defined STM32G474xx
#include "stm32g4xx_hal.h"
# else
# if defined STM32H523xx
#include "stm32h523xx"
# endif
# endif
#include "imu_sensor_read.h"
  
uint8_t regGyroX1, regGyroX2, regGyroY1, regGyroY2, regGyroZ1, regGyroZ2;
uint8_t regAccelX1, regAccelX2, regAccelY1, regAccelY2, regAccelZ1, regAccelZ2;
uint8_t regTemp1, regTemp2;

void imuSetConfigRegs(I2C_HandleTypeDef *hi2c1) {
    uint8_t writePointer = CONF_RESET;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, CTRL_RESET_REG, 1, &writePointer, 1, 200);
    writePointer = GRYO_240HZ;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_CTRL_REG, 1, &writePointer, 1, 200);
    writePointer = GYRO_250DPS;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_DPS_REG, 1, &writePointer, 1, 200);
    writePointer = GYRO_ENABLE_FILTER;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_FILTER_REG, 1, &writePointer, 1, 200);
    writePointer = ACCEL_240HZ;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ACCEL_CTRL_REG, 1, &writePointer, 1, 200);
    writePointer = ACCEL_ENABLE_FILTER;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ACCEL_FILTER_REG, 1, &writePointer, 1, 200);

    // enable interrupts
    writePointer = INT1_ENABLE;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, INT1_ENABLE_REG, 1, &writePointer, 1, 200);
    writePointer = INT2_ENABLE;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, INT2_ENABLE_REG, 1, &writePointer, 1, 200);
}

float readGyroX(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroX1, regGyroX2;

    // Read gyroscope X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_X1, 1, &regGyroX1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_X2, 1, &regGyroX2, 1, 50); 
    int16_t gyroXraw = (int16_t)(regGyroX2 << 8) | regGyroX1;

    // convert to dps
    float gyroX=gyroXraw*GYRO_DPS;

    return gyroX;
}

float readGyroY(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroY1, regGyroY2;

    // Read gyroscope Y values then combine
	HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_Y1, 1, &regGyroY1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_Y2, 1, &regGyroY2, 1, 50);
    int16_t gyroYraw = (int16_t)(regGyroY2 << 8) | regGyroY1;

    // convert to dps
    float gyroY=gyroYraw*GYRO_DPS;

    return gyroY;
}

float readGyroZ(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroZ1, regGyroZ2;

    // Read gyroscope Z values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_Z1, 1, &regGyroZ1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GRYO_Z2, 1, &regGyroZ2, 1, 50);
    int16_t gyroZraw = (int16_t)(regGyroZ2 << 8) | regGyroZ1;

    // convert to dps 
    float gyroZ=gyroZraw*GYRO_DPS;

    return gyroZ;
}

float readAccelX(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelX1, regAccelX2;

    // Read accelerometers X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_X1, 1, &regAccelX1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_X2, 1, &regAccelX2, 1, 50); 
    int16_t accelXraw = (int16_t)(regAccelX2 << 8) | regAccelX1;

    // convert to mgs
    float accelX=accelXraw*ACCEL_G;

    return accelX;
}

float readAccelY(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelY1, regAccelY2;

    // Read accelerometers Y values then combine
	HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Y1, 1, &regAccelY1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Y2, 1, &regAccelY2, 1, 50);
    int16_t accelYraw = (int16_t)(regAccelY2 << 8) | regAccelY1;

    // convert to mgs
    float accelY=accelYraw*ACCEL_G;

    return accelY;
}

float readAccelZ(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelZ1, regAccelZ2;

    // Read accelerometers Y values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Z1, 1, &regAccelZ1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Z2, 1, &regAccelZ2, 1, 50);
    int16_t accelZraw = (int16_t)(regAccelZ2 << 8) | regAccelZ1;

    // convert to mgs
    float accelZ=accelZraw*ACCEL_G;

    return accelZ;
}

float readTempIMU(I2C_HandleTypeDef *hi2c1) {
    uint8_t regTemp1, regTemp2;

    // Read temperature
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_TEMP1, 1, &regTemp1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_TEMP2, 1, &regTemp2, 1, 50);
    int16_t tempRaw = (int16_t)(regTemp2 << 8) | regTemp1;

    // convert to degrees C
    float temp=tempRaw*TEMP_C+25; // this is the coversion factor dont ask

    return temp;
}