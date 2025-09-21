/**
 * Copyright (c) 2024 Your Name
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// Device I2C addresses
const uint8_t ACCEL_MAG_ADDR = 0x1D;
const uint8_t GYRO_ADDR = 0x6B;

// Register addresses
const uint8_t CTRL_REG1_G = 0x20;
const uint8_t CTRL_REG1_XM = 0x20; 
const uint8_t CTRL_REG5_XM = 0x24; 
const uint8_t CTRL_REG7_XM = 0x26; // New register for magnetometer mode

const uint8_t OUT_X_L_A = 0x28;
const uint8_t OUT_X_L_M = 0x08;
const uint8_t OUT_X_L_G = 0x28;

// Function to write a byte to a register, now with error checking
int reg_write(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    if (i2c_write_blocking(I2C_PORT, addr, buf, 2, false) == PICO_ERROR_GENERIC) {
        return 1;
    }
    return 0;
}

// Function to read a block of data, now with error checking
int reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (i2c_write_blocking(I2C_PORT, addr, &reg, 1, true) == PICO_ERROR_GENERIC) {
        return 1;
    }
    if (i2c_read_blocking(I2C_PORT, addr, buf, len, false) == PICO_ERROR_GENERIC) {
        return 1;
    }
    return 0;
}

const double g = 9.81;

uint8_t data[6];
int16_t accel_raw[3], gyro_raw[3], mag_raw[3];
double accel[3], gyro[3], mag[3];

const double lsb_acc = 0.000061 * g;    // m/s^2
const double lsb_gyro = 0.00875;        // deg/s
const double lsb_mag = 0.00008;         // gause

void read_imu(void);
void parse_imu(double lsb_acc, double lsb_gyro, double lsb_mag);
void stream_imu(void);


int main() {
    stdio_init_all();
    // Wait 5 seconds before starting
    sleep_ms(5000); 

    // --- I2C Initialization ---
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // --- Sensor Initialization with Error Checking ---
    reg_write(GYRO_ADDR, CTRL_REG1_G, 0x0F);       // Enable Gyroscope
    reg_write(ACCEL_MAG_ADDR, CTRL_REG1_XM, 0x67); // Enable Accelerometer
    reg_write(ACCEL_MAG_ADDR, CTRL_REG5_XM, 0x74); // High-res magnetometer
    reg_write(ACCEL_MAG_ADDR, CTRL_REG7_XM, 0x00); // Set magnetometer to continuous mode

    // Print CSV header once
    printf("timestamp_us,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,gyro_x,gyro_y,gyro_z\n");


    uint16_t imu_sample_rate = 100; // Hz
    uint32_t imu_sample_time = 0;
    uint32_t now_time = 0;


    while (true) {
        now_time = time_us_32();

        if (now_time - imu_sample_time >= 1000000/imu_sample_rate) {
            imu_sample_time = time_us_32();
            
            read_imu();
            parse_imu(lsb_acc, lsb_gyro, lsb_mag);
            stream_imu();

        }
    
    }
    return 0;
}




void read_imu(void){
    // --- Read all sensors ---
    bool accel_ok = reg_read(ACCEL_MAG_ADDR, OUT_X_L_A | 0x80, data, 6) == 0;
    if (accel_ok) {
        accel_raw[0] = (data[1] << 8) | data[0];
        accel_raw[1] = (data[3] << 8) | data[2];
        accel_raw[2] = (data[5] << 8) | data[4];
    }

    bool mag_ok = reg_read(ACCEL_MAG_ADDR, OUT_X_L_M | 0x80, data, 6) == 0;
    if (mag_ok) {
        mag_raw[0] = (data[1] << 8) | data[0];
        mag_raw[1] = (data[3] << 8) | data[2];
        mag_raw[2] = (data[5] << 8) | data[4];
    }

    bool gyro_ok = reg_read(GYRO_ADDR, OUT_X_L_G | 0x80, data, 6) == 0;
    if (gyro_ok) {
        gyro_raw[0] = (data[1] << 8) | data[0];
        gyro_raw[1] = (data[3] << 8) | data[2];
        gyro_raw[2] = (data[5] << 8) | data[4];
    }
}

void parse_imu(double lsb_acc, double lsb_gyro, double lsb_mag) {

    accel[0] = lsb_acc * accel_raw[0];
    accel[1] = lsb_acc * accel_raw[1];
    accel[2] = lsb_acc * accel_raw[2];

    gyro[0] = lsb_gyro * gyro_raw[0];
    gyro[1] = lsb_gyro * gyro_raw[1];
    gyro[2] = lsb_gyro * gyro_raw[2];

    mag[0] = lsb_mag * mag_raw[0];
    mag[1] = lsb_mag * mag_raw[1];
    mag[2] = lsb_mag * mag_raw[2];

}

void stream_imu(void) {
    // timestamp in microseconds
    uint32_t ts = time_us_32();

    // Primary: print as floating point CSV (easy to read)
    // Note: some pico/newlib builds strip float support from printf.
    // If you get garbage or no decimals, use the integer fallback below.
    printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           (unsigned long)ts,
           accel[0], accel[1], accel[2],
           mag[0], mag[1], mag[2],
           gyro[0], gyro[1], gyro[2]);
}

