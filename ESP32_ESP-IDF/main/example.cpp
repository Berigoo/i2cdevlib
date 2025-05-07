/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include <cstdint>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"

#define PIN_SDA 21
#define PIN_CLK 22

uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU

i2c_master_bus_handle_t handle;
soft_i2c_master_bus_t handle2;

static void dmpPrintYPR(MPU6050 &mpu, uint8_t *fifoBuffer,
                          uint16_t &fifoCount, uint16_t packetSize,
                          const char *name);

void task_initI2C(void *ignore) {
       {
           i2c_master_bus_config_t i2c_mst_config = {};
           i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
           i2c_mst_config.i2c_port = I2C_NUM_0;
           i2c_mst_config.scl_io_num = GPIO_NUM_4; 
           i2c_mst_config.sda_io_num = GPIO_NUM_5;
           i2c_mst_config.glitch_ignore_cnt = 7;
           i2c_mst_config.flags.enable_internal_pullup = true;

           ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &handle));
       }
       {
           soft_i2c_master_config_t conf{};
           conf.scl_pin = GPIO_NUM_12;
           conf.sda_pin = GPIO_NUM_13;
           conf.freq = SOFT_I2C_200KHZ;

           ESP_ERROR_CHECK(soft_i2c_master_new(&conf, &handle2));
       }

	// vTaskDelete(NULL);
}

void task_display(void *) {
        task_initI2C(nullptr);
     
        MPU6050 mpu(handle, 400000, false);
        MPU6050 mpu2(handle2, true);

	mpu.initialize();
        mpu.dmpInitialize();
        mpu2.initialize();
	mpu2.dmpInitialize();

	// This need to be setup individually
	// mpu.setXGyroOffset(220);
	// mpu.setYGyroOffset(76);
	// mpu.setZGyroOffset(-85);
	// mpu.setZAccelOffset(1788);
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu2.CalibrateAccel(6);
        mpu2.CalibrateGyro(6);

        mpu.setDMPEnabled(true);
	mpu2.setDMPEnabled(true);

	while(1){
          dmpPrintYPR(mpu, fifoBuffer, fifoCount, packetSize, "MPU 1");
          dmpPrintYPR(mpu2, fifoBuffer1, fifoCount1, packetSize, "MPU 2");
	}

	vTaskDelete(NULL);
}

void dmpPrintYPR(MPU6050 &mpu, uint8_t *fifoBuffer, uint16_t &fifoCount,
                uint16_t packetSize, const char *name) {
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    // 
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    printf("[%s] YAW: %3.1f, ", name, ypr[0] * 180/M_PI);
    printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
    printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
  }
}
