/*
 * MIT License
 *
 * Copyright (c) 2017 M5Stack
 * Copyright (c) 2021 JASA(Japan Embedded Systems Technology Association)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __IMU__H_
#define __IMU__H_

#include "Arduino.h"

#define ACC_8G_RES (0.000244140625f)
#define GYRO_2000DPS_RES (0.06103515625f)

typedef union {
  uint8_t raw[14];
  struct {
    uint8_t acc_x_h;
    uint8_t acc_x_l;
    uint8_t acc_y_h;
    uint8_t acc_y_l;
    uint8_t acc_z_h;
    uint8_t acc_z_l;
    uint8_t temp_h;
    uint8_t temp_l;
    uint8_t gyro_x_h;
    uint8_t gyro_x_l;
    uint8_t gyro_y_h;
    uint8_t gyro_y_l;
    uint8_t gyro_z_h;
    uint8_t gyro_z_l;
  } value;
} ImuData_t;


float getAngle();
void ImuTaskStart(int16_t x_offset, int16_t y_offset, int16_t z_offset, SemaphoreHandle_t* i2c_lock);

#endif
