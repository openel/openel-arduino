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

#ifndef OPENEL_SENSOR_M5STACK_GRAY_MPU6886_HPP
#define OPENEL_SENSOR_M5STACK_GRAY_MPU6886_HPP

#define M5STACK_MPU6886

#include <M5Stack.h>
#include "Sensor.hpp"

#define MPU6886_ADDRESS            0x68
#define MPU6886_WHOAMI             0x75
#define MPU6886_ACCEL_INTEL_CTRL   0x69
#define MPU6886_SMPLRT_DIV         0x19
#define MPU6886_INT_PIN_CFG        0x37
#define MPU6886_INT_ENABLE         0x38
#define MPU6886_FIFO_WM_INT_STATUS 0x39
#define MPU6886_INT_STATUS         0x3A
#define MPU6886_ACCEL_WOM_X_THR    0x20
#define MPU6886_ACCEL_WOM_Y_THR    0x21
#define MPU6886_ACCEL_WOM_Z_THR    0x22

#define MPU6886_ACCEL_XOUT_H       0x3B
#define MPU6886_ACCEL_XOUT_L       0x3C
#define MPU6886_ACCEL_YOUT_H       0x3D
#define MPU6886_ACCEL_YOUT_L       0x3E
#define MPU6886_ACCEL_ZOUT_H       0x3F
#define MPU6886_ACCEL_ZOUT_L       0x40

#define MPU6886_TEMP_OUT_H         0x41
#define MPU6886_TEMP_OUT_L         0x42

#define MPU6886_GYRO_XOUT_H        0x43
#define MPU6886_GYRO_XOUT_L        0x44
#define MPU6886_GYRO_YOUT_H        0x45
#define MPU6886_GYRO_YOUT_L        0x46
#define MPU6886_GYRO_ZOUT_H        0x47
#define MPU6886_GYRO_ZOUT_L        0x48

#define MPU6886_USER_CTRL          0x6A
#define MPU6886_PWR_MGMT_1         0x6B
#define MPU6886_PWR_MGMT_2         0x6C
#define MPU6886_CONFIG             0x1A
#define MPU6886_GYRO_CONFIG        0x1B
#define MPU6886_ACCEL_CONFIG       0x1C
#define MPU6886_ACCEL_CONFIG2      0x1D
#define MPU6886_FIFO_EN            0x23

#define MPU6886_FIFO_ENABLE        0x23
#define MPU6886_FIFO_COUNT         0x72
#define MPU6886_FIFO_R_W           0x74

//#define G (9.8)
#define RtA     57.324841
#define AtR     0.0174533
#define Gyro_Gr 0.0010653

#define MPU6886_DATA_NUM 14

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

enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

class SensorM5StackGrayMPU6886 : public Sensor
{
private:
    static bool calibration_mode;
    static std::string strDevName;
    static std::vector<std::string> strFncLst;
    static uint8_t Acscale;
    static uint8_t Gyscale;
    static float aRes;
    static float gRes;
    static int16_t accAdcX;
    static int16_t accAdcY;
    static int16_t accAdcZ;
    static int16_t gyroAdcX;
    static int16_t gyroAdcY;
    static int16_t gyroAdcZ;
    static int16_t tempAdc;
    static float accDataX;
    static float accDataY;
    static float accDataZ;
    static float gyroDataX;
    static float gyroDataY;
    static float gyroDataZ;
    static float tempData;
    static uint8_t fifo_buff[1024];

    static int init_MPU6886();
    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer);
    static void getGres();
    static void getAres();
    static void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az);
    static void getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz);
    static void getTempAdc(int16_t* t);
    static void getAccelData(float* ax, float* ay, float* az);
    static void getGyroData(float* gx, float* gy, float* gz);
    static void getTempData(float* t);
    static void readFIFO(uint8_t *dataBuff, uint16_t length);
    static uint16_t readFIFOCount();

public:
    static Property SensorM5StackGrayMPU6886_property;

    static ReturnCode fncInit(HALComponent *pHALComponent);
    static ReturnCode fncReInit(HALComponent *pHALComponent);
    static ReturnCode fncFinalize(HALComponent *pHALComponent);
    static ReturnCode fncAddObserver(HALComponent *pHALComponent, HALObserver **halObserver);
    static ReturnCode fncRemoveObserver(HALComponent *pHALComponent, HALObserver **halObserver);
    static ReturnCode fncGetProperty(HALComponent *pHALComponent, Property **property);
    static ReturnCode fncGetTime(HALComponent *pHALComponent, unsigned int **timeValue);
    static ReturnCode fncGetValLst(HALComponent *pHALComponent, float **valueList, int **num);
    static ReturnCode fncGetTimedValLst(HALComponent *pHALComponent, float **valueList, unsigned int **time, int **num);
    static ReturnCode fncSetValue(HALComponent *pHALComponent, int request, float value);
    static ReturnCode fncGetValue(HALComponent *pHALComponent, int request, float **value);
    static ReturnCode fncNop(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd);
    static ReturnCode fncDeviceVendorSpec(HALComponent* pHALComponent, HAL_ARGUMENT_T *pCmd, HAL_ARGUMENT_DEVICE_T *pCmdDev);

};

extern HAL_FNCTBL_T HalSensorM5StackGrayMPU6886Tbl;

#endif // OPENEL_SENSOR_M5STTACK_GRAY_MPU6886_HPP
