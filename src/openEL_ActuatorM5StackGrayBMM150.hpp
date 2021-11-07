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

#ifndef OPENEL_ACTUATOR_M5STACK_GRAY_BMM150_HPP
#define OPENEL_ACTUATOR_M5STACK_GRAY_BMM150_HPP

#include <M5Stack.h>
#include "Actuator.hpp"

#define MOTOR_ADDRESS    0x3A
#define MOTOR_SPEED_L    0x00
#define MOTOR_SPEED_R    0x02
#define MOTOR_ENCODER_L  0x10
#define MOTOR_ENCODER_R  0x14

#define MOTOR_LENGTH_SPEED   (2)
#define MOTOR_LENGTH_ENCODER (4)

enum Instance {
    MOTOR_LEFT = 1,
    MOTOR_RIGHT,
    InstanceNum
};

class ActuatorM5StackGrayBMM150 : public Actuator
{
private:
    static std::string strDevName;
    static std::vector<std::string> strFncLst;

    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer);
    static void getEncoder(int32_t* wheel, uint8_t sw);
    static void setEncoder(int32_t wheel, uint8_t sw);
    static void getSpeed(int16_t* sp, uint8_t sw);
    static void setSpeed(int16_t sp, uint8_t sw);

public:
    static Property ActuatorM5StackGrayBMM150_property;

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

extern HAL_FNCTBL_T HalActuatorM5StackGrayBMM150Tbl;

#endif // OPENEL_ACTUATOR_M5STTACK_GRAY_BMM150_HPP
