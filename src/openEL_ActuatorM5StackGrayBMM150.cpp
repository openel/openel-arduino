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

#include "openEL_ActuatorM5StackGrayBMM150.hpp"

std::string ActuatorM5StackGrayBMM150::strDevName = "M5StackGrayBMM150";

std::vector<std::string> ActuatorM5StackGrayBMM150::strFncLst =
{
    "Init",
    "ReInit",
    "Finalize",
    "AddObserver",
    "RemoveObserver",
    "SetValue",
    "GetValue"
};

Property ActuatorM5StackGrayBMM150::ActuatorM5StackGrayBMM150_property;

ReturnCode ActuatorM5StackGrayBMM150::fncInit(HALComponent *pHALComponent)
{
    Wire.begin(21, 22);

    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncReInit(HALComponent *pHALComponent)
{
    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncFinalize(HALComponent *pHALComponent)
{
    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncAddObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncRemoveObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncGetProperty(HALComponent *pHALComponent, Property **property)
{
    ActuatorM5StackGrayBMM150_property.deviceName = strDevName;
    ActuatorM5StackGrayBMM150_property.functionList = strFncLst;
    (**property).functionList.resize(strFncLst.size());
    **property = ActuatorM5StackGrayBMM150_property;

    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncGetTime(HALComponent *pHALComponent, unsigned int **timeValue)
{
    **timeValue = (unsigned int)millis();

    return HAL_OK;
}

ReturnCode ActuatorM5StackGrayBMM150::fncGetValLst(HALComponent *pHALComponent, float **valueList, int **num)
{
    return HAL_ERROR;
}

ReturnCode ActuatorM5StackGrayBMM150::fncGetTimedValLst(HALComponent *pHALComponent, float **valueList, unsigned int **timeValue, int **num)
{
    return HAL_ERROR;
}

ReturnCode ActuatorM5StackGrayBMM150::fncSetValue(HALComponent *pHALComponent, int request, float value)
{
    ReturnCode retCode = HAL_OK;

    if (pHALComponent->hALId.instanceId >= InstanceNum)
    {
        return HAL_ERROR;
    }

    switch (request)
    {
    case HAL_REQUEST_POSITION_CONTROL:
        setEncoder((int32_t)value, pHALComponent->hALId.instanceId);
        break;
    case HAL_REQUEST_VELOCITY_CONTROL:
        setSpeed((int16_t)value, pHALComponent->hALId.instanceId);
        break;
    case HAL_REQUEST_TORQUE_CONTROL:
        break;
    default:
        retCode = HAL_ERROR;
        break;
    }

    return retCode;
}

ReturnCode ActuatorM5StackGrayBMM150::fncGetValue(HALComponent *pHALComponent, int request, float **value)
{
    int32_t wheel;
    int16_t sp;
    ReturnCode retCode = HAL_OK;

    if (pHALComponent->hALId.instanceId >= InstanceNum)
    {
        return HAL_ERROR;
    }

    switch (request)
    {
    case HAL_REQUEST_POSITION_CONTROL:
        getEncoder(&wheel, pHALComponent->hALId.instanceId);
        **value = (float)wheel;
        break;
    case HAL_REQUEST_VELOCITY_CONTROL:
        getSpeed(&sp, pHALComponent->hALId.instanceId);
        **value = (float)sp;
        break;
    case HAL_REQUEST_TORQUE_CONTROL:
        break;
    default:
        retCode = HAL_ERROR;
        break;
    }

    return retCode;
}

ReturnCode ActuatorM5StackGrayBMM150::fncNop(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd)
{
    return HAL_ERROR;
}

ReturnCode ActuatorM5StackGrayBMM150::fncDeviceVendorSpec(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd, HAL_ARGUMENT_DEVICE_T *pCmdDev)
{
    return HAL_ERROR;
}

HAL_FNCTBL_T HalActuatorM5StackGrayBMM150Tbl =
{
    ActuatorM5StackGrayBMM150::fncInit,			/* 0x00 */
    ActuatorM5StackGrayBMM150::fncReInit,			/* 0x01 */
    ActuatorM5StackGrayBMM150::fncFinalize,		/* 0x02 */
    ActuatorM5StackGrayBMM150::fncAddObserver,		/* 0x03 */
    ActuatorM5StackGrayBMM150::fncRemoveObserver,		/* 0x04 */
    ActuatorM5StackGrayBMM150::fncGetProperty,		/* 0x05 */
    ActuatorM5StackGrayBMM150::fncGetTime,		/* 0x06 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x07 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x08 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x09 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0A */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0B */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0C */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0D */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0E */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x0F */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x10 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x11 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x12 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x13 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x14 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x15 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x16 */
    ActuatorM5StackGrayBMM150::fncNop,			/* 0x17 */
    ActuatorM5StackGrayBMM150::fncSetValue,		/* 0x18 */
    ActuatorM5StackGrayBMM150::fncGetValue,		/* 0x19 */
    ActuatorM5StackGrayBMM150::fncGetValLst,	/* 0x1A */
    ActuatorM5StackGrayBMM150::fncGetTimedValLst,	/* 0x1B */
    ActuatorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1C */
    ActuatorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1D */
    ActuatorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1E */
    ActuatorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1F */
};

void ActuatorM5StackGrayBMM150::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
{
    uint8_t i = 0;

    Wire.beginTransmission(driver_Addr);  // Initialize the Tx buffer
    Wire.write(start_Addr);               // Put slave register address in Tx buffer
    Wire.endTransmission(false);
    Wire.requestFrom(driver_Addr, number_Bytes);

    //! Put read results in the Rx buffer
    while (Wire.available())
    {
        read_Buffer[i++] = Wire.read();
    }
}

void ActuatorM5StackGrayBMM150::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer)
{
    uint8_t i = 0;

    Wire.beginTransmission(driver_Addr);  // Initialize the Tx buffer
    Wire.write(start_Addr);               // Put slave register address in Tx buffer
    Wire.endTransmission(false);
    Wire.requestFrom(driver_Addr, number_Bytes);

    //! Put read results in the Rx buffer
    while (Wire.available())
    {
        read_Buffer[i++] = Wire.read();
    }
}

void ActuatorM5StackGrayBMM150::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
{
    uint8_t i;

    Wire.beginTransmission(driver_Addr);  // Initialize the Tx buffer
    Wire.write(start_Addr);               // Put slave register address in Tx buffer
    for(i = 0; i < number_Bytes; i++)
    {
        Wire.write(*(write_Buffer+i));    // Put data in Tx buffer
    }
    Wire.endTransmission();
}

void ActuatorM5StackGrayBMM150::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer)
{
    uint8_t i;

    Wire.beginTransmission(driver_Addr);  // Initialize the Tx buffer
    Wire.write(start_Addr);               // Put slave register address in Tx buffer
    for(i = 0; i < number_Bytes; i++)
    {
        Wire.write(*(write_Buffer+i));    // Put data in Tx buffer
    }
    Wire.endTransmission();
}

void ActuatorM5StackGrayBMM150::getEncoder(int32_t* wheel, uint8_t sw)
{
    uint8_t buf[4];

    switch (sw)
    {
    case MOTOR_LEFT:
        I2C_Read_NBytes(MOTOR_ADDRESS, MOTOR_ENCODER_L, MOTOR_LENGTH_ENCODER, buf);
        break;
    case MOTOR_RIGHT:
        I2C_Read_NBytes(MOTOR_ADDRESS, MOTOR_ENCODER_R, MOTOR_LENGTH_ENCODER, buf);
        break;
    }
    *wheel = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

}

void ActuatorM5StackGrayBMM150::setEncoder(int32_t wheel, uint8_t sw)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(wheel >> 24);
    buf[1] = (uint8_t)(wheel >> 16);
    buf[2] = (uint8_t)(wheel >> 8);
    buf[3] = (uint8_t)(wheel >> 0);

    switch (sw)
    {
    case MOTOR_LEFT:
        I2C_Write_NBytes(MOTOR_ADDRESS, MOTOR_ENCODER_L, MOTOR_LENGTH_ENCODER, buf);
        break;
    case MOTOR_RIGHT:
        I2C_Write_NBytes(MOTOR_ADDRESS, MOTOR_ENCODER_R, MOTOR_LENGTH_ENCODER, buf);
        break;
    }

}

void ActuatorM5StackGrayBMM150::getSpeed(int16_t* sp, uint8_t sw)
{
    int8_t buf[2];

    switch (sw)
    {
    case MOTOR_LEFT:
        I2C_Read_NBytes(MOTOR_ADDRESS, MOTOR_SPEED_L, MOTOR_LENGTH_SPEED, buf);
        break;
    case MOTOR_RIGHT:
        I2C_Read_NBytes(MOTOR_ADDRESS, MOTOR_SPEED_R, MOTOR_LENGTH_SPEED, buf);
        break;
    }
    *sp = (int16_t)((buf[0] << 8) | (uint8_t)buf[1]);

}

void ActuatorM5StackGrayBMM150::setSpeed(int16_t sp, uint8_t sw)
{
    int8_t buf[2];

    buf[0] = (int8_t)(sp >> 8);
    buf[1] = (int8_t)(sp >> 0);

    switch (sw)
    {
    case MOTOR_LEFT:
        I2C_Write_NBytes(MOTOR_ADDRESS, MOTOR_SPEED_L, MOTOR_LENGTH_SPEED, buf);
        break;
    case MOTOR_RIGHT:
        I2C_Write_NBytes(MOTOR_ADDRESS, MOTOR_SPEED_R, MOTOR_LENGTH_SPEED, buf);
        break;
    }

}

