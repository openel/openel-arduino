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

#include "openEL_SensorM5StackGrayBMM150.hpp"

std::string SensorM5StackGrayBMM150::strDevName = "M5StackGrayBMM150";

std::vector<std::string> SensorM5StackGrayBMM150::strFncLst =
{
    "Init",
    "ReInit",
    "Finalize",
    "AddObserver",
    "RemoveObserver",
    "GetProperty",
    "GetTime",
    "GetValueList",
    "GetTimedValueList"
};

int16_t SensorM5StackGrayBMM150::magRawX;
int16_t SensorM5StackGrayBMM150::magRawY;
int16_t SensorM5StackGrayBMM150::magRawZ;
int16_t SensorM5StackGrayBMM150::magDataX;
int16_t SensorM5StackGrayBMM150::magDataY;
int16_t SensorM5StackGrayBMM150::magDataZ;
struct BMM150Setting SensorM5StackGrayBMM150::settings;
struct BMM150TrimReg SensorM5StackGrayBMM150::trimData;

Property SensorM5StackGrayBMM150::SensorM5StackGrayBMM150_property;

ReturnCode SensorM5StackGrayBMM150::fncInit(HALComponent *pHALComponent)
{
    init_BMM150();

    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncReInit(HALComponent *pHALComponent)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncFinalize(HALComponent *pHALComponent)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncAddObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncRemoveObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncGetProperty(HALComponent *pHALComponent, Property **property)
{
    SensorM5StackGrayBMM150_property.deviceName = strDevName;
    SensorM5StackGrayBMM150_property.functionList = strFncLst;
    (**property).functionList.resize(strFncLst.size());
    **property = SensorM5StackGrayBMM150_property;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncGetTime(HALComponent *pHALComponent, unsigned int **timeValue)
{
    **timeValue = (unsigned int)millis();

    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncGetValLst(HALComponent *pHALComponent, float **valueList, int **num)
{
    getMag(&magRawX, &magRawY, &magRawZ);
    getMagData(&magDataX, &magDataY, &magDataZ);

    *(*valueList+0) = magDataX;
    *(*valueList+1) = magDataY;
    *(*valueList+2) = magDataZ;
    *(*valueList+3) = magRawX;
    *(*valueList+4) = magRawY;
    *(*valueList+5) = magRawZ;

    **num = DATA_NUM;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncGetTimedValLst(HALComponent *pHALComponent, float **valueList, unsigned int **timeValue, int **num)
{
    getMag(&magRawX, &magRawY, &magRawZ);
    getMagData(&magDataX, &magDataY, &magDataZ);

    *(*valueList+0) = magDataX;
    *(*valueList+1) = magDataY;
    *(*valueList+2) = magDataZ;
    *(*valueList+3) = magRawX;
    *(*valueList+4) = magRawY;
    *(*valueList+5) = magRawZ;

    **timeValue = (unsigned int)millis();
    **num = DATA_NUM;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayBMM150::fncSetValue(HALComponent *pHALComponent, int request, float value)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayBMM150::fncGetValue(HALComponent *pHALComponent, int request, float **value)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayBMM150::fncNop(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayBMM150::fncDeviceVendorSpec(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd, HAL_ARGUMENT_DEVICE_T *pCmdDev)
{
    return HAL_ERROR;
}

HAL_FNCTBL_T HalSensorM5StackGrayBMM150Tbl =
{
    SensorM5StackGrayBMM150::fncInit,			/* 0x00 */
    SensorM5StackGrayBMM150::fncReInit,			/* 0x01 */
    SensorM5StackGrayBMM150::fncFinalize,		/* 0x02 */
    SensorM5StackGrayBMM150::fncAddObserver,		/* 0x03 */
    SensorM5StackGrayBMM150::fncRemoveObserver,		/* 0x04 */
    SensorM5StackGrayBMM150::fncGetProperty,		/* 0x05 */
    SensorM5StackGrayBMM150::fncGetTime,		/* 0x06 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x07 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x08 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x09 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0A */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0B */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0C */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0D */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0E */
    SensorM5StackGrayBMM150::fncNop,			/* 0x0F */
    SensorM5StackGrayBMM150::fncNop,			/* 0x10 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x11 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x12 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x13 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x14 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x15 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x16 */
    SensorM5StackGrayBMM150::fncNop,			/* 0x17 */
    SensorM5StackGrayBMM150::fncSetValue,		/* 0x18 */
    SensorM5StackGrayBMM150::fncGetValue,		/* 0x19 */
    SensorM5StackGrayBMM150::fncGetValLst,		/* 0x1A */
    SensorM5StackGrayBMM150::fncGetTimedValLst,		/* 0x1B */
    SensorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1C */
    SensorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1D */
    SensorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1E */
    SensorM5StackGrayBMM150::fncDeviceVendorSpec,	/* 0x1F */
};

int SensorM5StackGrayBMM150::init_BMM150()
{
    uint8_t buf[1];
    Wire.begin(21, 22);
    /* Power up the sensor from suspend to sleep mode */
    setOpMode(BMM150_SLEEP_MODE);
    delay(3);

    /* Check chip ID */
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_CHIP_ID_ADDR, 1, buf);
    if(buf[0] != BMM150_CHIP_ID)
        return -1;

    /* Function to update trim values */
    readTrimRegisters();

    /* Setting the power mode as normal */
    setOpMode(BMM150_NORMAL_MODE);

    /* Setting the preset mode as Low power mode 
    i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
    setPresetMode(BMM150_PRESETMODE_LOWPOWER);
    // setPresetMode(BMM150_HIGHACCURACY_REPZ);

    return 0;
}

void SensorM5StackGrayBMM150::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
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

void SensorM5StackGrayBMM150::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer)
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

void SensorM5StackGrayBMM150::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
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

void SensorM5StackGrayBMM150::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer)
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

void SensorM5StackGrayBMM150::setOpMode(uint8_t pwr_mode)
{
    /* Select the power mode to set */
    switch (pwr_mode)
    {
    case BMM150_NORMAL_MODE:
        /* If the sensor is in suspend mode
        put the device to sleep mode */
        setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
        /* Start-up time delay of 3ms*/
        delay(3);
        /* write the op mode */
        writeOpMode(pwr_mode);
        break;
    case BMM150_FORCED_MODE:
        /* If the sensor is in suspend mode
        put the device to sleep mode */
        setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
        /* Start-up time delay of 3ms*/
        delay(3);
        /* write the op mode */
        writeOpMode(pwr_mode);
        break;
    case BMM150_SLEEP_MODE:
        /* If the sensor is in suspend mode
        put the device to sleep mode */
        setPowerControlBit(BMM150_POWER_CNTRL_ENABLE);
        /* Start-up time delay of 3ms*/
        delay(3);
        /* write the op mode */
        writeOpMode(pwr_mode);
        break;
    case BMM150_SUSPEND_MODE:
        /* Set the power control bit to zero */
        setPowerControlBit(BMM150_POWER_CNTRL_DISABLE);
        break;
    default:
        break;
    }
}

void SensorM5StackGrayBMM150::setPowerControlBit(uint8_t pwrcntrl_bit)
{
    uint8_t regdata;
    /* Power control register 0x4B is read */
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_POWER_CONTROL_ADDR, 1, &regdata);
    /* Sets the value of power control bit */
    regdata = BMM150_SET_BITS_POS_0(regdata, BMM150_PWR_CNTRL, pwrcntrl_bit);
    I2C_Write_NBytes(BMM150_I2C_ADDRESS, BMM150_POWER_CONTROL_ADDR, 1, &regdata);
}

void SensorM5StackGrayBMM150::writeOpMode(uint8_t op_mode)
{
    uint8_t regdata;
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_OP_MODE_ADDR, 1, &regdata);
    /* Set the op_mode value in Opmode bits of 0x4C */
    regdata = BMM150_SET_BITS(regdata, BMM150_OP_MODE, op_mode);
    I2C_Write_NBytes(BMM150_I2C_ADDRESS, BMM150_OP_MODE_ADDR, 1, &regdata);
}

void SensorM5StackGrayBMM150::readTrimRegisters()
{
    uint8_t x1y1[2];
    uint8_t xyz[4];
    uint8_t xy1xy2[10];
    uint16_t tempmsb;

    /* Trim register value is read */
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_DIG_X1, 2, x1y1);
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_DIG_Z4_LSB, 4, xyz);
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_DIG_Z2_LSB, 10, xy1xy2);
    /* Trim data which is read is updated
    in the device structure */
    trimData.digX1 = (int8_t)x1y1[0];
    trimData.digY1 = (int8_t)x1y1[1];
    trimData.digX2 = (int8_t)xyz[2];
    trimData.digY2 = (int8_t)xyz[3];
    tempmsb = ((uint16_t)xy1xy2[3]) << 8;
    trimData.digZ1 = (uint16_t)(tempmsb | xy1xy2[2]);
    tempmsb = ((uint16_t)xy1xy2[1]) << 8;
    trimData.digZ2 = (int16_t)(tempmsb | xy1xy2[0]);
    tempmsb = ((uint16_t)xy1xy2[7]) << 8;
    trimData.digZ3 = (int16_t)(tempmsb | xy1xy2[6]);
    tempmsb = ((uint16_t)xyz[1]) << 8;
    trimData.digZ4 = (int16_t)(tempmsb | xyz[0]);
    trimData.digXY1 = xy1xy2[9];
    trimData.digXY2 = (int8_t)xy1xy2[8];
    tempmsb = ((uint16_t)(xy1xy2[5] & 0x7F)) << 8;
    trimData.digXYZ1 = (uint16_t)(tempmsb | xy1xy2[4]);

}

void SensorM5StackGrayBMM150::setPresetMode(uint8_t preset_mode)
{
    switch (preset_mode)
    {
    case BMM150_PRESETMODE_LOWPOWER:
        /* Set the data rate x,y,z repetition
        for Low Power mode */
        settings.data_rate = BMM150_DATA_RATE_10HZ;
        settings.xy_rep = BMM150_LOWPOWER_REPXY;
        settings.z_rep = BMM150_LOWPOWER_REPZ;
        setOdrXYZRep(settings);
        break;
    case BMM150_PRESETMODE_REGULAR:
        /* Set the data rate x,y,z repetition
        for Regular mode */
        settings.data_rate = BMM150_DATA_RATE_10HZ;
        settings.xy_rep = BMM150_REGULAR_REPXY;
        settings.z_rep = BMM150_REGULAR_REPZ;
        setOdrXYZRep(settings);
        break;
    case BMM150_PRESETMODE_HIGHACCURACY:
        /* Set the data rate x,y,z repetition
        for High Accuracy mode */
        settings.data_rate = BMM150_DATA_RATE_20HZ;
        settings.xy_rep = BMM150_HIGHACCURACY_REPXY;
        settings.z_rep = BMM150_HIGHACCURACY_REPZ;
        setOdrXYZRep(settings);
        break;
    case BMM150_PRESETMODE_ENHANCED:
        /* Set the data rate x,y,z repetition
        for Enhanced Accuracy mode */
        settings.data_rate = BMM150_DATA_RATE_10HZ;
        settings.xy_rep = BMM150_ENHANCED_REPXY;
        settings.z_rep = BMM150_ENHANCED_REPZ;
        setOdrXYZRep(settings);
        break;
    default:
        break;
    }
}

void SensorM5StackGrayBMM150::setOdrXYZRep(struct BMM150Setting settings)
{
    uint8_t regdata;
    /* Set the ODR */
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_OP_MODE_ADDR, 1, &regdata);
    regdata = BMM150_SET_BITS(regdata, BMM150_ODR, settings.data_rate);
    I2C_Write_NBytes(BMM150_I2C_ADDRESS, BMM150_OP_MODE_ADDR, 1, &regdata);
    /* Set the XY-repetitions number */
    I2C_Write_NBytes(BMM150_I2C_ADDRESS, BMM150_REP_XY_ADDR, 1, &settings.xy_rep);
    /* Set the Z-repetitions number */
    I2C_Write_NBytes(BMM150_I2C_ADDRESS, BMM150_REP_Z_ADDR, 1, &settings.z_rep);
}

void SensorM5StackGrayBMM150::getMag(int16_t* mx, int16_t* my, int16_t* mz)
{
    int8_t buf[8];
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_DATA_X_LSB, 8, buf);

    /* Mag X axis data */
    buf[0] = BMM150_GET_BITS(buf[0], BMM150_DATA_X);
    /* Mag Y axis data */
    buf[2] = BMM150_GET_BITS(buf[2], BMM150_DATA_Y);
    /* Mag Z axis data */
    buf[4] = BMM150_GET_BITS(buf[4], BMM150_DATA_Z);

    *mx = (int16_t)((((int16_t)((int8_t)buf[1])) << 5) | buf[0]); /* Raw mag X axis data */
    *my = (int16_t)((((int16_t)((int8_t)buf[3])) << 5) | buf[2]); /* Raw mag Y axis data */
    *mz = (int16_t)((((int16_t)((int8_t)buf[5])) << 7) | buf[4]); /* Raw mag Z axis data */
}

void SensorM5StackGrayBMM150::getMagData(int16_t* mx, int16_t* my, int16_t* mz)
{
    int8_t buf[2];
    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;
    uint16_t rhall = 0;
    getMag(&magX, &magY, &magZ);
    I2C_Read_NBytes(BMM150_I2C_ADDRESS, BMM150_DATA_READY_STATUS, 2, buf);

    /* Mag R-HALL data */
    buf[0] = BMM150_GET_BITS(buf[0], BMM150_DATA_RHALL);
    rhall = (uint16_t)(((uint16_t)buf[1] << 6) | buf[0]);

    *mx = compensate_x(magX, rhall); /* Compensated Mag X data in int16_t format */
    *my = compensate_y(magY, rhall); /* Compensated Mag Y data in int16_t format */
    *mz = compensate_z(magZ, rhall); /* Compensated Mag Z data in int16_t format */
}

int16_t SensorM5StackGrayBMM150::compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
    int16_t retval;
    uint16_t process_comp_x0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;

    /* Overflow condition check */
    if(mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
    {
        if(data_rhall != 0)
        {
            /* Availability of valid data*/
            process_comp_x0 = data_rhall;
        }
        else if (trimData.digXYZ1 != 0)
        {
            process_comp_x0 = trimData.digXYZ1;
        }
        else
        {
            process_comp_x0 = 0;
        }
        if(process_comp_x0 != 0)
        {
            /* Processing compensation equations */
            process_comp_x1 = ((int32_t)trimData.digXYZ1) * 16384;
            process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_x2);
            process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4 = (((int32_t)trimData.digXY2) * (process_comp_x3 / 128));
            process_comp_x5 = (int32_t)(((int16_t)trimData.digXY1) * 128);
            process_comp_x6 = ((int32_t)retval) * process_comp_x5;
            process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8 = ((int32_t)(((int16_t)trimData.digX2) + ((int16_t)0xA0)));
            process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval = ((int16_t)(process_comp_x10 / 8192));
            retval = (retval + (((int16_t)trimData.digX1) * 8)) / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return retval;
}

int16_t SensorM5StackGrayBMM150::compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
    int16_t retval;
    uint16_t process_comp_y0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;

    /* Overflow condition check */
    if(mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
    {
        if(data_rhall != 0)
        {
            /* Availability of valid data*/
            process_comp_y0 = data_rhall;
        }
        else if (trimData.digXYZ1 != 0)
        {
            process_comp_y0 = trimData.digXYZ1;
        }
        else
        {
            process_comp_y0 = 0;
        }
        if(process_comp_y0 != 0)
        {
            /* Processing compensation equations */
            process_comp_y1 = (((int32_t)trimData.digXYZ1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t)retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)trimData.digXY2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)trimData.digXY1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = (int32_t)(((int16_t)trimData.digY2) + ((int16_t)0xA0));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval = (int16_t)(process_comp_y9 / 8192);
            retval = (retval + (((int16_t)trimData.digY1) * 8)) / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return retval;
}

int16_t SensorM5StackGrayBMM150::compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;

    /* Overflow condition check */
    if(mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL)
    {
        if ((trimData.digZ1 != 0) && (trimData.digZ2 != 0) &&
            (data_rhall != 0) && (trimData.digXYZ1 != 0))
        {
            /* Processing compensation equations */
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t)trimData.digXYZ1);
            process_comp_z1 = (((int32_t)trimData.digZ3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - trimData.digZ4)) * 32768);
            process_comp_z3 = ((int32_t)trimData.digZ1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval = ((process_comp_z2 - process_comp_z1) / (trimData.digZ2 + process_comp_z4));

            /* saturate result to +/- 2 micro-tesla */
            if (retval > BMM150_POSITIVE_SATURATION_Z)
            {
                retval =  BMM150_POSITIVE_SATURATION_Z;
            }
            else
            {
                if(retval < BMM150_NEGATIVE_SATURATION_Z)
                    retval = BMM150_NEGATIVE_SATURATION_Z;
            }
            /* Conversion of LSB to micro-tesla*/
            retval = retval / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return (int16_t)retval;
}
