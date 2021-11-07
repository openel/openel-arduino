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

#include "openEL_SensorM5StackGrayMPU6886.hpp"

std::string SensorM5StackGrayMPU6886::strDevName = "M5StackGrayMPU6886";

std::vector<std::string> SensorM5StackGrayMPU6886::strFncLst =
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

uint8_t SensorM5StackGrayMPU6886::Acscale = AFS_8G;
uint8_t SensorM5StackGrayMPU6886::Gyscale = GFS_2000DPS;
float SensorM5StackGrayMPU6886::aRes;
float SensorM5StackGrayMPU6886::gRes;
int16_t SensorM5StackGrayMPU6886::accAdcX;
int16_t SensorM5StackGrayMPU6886::accAdcY;
int16_t SensorM5StackGrayMPU6886::accAdcZ;
int16_t SensorM5StackGrayMPU6886::gyroAdcX;
int16_t SensorM5StackGrayMPU6886::gyroAdcY;
int16_t SensorM5StackGrayMPU6886::gyroAdcZ;
int16_t SensorM5StackGrayMPU6886::tempAdc;
float SensorM5StackGrayMPU6886::accDataX;
float SensorM5StackGrayMPU6886::accDataY;
float SensorM5StackGrayMPU6886::accDataZ;
float SensorM5StackGrayMPU6886::gyroDataX;
float SensorM5StackGrayMPU6886::gyroDataY;
float SensorM5StackGrayMPU6886::gyroDataZ;
float SensorM5StackGrayMPU6886::tempData;
uint8_t SensorM5StackGrayMPU6886::fifo_buff[1024];

Property SensorM5StackGrayMPU6886::SensorM5StackGrayMPU6886_property;

ReturnCode SensorM5StackGrayMPU6886::fncInit(HALComponent *pHALComponent)
{
    init_MPU6886();

    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncReInit(HALComponent *pHALComponent)
{
    return HAL_OK;
}


ReturnCode SensorM5StackGrayMPU6886::fncFinalize(HALComponent *pHALComponent)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncAddObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncRemoveObserver(HALComponent *pHALComponent, HALObserver **halObserver)
{
    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncGetProperty(HALComponent *pHALComponent, Property **property)
{
    SensorM5StackGrayMPU6886_property.deviceName = strDevName;
    SensorM5StackGrayMPU6886_property.functionList = strFncLst;
    (**property).functionList.resize(strFncLst.size());
    **property = SensorM5StackGrayMPU6886_property;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncGetTime(HALComponent *pHALComponent, unsigned int **timeValue)
{
    **timeValue = (unsigned int)millis();

    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncGetValLst(HALComponent *pHALComponent, float **valueList, int **num)
{
    uint16_t fifo_count = 0;
    uint16_t data_number = 0;
    ImuData_t* imu_data;
    float acc_x = 0, acc_y = 0, acc_z = 0; 
    float gyro_x, gyro_y, gyro_z;

    fifo_count = readFIFOCount();
    readFIFO(fifo_buff, fifo_count);

    imu_data = (ImuData_t *)fifo_buff;
    data_number = fifo_count / 14;

    for(uint8_t i = 0; i < data_number; i++)
    {
        *(*valueList+(i*14)+0) = aRes * (int16_t)(imu_data[i].value.acc_x_l | (imu_data[i].value.acc_x_h << 8));
        *(*valueList+(i*14)+1) = aRes * (int16_t)(imu_data[i].value.acc_y_l | (imu_data[i].value.acc_y_h << 8));
        *(*valueList+(i*14)+2) = aRes * (int16_t)(imu_data[i].value.acc_z_l | (imu_data[i].value.acc_z_h << 8));
        *(*valueList+(i*14)+3) = gRes * (int16_t)(imu_data[i].value.gyro_x_l | (imu_data[i].value.gyro_x_h << 8));
        *(*valueList+(i*14)+4) = gRes * (int16_t)(imu_data[i].value.gyro_y_l | (imu_data[i].value.gyro_y_h << 8));
        *(*valueList+(i*14)+5) = gRes * (int16_t)(imu_data[i].value.gyro_z_l | (imu_data[i].value.gyro_z_h << 8));
        *(*valueList+(i*14)+6) = ((int16_t)(imu_data[i].value.temp_l | (imu_data[i].value.temp_h << 8))) / 326.8 + 25.0;
        *(*valueList+(i*14)+7) = (int16_t)(imu_data[i].value.acc_x_l | (imu_data[i].value.acc_x_h << 8));
        *(*valueList+(i*14)+8) = (int16_t)(imu_data[i].value.acc_y_l | (imu_data[i].value.acc_y_h << 8));
        *(*valueList+(i*14)+9) = (int16_t)(imu_data[i].value.acc_z_l | (imu_data[i].value.acc_z_h << 8));
        *(*valueList+(i*14)+10) = (int16_t)(imu_data[i].value.gyro_x_l | (imu_data[i].value.gyro_x_h << 8));
        *(*valueList+(i*14)+11) = (int16_t)(imu_data[i].value.gyro_y_l | (imu_data[i].value.gyro_y_h << 8));
        *(*valueList+(i*14)+12) = (int16_t)(imu_data[i].value.gyro_z_l | (imu_data[i].value.gyro_z_h << 8));
        *(*valueList+(i*14)+13) = (int16_t)(imu_data[i].value.temp_l | (imu_data[i].value.temp_h << 8));
    }
    **num = data_number * DATA_NUM;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncGetTimedValLst(HALComponent *pHALComponent, float **valueList, unsigned int **timeValue, int **num)
{
    uint16_t fifo_count = 0;
    uint16_t data_number = 0;
    ImuData_t* imu_data;

    fifo_count = readFIFOCount();
    readFIFO(fifo_buff, fifo_count);

    imu_data = (ImuData_t *)fifo_buff;
    data_number = fifo_count / 14;

    for(uint8_t i = 0; i < data_number; i++)
    {
        *(*valueList+(i*14)+0) = aRes * (int16_t)(imu_data[i].value.acc_x_l | (imu_data[i].value.acc_x_h << 8));
        *(*valueList+(i*14)+1) = aRes * (int16_t)(imu_data[i].value.acc_y_l | (imu_data[i].value.acc_y_h << 8));
        *(*valueList+(i*14)+2) = aRes * (int16_t)(imu_data[i].value.acc_z_l | (imu_data[i].value.acc_z_h << 8));
        *(*valueList+(i*14)+3) = gRes * (int16_t)(imu_data[i].value.gyro_x_l | (imu_data[i].value.gyro_x_h << 8));
        *(*valueList+(i*14)+4) = gRes * (int16_t)(imu_data[i].value.gyro_y_l | (imu_data[i].value.gyro_y_h << 8));
        *(*valueList+(i*14)+5) = gRes * (int16_t)(imu_data[i].value.gyro_z_l | (imu_data[i].value.gyro_z_h << 8));
        *(*valueList+(i*14)+6) = ((int16_t)(imu_data[i].value.temp_l | (imu_data[i].value.temp_h << 8))) / 326.8 + 25.0;
        *(*valueList+(i*14)+7) = (int16_t)(imu_data[i].value.acc_x_l | (imu_data[i].value.acc_x_h << 8));
        *(*valueList+(i*14)+8) = (int16_t)(imu_data[i].value.acc_y_l | (imu_data[i].value.acc_y_h << 8));
        *(*valueList+(i*14)+9) = (int16_t)(imu_data[i].value.acc_z_l | (imu_data[i].value.acc_z_h << 8));
        *(*valueList+(i*14)+10) = (int16_t)(imu_data[i].value.gyro_x_l | (imu_data[i].value.gyro_x_h << 8));
        *(*valueList+(i*14)+11) = (int16_t)(imu_data[i].value.gyro_y_l | (imu_data[i].value.gyro_y_h << 8));
        *(*valueList+(i*14)+12) = (int16_t)(imu_data[i].value.gyro_z_l | (imu_data[i].value.gyro_z_h << 8));
        *(*valueList+(i*14)+13) = (int16_t)(imu_data[i].value.temp_l | (imu_data[i].value.temp_h << 8));
    }
    **timeValue = (unsigned int)millis();
    **num = data_number * DATA_NUM;

    return HAL_OK;
}

ReturnCode SensorM5StackGrayMPU6886::fncSetValue(HALComponent *pHALComponent, int request, float value)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayMPU6886::fncGetValue(HALComponent *pHALComponent, int request, float **value)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayMPU6886::fncNop(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd)
{
    return HAL_ERROR;
}

ReturnCode SensorM5StackGrayMPU6886::fncDeviceVendorSpec(HALComponent *pHALComponent, HAL_ARGUMENT_T *pCmd, HAL_ARGUMENT_DEVICE_T *pCmdDev)
{
    return HAL_ERROR;
}

HAL_FNCTBL_T HalSensorM5StackGrayMPU6886Tbl =
{
    SensorM5StackGrayMPU6886::fncInit,			/* 0x00 */
    SensorM5StackGrayMPU6886::fncReInit,			/* 0x01 */
    SensorM5StackGrayMPU6886::fncFinalize,		/* 0x02 */
    SensorM5StackGrayMPU6886::fncAddObserver,		/* 0x03 */
    SensorM5StackGrayMPU6886::fncRemoveObserver,		/* 0x04 */
    SensorM5StackGrayMPU6886::fncGetProperty,		/* 0x05 */
    SensorM5StackGrayMPU6886::fncGetTime,		/* 0x06 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x07 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x08 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x09 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0A */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0B */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0C */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0D */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0E */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x0F */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x10 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x11 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x12 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x13 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x14 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x15 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x16 */
    SensorM5StackGrayMPU6886::fncNop,			/* 0x17 */
    SensorM5StackGrayMPU6886::fncSetValue,		/* 0x18 */
    SensorM5StackGrayMPU6886::fncGetValue,		/* 0x19 */
    SensorM5StackGrayMPU6886::fncGetValLst,		/* 0x1A */
    SensorM5StackGrayMPU6886::fncGetTimedValLst,		/* 0x1B */
    SensorM5StackGrayMPU6886::fncDeviceVendorSpec,	/* 0x1C */
    SensorM5StackGrayMPU6886::fncDeviceVendorSpec,	/* 0x1D */
    SensorM5StackGrayMPU6886::fncDeviceVendorSpec,	/* 0x1E */
    SensorM5StackGrayMPU6886::fncDeviceVendorSpec,	/* 0x1F */
};

int SensorM5StackGrayMPU6886::init_MPU6886()
{
    unsigned char tempdata[1];
    unsigned char regdata;

    Wire.begin(21, 22);

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_WHOAMI, 1, tempdata);
    if(tempdata[0] != 0x19)
        return -1;
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 7);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 0);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = 0x10;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x18;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1,&regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
    delay(1);

    regdata = 0x22;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);

    delay(100);
    getGres();
    getAres();

    // FIFO
    regdata = 0x18;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_ENABLE, 1, &regdata);

    regdata = 0x40;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);

    regdata = 0x44;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);

    return 0;
}

void SensorM5StackGrayMPU6886::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
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

void SensorM5StackGrayMPU6886::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer)
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

void SensorM5StackGrayMPU6886::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
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

void SensorM5StackGrayMPU6886::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer)
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

void SensorM5StackGrayMPU6886::getAres()
{
    switch (Acscale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
        aRes = 2.0/32768.0;
        break;
    case AFS_4G:
        aRes = 4.0/32768.0;
        break;
    case AFS_8G:
        aRes = 8.0/32768.0;
        break;
    case AFS_16G:
        aRes = 16.0/32768.0;
        break;
    }
}

void SensorM5StackGrayMPU6886::getGres()
{
    switch (Gyscale)
    {
    // Possible gyro scales (and their register bit settings) are:
    case GFS_250DPS:
        gRes = 250.0/32768.0;
        break;
    case GFS_500DPS:
        gRes = 500.0/32768.0;
        break;
    case GFS_1000DPS:
        gRes = 1000.0/32768.0;
        break;
    case GFS_2000DPS:
        gRes = 2000.0/32768.0;
        break;
    }
}

void SensorM5StackGrayMPU6886::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, 6, buf);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

void SensorM5StackGrayMPU6886::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, 6, buf);

    *gx = ((int16_t)buf[0] << 8) | buf[1];
    *gy = ((int16_t)buf[2] << 8) | buf[3];
    *gz = ((int16_t)buf[4] << 8) | buf[5];
}

void SensorM5StackGrayMPU6886::getTempAdc(int16_t* t)
{
    uint8_t buf[2];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, 6, buf);

    *t = ((int16_t)buf[0] << 8) | buf[1];
}

void SensorM5StackGrayMPU6886::getAccelData(float* ax, float* ay, float* az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

void SensorM5StackGrayMPU6886::getGyroData(float* gx, float* gy, float* gz)
{
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    getGyroAdc(&gyroX, &gyroY, &gyroZ);

    *gx = (float)gyroX * gRes;
    *gy = (float)gyroY * gRes;
    *gz = (float)gyroZ * gRes;
}

void SensorM5StackGrayMPU6886::getTempData(float* t)
{
    int16_t temp = 0;
    getTempAdc(&temp);

    *t = (float)temp / 326.8 + 25.0;
}

void SensorM5StackGrayMPU6886::readFIFO(uint8_t *dataBuff, uint16_t length)
{
    uint8_t number = length / 210;

    for(uint8_t i = 0; i < number; i++)
    {
        I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, 210, &dataBuff[i*210]);
    }
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, length % 210, &dataBuff[number*210]);
}

uint16_t SensorM5StackGrayMPU6886::readFIFOCount()
{
    uint8_t buf[2];
    uint16_t regdata;

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_COUNT, 2, buf);
    regdata = ((uint16_t)buf[0] << 8) | buf[1];

    return regdata;
}
