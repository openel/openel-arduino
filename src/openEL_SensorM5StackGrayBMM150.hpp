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

#ifndef OPENEL_SENSOR_M5STACK_GRAY_BMM150_HPP
#define OPENEL_SENSOR_M5STACK_GRAY_BMM150_HPP

#include <M5Stack.h>
#include "Sensor.hpp"

#define BMM150_I2C_ADDRESS         0x10

#define BMM150_CHIP_ID             0x32
#define BMM150_SET_SOFT_RESET      0x82

#define BMM150_NORMAL_MODE         0x00
#define BMM150_FORCED_MODE         0x01
#define BMM150_SLEEP_MODE          0x03
#define BMM150_SUSPEND_MODE        0x04

#define BMM150_PRESETMODE_LOWPOWER     0x01
#define BMM150_PRESETMODE_REGULAR      0x02
#define BMM150_PRESETMODE_HIGHACCURACY 0x03
#define BMM150_PRESETMODE_ENHANCED     0x04

#define BMM150_POWER_CNTRL_DISABLE 0x00
#define BMM150_POWER_CNTRL_ENABLE  0x01

#define BMM150_CHIP_ID_ADDR        0x40
#define BMM150_DATA_X_LSB          0x42
#define BMM150_DATA_X_MSB          0x43
#define BMM150_DATA_Y_LSB          0x44
#define BMM150_DATA_Y_MSB          0x45
#define BMM150_DATA_Z_LSB          0x46
#define BMM150_DATA_Z_MSB          0x47
#define BMM150_DATA_READY_STATUS   0x48
#define BMM150_INTERRUPT_STATUS    0x4A
#define BMM150_POWER_CONTROL_ADDR  0x4B
#define BMM150_OP_MODE_ADDR        0x4C
#define BMM150_INT_CONFIG_ADDR     0x4D
#define BMM150_AXES_ENABLE_ADDR    0x4E
#define BMM150_LOW_THRESHOLD_ADDR  0x4F
#define BMM150_HIGH_THRESHOLD_ADDR 0x50
#define BMM150_REP_XY_ADDR         0x51
#define BMM150_REP_Z_ADDR          0x52

#define BMM150_DATA_RATE_10HZ      (0x00)
#define BMM150_DATA_RATE_02HZ      (0x01)
#define BMM150_DATA_RATE_06HZ      (0x02)
#define BMM150_DATA_RATE_08HZ      (0x03)
#define BMM150_DATA_RATE_15HZ      (0x04)
#define BMM150_DATA_RATE_20HZ      (0x05)
#define BMM150_DATA_RATE_25HZ      (0x06)
#define BMM150_DATA_RATE_30HZ      (0x07)

#define BMM150_DIG_X1              UINT8_C(0x5D)
#define BMM150_DIG_Y1              UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB          UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB          UINT8_C(0x63)
#define BMM150_DIG_X2              UINT8_C(0x64)
#define BMM150_DIG_Y2              UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB          UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB          UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB          UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB          UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB        UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB        UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB          UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB          UINT8_C(0x6F)
#define BMM150_DIG_XY2             UINT8_C(0x70)
#define BMM150_DIG_XY1             UINT8_C(0x71)

#define BMM150_LOWPOWER_REPXY         (1)
#define BMM150_REGULAR_REPXY          (4)
#define BMM150_ENHANCED_REPXY         (7)
#define BMM150_HIGHACCURACY_REPXY     (23)
#define BMM150_LOWPOWER_REPZ          (2)
#define BMM150_REGULAR_REPZ           (14)
#define BMM150_ENHANCED_REPZ          (26)
#define BMM150_HIGHACCURACY_REPZ      (82)

#define BMM150_PWR_CNTRL_MSK          (0x01)
#define BMM150_CONTROL_MEASURE_MSK    (0x38)
#define BMM150_CONTROL_MEASURE_POS    (0x03)
#define BMM150_POWER_CONTROL_BIT_MSK  (0x01)
#define BMM150_POWER_CONTROL_BIT_POS  (0x00)
#define BMM150_OP_MODE_MSK            (0x06)
#define BMM150_OP_MODE_POS            (0x01)
#define BMM150_ODR_MSK                (0x38)
#define BMM150_ODR_POS                (0x03)
#define BMM150_DATA_X_MSK             (0xF8)
#define BMM150_DATA_X_POS             (0x03)
#define BMM150_DATA_Y_MSK             (0xF8)
#define BMM150_DATA_Y_POS             (0x03)
#define BMM150_DATA_Z_MSK             (0xFE)
#define BMM150_DATA_Z_POS             (0x01)
#define BMM150_DATA_RHALL_MSK         (0xFC)
#define BMM150_DATA_RHALL_POS         (0x02)
#define BMM150_SELF_TEST_MSK          (0x01)
#define BMM150_ADV_SELF_TEST_MSK      (0xC0)
#define BMM150_ADV_SELF_TEST_POS      (0x06)
#define BMM150_DRDY_EN_MSK            (0x80)
#define BMM150_DRDY_EN_POS            (0x07)
#define BMM150_INT_PIN_EN_MSK         (0x40)
#define BMM150_INT_PIN_EN_POS         (0x06)
#define BMM150_DRDY_POLARITY_MSK      (0x04)
#define BMM150_DRDY_POLARITY_POS      (0x02)
#define BMM150_INT_LATCH_MSK          (0x02)
#define BMM150_INT_LATCH_POS          (0x01)
#define BMM150_INT_POLARITY_MSK       (0x01)
#define BMM150_DATA_OVERRUN_INT_MSK   (0x80)
#define BMM150_DATA_OVERRUN_INT_POS   (0x07)
#define BMM150_OVERFLOW_INT_MSK       (0x40)
#define BMM150_OVERFLOW_INT_POS       (0x06)
#define BMM150_HIGH_THRESHOLD_INT_MSK (0x38)
#define BMM150_HIGH_THRESHOLD_INT_POS (0x03)
#define BMM150_LOW_THRESHOLD_INT_MSK  (0x07)
#define BMM150_DRDY_STATUS_MSK        (0x01)

#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL (-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL  (-16384)
#define BMM150_OVERFLOW_OUTPUT             (-32768)
#define BMM150_NEGATIVE_SATURATION_Z       (-32767)
#define BMM150_POSITIVE_SATURATION_Z       (32767)

#define BMM150_SET_BITS(reg_data, bitname, data) \
                ((reg_data & ~(bitname##_MSK)) | \
                ((data << bitname##_POS) & bitname##_MSK))

#define BMM150_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
                (bitname##_POS))

#define BMM150_SET_BITS_POS_0(reg_data, bitname, data) \
                ((reg_data & ~(bitname##_MSK)) | \
                (data & bitname##_MSK))

#define BMM150_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

#define BMM150_DATA_NUM 6

struct BMM150Setting {
    /*! Control measurement of XYZ axes */
    uint8_t xyz_axes_control;
    /*! Power control bit value */
    uint8_t pwr_cntrl_bit;
    /*! Power control bit value */
    uint8_t pwr_mode;
    /*! Data rate value (ODR) */
    uint8_t data_rate;
    /*! XY Repetitions */
    uint8_t xy_rep;
    /*! Z Repetitions */
    uint8_t z_rep;
    /*! Preset mode of sensor */
    uint8_t preset_mode;
    /*! Interrupt configuration settings */
    // struct bmm150_int_ctrl_settings int_settings;
};

struct BMM150TrimReg {
    /*! trim x1 data */
    int8_t digX1;
    /*! trim y1 data */
    int8_t digY1;
    /*! trim x2 data */
    int8_t digX2;
    /*! trim y2 data */
    int8_t digY2;
    /*! trim z1 data */
    uint16_t digZ1;
    /*! trim z2 data */
    int16_t digZ2;
    /*! trim z3 data */
    int16_t digZ3;
    /*! trim z4 data */
    int16_t digZ4;
    /*! trim xy1 data */
    uint8_t digXY1;
    /*! trim xy2 data */
    int8_t digXY2;
    /*! trim xyz1 data */
    uint16_t digXYZ1;
};

class SensorM5StackGrayBMM150 : public Sensor
{
private:
    static bool calibration_mode;
    static std::string strDevName;
    static std::vector<std::string> strFncLst;
    static int16_t magRawX;
    static int16_t magRawY;
    static int16_t magRawZ;
    static int16_t magDataX;
    static int16_t magDataY;
    static int16_t magDataZ;
    static struct BMM150Setting settings;
    static struct BMM150TrimReg trimData;

    static int init_BMM150();
    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    static void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *read_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    static void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, int8_t *write_Buffer);
    static void setOpMode(uint8_t pwr_mode);
    static void setPowerControlBit(uint8_t pwrcntrl_bit);
    static void writeOpMode(uint8_t op_mode);
    static void readTrimRegisters();
    static void setPresetMode(uint8_t preset_mode);
    static void setOdrXYZRep(struct BMM150Setting settings);
    static void getMag(int16_t* mx, int16_t* my, int16_t* mz);
    static void getMagData(int16_t* mx, int16_t* my, int16_t* mz);
    static int16_t compensate_x(int16_t mag_data_x, uint16_t data_rhall);
    static int16_t compensate_y(int16_t mag_data_y, uint16_t data_rhall);
    static int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall);

public:
    static Property SensorM5StackGrayBMM150_property;

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

extern HAL_FNCTBL_T HalSensorM5StackGrayBMM150Tbl;

#endif // OPENEL_SENSOR_M5STTACK_GRAY_BMM150_HPP
