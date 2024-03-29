/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2018-2021, Japan Embedded Systems Technology Association(JASA)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "openEL.hpp"
#include "Actuator.hpp"
#include "Sensor.hpp"
#include "openEL_ActuatorM5StackGrayBMM150.hpp"
#include "openEL_SensorM5StackGrayMPU6886.hpp"
#include "openEL_SensorM5StackGrayBMM150.hpp"

const HAL_REG_T HalRegTbl[] = {
    {0x0001, 0x0000000A, 0x00000001, &HalActuatorM5StackGrayBMM150Tbl, sizeof(Actuator) },
    {0x0002, 0x0000000A, 0x00000001, &HalSensorM5StackGrayMPU6886Tbl,  sizeof(Sensor)   },
    {0x000D, 0x0000000A, 0x00000001, &HalSensorM5StackGrayBMM150Tbl,   sizeof(Sensor)   },
};

const int32_t hal_szRegTbl = sizeof(HalRegTbl)/sizeof(HAL_REG_T);

