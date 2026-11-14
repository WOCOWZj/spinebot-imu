//****************************************************************************************
// @file    SCH1.h
// @brief   Header file for the SCH1 library functions.
//
// @attention
//
// This software is released under the BSD license as follows.
// Copyright (c) 2024, Murata Electronics Oy.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following
// conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//    3. Neither the name of Murata Electronics Oy nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************************

#ifndef SCH1_H
#define SCH1_H

#include "SCH16t_hw.h"
#include "SCH16t_reg.h"

/**
 * API return codes
 */
#define SCH1_OK                      0
#define SCH1_ERR_NULL_POINTER       (-1)
#define SCH1_ERR_INVALID_PARAM      (-2)
#define SCH1_ERR_SENSOR_INIT        (-3)
#define SCH1_ERR_OTHER              (-4)

/**
 * Macros
 */
#define SPI48_DATA_INT32(a)         (((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)        ((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a)        ((uint16_t)(((a) >> 8)  & 0x0000ffffUL))
#define GET_TEMPERATURE(a)          ((a) / 100.0f)

// SCH1 raw unprocessed data values
typedef struct _SCH1_raw_data {
    int32_t Rate1_raw[3];
    int32_t Rate2_raw[3];
    int32_t Acc1_raw[3];
    int32_t Acc2_raw[3];
    int32_t Acc3_raw[3];
    int32_t Temp_raw;
    bool frame_error;
} SCH1_raw_data;


// SCH1 scaled measurement results
typedef struct _SCH1_result {
    float Rate1[3];
    float Rate2[3];
    float Acc1[3];
    float Acc2[3];
    float Acc3[3];
    float Temp;
} SCH1_result;


// SCH1 status data
typedef struct _SCH1_status {
    uint16_t Summary;
    uint16_t Summary_Sat;
    uint16_t Common;
    uint16_t Rate_Common;
    uint16_t Rate_X;
    uint16_t Rate_Y;
    uint16_t Rate_Z;
    uint16_t Acc_X;
    uint16_t Acc_Y;
    uint16_t Acc_Z;
} SCH1_status;


// SCH1 filters
typedef struct _SCH1_filter {
    uint16_t Rate12;
    uint16_t Acc12;
    uint16_t Acc3;
} SCH1_filter;


// SCH1 sensitivities
typedef struct _SCH1_sensitivity {
    uint16_t Rate1;
    uint16_t Rate2;
    uint16_t Acc1;
    uint16_t Acc2;
    uint16_t Acc3;
} SCH1_sensitivity;


// SCH1 decimation
typedef struct _SCH1_decimation {
    uint16_t Rate2;
    uint16_t Acc2;
} SCH1_decimation;


// Measurement axes
enum
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};


/**
 * Function prototypes
 */
int         SCH1_init();
void        SCH1_getData(SCH1_raw_data *data,
                         bool read_gyro, bool read_accel, bool read_temp);
bool        SCH1_getData(float gyro[3], float accel[3], float *temperature,
                         bool read_gyro, bool read_accel, bool read_temp);

int         SCH1_getStatus(SCH1_status *Status);
void        SCH1_convert_data(SCH1_raw_data *data_in, SCH1_result *data_out);
bool        SCH1_check_48bit_frame_error(uint64_t *data, int size);
uint64_t    SCH1_sendRequest(uint64_t Request);

int         SCH1_setFilters();
int         SCH1_setRateSensDec();
int         SCH1_setAccSensDec();

int         SCH1_enableMeas(bool enableSensor, bool setEOI);
int         SCH1_setDRY(int8_t polarity, bool enable);
void        SCH1_sendSPIreset();
bool        SCH1_verifyStatus(SCH1_status *Status);

char        *SCH1_getSnbr();
void        SCH1_reset();

#endif /* SCH1_H */
