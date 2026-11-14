//****************************************************************************************
// @file    SCH1.c
// @brief   SCH1 library functions.
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

#include "SCH16t.h"

uint32_t SCH16T_Filter_GYRO = SCH1_FILTER_RATE_370HZ;
uint32_t SCH16T_Filter_ACCEL12 = SCH1_FILTER_ACC_210HZ;
uint32_t SCH16T_Filter_ACCEL3 = SCH1_FILTER_ACC_500HZ;

uint32_t SCH16T_Range_GYRO = SCH1_RANGE_RATE_DYN_5200DPS;
float SCH16T_LSB_perDPS_GYRO = SCH1_LSB_PER_DPS_RATE_5200DPS;

uint32_t SCH16T_Range_ACCEL12 = SCH1_RANGE_ACC_12_DYN_8G;
float SCH16T_LSB_perMS2_ACCEL12 = SCH1_LSB_PER_MS2_ACC_12_8G;

uint32_t SCH16T_Range_ACCEL3 = SCH1_RANGE_ACC_3_DYN_26G;
float SCH16T_LSB_perMS2_ACCEL3 = SCH1_LSB_PER_MS2_ACC_3_26G;

uint32_t SCH16T_Decimation_GYRO = SCH1_DECIMATION_1;
uint32_t SCH16T_Decimation_ACCEL12 = SCH1_DECIMATION_1;

float SCH16T_LSB_PerDegreeCelsius = SCH1_LSB_PER_DEGREE_C;

bool SCH16T_DRY_EN = false;  // 0=disable, 1=enable
bool SCH16T_HighSpeedSPI_EN = false;

/**
 * Internal function prototypes
 */          
static uint8_t CRC8(uint64_t SPIframe);
static uint8_t CRC3(uint32_t SPIframe);

/**
 * Internal function definitions
 */          


/**
 * @brief Resets SCH1 using EXTRESN pin.
 *
 * @note Resets SCH1 by setting EXTRESN pin low and high.
 *
 * @param None
 * 
 * @return None                           
 */
void SCH1_reset(void)
{              
    hw_EXTRESN_Low();
    hw_delay(2);
    hw_EXTRESN_High();
}


/**
 * @brief Resets SCH1 using SPI command.
 *
 * @note Resets SCH1 by setting SOFTRESET_CTRL-bit in CTRL_RESET register.
 *
 * @param None
 * 
 * @return None                           
 */
void SCH1_sendSPIreset(void)
{
    SCH1_sendRequest(REQ_SOFTRESET);
}

int SCH1_setFilters()
{
    uint32_t dataField;
    uint64_t requestFrame_Rate12;
    uint64_t responseFrame_Rate12;
    uint64_t requestFrame_Acc12;
    uint64_t responseFrame_Acc12;
    uint64_t requestFrame_Acc3;
    uint64_t responseFrame_Acc3;
    uint8_t  CRCvalue;

    // Set filters for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    requestFrame_Rate12 = REQ_SET_FILT_RATE;
    dataField = SCH16T_Filter_GYRO << 6 | SCH16T_Filter_GYRO << 3 | SCH16T_Filter_GYRO;
    requestFrame_Rate12 |= dataField;
    requestFrame_Rate12 <<= 8;
    CRCvalue = CRC8(requestFrame_Rate12);
    requestFrame_Rate12 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Rate12);

    // Set filters for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    requestFrame_Acc12 = REQ_SET_FILT_ACC12;
    dataField = SCH16T_Filter_ACCEL12 << 6 | SCH16T_Filter_ACCEL12 << 3 | SCH16T_Filter_ACCEL12;
    requestFrame_Acc12 |= dataField;
    requestFrame_Acc12 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12);
    requestFrame_Acc12 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc12);

    // Set filters for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3 = REQ_SET_FILT_ACC3;
    dataField = SCH16T_Filter_ACCEL3 << 6 | SCH16T_Filter_ACCEL3 << 3 | SCH16T_Filter_ACCEL3;
    requestFrame_Acc3 |= dataField;
    requestFrame_Acc3 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3);
    requestFrame_Acc3 |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc3);
    
    // Read back filter register contents.
    SCH1_sendRequest(REQ_READ_FILT_RATE);        
    responseFrame_Rate12 = SCH1_sendRequest(REQ_READ_FILT_ACC12);
    responseFrame_Acc12 = SCH1_sendRequest(REQ_READ_FILT_ACC3);
    responseFrame_Acc3 = SCH1_sendRequest(REQ_READ_FILT_ACC3);
    
    // Check that return frame is not blank.
    if ((responseFrame_Rate12 == 0xFFFFFFFFFFFF) || (responseFrame_Rate12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc12 == 0xFFFFFFFFFFFF) || (responseFrame_Acc12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3 == 0xFFFFFFFFFFFF) || (responseFrame_Acc3 == 0x00))
        return SCH1_ERR_OTHER;
    
    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate12 & DATA_FIELD_MASK) != (responseFrame_Rate12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc12 & DATA_FIELD_MASK) != (responseFrame_Acc12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3 & DATA_FIELD_MASK) != (responseFrame_Acc3 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}


/**
 * @brief Sets Rate_XYZ1 & Rate_XYZ2 channel sensitivities and Rate_XYZ2 channel decimation for the SCH1.
 *
 * @return SCH1_OK = success
 *         SCH1_ERR_* = failure. Please see header file for error definitions.                      
 */
int SCH1_setRateSensDec()
{
    uint64_t requestFrame_Rate_Ctrl;
    uint64_t responseFrame_Rate_Ctrl;
    uint8_t  CRCvalue;

    // Set sensitivities for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    // Also set decimation for Rate_XYZ2.
    requestFrame_Rate_Ctrl = REQ_SET_RATE_CTRL;
    requestFrame_Rate_Ctrl |= SCH16T_Range_GYRO << 12 | SCH16T_Range_GYRO << 9 | SCH16T_Decimation_GYRO << 6 | SCH16T_Decimation_GYRO << 3 | SCH16T_Decimation_GYRO;

    requestFrame_Rate_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Rate_Ctrl);
    requestFrame_Rate_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Rate_Ctrl);

    // Read back rate control register contents.
    SCH1_sendRequest(REQ_READ_RATE_CTRL);
    responseFrame_Rate_Ctrl = SCH1_sendRequest(REQ_READ_RATE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Rate_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Rate_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate_Ctrl & DATA_FIELD_MASK) != (responseFrame_Rate_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}

/**
 * @brief Sets Acc_XYZ1/2/3 channel sensitivities and Acc_XYZ2 channel decimation for the SCH1.
 *
 * @return SCH1_OK = success
 *         SCH1_ERR_* = failure. Please see header file for error definitions.                      
 */
int SCH1_setAccSensDec()
{
    uint32_t dataField;
    uint32_t bitField;
    uint64_t requestFrame_Acc12_Ctrl;
    uint64_t responseFrame_Acc12_Ctrl;
    uint64_t requestFrame_Acc3_Ctrl;
    uint64_t responseFrame_Acc3_Ctrl;
    uint8_t  CRCvalue;

    // Set sensitivities for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    // Also set decimation for Acc_XYZ2.
    requestFrame_Acc12_Ctrl = REQ_SET_ACC12_CTRL;
    requestFrame_Acc12_Ctrl |= SCH16T_Range_ACCEL12 << 12 | SCH16T_Range_ACCEL12 << 9 | SCH16T_Decimation_ACCEL12 << 6 | SCH16T_Decimation_ACCEL12 << 3 | SCH16T_Decimation_ACCEL12;

    requestFrame_Acc12_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12_Ctrl);
    requestFrame_Acc12_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc12_Ctrl);

    // Set sensitivity for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3_Ctrl = REQ_SET_ACC3_CTRL;
    requestFrame_Acc3_Ctrl |= SCH16T_Range_ACCEL3;

    requestFrame_Acc3_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3_Ctrl);
    requestFrame_Acc3_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Acc3_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_ACC12_CTRL);
    responseFrame_Acc12_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);
    responseFrame_Acc3_Ctrl = SCH1_sendRequest(REQ_READ_ACC3_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Acc12_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc12_Ctrl == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc3_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Acc12_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Acc12_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc12_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc3_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}

/**
 * @brief Activates/deactivates SCH1 measurement mode and sets the EOI (End Of Initialization) bit if needed.
 *
 * @param enableSensor - Enables/disables the sensor.
 * @param setEOI - Sets EOI-bit. Locks all R/W registers, except soft reset. Can only be set when no errors in common status.
 *
 * @return SCH1_OK = success
 *         SCH1_ERR_* = failure. Please see header file for error definitions.                      
 */
int SCH1_enableMeas(bool enableSensor, bool setEOI)
{
    uint64_t requestFrame_Mode_Ctrl;
    uint64_t responseFrame_Mode_Ctrl;
    uint8_t  CRCvalue;

    requestFrame_Mode_Ctrl = REQ_SET_MODE_CTRL;

    // Handle EN_SENSOR -bit
    if (enableSensor)
        requestFrame_Mode_Ctrl |= 0x01;

    // Handle EOI_CTRL -bit
    if (setEOI)
        requestFrame_Mode_Ctrl |= 0x02;

    requestFrame_Mode_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Mode_Ctrl);
    requestFrame_Mode_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_Mode_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_MODE_CTRL);
    responseFrame_Mode_Ctrl = SCH1_sendRequest(REQ_READ_MODE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Mode_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Mode_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Mode_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Mode_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}

/**
 * @brief Configures Data Ready (DRY) -pin
 *
 * @param polarity - Data Ready polarity control: 0 - high active (default), 1 - low active. -1 = don't care
 * @param enable   - Data Ready -pin disable/enable
 *
 * @return SCH1_OK = success
 *         SCH1_ERR_* = failure. Please see header file for error definitions.                      
 */
int SCH1_setDRY(int8_t polarity, bool enable_DRY_Ready, bool enable_HighSpeedSPI)
{
    uint64_t requestFrame_User_If_Ctrl;
    uint64_t responseFrame_User_If_Ctrl;
    uint64_t dataContent;
    uint8_t  CRCvalue;

    if ((polarity < -1) || (polarity > 1))
        return SCH1_ERR_INVALID_PARAM;
    
    // Read USER_IF_CTRL -register content
    SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    dataContent = (responseFrame_User_If_Ctrl & DATA_FIELD_MASK) >> 8;
    
    if (polarity == 0)
        dataContent &= (uint16_t)~0x40;   // Set DRY active high (0b01000000)
    else if (polarity == 1)
        dataContent |= 0x40;              // Set DRY active low
    
    if (enable_DRY_Ready)
        dataContent |= 0x20;              // Set DRY enabled (0b00100000)
    else
        dataContent &= (uint16_t)~0x20;   // Set DRY disabled

    if (enable_HighSpeedSPI)
        dataContent |= 0x01;             // Set MISO_HI_SPD enabled (0b00000001)
    else
        dataContent &= (uint16_t)~0x01;  // Set MISO_HI_SPD disabled

    requestFrame_User_If_Ctrl = REQ_SET_USER_IF_CTRL;
    requestFrame_User_If_Ctrl |= dataContent;
    requestFrame_User_If_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_User_If_Ctrl);
    requestFrame_User_If_Ctrl |= CRCvalue;
    SCH1_sendRequest(requestFrame_User_If_Ctrl);

    // Read back sensitivity control register contents.
    SCH1_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH1_sendRequest(REQ_READ_USER_IF_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_User_If_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_User_If_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_User_If_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_User_If_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    // Check that read and written data match.
    if ((requestFrame_User_If_Ctrl & DATA_FIELD_MASK) != (responseFrame_User_If_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}

/**
 * @brief Read status register values.
 *
.* @param Status - reference to SCH1 status register structure.
 *
 * @return SCH1_MT_OK = success, SCH1_MT_ERR_* = failure. Please see header file for error definitions.
 */
int SCH1_getStatus(SCH1_status *Status)
{
    if (Status == nullptr) {
        return SCH1_ERR_NULL_POINTER;
    }
    
    SCH1_sendRequest(REQ_READ_STAT_SUM);
    Status->Summary     = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_SUM_SAT));
    Status->Summary_Sat = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_COM));
    Status->Common      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_COM));
    Status->Rate_Common = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_X));
    Status->Rate_X      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_Y));
    Status->Rate_Y      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_RATE_Z));
    Status->Rate_Z      = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_X));    
    Status->Acc_X       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Y));
    Status->Acc_Y       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Z));
    Status->Acc_Z       = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_STAT_ACC_Z));

    return SCH1_OK;
}

/**
 * @brief Verify if all status registers show OK condition.
 *
.* @param Status - reference to SCH1 status register structure.
 *
 * @return true = no status failures
 *         false = at least one status failure.
 */
bool SCH1_verifyStatus(SCH1_status *Status)
{
    if (Status == nullptr) {
        return SCH1_ERR_NULL_POINTER;
    }

    if (Status->Summary != 0xffff)
        return false;
    if (Status->Summary_Sat != 0xffff)
        return false;
    if (Status->Common != 0xffff)
        return false;
    if (Status->Rate_Common != 0xffff)
        return false;
    if (Status->Rate_X != 0xffff)
        return false;
    if (Status->Rate_Y != 0xffff)
        return false;
    if (Status->Rate_Z != 0xffff)
        return false;
    if (Status->Acc_X != 0xffff)
        return false;
    if (Status->Acc_Y != 0xffff)
        return false;
    if (Status->Acc_Z != 0xffff)
        return false;
    
    return true;
}

/**
 * @brief Read serial number of the SCH1.
 *
 * @param None
 * 
 * @return Serial number string                        
 */
char* SCH1_getSnbr(void)
{
//    uint16_t sn_id1;
//    uint16_t sn_id2;
//    uint16_t sn_id3;
    static char strBuffer[15];
//
//    SCH1_sendRequest(REQ_READ_SN_ID1);
//    sn_id1 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID2));
//    sn_id2 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID3));
//    sn_id3 = SPI48_DATA_UINT16(SCH1_sendRequest(REQ_READ_SN_ID3));
//
//    // Build serial number string
//    snprintf(strBuffer, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
//
    return strBuffer;
}

/**
 * @brief Send SPI request to SCH1.
 *
 * @param  Request - 48-bit MOSI data
 * 
 * @return 48-bit received MISO line data.                     
 */
inline uint64_t SCH1_sendRequest(uint64_t Request)
{
    return hw_SPI48_Send_Request(Request);
}

/**
 * @brief Calculate CRC8 for the 48-bit SPI-frame.
 * 
 * @param SPIframe - 48-bit SPI-frame for which the CRC is calculated.
 * 
 * @return CRC for the given SPI-frame.                   
 */
uint8_t CRC8(uint64_t SPIframe)
{
    uint64_t data = SPIframe & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }
    
    return crc;
}

/**
 * @brief Calculate CRC3 for the 32-bit SPI-frame.
 * 
 * @param SPIframe - 32-bit SPI-frame for which the CRC is calculated.
 * 
 * @return CRC for the given SPI-frame.                   
 */
uint8_t CRC3(uint32_t SPIframe)
{
    uint32_t data = SPIframe & 0xFFFFFFF8;
    uint8_t crc = 0x05;
 
    for (int i = 31; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
        crc &= 0x07;
    }
 
    return crc;
}

/**
 * @brief Initialize the SCH1 sensor.
 *
 * @param sFilter - structure containing filter settings for all channels.
 *        sSensitivity - structure containing sensitivity settings for all channels.
 *        sDecimation - structure containing decimation settings for decimated channels.
 *        enableDRY - true = enable DRY interrupt, false = disable DRY interrupt.
 * 
 * @return SCH1_OK = success
 *         SCH1_ERR_* = failure. Please see header file for error definitions.                      
 */
int SCH1_init()
{
    int ret = SCH1_OK;
    uint8_t startup_attempt = 0;
    bool SCH1status = false;
    SCH1_status SCH1statusAll;
        
    // SCH1 startup sequence specified in section "5 Component Operation,
    // Reset and Power Up" in the data sheet.

    // DIS_EN_SENSOR
    hw_delay(100);
//    ret = SCH1_enableMeas(false, false);
    SCH1_sendSPIreset();
    hw_delay(100);

    for (startup_attempt = 0; startup_attempt < 2; startup_attempt++) {
                                    
        // Wait 32 ms for the non-volatile memory (NVM) Read
        hw_delay(32);

        // Set user controls
        ret = SCH1_setFilters();
        ret = SCH1_setRateSensDec();
        ret = SCH1_setAccSensDec();
        ret = SCH1_setDRY(0, SCH16T_DRY_EN, SCH16T_HighSpeedSPI_EN);   // 0 = DRY active high

        
        // Write EN_SENSOR = 1
        ret = SCH1_enableMeas(true, false);

        // Wait 215 ms
        hw_delay(215);
    
        // Read all status registers once.
        ret = SCH1_getStatus(&SCH1statusAll);

        // Write EOI = 1 (End of Initialization command)
        ret = SCH1_enableMeas(true, true);
        
        // Wait 3 ms
        hw_delay(3);
        
        // Read all status registers twice.
        ret = SCH1_getStatus(&SCH1statusAll);
        ret = SCH1_getStatus(&SCH1statusAll);

        // Read all user control registers and verify content - Add verification here if needed for FuSa.

        // Check that all status registers have OK status.
        if (!SCH1_verifyStatus(&SCH1statusAll)) {
            SCH1status = false;            
            SCH1_reset();    // Sensor failed, reset and retry.
        }
        else {
            SCH1status = true;           
            break;
        }
        
    } // for (startup_attempt = 0; startup_attempt < 2; startup_attempt++) 

    if (!SCH1status)
        ret = SCH1_ERR_SENSOR_INIT;
             
    return ret;
}


/**
 * @brief Read rate, acceleration and temperature data from sensor. Called by sampling_callback()
 *
 * @param data - pointer to "raw" data from sensor
 * 
 * @return None                       
 */
void SCH1_getData(SCH1_raw_data *data,
                  bool read_gyro, bool read_accel, bool read_temp)
{
    static uint64_t miso_buffer[7];
    int miso_buffer_index = 0;
    bool first_frame_sent = false;

    if(read_gyro){
        SCH1_sendRequest(REQ_READ_RATE_X1);
        miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_RATE_Y1);
        miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_RATE_Z1);
        first_frame_sent = true;
    }
    if(read_accel){
        if(!first_frame_sent){
            SCH1_sendRequest(REQ_READ_ACC_X1);
            first_frame_sent = true;
        }else{
            miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_ACC_X1);
        }
        miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_ACC_Y1);
        miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_ACC_Z1);
    }
    if(read_temp){
        if(!first_frame_sent){
            SCH1_sendRequest(REQ_READ_TEMP);
            first_frame_sent = true;
        }else{
            miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_TEMP);
        }
    }
    // null frame(temp been chosen to read the final data)
    if(first_frame_sent){
        miso_buffer[miso_buffer_index++] = SCH1_sendRequest(REQ_READ_TEMP);
    }

    data->frame_error = SCH1_check_48bit_frame_error(miso_buffer, miso_buffer_index);

    miso_buffer_index = 0;
    if(read_gyro){
        data->Rate1_raw[AXIS_X] = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
        data->Rate1_raw[AXIS_Y] = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
        data->Rate1_raw[AXIS_Z] = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
    }
    if(read_accel){
        data->Acc1_raw[AXIS_X]  = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
        data->Acc1_raw[AXIS_Y]  = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
        data->Acc1_raw[AXIS_Z]  = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]);
    }
    if(read_temp){
        data->Temp_raw = SPI48_DATA_INT32(miso_buffer[miso_buffer_index++]) >> 4;
    }
}

bool SCH1_getData(float gyro[3], float accel[3], float *temperature,
                  bool read_gyro, bool read_accel, bool read_temp){
    static SCH1_raw_data raw_data = {0};
    raw_data.frame_error=true;
    SCH1_getData(&raw_data, read_gyro, read_accel, read_temp);

    if(read_gyro){
        gyro[AXIS_X] = (float)raw_data.Rate1_raw[AXIS_X] / (SCH16T_LSB_perDPS_GYRO) / 180 * M_PI;
        gyro[AXIS_Y] = (float)raw_data.Rate1_raw[AXIS_Y] / (SCH16T_LSB_perDPS_GYRO) / 180 * M_PI;
        gyro[AXIS_Z] = (float)raw_data.Rate1_raw[AXIS_Z] / (SCH16T_LSB_perDPS_GYRO) / 180 * M_PI;
    }

    if(read_accel){
        accel[AXIS_X] = (float)raw_data.Acc1_raw[AXIS_X] / (SCH16T_LSB_perMS2_ACCEL12);
        accel[AXIS_Y] = (float)raw_data.Acc1_raw[AXIS_Y] / (SCH16T_LSB_perMS2_ACCEL12);
        accel[AXIS_Z] = (float)raw_data.Acc1_raw[AXIS_Z] / (SCH16T_LSB_perMS2_ACCEL12);
    }

    if(read_temp){
        *temperature = (float)raw_data.Temp_raw / SCH16T_LSB_PerDegreeCelsius;
    }
    return raw_data.frame_error;
}


/**
 * @brief Convert summed raw data from sensor to real values. Also calculate averages values.
 *
 * @param data_in - pointer to summed raw data from sensor
 *        data_out - pointer to converted values
 * 
 * @return None                       
 */
void SCH1_convert_data(SCH1_raw_data *data_in, SCH1_result *data_out)
{
    // Convert from raw counts to sensitivity and calculate averages here for faster execution
    data_out->Rate1[AXIS_X] = (float)data_in->Rate1_raw[AXIS_X] / (SCH16T_LSB_perDPS_GYRO);
    data_out->Rate1[AXIS_Y] = (float)data_in->Rate1_raw[AXIS_Y] / (SCH16T_LSB_perDPS_GYRO);
    data_out->Rate1[AXIS_Z] = (float)data_in->Rate1_raw[AXIS_Z] / (SCH16T_LSB_perDPS_GYRO);
    data_out->Acc1[AXIS_X]  = (float)data_in->Acc1_raw[AXIS_X] / (SCH16T_LSB_perMS2_ACCEL12);
    data_out->Acc1[AXIS_Y]  = (float)data_in->Acc1_raw[AXIS_Y] / (SCH16T_LSB_perMS2_ACCEL12);
    data_out->Acc1[AXIS_Z]  = (float)data_in->Acc1_raw[AXIS_Z] / (SCH16T_LSB_perMS2_ACCEL12);

    // Convert temperature and calculate average
    data_out->Temp = (float)data_in->Temp_raw / SCH16T_LSB_PerDegreeCelsius;
        
}


/**
 * @brief Check if 48-bit MISO frames have any error bits set. Return true on the first error encountered.
 *
 * @param data - pointer to 48-bit MISO frames from sensor
 *        size - number of frames to check
 * 
 * @return true = any error bit set
 *         false = no error
 */
bool SCH1_check_48bit_frame_error(uint64_t *data, int size)
{
    for (int i = 0; i < size; i++) 
    {
        uint64_t value = data[i];
        if (value & ERROR_FIELD_MASK)
            return true;
        // 2) 数据场全 0 或全 1（在断线/掉电时常见）也认为是错误
        uint64_t data_field = value & DATA_FIELD_MASK;
        if (data_field == 0x0ULL)
            return true;
        if (data_field == DATA_FIELD_MASK)
            return true;
    }
    
    return false;
}
