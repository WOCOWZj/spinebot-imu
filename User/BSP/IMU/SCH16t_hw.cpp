//****************************************************************************************
// @file    hw.c
// @brief   Hardware related functions.
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

#include "SCH16t_hw.h"

extern SPI_HandleTypeDef hspi1;

/**
  * @brief  Set SCH1600 EXTRESN pin high
  *
  * @param  None
  *
  * @return None
  */
void hw_EXTRESN_High(void)
{
//    HAL_GPIO_WritePin(EXTRESN_PORT, EXTRESN_PIN, GPIO_PIN_SET);
}


/**
  * @brief  Set SCH1600 EXTRESN pin low
  *
  * @param  None
  *
  * @return None
  */
void hw_EXTRESN_Low(void)
{
//    HAL_GPIO_WritePin(EXTRESN_PORT, EXTRESN_PIN, GPIO_PIN_RESET);
}


/**
  * @brief  Set SPI chip select high
  *
  * @param  None
  *
  * @return None
  */
void hw_CS_High(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


/**
  * @brief  Set SPI chip select low
  *
  * @param  None
  *
  * @return None
  */
void hw_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}


/**
 * @brief  Hardware delay function
 *                
 * @params ms - Delay time in milliseconds
 *
 * @return None
 */
void hw_delay(uint32_t ms)
{
    HAL_Delay(ms);
}


/**
  * @brief  Send 48bit request to SPI device, receive 48bit response simultaneously (off-frame protocol).
  *
  * @param  Request - 48 bit MOSI data
  *
  * @return ReceivedData - 48 bit MISO data
  */
uint64_t hw_SPI48_Send_Request(uint64_t Request)
{

    uint64_t ReceivedData = 0;
    uint16_t txBuffer[3];
    uint16_t rxBuffer[3];
    uint8_t index;
    uint8_t size = 3;   // 48-bit SPI-transfer consists of three 16-bit transfers.
    
    // Split Request qword (MOSI data) to tx buffer.
    for (index = 0; index < size; index++)
    {
        txBuffer[size - index - 1] = (Request >> (index << 4)) & 0xFFFF;
    }

    // Send tx buffer and receive rx buffer simultaneously.
    hw_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txBuffer, (uint8_t*)rxBuffer, size, 10);
    hw_CS_High();

    // Create ReceivedData qword from received rx buffer (MISO data).
    for (index = 0; index < size; index++)
    {
    	ReceivedData |= (uint64_t)rxBuffer[index] << ((size - index - 1) << 4);
    }

    return ReceivedData;
}