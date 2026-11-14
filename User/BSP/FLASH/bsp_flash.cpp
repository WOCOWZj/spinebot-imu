//
// Created by 92428 on 2025/7/5.
//

#include "bsp_Flash.h"

void bsp_Flash::Init() {
    OSPI_W25Qxx_Init();
}

uint8_t bsp_Flash::Write_3Float_WithCRC(float data_0, float data_1, float data_2) {
    uint32_t W25Qxx_TestAddr = 0x00;
    int8_t OSPI_Status;

    OSPI_Status = OSPI_W25Qxx_SectorErase(W25Qxx_TestAddr);	// 擦除32K字节
    if(OSPI_Status != OSPI_W25Qxx_OK) return 0;

    W25Qxx_DataBuffer.data0 = data_0;
    W25Qxx_DataBuffer.data1 = data_1;
    W25Qxx_DataBuffer.data2 = data_2;
    W25Qxx_DataBuffer.crc = crc16((uint8_t*)&W25Qxx_DataBuffer, sizeof(W25Qxx_DataBuffer) - sizeof(W25Qxx_DataBuffer.crc));

    OSPI_Status				= OSPI_W25Qxx_WriteBuffer((uint8_t*)&W25Qxx_DataBuffer,W25Qxx_TestAddr,sizeof(Float3Data)); // 写入数据

    if(OSPI_Status != OSPI_W25Qxx_OK) return 0;
    return 1;
}

uint8_t bsp_Flash::Read_3Float_WithCRC(float &data_0, float &data_1, float &data_2) {
    uint32_t W25Qxx_TestAddr = 0;
    int8_t OSPI_Status;

    OSPI_Status = OSPI_W25Qxx_ReadBuffer((uint8_t*)&W25Qxx_DataBuffer, W25Qxx_TestAddr, (uint32_t)sizeof(Float3Data)); // 读取数据

    if(OSPI_Status != OSPI_W25Qxx_OK) return false;

    // CRC校验
    uint16_t crc_calculated = crc16((uint8_t*)&W25Qxx_DataBuffer, sizeof(W25Qxx_DataBuffer) - sizeof(W25Qxx_DataBuffer.crc));
    if(crc_calculated != W25Qxx_DataBuffer.crc) return false;

    data_0 = W25Qxx_DataBuffer.data0;
    data_1 = W25Qxx_DataBuffer.data1;
    data_2 = W25Qxx_DataBuffer.data2;

    return true;
}



