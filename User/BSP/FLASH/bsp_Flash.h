//
// Created by 92428 on 2025/7/5.
//

#ifndef H7_IMU_BSP_FLASH_H
#define H7_IMU_BSP_FLASH_H

#include "w25q64.h"

class bsp_Flash {
public:
    typedef struct __attribute__((packed)){
        float data0;
        float data1;
        float data2;
        uint32_t crc; // CRC16校验
    }Float3Data;

    void Init();

    uint8_t Write_3Float_WithCRC(float data_0, float data_1, float data_2);

    uint8_t Read_3Float_WithCRC(float &data_0, float &data_1, float &data_2);

private:
    Float3Data W25Qxx_DataBuffer{};

    static uint16_t crc16(const uint8_t *data, uint16_t length) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

};


#endif //H7_IMU_BSP_FLASH_H
