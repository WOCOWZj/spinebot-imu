//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_LED_H
#define H7_IMU_BSP_LED_H

#include "spi.h"
#include "main.h"

class BSP_LED{
public:
    void Init();
    void Change_Color(uint8_t r, uint8_t g, uint8_t b);
    void On();
    void Off();

private:
    inline void Ctrl(uint8_t r, uint8_t g, uint8_t b);
    uint8_t WS2812_LowLevel = 0xC0;
    uint8_t WS2812_HighLevel = 0xF0;
    SPI_HandleTypeDef * _hspi = &hspi6;
    uint8_t  _r = 0;
    uint8_t  _g = 0;
    uint8_t  _b = 0;
};

#endif //H7_IMU_BSP_LED_H
