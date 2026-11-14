//
// Created by 92428 on 2024/8/13.
//

#include "bsp_led.h"

void BSP_LED::Init(){

}

void BSP_LED::Change_Color(uint8_t r, uint8_t g, uint8_t b){
    this->_r = r;
    this->_g = g;
    this->_b = b;

    Ctrl(this->_r,this->_g,this->_b);
}

void BSP_LED::On() {
    Ctrl(this->_r,this->_g,this->_b);
}

void BSP_LED::Off() {
    Ctrl(0,0,0);
}

void BSP_LED::Ctrl(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(_hspi, &res, 0, 0xFFFF);
    while (_hspi->State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(_hspi, txbuf, 24, 0xFFFF);
}
