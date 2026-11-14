//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_VBAT_H
#define H7_IMU_BSP_VBAT_H

#include "adc.h"

class BSP_Vbat{
public:
    void Init();
    float GetVoltage();

private:
    uint16_t adc_val[2];
};

#endif //H7_IMU_BSP_VBAT_H
