//
// Created by 92428 on 2024/8/13.
//

#include "bsp_vbat.h"

void BSP_Vbat::Init() {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
}

float BSP_Vbat::GetVoltage() {
    SCB_InvalidateDCache_by_Addr ((uint32_t *)adc_val, 4);
    return (float)(adc_val[0]*3.3f/65535)*11.0f;
}