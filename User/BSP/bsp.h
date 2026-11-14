//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_H
#define H7_IMU_BSP_H

#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "bsp_usb.h"
#include "bsp_imu.h"
#include "bsp_vbat.h"
#include "bsp_heater.h"
#include "bsp_time_stamp.h"
#include "bsp_reset.h"
#include "bsp_Flash.h"
#include "bsp_fdcan.h"

class BSP_Pack{
public:
    void Init();


    BSP_Buzzer buzzer;
    BSP_LED led;
    BSP_USB usb;
    BSP_IMU imu;
    BSP_Vbat vbat;
    BSP_Heater heater;
    BSP_Time_Stamp_us time_stamp_us;
    BSP_Reset software_reset;
    bsp_Flash flash;
};


#endif //H7_IMU_BSP_H
