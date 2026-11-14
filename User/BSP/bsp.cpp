//
// Created by 92428 on 2024/8/13.
//

#include "bsp.h"

BSP_Pack BSP_Instance;

static uint8_t BSP_USB_Buffer[1024] = {0};

void BSP_Pack::Init() {
    buzzer.Init();
    led.Init();
    usb.Init(BSP_USB_Buffer, 1024);
    imu.Init();
    vbat.Init();
    heater.Init();
    time_stamp_us.init();
    flash.Init();

    FDCAN1_Manager_Obj.Init(&hfdcan1);
}
