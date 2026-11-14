//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_USB_H
#define H7_IMU_BSP_USB_H

#include "usbd_cdc_if.h"

class BSP_USB{
public:
    void Init(uint8_t * pdata, uint16_t buffer_size);
    void Add_Data(uint8_t *data, uint16_t len, uint8_t _id);
    void Send_Data();

private:
    struct usb_data {
        uint8_t id;
        uint8_t is_send;
        uint16_t len;
        uint8_t *data;
    } ;
    uint16_t  usb_send_data_size = 10;
    usb_data usb_send_data_list[10];
    uint8_t * send_buffer;
    uint16_t send_buffer_size = 0;
};


#endif //H7_IMU_BSP_USB_H
