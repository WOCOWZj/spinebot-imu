//
// Created by 92428 on 2024/8/13.
//

#include "bsp_usb.h"

void BSP_USB::Init(uint8_t * pdata, uint16_t buffer_size){
    send_buffer = pdata;
    send_buffer_size = buffer_size;

    for(int i = 0; i < usb_send_data_size; i++) {
        usb_send_data_list[i].id = i;
        usb_send_data_list[i].is_send = 1;
        usb_send_data_list[i].len = 0;
        usb_send_data_list[i].data = nullptr;
    }
}

void BSP_USB::Add_Data(uint8_t *data, uint16_t len, uint8_t _id) {
    if(_id < usb_send_data_size) {
        usb_send_data_list[_id].data = data;
        usb_send_data_list[_id].len = len;
        usb_send_data_list[_id].is_send = 0;
    }
}

void BSP_USB::Send_Data() {
    uint16_t temp_len = 0;
    for(int i = 0; i < usb_send_data_size; i++) {
        if (usb_send_data_list[i].is_send == 0) {
            if(temp_len + usb_send_data_list[i].len > send_buffer_size) {
                break;
            }
            memcpy(send_buffer + temp_len, usb_send_data_list[i].data, usb_send_data_list[i].len);
            temp_len += usb_send_data_list[i].len;
            usb_send_data_list[i].is_send = 1;
        }
    }
    if (temp_len > 0)
        SCB_CleanDCache_by_Addr((uint32_t *)send_buffer, temp_len);
        CDC_Transmit_HS(send_buffer, temp_len);
}

