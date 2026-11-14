//
// Created by 92428 on 2024/8/14.
//

#ifndef H7_IMU_BSP_HEATER_H
#define H7_IMU_BSP_HEATER_H


class BSP_Heater{
public:
    void Init();
    void Set(float out);
    void close();
    float  MAX_OUT = 8000;
private:

};


#endif //H7_IMU_BSP_HEATER_H
