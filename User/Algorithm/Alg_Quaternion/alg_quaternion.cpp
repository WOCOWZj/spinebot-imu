//
// Created by 92428 on 2024/8/21.
//

#include "alg_quaternion.h"

Quaternion Gravity_Direction_To_Quaternion(Vectorxf<3> & gravity_direction){
    Quaternion quaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // 导航坐标系下的标准重力向量
    Vectorxf<3> gravity({0.0f, 0.0f, 1.0f});

    // 测量的加速度从载体系转换到导航系
    Vectorxf<3> acceleration = (quaternion.toRotationMatrix() * gravity_direction);

    // 计算旋转轴
    Vectorxf<3> rotate_vector = acceleration.cross(gravity);

    // 计算旋转角度
    float theta = acos(acceleration.dot(gravity));

    // 得到旋转对应的四元数
    Vectorxf<3> v = rotate_vector.normalize() * std::sin(theta / 2.0f);
    Quaternion q = Quaternion(std::cos(theta / 2.0f), v[0], v[1], v[2]).normalize();

    // 应用旋转
    // I don't know why the factor to let norm > 1.2 should be multiplied,but it seems work well.
    quaternion = (q * quaternion).normalize() ;

    return quaternion;
}