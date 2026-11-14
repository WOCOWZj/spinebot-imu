//
// Created by 92428 on 2024/8/21.
//

#ifndef H7_IMU_ALG_EKF_QUATERNION_H
#define H7_IMU_ALG_EKF_QUATERNION_H

#include "alg_matrix.h"
#include "alg_quaternion.h"

class Alg_Ekf_Quaternion{
public:
    float dt_;
    Matrixf<3,1> gyro_omega_original;

    void setState(Quaternion & x0);
    Quaternion getState();
    Quaternion predict();
    Quaternion update(Matrixf<3,1> & z,float a_square);
    float chi_square_test(Matrixf<3,1> & z,float a_square);

    float decrease_factor = 1.0f;

private:
    // EKF
    // state: q0, q1, q2, q3
    // measurement: ax, ay, az

//    float r_accel_ = 0.002;
//    float r_accel_ = 0.02;
//    float r_accel_ = 0.00004;
    float r_accel_ = 0.00006;
    float k_move = 100 * r_accel_;        //  k * sqrt(a^2 - g^2)/2/g + r
    float r_gyro_ = 20.0; // r*r*dt*dt will be the final noise

    float decrease_rate = 1.0f; // act on r_gyro_ when no update

    Quaternion f(Quaternion & x);
    Matrixf<4,4> jacobian_f(Quaternion & x);
    Matrixf<4,4> F;

    Matrixf<3,1> h(Quaternion & x);
    Matrixf<3,4> jacobian_h(Quaternion & x);
    Matrixf<3,4> H;

    Matrixf<4,4> jacobian_c(Quaternion & x);
    Matrixf<4,4> C;

    Matrixf<4,3> update_W(Quaternion & x);
    Matrixf<4,3> W;

    Matrixf<3,3> update_R(float a_square);
    Matrixf<3,3> R;

    Matrixf<3,3> update_Q();
    Matrixf<3,3> Q;

    Matrixf<4,4> P_Pri;
    Matrixf<4,4> P_Post = matrixf::Identity<4,4>()*1e-10;

    Matrixf<4,3> K;

    Matrixf <4,4> I = matrixf::Identity<4,4>();

    Quaternion x_Pri;
    Quaternion x_Post;
};


#endif //H7_IMU_ALG_EKF_QUATERNION_H
