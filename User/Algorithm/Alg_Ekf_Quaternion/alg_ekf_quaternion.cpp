//
// Created by 92428 on 2024/8/21.
//
#include "alg_ekf_quaternion.h"

void Alg_Ekf_Quaternion::setState(Quaternion &x0) {x_Post = x0; x_Pri = x0;}

Quaternion Alg_Ekf_Quaternion::getState() { return x_Post; }

Quaternion Alg_Ekf_Quaternion::predict() {
//    F = jacobian_f(x_Post),Q = update_Q(), W = update_W(x_Post);
    F = jacobian_f(x_Post), W = update_W(x_Post);

//    x_Pri = f(x_Post);
    x_Pri = F * x_Post;
//    P_Pri = F * P_Post * F.trans() + W * Q * W.trans();
    P_Pri = F * P_Post * F.trans() + W * W.trans() * r_gyro_ * r_gyro_ * dt_ * dt_ * decrease_factor * decrease_factor;
    decrease_factor *= decrease_rate;

// handle the case when there will be no measurement before the next predict
    x_Post = x_Pri;
    P_Post = P_Pri;

    return x_Pri;
}


Quaternion Alg_Ekf_Quaternion::update(Matrixf<3, 1> &z,float a_square) {
    H = jacobian_h(x_Pri), R = update_R(a_square);
    Matrixf<3, 3> temp = H * P_Pri * H.trans() + R;
    K = P_Pri * H.trans() * (matrixf::inv(temp));
    x_Post = x_Pri + K * (z - h(x_Pri));
    Matrixf<4, 4> temp2 = I - K * H;
    P_Post = temp2 * P_Pri * temp2.trans() + K * R *K.trans();
//    P_Post = temp2 * P_Pri;
    C = jacobian_c(x_Post);
    P_Post = C * P_Post * C.trans();

    decrease_factor = 1.0f;
    return x_Post;
}


float Alg_Ekf_Quaternion::chi_square_test(Matrixf<3, 1> &z,float a_square) {
    Matrixf<3,1> e_k = z - h(x_Pri);
    Matrixf<3,3> D_k = jacobian_h(x_Pri) * P_Pri * jacobian_h(x_Pri).trans() + update_R(a_square);
    Matrixf<3,3> D_k_inv = matrixf::inv(D_k);
    float r_k = (e_k.trans() * D_k_inv * e_k)[0][0] * (D_k[0][0] + D_k[1][1] + D_k[2][2]) / 3;
    return r_k;
}


Quaternion Alg_Ekf_Quaternion::f(Quaternion & x) {
    float omega_matrix_buffer[4*4] =
            {0, -gyro_omega_original[0][0], -gyro_omega_original[1][0], -gyro_omega_original[2][0],
            gyro_omega_original[0][0], 0, gyro_omega_original[2][0], -gyro_omega_original[1][0],
            gyro_omega_original[1][0], -gyro_omega_original[2][0], 0, gyro_omega_original[0][0],
            gyro_omega_original[2][0], gyro_omega_original[1][0], -gyro_omega_original[0][0], 0};
    Matrixf<4, 4> omega_matrix(omega_matrix_buffer);
    Matrixf<4 ,1> x_new = x + (1.0f / 2 * dt_ * omega_matrix * x);
    return x_new;
}

Matrixf<4, 4> Alg_Ekf_Quaternion::jacobian_f(Quaternion &x) {
    float omega_matrix_buffer[4*4] =
            {0, -gyro_omega_original[0][0], -gyro_omega_original[1][0], -gyro_omega_original[2][0],
             gyro_omega_original[0][0], 0, gyro_omega_original[2][0], -gyro_omega_original[1][0],
             gyro_omega_original[1][0], -gyro_omega_original[2][0], 0, gyro_omega_original[0][0],
             gyro_omega_original[2][0], gyro_omega_original[1][0], -gyro_omega_original[0][0], 0};
    Matrixf<4, 4> omega_matrix(omega_matrix_buffer);
    Matrixf<4, 4> j_f_matrix = matrixf::Identity<4,4>() + 1.0f / 2 * dt_ * omega_matrix;
    return j_f_matrix;
}

Matrixf<4, 3> Alg_Ekf_Quaternion::update_W(Quaternion &x) {
    float u_w_matrix_buffer[4*3] =
            {-x[1], -x[2], -x[3],
            x[0], -x[3], x[2],
            x[3], x[0], -x[1],
            -x[2], x[1], x[0]};
    Matrixf<4, 3> u_w_matrix(u_w_matrix_buffer);
    u_w_matrix = 1.0f / 2 * dt_ * u_w_matrix / x.norm();
    return u_w_matrix;
}

Matrixf<3, 1> Alg_Ekf_Quaternion::h(Quaternion &x) {
    Matrixf<3, 1> h;
    Quaternion x_normalized = x / x.norm();
    h[0][0] = 2 * (x_normalized[1] * x_normalized[3] - x_normalized[0] * x_normalized[2]);
    h[1][0] = 2 * (x_normalized[2] * x_normalized[3] + x_normalized[0] * x_normalized[1]);
    h[2][0] = x_normalized[0]*x_normalized[0] - x_normalized[1]*x_normalized[1] - x_normalized[2]*x_normalized[2] + x_normalized[3]*x_normalized[3];
    return h;
}

Matrixf<3, 4> Alg_Ekf_Quaternion::jacobian_h(Quaternion &x) {
    Quaternion x_normalized = x / x.norm();
    float j_h_matrix_buffer[3*4] =
            {   -2*x_normalized[2], 2*x_normalized[3], -2*x_normalized[0], 2*x_normalized[1],
                2*x_normalized[1], 2*x_normalized[0], 2*x_normalized[3], 2*x_normalized[2],
                2*x_normalized[0], -2*x_normalized[1], -2*x_normalized[2], 2*x_normalized[3]};
    Matrixf<3, 4> j_h_matrix = Matrixf<3, 4>(j_h_matrix_buffer);
    return j_h_matrix;
}

Matrixf<4, 4> Alg_Ekf_Quaternion::jacobian_c(Quaternion &x) {
    float j_c_matrix_buffer[4*4] = {
            x[1]*x[1] + x[2]*x[2] + x[3]*x[3],    -x[0]*x[1], -x[0]*x[2], -x[0]*x[3],
            -x[1]*x[0],                                       x[0]*x[0] + x[2]*x[2] + x[3]*x[3], -x[1]*x[2], -x[1]*x[3],
            -x[2]*x[0],                                       -x[2]*x[1], x[0]*x[0] + x[1]*x[1] + x[3]*x[3], -x[2]*x[3],
            -x[3]*x[0],                                     -x[3]*x[1], -x[3]*x[2], x[0]*x[0] + x[1]*x[1] + x[2]*x[2]
    };
    Matrixf<4, 4> j_c_matrix(j_c_matrix_buffer);
    float d = x.norm();
    j_c_matrix = 1.0f / (d * d * d) * j_c_matrix;
    return j_c_matrix;
}

Matrixf<3, 3> Alg_Ekf_Quaternion::update_R(float a_square) {
    float r_factor = (k_move * sqrt(fabs(a_square - 96.0f))/2.0f/9.8f + r_accel_);
//    float r_factor = r_accel_;
    r_factor *= r_factor;
    float u_r_matrix_buffer[3*3] = {
            r_factor, 0, 0,
            0, r_factor, 0,
            0, 0, r_factor
    };
    Matrixf<3, 3> u_r_matrix(u_r_matrix_buffer);
    return u_r_matrix;
}

Matrixf<3, 3> Alg_Ekf_Quaternion::update_Q() {
    float u_q_matrix_buffer[3*3] = {
            r_gyro_ * r_gyro_, 0, 0,
            0, r_gyro_ * r_gyro_, 0,
            0, 0, r_gyro_ * r_gyro_
    };
    Matrixf<3, 3> u_q_matrix(u_q_matrix_buffer);
    return u_q_matrix;
}
