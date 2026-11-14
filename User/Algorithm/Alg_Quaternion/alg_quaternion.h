//
// Created by 92428 on 2024/8/21.
//

#ifndef H7_IMU_ALG_QUATERNION_H
#define H7_IMU_ALG_QUATERNION_H

#include "alg_matrix.h"

//using Quaternion = Matrixf<4, 1>;

template <int Size>
//using Vectorxf = Matrixf<Size, 1>;
class Vectorxf : public Matrixf<Size, 1>{
public:
    // constructor
    using Matrixf<Size,1>::Matrixf;
    Vectorxf(const Matrixf<Size, 1> & mat) : Matrixf<Size, 1>(mat){}
    // from {} like eigen
    Vectorxf(const float (&arr)[Size]) : Matrixf<Size, 1>(){
        memcpy(this->data_, arr, sizeof(float) * Size);
    }
    //Element
    float operator[](const int& row) const { return this->data_[row]; }

    // Assignment operator to allow assignment from an initializer list
    Vectorxf<Size>& operator=(const float (&arr)[Size]) {
        memcpy(this->data_, arr, sizeof(float) * Size);
        return *this;
    }

    // Modulus
    float modulus_square(){
        float result = 0;
        for (int i = 0; i < Size; i++) {
            result += this->data_[i] * this->data_[i];
        }
        return result;
    }
    // Norm
    float norm() {
        float temp = this->modulus_square();
        arm_sqrt_f32((this->trans() * *this)[0][0],&temp);
        return temp; }
    // Normalize
    Vectorxf<Size> normalize(){
        return *this / this->norm();
    }
    // Scale
    void scale(float _scale){
        for(int i = 0; i < Size; i++){
            this->data_[i] *= _scale;
        }
    }
    // Sum
    float sum(){
        float result = 0;
        for(int i = 0; i < Size; i++){
            result += this->data_[i];
        }
        return result;
    }
    // cross
    Vectorxf<Size> cross(Vectorxf<Size> & vec){
        static_assert(Size == 3, "cross product is only defined for 3D vectors");
        return Vectorxf<Size>({this->data_[1] * vec[2] - this->data_[2] * vec[1],
                             this->data_[2] * vec[0] - this->data_[0] * vec[2],
                             this->data_[0] * vec[1] - this->data_[1] * vec[0]});
    }
    // dot
    float dot(Vectorxf<Size> & vec){
        float result = 0;
        for(int i = 0; i < Size; i++){
            result += this->data_[i] * vec[i];
        }
        return result;
    }
};

class Quaternion : public Vectorxf<4>{
public:
    using Vectorxf<4>::Vectorxf;
    using Vectorxf<4>::operator[];

    Quaternion() : Vectorxf<4>(){ data_[0] = 1; data_[1] = 0; data_[2] = 0; data_[3] = 0;};
    Quaternion(float q0, float q1, float q2, float q3) : Vectorxf<4>(){
        data_[0] = q0;
        data_[1] = q1;
        data_[2] = q2;
        data_[3] = q3;
    }

    // operators
    Quaternion & operator*(const Quaternion & p){
        static Quaternion result;
        result.data_[0] = data_[0] * p[0] - data_[1] * p[1] - data_[2] * p[2] - data_[3] * p[3];
        result.data_[1] = data_[0] * p[1] + data_[1] * p[0] + data_[2] * p[3] - data_[3] * p[2];
        result.data_[2] = data_[0] * p[2] - data_[1] * p[3] + data_[2] * p[0] + data_[3] * p[1];
        result.data_[3] = data_[0] * p[3] + data_[1] * p[2] - data_[2] * p[1] + data_[3] * p[0];
        return result;
    }

    Matrixf<3,3> toRotationMatrix(){
        static Matrixf<3,3> R;
        static float tx, ty, tz, twx, twy, twz, txx, txy, txz, tyy, tyz, tzz;
        static float norm;
        static float normalized_data[4];
        // Normalize
        norm = this->norm();
        if(norm < 1e-6f){
            norm = 1.0f;
        }
        for(int i = 0; i < 4; i++){
            normalized_data[i] = data_[i] / norm;
        }

        tx = 2 * normalized_data[1];
        ty = 2 * normalized_data[2];
        tz = 2 * normalized_data[3];
        twx = tx * normalized_data[0];
        twy = ty * normalized_data[0];
        twz = tz * normalized_data[0];
        txx = tx * normalized_data[1];
        txy = ty * normalized_data[1];
        txz = tz * normalized_data[1];
        tyy = ty * normalized_data[2];
        tyz = tz * normalized_data[2];
        tzz = tz * normalized_data[3];


        R[0][0] = 1 - (tyy + tzz);
        R[0][1] = txy - twz;
        R[0][2] = txz + twy;
        R[1][0] = txy + twz;
        R[1][1] = 1 - (txx + tzz);
        R[1][2] = tyz - twx;
        R[2][0] = txz - twy;
        R[2][1] = tyz + twx;
        R[2][2] = 1 - (txx + tyy);

        return R;
    }

    void toEulerAngle(float & roll, float & pitch, float & yaw){
        Quaternion norm = this->normalize();
        roll = atan2(2 * (norm[0] * norm[1] + norm[2] * norm[3]), 1 - 2 * (norm[1] * norm[1] + norm[2] * norm[2]));
        float sinp = 2 * (norm[0] * norm[2] - norm[3] * norm[1]);
        if(fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp);
        else
            pitch = asin(sinp);
        yaw = atan2(2 * (norm[0] * norm[3] + norm[1] * norm[2]), 1 - 2 * (norm[2] * norm[2] + norm[3] * norm[3]));
    }
};




Quaternion Gravity_Direction_To_Quaternion(Vectorxf<3> & gravity_direction);

#endif //H7_IMU_ALG_QUATERNION_H
