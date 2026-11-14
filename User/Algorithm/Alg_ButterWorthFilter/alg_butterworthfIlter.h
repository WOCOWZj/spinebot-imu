//
// Created by 92428 on 2024/8/26.
//

#ifndef H7_IMU_ALG_BUTTERWORTHFILTER_H
#define H7_IMU_ALG_BUTTERWORTHFILTER_H

class Alg_ButterWorthFilter {
public:
    float Update(float input_sample) {
        float output = b[0] * input_sample;

        // Apply recursive IIR calculation
        for (int i = 1; i <= ORDER; ++i) {
            int p = (pos + ORDER - i) % ORDER;
            output += b[i] * old_x[p] - a[i] * old_y[p];
        }

        old_x[pos] = input_sample;
        old_y[pos] = output;

        pos = (pos + 1) % ORDER;

        return output;
    }

    float GetLast() const {
        int last_pos = (pos + ORDER - 1) % ORDER;
        return old_y[last_pos];
    }

private:
    static constexpr int ORDER = 4;

    // 系数（注意：a[0] = 1 不参与计算）
    const float b[ORDER + 1] = {
            0.0186   , 0.0743  ,  0.1114  ,  0.0743  ,  0.0186
    };
    const float a[ORDER + 1] = {
            1.0000  , -1.5704 ,   1.2756 ,  -0.4844  ,  0.0762
    };

    float old_x[ORDER] = {0.0f};   // 输入延迟线
    float old_y[ORDER] = {0.0f};   // 输出延迟线
    int pos = 0;                   // 当前缓冲区位置指针
};

#endif //H7_IMU_ALG_BUTTERWORTHFILTER_H
