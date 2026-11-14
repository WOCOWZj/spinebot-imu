#ifndef H7_IMU_ALG_SLIDE_STATISTICS_H
#define H7_IMU_ALG_SLIDE_STATISTICS_H

template<int BufferSize>
class alg_slide_statistics {
public:
    alg_slide_statistics() : current_index(0), mean(0.0f), variance(0.0f) {
        for (int i = 0; i < BufferSize; ++i) {
            data_buffer[i] = 0.0f;
        }
        beta_n = 1.0f / (BufferSize - 1);
        beta_n_plus_1 = 1.0f / BufferSize;
    }

    void Update(float data){
        mean = mean - (data_buffer[current_index] - mean) * beta_n;
        variance = (variance - beta_n_plus_1*(data_buffer[current_index] - mean)*(data_buffer[current_index] - mean)) / (1.0f - beta_n);
        data_buffer[current_index] = data;
        variance = (1.0f - beta_n) * variance + beta_n_plus_1*(data_buffer[current_index] - mean) * (data_buffer[current_index] - mean);
        mean = mean + (data_buffer[current_index] - mean) * beta_n_plus_1;
        current_index = (current_index + 1) % BufferSize;

        if(update_times < BufferSize){
            update_times++;
        }
    };
    float GetVariance(){ return variance; };
    float GetMean(){ return mean;};
    int GetBufferSize() { return BufferSize; };
    bool isBufferFull() {return update_times >= BufferSize;};

private:
    int current_index;
    float data_buffer[BufferSize]{};
    float mean;
    float variance;

    float beta_n;
    float beta_n_plus_1;
    int update_times = 0;
};


#endif //H7_IMU_ALG_SLIDE_STATISTICS_H
