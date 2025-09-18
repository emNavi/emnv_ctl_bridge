#ifndef ACC_FUSION_HPP
#define ACC_FUSION_HPP

#include <vector>
#include <cmath>
#include <cstdlib>

class AccelerationFusion {
public:
    AccelerationFusion(double fs,
                       double notchFreq = 3.0,
                       double notchQ = 10.0,
                       double lowPassCutoff = 1.0,
                       double w_imu = 0.7,
                       double w_pos = 0.3)
        : fs_(fs), dt_(1.0/fs), notchFreq_(notchFreq), notchQ_(notchQ),
          lowPassCutoff_(lowPassCutoff), w_imu_(w_imu), w_pos_(w_pos) {}
    struct DataPoint {
        double time;
        double pos_z;
        double acc;
    };

    #include <deque>

    std::deque<DataPoint> data_queue_;
    std::deque<DataPoint> calibrated_acc_queue_;
    void addDataPoint(double time, double pos_z, double acc) {
        DataPoint point{time, pos_z, acc};
        data_queue_.push_back(point);
        while (!data_queue_.empty() && (point.time - data_queue_.front().time) > 3.5) {
            data_queue_.pop_front();
        }
    }
    double getDataDuration() const {
        if (data_queue_.empty()) return 0.0;
        return data_queue_.back().time - data_queue_.front().time;
    }
    void clearData() {
        std::deque<DataPoint> empty;
        std::swap(data_queue_, empty);
    }
    void update(std::vector<double> &t, std::vector<double> &acc_fused){
        std::vector<double> acc_raw;
        std::vector<double> pos;
        for(const auto& dp : data_queue_){
            acc_raw.push_back(dp.acc);
            pos.push_back(dp.pos_z);
            t.push_back(dp.time);
        }
        fuse(acc_raw, pos, acc_fused);
    }




    // 主融合接口
    void fuse(const std::vector<double>& acc_raw,
              const std::vector<double>& pos,
              std::vector<double>& acc_fused)
    {
        std::vector<double> acc_notch;
        notchFilter(acc_raw, acc_notch, notchFreq_, notchQ_, fs_);

        std::vector<double> pos_low;
        butterworthLowPass(pos, pos_low, lowPassCutoff_, fs_);

        std::vector<double> acc_pos_low;
        positionToAcceleration(pos_low, acc_pos_low, dt_);

        // 对齐IMU长度
        std::vector<double> acc_notch_aligned(acc_pos_low.size());
        for(size_t i=0;i<acc_pos_low.size();++i){
            acc_notch_aligned[i] = acc_notch[i+1]; // 中心差分对齐
        }

        // 融合
        acc_fused.resize(acc_pos_low.size());
        for(size_t i=0;i<acc_pos_low.size();++i){
            acc_fused[i] = w_imu_ * acc_notch_aligned[i] + w_pos_ * acc_pos_low[i];
        }
    }


private:
    double fs_;
    double dt_;
    double notchFreq_;
    double notchQ_;
    double lowPassCutoff_;
    double w_imu_;
    double w_pos_;

    // ----------------------------
    // 低通滤波 (一阶Butterworth)
    // ----------------------------
    void butterworthLowPass(const std::vector<double>& input, std::vector<double>& output,
                            double cutoffFreq, double fs)
    {
        output.resize(input.size());
        double RC = 1.0 / (2 * M_PI * cutoffFreq);
        double alpha = dt_ / (RC + dt_);

        output[0] = input[0];
        for(size_t i = 1; i < input.size(); ++i) {
            output[i] = output[i-1] + alpha * (input[i] - output[i-1]);
        }
    }

    // ----------------------------
    // 简单 IIR notch滤波
    // ----------------------------
    void notchFilter(const std::vector<double>& input, std::vector<double>& output,
                     double f0, double Q, double fs)
    {
        output.resize(input.size());
        double w0 = 2 * M_PI * f0 / fs;
        double alpha = sin(w0)/(2*Q);
        double b0 = 1;
        double b1 = -2*cos(w0);
        double b2 = 1;
        double a0 = 1 + alpha;
        double a1 = -2*cos(w0);
        double a2 = 1 - alpha;

        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;

        double x1=0, x2=0, y1=0, y2=0;
        for(size_t i=0;i<input.size();++i){
            double x0 = input[i];
            double y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            output[i] = y0;
            x2 = x1; x1 = x0;
            y2 = y1; y1 = y0;
        }
    }

    // ----------------------------
    // 位置二阶差分得到低频加速度
    // ----------------------------
    void positionToAcceleration(const std::vector<double>& pos, std::vector<double>& acc, double dt)
    {
        size_t N = pos.size();
        acc.resize(N-2);
        for(size_t i=1;i<N-1;++i){
            acc[i-1] = (pos[i+1] - 2*pos[i] + pos[i-1]) / (dt*dt);
        }
    }
};

#endif // ACC_FUSION_HPP
