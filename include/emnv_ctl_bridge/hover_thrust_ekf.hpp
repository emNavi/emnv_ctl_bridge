#ifndef __HOVER_THRUST_EKF_HPP
#define __HOVER_THRUST_EKF_HPP
#include <cmath>
#include <iostream>
#include "emnv_ctl_bridge/my_math.hpp"

static constexpr float CONSTANTS_ONE_G = 9.80665f;						// m/s^2

/**
 * @brief 悬停油门估计
 *
 * 
 */
class HoverThrustEkf
{
private:
    // ////////// 
    bool use_gate_flag{false};

    // //////////
    float _process_var{12.5e-6f}; ///< Hover thrust process noise variance (thrust^2/s^2)
	float _state_var{0.01f}; ///< Initial hover thrust uncertainty variance (thrust^2)
	
    float _dt{0.02f};
    float _acc_var{5.f}; ///< Acceleration variance (m^2/s^3)
	float _acc_var_scale{1.f}; ///< Multiplicator of the measurement variance, used to decrease sensivity 
	float _gate_size{3.f};


	float _hover_thr{0.5f};
	float _hover_thr_min{0.1f};
	float _hover_thr_max{0.9f};

	float _signed_innov_test_ratio_lpf{}; ///< used as a delay to trigger the recovery logic


    float computeH(float thrust) const;
    float computeInnov(float acc_z, float thrust) const;
	float computeInnovVar(float H) const;
	float computeKalmanGain(float H, float innov_var) const;

	void updateState(float K, float innov);
	void updateStateCovariance(float K, float H);

	float computeInnovTestRatio(float innov, float innov_var) const;
	bool isLargeOffsetDetected() const;
	void updateLpf(float residual, float signed_innov_test_ratio);
	void updateMeasurementNoise(float residual, float H);
	void bumpStateVariance();

	float _residual_lpf{}; ///< used to remove the constant bias of the residual

    float _innov{0.f}; ///< Measurement innovation (m/s^2)
	float _innov_var{0.f}; ///< Measurement innovation variance (m^2/s^3)
	float _innov_test_ratio{0.f}; ///< Noramlized Innovation Squared test ratio

    static constexpr float _noise_learning_time_constant = 2.f; ///< in seconds
	static constexpr float _lpf_time_constant = 1.f; ///< in seconds


public:
    /**
     * @brief 初始化参数
     * 默认取消了Gate的使用，少定义了一些变量，可使用enableGate(打开)
     * @param init_hover_thrust 初始悬停油门估计
     * @param hover_thrust_noise 过程噪声|油门噪声 HTE_HT_ERR_INIT
     * @param process_noise 观测噪声|加速度噪声 HTE_HT_NOISE
     * 
     * @return 两个整数的和
     */
    HoverThrustEkf(double init_hover_thrust,double hover_thrust_noise,double process_noise,double thr_max);
    ~HoverThrustEkf();
    void predict(float dt);
	void fuseAccZ(float acc_z, float thrust);
    void enableGate(double gate_size);
    void printLog();
    double getHoverThrust();

};


inline HoverThrustEkf::HoverThrustEkf(double init_hover_thrust,double hover_thrust_noise,double process_noise,double thr_max=0.9)
{
    // 更新频率与position callback频率相同

    _hover_thr = init_hover_thrust;
    _hover_thr_max = thr_max;
    _hover_thr_min = 0.1;

    _state_var = pow(hover_thrust_noise,2);
    _process_var = pow(process_noise,2);
}


inline HoverThrustEkf::~HoverThrustEkf()
{
}
inline void HoverThrustEkf::enableGate(double gate_size)
{
    use_gate_flag=true; 
    _gate_size = gate_size;

}
inline void HoverThrustEkf::printLog()
{
    std::cout<< "hover_thr" << _hover_thr <<" innov "<< _innov << " innov_var " << _innov_var <<" innov_test_ratio "<<_innov_test_ratio<<std::endl;
}


inline double HoverThrustEkf::getHoverThrust(){
    return _hover_thr;
}

// 
/**
 * @brief Predict
 * State is constant,Predict state covariance only
 * @param dt 
 */
inline void HoverThrustEkf::predict(const float dt){
	_state_var += _process_var * dt * dt; // 量纲统一
	_dt = dt;
}

// Update
// 
inline float HoverThrustEkf::computeH(const float thrust) const
{
	return -CONSTANTS_ONE_G * thrust / (_hover_thr * _hover_thr);
}
//Innovation or measurement residual
inline float HoverThrustEkf::computeInnov(const float acc_z, const float thrust) const
{
	const float predicted_acc_z = CONSTANTS_ONE_G * thrust / _hover_thr - CONSTANTS_ONE_G;
	return acc_z - predicted_acc_z;
}

inline float HoverThrustEkf::computeInnovVar(const float H) const
{
	const float R = _acc_var * _acc_var_scale;
	const float P = _state_var;
	return MyMath::max(H * P * H + R, R);
}
inline float HoverThrustEkf::computeKalmanGain(const float H, const float innov_var) const
{
	return _state_var * H / innov_var;
}
inline float HoverThrustEkf::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline void HoverThrustEkf::updateState(const float K, const float innov)
{
	_hover_thr = MyMath::constrain(_hover_thr + K * innov, _hover_thr_min, _hover_thr_max);
}
inline void HoverThrustEkf::updateStateCovariance(const float K, const float H)
{
	_state_var = MyMath::constrain((1.f - K * H) * _state_var, 1e-10f, 1.f);
}

/**
 * @brief update
 * 数据融合
 * @param acc_z z轴方向的加速度
 * @param thrust_z 和油门在z轴方向的分量
 */
inline void HoverThrustEkf::fuseAccZ(const float acc_z, const float thrust_z)
{
	const float H = computeH(thrust_z);
	// std::cout << "H" << H << std::endl;
	const float innov_var = computeInnovVar(H);

	const float innov = computeInnov(acc_z, thrust_z);
	// std::cout << "innov" << innov << std::endl;

	const float K = computeKalmanGain(H, innov_var);
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	float residual = innov;

	if (innov_test_ratio<1.f || !use_gate_flag) {
		updateState(K, innov);
		updateStateCovariance(K, H);
		residual = computeInnov(acc_z, thrust_z); // residual != innovation since the hover thrust changed

	} else if (isLargeOffsetDetected()) {
		// Rejecting all the measurements for some time,
		// it means that the hover thrust suddenly changed or that the EKF
		// is diverging. To recover, we bump the state variance
		bumpStateVariance();
	}

	const float signed_innov_test_ratio = MyMath::sign(innov) * innov_test_ratio;
	updateLpf(residual, signed_innov_test_ratio);
	updateMeasurementNoise(residual, H);

	// save for logging
	_innov = innov;
	_innov_var = innov_var;
	_innov_test_ratio = innov_test_ratio;
}
inline bool HoverThrustEkf::isLargeOffsetDetected() const
{
	return fabsf(_signed_innov_test_ratio_lpf) > 0.2f;
}

inline void HoverThrustEkf::bumpStateVariance()
{
	_state_var += 1e3f * _process_var * _dt * _dt;
}

inline void HoverThrustEkf::updateLpf(const float residual, const float signed_innov_test_ratio)
{
	const float alpha = _dt / (_lpf_time_constant + _dt);
	_residual_lpf = (1.f - alpha) * _residual_lpf + alpha * residual;
	_signed_innov_test_ratio_lpf = (1.f - alpha) * _signed_innov_test_ratio_lpf + alpha * MyMath::constrain(
					       signed_innov_test_ratio, -1.f, 1.f);
}

inline void HoverThrustEkf::updateMeasurementNoise(const float residual, const float H)
{
	const float alpha = _dt / (_noise_learning_time_constant + _dt);
	const float res_no_bias = residual - _residual_lpf;
	const float P = _state_var;
	_acc_var = MyMath::constrain((1.f - alpha) * _acc_var  + alpha * (res_no_bias * res_no_bias + H * P * H), 1.f, 400.f);
}

#endif