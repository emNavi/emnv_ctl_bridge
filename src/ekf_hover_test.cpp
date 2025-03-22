#include "ctrl_bridge/hover_thrust_ekf.hpp"
#include <random>
#include <cmath>
#include <iostream>
#include <fstream>

int main()
{
    std::ofstream outputFile("../hover_thrust.txt");
    // 检查文件是否成功打开
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return 1;
    }
    // 创建一个随机数引擎对象
    std::random_device rd;
    std::mt19937 gen(rd());

    // 参考6c 默认参数
    std::normal_distribution<> acc_z_noise(0.0, 0.036f);  // 平均值为0，标准差为0.036
    std::normal_distribution<> thrust_noise(0.0, 0.1f);  // 平均值为0，标准差为0.1

    const float hover_thrust_true = 0.6f; //悬停油门真值


    double thrust=0;
    double time=0;
    double acc_z=0;

    HoverThrustEkf hoverThrustEkf(0.4f,0.1f,0.036f);
    // pic6c中设置的 process noise 仅为0.0036 ， 这里放大了一点

    while (time<5)
    {
        // 数据仿真
        thrust = (sin(time/2)+3)/6.0 +thrust_noise(gen);
        acc_z= CONSTANTS_ONE_G * thrust / hover_thrust_true+ acc_z_noise(gen);
        //  - CONSTANTS_ONE_G 
        // EKF
        hoverThrustEkf.predict(0.02); //dt
        hoverThrustEkf.fuseAccZ(acc_z,thrust);         
        time +=0.02;
        // hoverThrustEkf.printLog();
        std::cout<<"_hover_thr "<<hoverThrustEkf.getHoverThrust() << " thrust " << thrust <<  " acc_z " << acc_z <<std::endl;
        outputFile << time << " "<< hoverThrustEkf.getHoverThrust() << std::endl;

    }
    outputFile.close();
}
