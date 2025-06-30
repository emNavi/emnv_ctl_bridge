#ifndef __PARAMS_PARSE_HPP_
#define __PARAMS_PARSE_HPP_
#include <Eigen/Eigen>
struct ParamsParse
{
    std::string ctrl_out_level;
    std::string ctrl_mode;

    double takeoff_height;
    int drone_id;
    double loop_rate;
    std::string name;
};

#endif
