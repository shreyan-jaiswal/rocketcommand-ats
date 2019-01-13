#include <utility>
#include "pi_controller.h"

pi_controller::pi_controller(double p_arg, double i_arg, double discrete_arg = 0.){
    p = p_arg;
    i = i_arg;
    discrete = discrete_arg;
}

double pi_controller::control(std::pair<double, double> e){
    double output = p * e.first + i * e.second;
    if(discrete != 0.){
        output = discrete * round(output/discrete);
    }
    if(output < 0.) return 0.;
    if(output > 90.) return 90.;
    return output;
}
