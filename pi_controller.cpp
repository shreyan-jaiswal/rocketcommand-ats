#include <utility>
#include "pi_controller.h"
#include "Eigen/Eigen"

pi_controller::pi_controller(double p_arg, double i_arg){
    p = p_arg;
    i = i_arg;
}

double pi_controller::control(Matrix<2,1 double> e){
    double output = p * e(0,0) + i * e(1,0);
    if(output < 0.) return 0.;
    if(output > 0.125) return 0.125;
    return output;
}
