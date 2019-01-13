#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include "LINR.h"
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

typedef pair<double, double> state_t;
typedef Matrix<double, 11, 1> Vector11d;
typedef Matrix<double, 11, 11> Matrix11d;

LINR::LINR(){
    // constant system parameters
    m = 1.24;
    arm_len = 0.3;
    hub_rad = 0.1;
    fin_wid = 0.16;
    rocket_rad = 0.25;

    // constant physical parameters
    g = 32.;
    ro = 0.00237; //slugs/ft3
    MATH_PI = 3.14159265358979;

    // constant general prediction parameters
    apo_goal = 5100.;
    dv = -1.;

    // constant LINR prediction parameters
    W_size = 11;
    inv_ff = 1/0.9995;

    // variable LINR prediction parameters
    W << 0.4374, -0.0016, 2.5e-6, 0., 0., 0., 0., 0.0088, 0., 0., 0.;
    P << Matrix11d::Identity();
}

double LINR::compute_cd(state_t X_v, double U_v){
    double v1 = X_v.first;
    double d1 = U_v;
    Vector11d Z;
    Z << 1., v1, pow(v1, 2), pow(v1, 3), pow(v1, 4), pow(v1, 5), pow(v1, 6), d1, pow(d1, 2), pow(d1, 3), pow(d1, 4);
    return Z.dot(W);
}
double LINR::compute_fd(state_t X_v, double U_v){
    double area = compute_area(U_v);
    return compute_cd(X_v, U_v) * 0.5 * ro * area * X_v.first * X_v.first;
}
double LINR::compute_area(double U_v){
    double theta = U_v;
    double area_flaps = 0.;
    area_flaps += fin_wid*hub_rad*sin((MATH_PI/180)*theta);
    area_flaps += sqrt(arm_len*arm_len-hub_rad*hub_rad*cos((MATH_PI/180)*theta)*cos((MATH_PI/180)*theta) );
    area_flaps -= sqrt(arm_len*arm_len-hub_rad*hub_rad);
    area_flaps *= 3;
    double area_rocket = MATH_PI*rocket_rad*rocket_rad;
    double area = area_flaps + area_rocket;
    return area;
}
state_t LINR::ss_predict(state_t X_v, double U_v){
    state_t X_vp = state_t();
    double v = X_v.first;
    double h = X_v.second;
    double fd = compute_fd(X_v, U_v);
    double dh = v*dv / (-g-(fd/m));
    X_vp.first = v+dv;
    X_vp.second = h+dh;
    return X_vp;
}
state_t LINR::ms_predict(state_t X_v, double U_v){
    double v = X_v.first;
    state_t X_vpm1;
    int loopmax = (int) round(v/fabs(dv));
    for(int i = 1; i <= loopmax ;i++){
        X_vpm1 = ss_predict(X_v,U_v);
        X_v = X_vpm1;
    }
    return X_v;
}
void LINR::update(state_t X_v, state_t X_vm1, double U_vm1){
    //calculation of the actual cd of the flaps over
    //the last time step
    double area = compute_area(U_vm1);

    double fd_vm1 = -m*( (X_v.first*X_v.first-X_vm1.first*X_vm1.first)/(2*(X_v.second-X_vm1.second))+g );

    double cd_vm1 = fd_vm1/(0.5*ro*area*X_vm1.first*X_vm1.first);

    //calculation of the predicted cd of the flaps
    //over the last time step
    double cd_vpm1 = compute_cd(X_vm1,U_vm1);

    //obtain relevant state data
    double d1 = U_vm1;
    double v1 = X_vm1.first;

    //perform recursive update with this convoluted formula
    Vector11d Z;
    Z << 1., v1, pow(v1, 2), pow(v1, 3), pow(v1, 4), pow(v1, 5), pow(v1, 6), d1, pow(d1, 2), pow(d1, 3), pow(d1, 4);
    Vector11d k = (P*Z*inv_ff) / (1+inv_ff*Z.dot(P*Z));
    double e = cd_vm1-cd_vpm1; //
    W = W + e*k;
    P = inv_ff*(P - k*(Z.transpose()*P));
}

void LINR::set_dv(double dv_arg){
    dv = dv_arg;
}
