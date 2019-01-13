#include "LINR.cpp"
#include "pi_controller.cpp"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

typedef pair<double, double> state_t;

int main(){

    int DISCRETE = 0;
    bool PREDICTED = true;

    LINR r = LINR();

    LINR s = LINR();
    s.set_dv(-.001);

    state_t X_v(580., 1250.);
    double U_v = 0.;

    // create a state and control corresponding to the last state and control
    // measured by the rocket
    state_t X_l(580., 1250.);
    double U_l = 0.;

    //finally, we initialize the controller and the error it operates on
    pi_controller c = pi_controller(0.001,0.006, (double) DISCRETE);
    cout << c.discrete << "\n";
    state_t e(0., 0.);

    vector<double> states;
    vector<double> control;
    int counter = 0;

    state_t X_vpf;

    while(X_v.first>0){
        X_v = s.ss_predict(X_v,U_v);

        //once the speed has changed by the amount specified by the rocket
        //object, we perform a control cycle
        if(X_v.first - X_l.first <= r.dv){

            //make a prediction and enact control

            X_vpf = r.ms_predict(X_v,U_v);
            if(PREDICTED) states.push_back(X_vpf.second);
            else states.push_back(X_v.second);
            control.push_back(U_l);
            e.first = X_vpf.second-r.apo_goal;
            e.second = e.second+e.first;
            U_v = c.control(e);

            //enact a recursive update on the predictive model using the state
            //seen by the model on the last velocity step
            //r.update(X_v,X_l,U_l);

            //update the last state seen by the controller to the current state
            X_l = X_v;
            U_l = U_v;
            counter++;
        }
    }

    ofstream fout;
    ostringstream oss;
    oss << DISCRETE ;
    if(PREDICTED) oss << "p";
    oss << ".txt";
    string fname = oss.str();
    fout.open(fname.c_str());
    for(int i = 0; i < counter; i++){
        fout << states[i];
        if(i != counter - 1) fout << " ";
    }
    fout.close();

    return 0;
}
