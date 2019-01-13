#include <vector>
#include <string>
#include <utility>
#include <Eigen/Eigen>
// include a lot more shit

using namespace std;
using namespace Eigen;

typedef pair<double, double> state_t;
typedef Matrix<double, 11, 1> Vector11d;
typedef Matrix<double, 11, 11> Matrix11d;

#define VELOCITY first;
#define ALTITUDE second;

class LINR{
public:
  //methods
  LINR();
  double compute_cd(state_t X_v, double U_v);
  double compute_fd(state_t X_v, double U_v);
  double compute_area(double U_v);
  state_t ss_predict(state_t X_v, double U_v);
  state_t ms_predict(state_t X_v, double U_v);
  void set_dv(double dv_arg);
  void update(state_t X_v, state_t X_vm1, double U_vm1);
  //constant system parameters
  double m, arm_len, hub_rad, fin_wid, rocket_rad;
  //constant physical parameters
  double g, ro, MATH_PI;
  // constant general prediction parameters
  double apo_goal, dv;
  // constant LINR prediction parameters
  size_t W_size;
  double inv_ff;
  //variable LINR prediction parameters
  Vector11d W;
  Matrix11d P;
};
