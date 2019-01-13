#include <utility>

using namespace std;

class pi_controller{
public:
  double p;
  double i;
  double discrete;
  pi_controller(double p, double i, double discrete);
  double control(pair<double, double> e);
};
