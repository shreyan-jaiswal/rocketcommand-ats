#include <utility>

using namespace std;

class pi_controller{
private:
  double p;
  double i;
public:
  pi_controller(double p, double i);
  double control(Matrix<2,1,double> e);
};
