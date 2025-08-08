#ifndef _IMM_H_
#define _IMM_H_
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#define modNum 3
using namespace std;
using namespace Eigen;

class immAlg {
 public:
  int dmS = 4, dmZ = 2;
  double dt = 0.016f;
  double w1, w2;
  MatrixXd S0;
  MatrixXd P0;
  MatrixXd F1;
  MatrixXd F2;
  MatrixXd F3;
  MatrixXd G;
  MatrixXd Q;
  MatrixXd H;
  MatrixXd R;
  MatrixXd Z;
  immAlg(int dim_s, int dim_z, double det_t);
  void kalmanFilter(MatrixXd F, int wM);
  MatrixXd immSingleEstimator(double Zx, double Zy);
  void immPrioriFusion();
  void immPosterioriFusion();

 private:
  MatrixXd E = MatrixXd(dmZ, 1);
  MatrixXd S = MatrixXd(dmZ, dmZ);
  MatrixXd SI = MatrixXd(dmZ, dmZ);
  MatrixXd K = MatrixXd(dmS, dmZ);

  MatrixXd uM = MatrixXd(1, modNum);
  MatrixXd uPhi = MatrixXd(modNum, modNum);

  MatrixXd xFus = MatrixXd(dmS, 1);

  std::vector<MatrixXd> xPrior = std::vector<MatrixXd>(3);  //3->3个模型
  std::vector<MatrixXd> pPrior = std::vector<MatrixXd>(3);
  std::vector<MatrixXd> xPoste = std::vector<MatrixXd>(3);  //3->3个模型
  std::vector<MatrixXd> pPoste = std::vector<MatrixXd>(3);

  std::vector<double> det = std::vector<double>(3);
};

#endif
