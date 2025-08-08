#ifndef _simPart_H_
#define _simPart_H_
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#define dimS 4  // 状态维数 - [x,y,vx,vy]
#define dimZ 2  // 量测维数 - [x,y]
#define dT 0.05f
#define TDs 6

#define pi 3.1415f
using namespace Eigen;

// 生成数据类 -> 用于生成测试数据
class createData {
 public:
  MatrixXd S0;
  MatrixXd F1;
  MatrixXd F2;
  MatrixXd F3;
  MatrixXd H;

  double w1, w2;
  std::vector<MatrixXd> s_true;
  std::vector<MatrixXd> z_k;
  createData();
  void trueTracking();

 private:
  MatrixXd R;
};

#endif
