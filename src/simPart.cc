// 模拟真实场景
// 生成量测数据
// MixCheng - 9/11
#include "simPart.h"

#include <random>

double genGauNoise(double mean, double stddev) {
  // 创建随机数引擎
  std::random_device rd;
  std::mt19937 gen(rd());

  // 创建正态分布对象
  std::normal_distribution<double> dist(mean, stddev);

  // 生成随机数
  return dist(gen);
}

createData::createData() {
  auto w1 = 50 * 2 * pi / 360.0f;
  auto w2 = -50 * 2 * pi / 360.0f;
  auto S0 = MatrixXd(dimS, 1);
  auto F1 = MatrixXd(dimS, dimS);
  auto F2 = MatrixXd(dimS, dimS);
  auto F3 = MatrixXd(dimS, dimS);
  auto H = MatrixXd(dimZ, dimS);
  auto R = MatrixXd(dimZ, dimZ);

  S0 << 10, 20, 5, 6;
  F1 << 1, 0, dT, 0, // CV模型 - 状态转移矩阵
      0, 1, 0, dT, 0, 0, 1, 0, 0, 0, 0, 1;
  F2 << 1, 0, sin(w1 * dT) / w1, -(1 - cos(w1 * dT)) / w1, 0, 1, (1 - cos(w1 * dT)) / w1,
      sin(w1 * dT) / w1, 0, 0, cos(w1 * dT), -sin(w1 * dT), 0, 0, sin(w1 * dT), cos(w1 * dT);
  F3 << 1, 0, sin(w2 * dT) / w2, -(1 - cos(w2 * dT)) / w2, 0, 1, (1 - cos(w2 * dT)) / w2,
      sin(w2 * dT) / w2, 0, 0, cos(w2 * dT), -sin(w2 * dT), 0, 0, sin(w2 * dT), cos(w2 * dT);
  H << 1, 0, 0, 0, 0, 1, 0, 0;
  R << 0.1 * 0.1, 0, 0, 0.5 * 0.5;
}

void createData::trueTracking() // 返回真实的状态序列(轨迹)
{
  int T1, T2, T3, i = 0;
  T1 = static_cast<int>(TDs / (3.0 * dT));
  T2 = static_cast<int>(2.0 * TDs / (3.0 * dT));
  T3 = static_cast<int>(TDs / dT);
  double r11 = R(0, 0), r22 = R(1, 1);
  MatrixXd tmpS(dimS, 1);
  MatrixXd tmpZ(dimZ, 1);
  MatrixXd v_k(dimZ, 1);
  tmpS = S0;
  v_k(0) = genGauNoise(0.0, sqrt(r11));
  v_k(1) = genGauNoise(0.0, sqrt(r22)); // cout << sqrt(r22);
  tmpZ = H * tmpS + v_k;                // cout << tmpZ;
  s_true.push_back(tmpS);
  z_k.push_back(tmpZ);

  // for (i = 1; i < T3; i++) {
  //	tmpS = (*F1) * tmpS;
  //	v_k(0) = genGauNoise(0.0, sqrt(r11));
  //	v_k(1) = genGauNoise(0.0, sqrt(r22));
  //	tmpZ = (*H) * tmpS + v_k;
  //	s_true.push_back(tmpS);
  //	z_k.push_back(tmpZ);
  // }

  for (i = 1; i < T1; i++) {
    tmpS = (F1)*tmpS;
    v_k(0) = genGauNoise(0.0, sqrt(r11));
    v_k(1) = genGauNoise(0.0, sqrt(r22));
    tmpZ = (H)*tmpS + v_k;
    s_true.push_back(tmpS);
    z_k.push_back(tmpZ);
  }
  for (; i < T2; i++) {
    tmpS = (F2)*tmpS;
    v_k(0) = genGauNoise(0.0, sqrt(r11));
    v_k(1) = genGauNoise(0.0, sqrt(r22));
    tmpZ = (H)*tmpS + v_k;
    s_true.push_back(tmpS);
    z_k.push_back(tmpZ);
  }
  for (; i < T3; i++) {
    tmpS = (F3)*tmpS;
    v_k(0) = genGauNoise(0.0, sqrt(r11));
    v_k(1) = genGauNoise(0.0, sqrt(r22));
    tmpZ = (H)*tmpS + v_k;
    s_true.push_back(tmpS);
    z_k.push_back(tmpZ);
  }
}
