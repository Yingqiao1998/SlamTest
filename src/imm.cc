// IMM算法
// 2023/9/11 - MixCheng

#include "imm.h"

#include <math.h>

#include "simPart.h"

/* 构造函数 */
// 初始化参数，模型未知情况下参数很重要，依旧实际情况修改
immAlg::immAlg(int dim_s, int dim_z, double det_t) {
  dmS = dim_s;
  dmZ = dim_z;
  dt = det_t;
  w1 = 150 * 2 * pi / 360.0f;
  w2 = -w1;
  S0 = MatrixXd(dmS, 1);
  P0 = MatrixXd(dmS, dmS);
  F1 = MatrixXd(dmS, dmS);
  F2 = MatrixXd(dmS, dmS);
  F3 = MatrixXd(dmS, dmS);
  G = MatrixXd(dmS, 2);
  Q = MatrixXd(2, 2); // 2维值噪声来源，和量测2维不是同一个意思
  H = MatrixXd(dmZ, dmS);
  R = MatrixXd(dmZ, dmZ);
  Z = MatrixXd(dmZ, 1);

  S0 << 10, 10, 1, 2; // 滤波一般不知道真正的初始状态
  P0 << 10e5, 0, 0, 0, 0, 10e5, 0, 0, 0, 0, 10e5, 0, 0, 0, 0,
      10e5;          // cout << *P0;
  F1 << 1, 0, dt, 0, // CV模型 - 状态转移矩阵
      0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
  F2 << 1, 0, sin(w1 * dt) / w1, -(1 - cos(w1 * dt)) / w1, // CT
      0, 1, (1 - cos(w1 * dt)) / w1, sin(w1 * dt) / w1, 0, 0, cos(w1 * dt), -sin(w1 * dt), 0, 0,
      sin(w1 * dt), cos(w1 * dt);
  F3 << 1, 0, sin(w2 * dt) / w2, -(1 - cos(w2 * dt)) / w2, 0, 1, (1 - cos(w2 * dt)) / w2,
      sin(w2 * dt) / w2, 0, 0, cos(w2 * dt), -sin(w2 * dt), 0, 0, sin(w2 * dt), cos(w2 * dt);
  G << dt * dt / 2, 0, 0, dt * dt / 2, dt, 0, 0, dt;
  H << 1, 0, 0, 0, 0, 1, 0, 0;

  // 实际测试，需要调参
  Q = MatrixXd::Identity(2, 2) * 1.0; // 允许更大模型噪声（每维1米²）

  R = MatrixXd::Identity(2, 2) * 1.0; // 标准差 = 1m

  uM << 1.0 / 3, 1.0 / 3, 1.0 / 3; // 初始均匀分布

  // uPhi << 0.75, 0.15, 0.10, 0.15, 0.75, 0.10, 0.15, 0.10, 0.75;
  uPhi << 0.9, 0.05, 0.05, 0.05, 0.9, 0.05, 0.05, 0.05, 0.9; // 初始均匀分布

  for (int i = 0; i < 3; i++)
    xPoste[i] = (S0);
  for (int i = 0; i < 3; i++)
    pPoste[i] = (P0);
}

/****** 卡尔曼滤波 ******/
// 传统的卡尔曼滤波，但保存了似然
void immAlg::kalmanFilter(MatrixXd Fk, int wM) {
  // 预测步
  xPrior[wM] = (Fk)*xPoste[wM];
  pPrior[wM] = (((Fk)*pPoste[wM]) * (Fk).transpose()) + (G) * (Q) * (G).transpose();

  // 更新步
  E = (Z) - (H)*xPrior[wM]; // 残差，也叫新息
  MatrixXd PHT = pPrior[wM] * ((H).transpose());
  S = (H)*PHT + R;
  SI = (S).inverse();
  K = PHT * (SI);
  xPoste[wM] = xPrior[wM] + (K) * (E);
  pPoste[wM] = pPrior[wM] - (K) * (H)*pPrior[wM];

  // 获取似然函数，来更新概率（权重）
  MatrixXd tmpM1d = (-0.5 * (E).transpose() * (SI) * (E));
  double tmpExp = exp(tmpM1d(0));
  double tmpSqrt =
      sqrt(2 * 3.14159 * (S).determinant()); // 2pi的次方为1，但定义是维度，感觉常数不影响
  det[wM] = tmpExp / tmpSqrt;
}

/****** IMM-输入交互 ******/
// 先于KF,综合多个模型上一时刻的估计信息
// 修改：局部变量维度
void immAlg::immPrioriFusion() {
  // 获得更新上一时刻后验信息，并保存概率矩阵
  MatrixXd ptr_xTmp = MatrixXd(dmS, dmS);
  Eigen::Matrix<double, 4, 1> xTmp;
  xTmp.setZero();
  Eigen::Matrix<double, 4, 4> pTmp;
  pTmp.setZero();
  MatrixXd uNorm = (uM) * (uPhi);
  MatrixXd uState = MatrixXd(modNum, modNum);
  for (int i = 0; i < modNum; i++) {
    for (int j = 0; j < modNum; j++) {
      (uState)(j, i) = (uM)(j) * (uPhi)(j, i) / uNorm(i);
    }
  }
  for (int i = 0; i < modNum; i++) {
    for (int j = 0; j < modNum; j++) {
      xTmp += (uState)(j, i) * xPoste[j];
    }
    for (int j = 0; j < modNum; j++) {
      Eigen::MatrixXd tmpM = (xPoste[j] - xTmp);
      pTmp += (uState)(j, i) * (pPoste[j] + tmpM * tmpM.transpose());
    }
    xPoste[i] = xTmp;
    pPoste[i] = pTmp;
    xTmp.setZero();
    pTmp.setZero();
  }
}

/****** IMM-后验融合 ******/
// 后于KF,按权重融合模型的滤波值
void immAlg::immPosterioriFusion() {
  // 1-归一化似然，似然值很小，先归一化
  double sumLike = 0.0;
  for (int i = 0; i < modNum; i++)
    sumLike += det[i];
  for (int i = 0; i < modNum; i++)
    det[i] = det[i] / sumLike;
  double UL = 0.0;
  for (int i = 0; i < modNum; i++)
    UL += det[i] * (uM)(i);
  for (int i = 0; i < modNum; i++)
    (uM)(i) = det[i] * (uM)(i) / UL;
  // cout << *uM << endl;
  // 2-融合
  Eigen::Matrix<double, 4, 1> xTmp;
  xTmp.setZero();
  Eigen::Matrix<double, 4, 4> pTmp;
  pTmp.setZero();
  for (int i = 0; i < modNum; i++) {
    xTmp += (uM)(i)*xPoste[i]; // 真实情况，只需要状态融合，不需要协方差融合
  }
  xFus = xTmp;
}

/****** IMM算法接口 ******/
// 输入一组量测数据，可得一组IMM滤波融合结果
MatrixXd immAlg::immSingleEstimator(double Zx, double Zy) {
  Z(0) = Zx;
  Z(1) = Zy;
  immPrioriFusion();
  kalmanFilter(F1, 0);
  kalmanFilter(F2, 1);
  kalmanFilter(F3, 2);
  immPosterioriFusion();
  MatrixXd xEst = xFus;
  return xEst;
}