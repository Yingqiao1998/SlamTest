/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once                // 防止头文件被重复包含
#include <ceres/ceres.h>    // Ceres非线性优化库主头文件
#include <ceres/rotation.h> // Ceres旋转相关函数库
#include <cstdlib>          // C标准库，包含内存分配等函数
#include <deque>            // 双端队列容器
#include <eigen3/Eigen/Dense> // Eigen线性代数库，用于矩阵和向量运算
#include <iostream>           // 标准输入输出流库
#include <map>                // 关联容器map
#include <opencv2/core/eigen.hpp> // OpenCV与Eigen的转换接口
#include <opencv2/opencv.hpp>     // OpenCV计算机视觉库
using namespace Eigen;            // 使用Eigen命名空间
using namespace std;              // 使用标准命名空间

// SFM（Structure from Motion）特征点结构体
// 用于存储三维重建过程中特征点的相关信息
struct SFMFeature {
  bool state; // 特征点状态，表示是否已被三角化
  int id;     // 特征点的唯一标识符
  vector<pair<int, Vector2d>> observation; // 观测数据：帧ID和对应的2D像素坐标
  double position[3];                      // 特征点的3D世界坐标位置
  double depth;                            // 特征点的深度信息
};

// 3D重投影误差结构体
// 用于Ceres优化器的代价函数，计算3D点投影到2D图像平面的误差
struct ReprojectionError3D {
  // 构造函数：初始化观测到的2D像素坐标
  ReprojectionError3D(double observed_u, double observed_v)
      : observed_u(observed_u), observed_v(observed_v) // 成员初始化列表
  {}

  // 重载运算符()，实现代价函数的计算
  // camera_R: 相机旋转四元数参数
  // camera_T: 相机平移向量参数
  // point: 3D点坐标参数
  // residuals: 输出的残差值
  template <typename T>
  bool operator()(const T *const camera_R, const T *const camera_T,
                  const T *point, T *residuals) const {
    T p[3];                                           // 转换后的3D点坐标
    ceres::QuaternionRotatePoint(camera_R, point, p); // 使用四元数旋转3D点
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];               // 加上平移向量
    T xp = p[0] / p[2];                // 计算归一化坐标x
    T yp = p[1] / p[2];                // 计算归一化坐标y
    residuals[0] = xp - T(observed_u); // x方向的重投影误差
    residuals[1] = yp - T(observed_v); // y方向的重投影误差
    return true;                       // 返回计算成功
  }

  // 静态工厂方法：创建Ceres代价函数对象
  // observed_x, observed_y: 观测到的2D像素坐标
  // 返回值: Ceres代价函数指针
  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<
            ReprojectionError3D, 2, 4, 3,
            3>( // 模板参数：残差维度2，参数维度4,3,3
        new ReprojectionError3D(observed_x, observed_y)));
  }

  double observed_u; // 观测到的u坐标（图像x坐标）
  double observed_v; // 观测到的v坐标（图像y坐标）
};

// 全局SFM（Structure from Motion）类
// 负责从多帧图像中恢复相机姿态和3D特征点位置
class GlobalSFM {
public:
  GlobalSFM(); // 默认构造函数

  // 主要构造函数：执行全局SFM重建
  // frame_num: 参与重建的帧数
  // q: 输出的相机旋转四元数数组
  // T: 输出的相机平移向量数组
  // l: 参考帧索引
  // relative_R: 相对旋转矩阵
  // relative_T: 相对平移向量
  // sfm_f: SFM特征点数据
  // sfm_tracked_points: 跟踪到的3D点坐标映射
  // 返回值: 重建是否成功
  bool construct(int frame_num, Quaterniond *q, Vector3d *T, int l,
                 const Matrix3d relative_R, const Vector3d relative_T,
                 vector<SFMFeature> &sfm_f,
                 map<int, Vector3d> &sfm_tracked_points);

private:
  // 使用PnP算法求解单帧相机姿态
  // R_initial: 输出的旋转矩阵
  // P_initial: 输出的平移向量
  // i: 当前帧索引
  // sfm_f: SFM特征点数据
  // 返回值: 求解是否成功
  bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
                       vector<SFMFeature> &sfm_f);

  // 三角化单个3D点
  // Pose0: 第一个相机的投影矩阵 [3x4]
  // Pose1: 第二个相机的投影矩阵 [3x4]
  // point0: 第一个相机中的2D观测点
  // point1: 第二个相机中的2D观测点
  // point_3d: 输出的三角化3D点坐标
  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1, Vector2d &point0,
                        Vector2d &point1, Vector3d &point_3d);

  // 三角化两帧之间的所有特征点
  // frame0: 第一帧的索引
  // Pose0: 第一帧的投影矩阵
  // frame1: 第二帧的索引
  // Pose1: 第二帧的投影矩阵
  // sfm_f: SFM特征点数据，会被修改以存储三角化结果
  void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                            int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                            vector<SFMFeature> &sfm_f);

  int feature_num; // 特征点总数
};