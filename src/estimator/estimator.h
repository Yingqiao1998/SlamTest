/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once // 防止头文件被重复包含

#include <ceres/ceres.h>            // Ceres非线性优化库
#include <eigen3/Eigen/Dense>       // Eigen稠密矩阵库
#include <eigen3/Eigen/Geometry>    // Eigen几何库
#include <mutex>                    // 互斥锁库
#include <opencv2/core/eigen.hpp>   // OpenCV与Eigen转换接口
#include <queue>                    // 队列容器
#include <std_msgs/msg/float32.hpp> // ROS2浮点数消息
#include <std_msgs/msg/header.hpp>  // ROS2标准消息头
#include <thread>                   // 多线程库
#include <unordered_map>            // 无序映射容器

#include "../factor/imu_factor.h"                     // IMU因子
#include "../factor/marginalization_factor.h"         // 边缘化因子
#include "../factor/pose_local_parameterization.h"    // 位姿局部参数化
#include "../factor/projectionOneFrameTwoCamFactor.h" // 单帧双相机投影因子
#include "../factor/projectionTwoFrameOneCamFactor.h" // 双帧单相机投影因子
#include "../factor/projectionTwoFrameTwoCamFactor.h" // 双帧双相机投影因子
#include "../featureTracker/feature_tracker.h"        // 特征跟踪器
#include "../initial/initial_alignment.h"             // 初始对齐
#include "../initial/initial_ex_rotation.h"           // 外参旋转初始化
#include "../initial/initial_sfm.h"                   // 初始SFM
#include "../initial/solve_5pts.h"                    // 五点法求解
#include "../utility/tic_toc.h"                       // 计时工具
#include "../utility/utility.h"                       // 通用工具函数
#include "feature_manager.h"                          // 特征点管理器
#include "parameters.h"                               // 参数配置文件
#include "rclcpp/rclcpp.hpp"                          // ROS2 C++客户端库

// VINS-Fusion估计器主类
// 负责融合IMU和视觉数据，进行状态估计和优化
class Estimator {
public:
  Estimator();         // 构造函数
  ~Estimator();        // 析构函数
  void setParameter(); // 设置参数

  // 外部接口函数
  // 初始化第一帧位姿
  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
  // 输入IMU数据：时间戳、线性加速度、角速度
  void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
  // 输入特征点数据：时间戳、特征点帧数据
  void inputFeature(double t,
                    const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
  // 输入图像数据：时间戳、左图像、右图像（可选）
  void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
  // 处理IMU数据：时间戳、时间差、线性加速度、角速度
  void processIMU(double t, double dt, const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity);
  // 处理图像数据：特征点数据、时间戳
  void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                    const double header);
  // 处理所有测量数据的主函数
  void processMeasurements();
  // 改变传感器类型：是否使用IMU、是否使用双目
  void changeSensorType(int use_imu, int use_stereo);

  // 内部处理函数
  void clearState();         // 清除状态
  bool initialStructure();   // 初始化结构
  bool visualInitialAlign(); // 视觉初始对齐
  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T,
                    int &l); // 计算相对位姿
  void slideWindow();        // 滑动窗口
  void slideWindowNew();     // 滑动窗口（新帧）
  void slideWindowOld();     // 滑动窗口（旧帧）
  void optimization();       // 非线性优化
  void vector2double();      // 向量转换为double数组
  void double2vector();      // double数组转换为向量
  bool failureDetection();   // 失效检测
  // 获取IMU时间区间内的数据
  bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                      vector<pair<double, Eigen::Vector3d>> &gyrVector);
  // 获取世界坐标系下的位姿
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();                  // 预测下一帧的特征点
  void outliersRejection(set<int> &removeIndex); // 外点剔除
  // 计算重投影误差
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj,
                           Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates(); // 更新最新状态
  // 快速IMU预测
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                      Eigen::Vector3d angular_velocity);
  bool IMUAvailable(double t); // 检查IMU数据是否可用
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector); // 初始化第一帧IMU位姿

  // 目标定位功能：根据像素位置计算目标到相机的NED坐标
  Eigen::Vector3d calculateTargetNED(double pixel_x, double pixel_y, int camera_id = 0);
  bool isInitialized() const;

      // 求解器状态枚举
      enum SolverFlag {
        INITIAL,   // 初始化状态
        NON_LINEAR // 非线性优化状态
      };

  // 边缘化标志枚举
  enum MarginalizationFlag {
    MARGIN_OLD = 0,       // 边缘化最老帧
    MARGIN_SECOND_NEW = 1 // 边缘化次新帧
  };

  std::mutex mProcess;                         // 处理过程互斥锁
  std::mutex mBuf;                             // 缓冲区互斥锁
  std::mutex mPropagate;                       // 传播过程互斥锁
  queue<pair<double, Eigen::Vector3d>> accBuf; // 加速度数据缓冲区：时间戳+加速度
  queue<pair<double, Eigen::Vector3d>> gyrBuf; // 角速度数据缓冲区：时间戳+角速度
  queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
      featureBuf;           // 特征点数据缓冲区
  double prevTime, curTime; // 前一时刻和当前时刻
  bool openExEstimation;    // 是否开启外参估计

  std::thread trackThread;   // 特征跟踪线程
  std::thread processThread; // 数据处理线程

  FeatureTracker featureTracker; // 特征跟踪器

  SolverFlag solver_flag;                   // 求解器状态标志
  MarginalizationFlag marginalization_flag; // 边缘化标志
  Vector3d g;                               // 重力向量

  Matrix3d ric[2]; // 相机到IMU的旋转外参（最多支持2个相机）
  Vector3d tic[2]; // 相机到IMU的平移外参（最多支持2个相机）

  Vector3d Ps[(WINDOW_SIZE + 1)];  // 滑动窗口内各帧的位置
  Vector3d Vs[(WINDOW_SIZE + 1)];  // 滑动窗口内各帧的速度
  Matrix3d Rs[(WINDOW_SIZE + 1)];  // 滑动窗口内各帧的旋转
  Vector3d Bas[(WINDOW_SIZE + 1)]; // 滑动窗口内各帧的加速度偏置
  Vector3d Bgs[(WINDOW_SIZE + 1)]; // 滑动窗口内各帧的陀螺仪偏置
  double td;                       // 时间偏移量（相机与IMU之间）

  Matrix3d back_R0, last_R, last_R0; // 后向、最后、最后初始旋转矩阵
  Vector3d back_P0, last_P, last_P0; // 后向、最后、最后初始位置向量
  double Headers[(WINDOW_SIZE + 1)]; // 滑动窗口内各帧的时间戳

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]; // IMU预积分对象数组
  Vector3d acc_0, gyr_0;                                // IMU初始加速度和角速度

  vector<double> dt_buf[(WINDOW_SIZE + 1)];                    // 时间差缓冲区
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)]; // 线性加速度缓冲区
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];    // 角速度缓冲区

  int frame_count;                                               // 帧计数器
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid; // 各种统计计数
  int inputImageCnt;                                             // 输入图像计数

  FeatureManager f_manager;              // 特征点管理器
  MotionEstimator m_estimator;           // 运动估计器
  InitialEXRotation initial_ex_rotation; // 初始外参旋转

  bool first_imu;        // 是否为第一个IMU数据
  bool is_valid, is_key; // 当前帧是否有效、是否为关键帧
  bool failure_occur;    // 是否发生失效

  vector<Vector3d> point_cloud;  // 点云数据
  vector<Vector3d> margin_cloud; // 边缘化点云数据
  vector<Vector3d> key_poses;    // 关键帧位姿
  double initial_timestamp;      // 初始时间戳

  // Ceres优化器参数数组
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE]; // 位姿参数：滑动窗口内每帧的位姿
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]; // 速度偏置参数：速度和IMU偏置
  double para_Feature[NUM_OF_F][SIZE_FEATURE];            // 特征点参数：特征点逆深度
  double para_Ex_Pose[2][SIZE_POSE];   // 外参位姿参数：相机到IMU的外参
  double para_Retrive_Pose[SIZE_POSE]; // 回环检测位姿参数
  double para_Td[1][1];                // 时间偏移参数
  double para_Tr[1][1];                // 时间比例参数

  int loop_window_index; // 回环窗口索引

  MarginalizationInfo *last_marginalization_info;         // 上次边缘化信息
  vector<double *> last_marginalization_parameter_blocks; // 上次边缘化参数块

  map<double, ImageFrame> all_image_frame; // 所有图像帧映射：时间戳->图像帧
  IntegrationBase *tmp_pre_integration;    // 临时预积分对象

  Eigen::Vector3d initP; // 初始位置
  Eigen::Matrix3d initR; // 初始旋转

  double latest_time; // 最新时间戳
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0,
      latest_gyr_0;            // 最新状态变量
  Eigen::Quaterniond latest_Q; // 最新四元数

  bool initFirstPoseFlag; // 初始化第一帧位姿标志
  bool initThreadFlag;    // 初始化线程标志
};
