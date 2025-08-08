/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h" // 包含估计器头文件
// #include "../utility/visualization.h" // 包含可视化工具头文件

Estimator::Estimator()
    : f_manager{Rs} // 构造函数，初始化特征管理器f_manager，传入旋转矩阵数组Rs作为参数
{
  RCLCPP_INFO(rclcpp::get_logger("estimator"), "init begins"); // 输出初始化开始的信息
  initThreadFlag = false;                                      // 初始化线程标志为false
  clearState(); // 调用清除状态函数，初始化所有变量
}

Estimator::~Estimator() // 析构函数
{
  if (MULTIPLE_THREAD) // 如果使用多线程模式
  {
    processThread.join();     // 等待处理线程结束
    printf("join thread \n"); // 输出线程已连接的信息
  }
}

void Estimator::clearState() // 清除所有状态变量的函数
{
  mProcess.lock();        // 加锁保护数据
  while (!accBuf.empty()) // 清空加速度缓冲区
    accBuf.pop();
  while (!gyrBuf.empty()) // 清空陀螺仪缓冲区
    gyrBuf.pop();
  while (!featureBuf.empty()) // 清空特征缓冲区
    featureBuf.pop();

  prevTime = -1;                       // 将前一时刻设置为-1
  curTime = 0;                         // 当前时间设置为0
  openExEstimation = 0;                // 关闭外参估计
  initP = Eigen::Vector3d(0, 0, 0);    // 初始化位置为零向量
  initR = Eigen::Matrix3d::Identity(); // 初始化旋转矩阵为单位矩阵
  inputImageCnt = 0;                   // 输入图像计数器置零
  initFirstPoseFlag = false;           // 初始第一帧位姿标志置为false

  for (int i = 0; i < WINDOW_SIZE + 1; i++) // 对滑动窗口中的每一帧进行操作
  {
    Rs[i].setIdentity();                // 将旋转矩阵初始化为单位矩阵
    Ps[i].setZero();                    // 将位置向量初始化为零向量
    Vs[i].setZero();                    // 将速度向量初始化为零向量
    Bas[i].setZero();                   // 将加速度计偏置初始化为零向量
    Bgs[i].setZero();                   // 将陀螺仪偏置初始化为零向量
    dt_buf[i].clear();                  // 清空时间间隔缓冲区
    linear_acceleration_buf[i].clear(); // 清空线性加速度缓冲区
    angular_velocity_buf[i].clear();    // 清空角速度缓冲区

    if (pre_integrations[i] != nullptr) // 如果预积分对象存在
    {
      delete pre_integrations[i]; // 释放预积分对象内存
    }
    pre_integrations[i] = nullptr; // 将预积分指针设为空
  }

  for (int i = 0; i < NUM_OF_CAM; i++) // 对每个相机进行操作
  {
    tic[i] = Vector3d::Zero();     // 初始化相机位置向量为零向量
    ric[i] = Matrix3d::Identity(); // 初始化相机旋转矩阵为单位矩阵
  }

  first_imu = false,       // 初始IMU标志设为false
      sum_of_back = 0;     // 后移滑动窗口次数计数器置零
  sum_of_front = 0;        // 前移滑动窗口次数计数器置零
  frame_count = 0;         // 帧计数器置零
  solver_flag = INITIAL;   // 求解器标志设为初始状态
  initial_timestamp = 0;   // 初始时间戳置零
  all_image_frame.clear(); // 清空所有图像帧容器

  if (tmp_pre_integration != nullptr)       // 如果临时预积分对象存在
    delete tmp_pre_integration;             // 释放临时预积分对象内存
  if (last_marginalization_info != nullptr) // 如果上一次边缘化信息存在
    delete last_marginalization_info;       // 释放边缘化信息内存

  tmp_pre_integration = nullptr;                 // 将临时预积分指针设为空
  last_marginalization_info = nullptr;           // 将上一次边缘化信息指针设为空
  last_marginalization_parameter_blocks.clear(); // 清空上一次边缘化参数块列表

  f_manager.clearState(); // 调用特征管理器的清除状态函数

  failure_occur = 0; // 失败发生标志置零

  mProcess.unlock(); // 解锁
}

void Estimator::setParameter() // 设置系统参数的函数
{
  mProcess.lock();                     // 加锁保护数据
  for (int i = 0; i < NUM_OF_CAM; i++) // 对每个相机进行操作
  {
    tic[i] = TIC[i]; // 设置相机位置向量为全局参数
    ric[i] = RIC[i]; // 设置相机旋转矩阵为全局参数
    cout << " exitrinsic cam " << i << endl
         << ric[i] << endl
         << tic[i].transpose() << endl; // 输出外参信息
  }
  f_manager.setRic(ric); // 设置特征管理器的相机旋转矩阵
  ProjectionTwoFrameOneCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity(); // 设置单目两帧投影因子的信息矩阵
  ProjectionTwoFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity(); // 设置双目两帧投影因子的信息矩阵
  ProjectionOneFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();    // 设置双目单帧投影因子的信息矩阵
  td = TD;                                          // 设置时间偏差为全局参数
  g = G;                                            // 设置重力向量为全局参数
  cout << "set g " << g.transpose() << endl;        // 输出重力向量
  featureTracker.readIntrinsicParameter(CAM_NAMES); // 读取相机内参

  std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n'; // 输出是否使用多线程
  if (MULTIPLE_THREAD && !initThreadFlag) // 如果使用多线程且未初始化线程
  {
    initThreadFlag = true; // 设置线程初始化标志
    processThread = std::thread(&Estimator::processMeasurements,
                                this); // 创建处理线程，执行processMeasurements函数
  }
  mProcess.unlock(); // 解锁
}

void Estimator::changeSensorType(int use_imu,
                                 int use_stereo) // 改变传感器类型的函数
{
  bool restart = false;                     // 重启系统标志
  mProcess.lock();                          // 加锁保护数据
  if (!use_imu && !use_stereo)              // 如果既不使用IMU也不使用双目
    printf("at least use two sensors! \n"); // 输出至少使用两个传感器的提示
  else {
    if (USE_IMU != use_imu) // 如果改变了IMU的使用状态
    {
      USE_IMU = use_imu; // 更新IMU使用状态
      if (USE_IMU)       // 如果现在要使用IMU
      {
        // reuse imu; restart system
        restart = true; // 需要重启系统
      } else            // 如果现在不使用IMU
      {
        if (last_marginalization_info != nullptr) // 如果边缘化信息存在
          delete last_marginalization_info;       // 释放边缘化信息内存

        tmp_pre_integration = nullptr;                 // 临时预积分指针置空
        last_marginalization_info = nullptr;           // 边缘化信息指针置空
        last_marginalization_parameter_blocks.clear(); // 清空边缘化参数块列表
      }
    }

    STEREO = use_stereo;                                   // 更新立体视觉使用状态
    printf("use imu %d use stereo %d\n", USE_IMU, STEREO); // 输出传感器使用状态
  }
  mProcess.unlock(); // 解锁
  if (restart)       // 如果需要重启系统
  {
    clearState();   // 清除所有状态
    setParameter(); // 重新设置参数
  }
}

void Estimator::inputImage(double t, const cv::Mat &_img,
                           const cv::Mat &_img1) // 输入图像处理函数
{
  inputImageCnt++; // 输入图像计数器加1
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame; // 声明特征帧数据结构
  TicToc featureTrackerTime;                                             // 创建计时器对象

  if (_img1.empty()) // 如果没有提供第二幅图像（单目情况）
    featureFrame = featureTracker.trackImage(t, _img);        // 调用单目跟踪函数
  else                                                        // 双目情况
    featureFrame = featureTracker.trackImage(t, _img, _img1); // 调用双目跟踪函数
  // printf("featureTracker time: %f\n", featureTrackerTime.toc());

  if (SHOW_TRACK) // 如果需要显示跟踪结果
  {
    cv::Mat imgTrack = featureTracker.getTrackImage(); // 获取跟踪图像
    // pubTrackImage(imgTrack, t);                        // 发布跟踪图像
  }

  if (MULTIPLE_THREAD) // 如果使用多线程模式
  {
    if (inputImageCnt % 2 == 0) // 每隔一帧处理一次，减少计算负担
    {
      mBuf.lock();                                 // 加锁保护缓冲区
      featureBuf.push(make_pair(t, featureFrame)); // 将特征帧添加到缓冲区
      mBuf.unlock();                               // 解锁
    }
  } else // 单线程模式
  {
    mBuf.lock();                                     // 加锁保护缓冲区
    featureBuf.push(make_pair(t, featureFrame));     // 将特征帧添加到缓冲区
    mBuf.unlock();                                   // 解锁
    TicToc processTime;                              // 创建计时器对象
    processMeasurements();                           // 直接处理测量值
    printf("process time: %f\n", processTime.toc()); // 输出处理时间
  }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration,
                         const Vector3d &angularVelocity) // 输入IMU数据处理函数
{
  mBuf.lock();                                   // 加锁保护缓冲区
  accBuf.push(make_pair(t, linearAcceleration)); // 将加速度数据添加到缓冲区
  gyrBuf.push(make_pair(t, angularVelocity));    // 将陀螺仪数据添加到缓冲区
  // printf("input imu with time %f \n", t);
  mBuf.unlock(); // 解锁

  if (solver_flag == NON_LINEAR) // 如果系统已经初始化完成，处于非线性优化状态
  {
    mPropagate.lock(); // 加锁保护传播过程
    fastPredictIMU(t, linearAcceleration,
                   angularVelocity); // 使用IMU数据快速预测位姿
    // pubLatestOdometry(latest_P, latest_Q, latest_V, t); // 发布最新的里程计信息
    mPropagate.unlock(); // 解锁
  }
}

void Estimator::inputFeature(double t,
                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
                                 &featureFrame) // 输入特征数据处理函数
{
  mBuf.lock();                                 // 加锁保护缓冲区
  featureBuf.push(make_pair(t, featureFrame)); // 将特征帧添加到缓冲区
  mBuf.unlock();                               // 解锁

  if (!MULTIPLE_THREAD)    // 如果是单线程模式
    processMeasurements(); // 直接处理测量值
}

bool Estimator::getIMUInterval(
    double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
    vector<pair<double, Eigen::Vector3d>> &gyrVector) // 获取指定时间段内的IMU数据
{
  if (accBuf.empty()) // 如果加速度缓冲区为空
  {
    printf("not receive imu\n"); // 输出未接收到IMU数据的提示
    return false;                // 返回失败
  }
  // printf("get imu from %f %f\n", t0, t1);
  // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first,
  // accBuf.back().first);
  if (t1 <= accBuf.back().first) // 如果请求的结束时间小于等于缓冲区中最新数据的时间
  {
    while (accBuf.front().first <= t0) // 丢弃t0之前的数据
    {
      accBuf.pop();
      gyrBuf.pop();
    }
    while (accBuf.front().first < t1) // 收集t0到t1之间的数据
    {
      accVector.push_back(accBuf.front()); // 收集加速度数据
      accBuf.pop();
      gyrVector.push_back(gyrBuf.front()); // 收集陀螺仪数据
      gyrBuf.pop();
    }
    accVector.push_back(accBuf.front()); // 添加t1时刻或之后的第一个数据点
    gyrVector.push_back(gyrBuf.front());
  } else // 如果请求的结束时间大于缓冲区中最新数据的时间
  {
    printf("wait for imu\n"); // 输出等待IMU数据的提示
    return false;             // 返回失败，需要等待更多IMU数据
  }
  return true; // 返回成功
}

bool Estimator::IMUAvailable(double t) // 检查指定时间的IMU数据是否可用
{
  if (!accBuf.empty() &&
      t <=
          accBuf.back().first) // 如果加速度缓冲区不为空，且请求的时间小于等于缓冲区中最新数据的时间
    return true;               // 返回可用
  else
    return false; // 返回不可用
}

void Estimator::processMeasurements() // 处理测量数据的主函数
{
  while (1) // 无限循环，多线程模式下持续运行
  {
    // printf("process measurments\n");
    pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>
        feature; // 声明特征数据变量
    vector<pair<double, Eigen::Vector3d>> accVector,
        gyrVector;           // 声明IMU数据向量
    if (!featureBuf.empty()) // 如果特征缓冲区不为空
    {
      feature = featureBuf.front(); // 获取最早的特征帧
      curTime = feature.first + td; // 计算当前时间（加上时间偏差）
      while (1)                     // 等待IMU数据可用
      {
        if ((!USE_IMU || IMUAvailable(feature.first + td))) // 如果不使用IMU或IMU数据可用
          break;                                            // 退出等待循环
        else {
          printf("wait for imu ... \n");     // 输出等待IMU数据的提示
          if (!MULTIPLE_THREAD)              // 如果是单线程模式
            return;                          // 直接返回，等待下次调用
          std::chrono::milliseconds dura(5); // 创建5毫秒的时间间隔对象
          std::this_thread::sleep_for(dura); // 睡眠5毫秒
        }
      }
      mBuf.lock(); // 加锁保护缓冲区
      if (USE_IMU) // 如果使用IMU
        getIMUInterval(prevTime, curTime, accVector,
                       gyrVector); // 获取prevTime到curTime之间的IMU数据

      featureBuf.pop(); // 移除已处理的特征帧
      mBuf.unlock();    // 解锁

      if (USE_IMU) // 如果使用IMU
      {
        if (!initFirstPoseFlag)                       // 如果还未初始化第一个位姿
          initFirstIMUPose(accVector);                // 使用IMU数据初始化第一个位姿
        for (size_t i = 0; i < accVector.size(); i++) // 处理所有收集到的IMU数据
        {
          double dt;                                          // 时间间隔
          if (i == 0)                                         // 如果是第一个IMU数据
            dt = accVector[i].first - prevTime;               // 计算与上一帧的时间间隔
          else if (i == accVector.size() - 1)                 // 如果是最后一个IMU数据
            dt = curTime - accVector[i - 1].first;            // 计算与当前帧的时间间隔
          else                                                // 中间的IMU数据
            dt = accVector[i].first - accVector[i - 1].first; // 计算与前一个IMU数据的时间间隔
          processIMU(accVector[i].first, dt, accVector[i].second,
                     gyrVector[i].second); // 处理IMU数据
        }
      }
      mProcess.lock();                             // 加锁保护处理过程
      processImage(feature.second, feature.first); // 处理图像特征数据
      prevTime = curTime;                          // 更新上一时刻为当前时刻

      // printStatistics(*this, 0); // 打印统计信息

      // std_msgs::Header header;                 // 创建ROS消息头
      // header.frame_id = "world";               // 设置坐标系为world
      // header.stamp = ros::Time(feature.first); // 设置时间戳

      // pubOdometry(*this, header);   // 发布里程计信息
      // pubKeyPoses(*this, header);   // 发布关键帧位姿
      // pubCameraPose(*this, header); // 发布相机位姿
      // pubPointCloud(*this, header); // 发布点云
      // pubKeyframe(*this);           // 发布关键帧
      // pubTF(*this, header);         // 发布坐标变换
      mProcess.unlock(); // 解锁
    }

    if (!MULTIPLE_THREAD) // 如果是单线程模式
      break;              // 处理完一帧后退出循环

    std::chrono::milliseconds dura(2); // 创建2毫秒的时间间隔对象
    std::this_thread::sleep_for(dura); // 睡眠2毫秒，避免CPU占用过高
  }
}

void Estimator::initFirstIMUPose(
    vector<pair<double, Eigen::Vector3d>> &accVector) // 初始化第一个IMU位姿
{
  printf("init first imu pose\n"); // 输出初始化第一个IMU位姿的提示
  initFirstPoseFlag = true;        // 设置初始化第一帧位姿标志
  // return;
  Eigen::Vector3d averAcc(0, 0, 0);             // 平均加速度向量初始化为零
  int n = (int)accVector.size();                // 获取加速度向量的数量
  for (size_t i = 0; i < accVector.size(); i++) // 遍历所有加速度数据
  {
    averAcc = averAcc + accVector[i].second; // 累加加速度向量
  }
  averAcc = averAcc / n; // 计算平均加速度
  printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(),
         averAcc.z());                 // 输出平均加速度
  Matrix3d R0 = Utility::g2R(averAcc); // 根据平均加速度计算初始旋转矩阵（重力对齐）
  double yaw = Utility::R2ypr(R0).x();                   // 提取yaw角
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // 消除yaw角影响
  Rs[0] = R0;                                            // 设置初始旋转矩阵
  cout << "init R0 " << endl
       << Rs[0] << endl; // 输出初始旋转矩阵
                         // Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p,
                              Eigen::Matrix3d r) // 使用给定的位置和旋转矩阵初始化第一个位姿
{
  Ps[0] = p; // 设置初始位置
  Rs[0] = r; // 设置初始旋转矩阵
  initP = p; // 保存初始位置
  initR = r; // 保存初始旋转矩阵
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity) // 处理IMU数据
{
  if (!first_imu) // 如果是第一个IMU数据
  {
    first_imu = true;            // 设置首次接收IMU数据的标志
    acc_0 = linear_acceleration; // 记录初始加速度
    gyr_0 = angular_velocity;    // 记录初始角速度
  }

  if (!pre_integrations[frame_count]) // 如果当前帧的预积分对象不存在
  {
    pre_integrations[frame_count] =
        new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; // 创建新的预积分对象
  }
  if (frame_count != 0) // 如果不是第一帧
  {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration,
                                             angular_velocity); // 将IMU数据添加到预积分中
    // if(solver_flag != NON_LINEAR)
    tmp_pre_integration->push_back(dt, linear_acceleration,
                                   angular_velocity); // 同时将IMU数据添加到临时预积分中

    dt_buf[frame_count].push_back(dt); // 保存时间间隔到缓冲区
    linear_acceleration_buf[frame_count].push_back(linear_acceleration); // 保存线性加速度到缓冲区
    angular_velocity_buf[frame_count].push_back(angular_velocity); // 保存角速度到缓冲区

    int j = frame_count;
    Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g; // 计算上一时刻的无偏加速度（世界坐标系下）
    Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j]; // 计算中值角速度并减去偏置
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); // 更新旋转矩阵（积分角速度）
    Vector3d un_acc_1 =
        Rs[j] * (linear_acceleration - Bas[j]) - g; // 计算当前时刻的无偏加速度（世界坐标系下）
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); // 计算中值加速度
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;  // 更新位置（积分速度和加速度）
    Vs[j] += dt * un_acc;                          // 更新速度（积分加速度）
  }
  acc_0 = linear_acceleration; // 更新上一时刻的加速度
  gyr_0 = angular_velocity;    // 更新上一时刻的角速度
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                             const double header) // 处理图像特征数据
{
  RCLCPP_DEBUG(
      rclcpp::get_logger("estimator"),
      "new image coming ------------------------------------------"); // 调试输出：新图像到来
  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "Adding feature points %lu",
               image.size()); // 调试输出：添加特征点的数量
  if (f_manager.addFeatureCheckParallax(frame_count, image,
                                        td)) // 添加特征并检查视差，判断是否为关键帧
  {
    marginalization_flag = MARGIN_OLD; // 如果是关键帧，设置边缘化旧帧的标志
                                       // printf("keyframe\n");
  } else {
    marginalization_flag = MARGIN_SECOND_NEW; // 如果不是关键帧，设置边缘化次新帧的标志
                                              // printf("non-keyframe\n");
  }

  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "%s",
               marginalization_flag ? "Non-keyframe" : "Keyframe"); // 调试输出：帧类型
  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "Solving %d",
               frame_count); // 调试输出：当前处理的帧序号
  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "number of feature: %d",
               f_manager.getFeatureCount()); // 调试输出：特征点数量
  Headers[frame_count] = header;             // 记录当前帧的时间戳

  ImageFrame imageframe(image, header);                  // 创建图像帧对象
  imageframe.pre_integration = tmp_pre_integration;      // 设置预积分对象
  all_image_frame.insert(make_pair(header, imageframe)); // 将图像帧添加到容器中
  tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count],
                                            Bgs[frame_count]}; // 创建新的临时预积分对象

  if (ESTIMATE_EXTRINSIC == 2) // 如果需要标定外参
  {
    RCLCPP_INFO(
        rclcpp::get_logger("estimator"),
        "calibrating extrinsic param, rotation movement is needed"); // 输出需要旋转运动以标定外参的提示
    if (frame_count != 0)                                            // 如果不是第一帧
    {
      vector<pair<Vector3d, Vector3d>> corres =
          f_manager.getCorresponding(frame_count - 1, frame_count); // 获取相邻帧之间的特征对应关系
      Matrix3d calib_ric; // 标定得到的相机与IMU之间的旋转矩阵
      if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q,
                                                    calib_ric)) // 标定外部旋转参数
      {
        RCLCPP_WARN(rclcpp::get_logger("estimator"),
                    "initial extrinsic rotation calib success"); // 输出外参标定成功的提示
        RCLCPP_WARN_STREAM(rclcpp::get_logger("estimator"),
                           "initial extrinsic rotation: " << endl
                                                          << calib_ric); // 输出标定得到的旋转矩阵
        ric[0] = calib_ric;     // 更新相机与IMU之间的旋转矩阵
        RIC[0] = calib_ric;     // 更新全局变量
        ESTIMATE_EXTRINSIC = 1; // 将标定外参的标志设为1（已标定）
      }
    }
  }

  if (solver_flag == INITIAL) // 如果系统处于初始化状态
  {
    // monocular + IMU initilization
    if (!STEREO && USE_IMU) // 如果是单目+IMU模式
    {
      if (frame_count == WINDOW_SIZE) // 如果滑动窗口已满
      {
        bool result = false; // 初始化结果标志
        if (ESTIMATE_EXTRINSIC != 2 &&
            (header - initial_timestamp) > 0.1) // 如果不需要标定外参，且与上次初始化尝试间隔足够
        {
          result = initialStructure(); // 尝试初始化结构
          initial_timestamp = header;  // 记录本次初始化时间戳
        }
        if (result) // 如果初始化成功
        {
          optimization();           // 进行一次优化
          updateLatestStates();     // 更新最新状态
          solver_flag = NON_LINEAR; // 将求解器状态设为非线性优化状态
          slideWindow();            // 滑动窗口
          RCLCPP_INFO(rclcpp::get_logger("estimator"),
                      "Initialization finish!"); // 输出初始化完成的信息
        } else                                   // 如果初始化失败
          slideWindow();                         // 仍然需要滑动窗口，丢弃旧的数据
      }
    }

    // stereo + IMU initilization
    if (STEREO && USE_IMU) // 如果是双目+IMU模式
    {
      f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic,
                                   ric); // 使用PnP算法初始化当前帧的位姿
      f_manager.triangulate(frame_count, Ps, Rs, tic, ric); // 三角化特征点
      if (frame_count == WINDOW_SIZE)                       // 如果滑动窗口已满
      {
        map<double, ImageFrame>::iterator frame_it;
        int i = 0;
        for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end();
             frame_it++) // 遍历所有图像帧
        {
          frame_it->second.R = Rs[i]; // 设置图像帧的旋转矩阵
          frame_it->second.T = Ps[i]; // 设置图像帧的位置
          i++;
        }
        solveGyroscopeBias(all_image_frame, Bgs); // 求解陀螺仪偏置
        for (int i = 0; i <= WINDOW_SIZE; i++) // 使用新的陀螺仪偏置重新进行预积分
        {
          pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
        }
        optimization();           // 进行一次优化
        updateLatestStates();     // 更新最新状态
        solver_flag = NON_LINEAR; // 将求解器状态设为非线性优化状态
        slideWindow();            // 滑动窗口
        RCLCPP_INFO(rclcpp::get_logger("estimator"),
                    "Initialization finish!"); // 输出初始化完成的信息
      }
    }

    // stereo only initilization
    if (STEREO && !USE_IMU) // 如果仅使用双目（不使用IMU）
    {
      f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic,
                                   ric); // 使用PnP算法初始化当前帧的位姿
      f_manager.triangulate(frame_count, Ps, Rs, tic, ric); // 三角化特征点
      optimization();                                       // 进行一次优化

      if (frame_count == WINDOW_SIZE) // 如果滑动窗口已满
      {
        optimization();           // 再次进行优化
        updateLatestStates();     // 更新最新状态
        solver_flag = NON_LINEAR; // 将求解器状态设为非线性优化状态
        slideWindow();            // 滑动窗口
        RCLCPP_INFO(rclcpp::get_logger("estimator"),
                    "Initialization finish!"); // 输出初始化完成的信息
      }
    }

    if (frame_count < WINDOW_SIZE) // 如果滑动窗口未满
    {
      frame_count++;                    // 帧计数器加1
      int prev_frame = frame_count - 1; // 获取上一帧的索引
      Ps[frame_count] = Ps[prev_frame]; // 将当前帧的位置初始化为上一帧的位置
      Vs[frame_count] = Vs[prev_frame]; // 将当前帧的速度初始化为上一帧的速度
      Rs[frame_count] = Rs[prev_frame]; // 将当前帧的旋转矩阵初始化为上一帧的旋转矩阵
      Bas[frame_count] = Bas[prev_frame]; // 将当前帧的加速度计偏置初始化为上一帧的偏置
      Bgs[frame_count] = Bgs[prev_frame]; // 将当前帧的陀螺仪偏置初始化为上一帧的偏置
    }

  } else // 如果系统已经初始化完成
  {
    TicToc t_solve; // 创建计时器对象
    if (!USE_IMU)   // 如果不使用IMU
      f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic,
                                   ric); // 使用PnP算法初始化当前帧的位姿
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric); // 三角化特征点
    optimization();                                       // 进行优化
    set<int> removeIndex;                 // 创建需要移除的特征点索引集合
    outliersRejection(removeIndex);       // 进行外点剔除
    f_manager.removeOutlier(removeIndex); // 从特征管理器中移除外点
    if (!MULTIPLE_THREAD)                 // 如果是单线程模式
    {
      featureTracker.removeOutliers(removeIndex); // 从特征跟踪器中移除外点
      predictPtsInNextFrame();                    // 预测下一帧中的特征点位置
    }

    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "solver costs: %fms",
                 t_solve.toc()); // 输出求解耗时

    if (failureDetection()) // 检测系统是否发生故障
    {
      RCLCPP_WARN(rclcpp::get_logger("estimator"), "failure detection!"); // 输出故障检测的警告
      failure_occur = 1;                                              // 设置故障发生标志
      clearState();                                                   // 清除所有状态
      setParameter();                                                 // 重新设置参数
      RCLCPP_WARN(rclcpp::get_logger("estimator"), "system reboot!"); // 输出系统重启的警告
      return; // 直接返回，不再进行后续处理
    }

    slideWindow();              // 滑动窗口
    f_manager.removeFailures(); // 移除失败的特征点
    // prepare output of VINS
    key_poses.clear(); // 清空关键帧位置容器
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.push_back(Ps[i]); // 收集所有关键帧位置

    last_R = Rs[WINDOW_SIZE]; // 保存最后一帧的旋转矩阵
    last_P = Ps[WINDOW_SIZE]; // 保存最后一帧的位置
    last_R0 = Rs[0];          // 保存第一帧的旋转矩阵
    last_P0 = Ps[0];          // 保存第一帧的位置
    updateLatestStates();     // 更新最新状态
  }
}

bool Estimator::initialStructure() // 初始化结构的函数
{
  TicToc t_sfm; // 创建计时器对象
  // check imu observibility
  {
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g; // 重力向量的累加和
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
         frame_it++) // 遍历所有图像帧（除了第一帧）
    {
      double dt = frame_it->second.pre_integration->sum_dt; // 获取预积分的时间间隔
      Vector3d tmp_g =
          frame_it->second.pre_integration->delta_v / dt; // 计算重力向量（速度变化除以时间）
      sum_g += tmp_g;                                     // 累加重力向量
    }
    Vector3d aver_g;                                          // 平均重力向量
    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1); // 计算平均重力向量
    double var = 0;                                           // 方差初始化为0
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end();
         frame_it++) // 再次遍历所有图像帧
    {
      double dt = frame_it->second.pre_integration->sum_dt; // 获取预积分的时间间隔
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt; // 计算重力向量
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);          // 计算方差
      // cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)all_image_frame.size() - 1)); // 计算标准差
    // ROS_WARN("IMU variation %f!", var);
    if (var < 0.25) // 如果标准差小于0.25
    {
      RCLCPP_INFO(rclcpp::get_logger("estimator"),
                  "IMU excitation not enouth!"); // 输出IMU激励不足的信息
      // return false; // 可以返回失败，但这里注释掉了
    }
  }
  // global sfm
  Quaterniond Q[frame_count + 1];           // 存储每一帧的旋转四元数
  Vector3d T[frame_count + 1];              // 存储每一帧的位置
  map<int, Vector3d> sfm_tracked_points;    // 存储SFM跟踪的3D点
  vector<SFMFeature> sfm_f;                 // SFM特征向量
  for (auto &it_per_id : f_manager.feature) // 遍历所有特征
  {
    int imu_j = it_per_id.start_frame - 1;                 // 初始IMU帧索引
    SFMFeature tmp_feature;                                // 创建临时SFM特征
    tmp_feature.state = false;                             // 设置特征状态为false
    tmp_feature.id = it_per_id.feature_id;                 // 设置特征ID
    for (auto &it_per_frame : it_per_id.feature_per_frame) // 遍历特征在各帧中的观测
    {
      imu_j++;                             // IMU帧索引递增
      Vector3d pts_j = it_per_frame.point; // 获取特征点
      tmp_feature.observation.push_back(
          make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()})); // 添加观测
    }
    sfm_f.push_back(tmp_feature); // 将特征添加到SFM特征向量
  }
  Matrix3d relative_R;                          // 相对旋转矩阵
  Vector3d relative_T;                          // 相对平移向量
  int l;                                        // 参考帧索引
  if (!relativePose(relative_R, relative_T, l)) // 计算相对位姿
  {
    RCLCPP_INFO(rclcpp::get_logger("estimator"),
                "Not enough features or parallax; Move device around"); // 输出特征或视差不足的提示
    return false;                                                       // 返回失败
  }
  GlobalSFM sfm; // 创建全局SFM对象
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) // 构建全局SFM
  {
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "global SFM failed!"); // 输出全局SFM失败的信息
    marginalization_flag = MARGIN_OLD; // 设置边缘化旧帧的标志
    return false;                      // 返回失败
  }

  // solve pnp for all frame
  map<double, ImageFrame>::iterator frame_it;
  map<int, Vector3d>::iterator it;
  frame_it = all_image_frame.begin();                            // 初始化图像帧迭代器
  for (int i = 0; frame_it != all_image_frame.end(); frame_it++) // 遍历所有图像帧
  {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;        // OpenCV矩阵
    if ((frame_it->first) == Headers[i]) // 如果当前帧是关键帧
    {
      frame_it->second.is_key_frame = true;                              // 设置为关键帧
      frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose(); // 设置旋转矩阵
      frame_it->second.T = T[i];                                         // 设置位置
      i++;                                                               // 索引递增
      continue;                                                          // 继续下一次循环
    }
    if ((frame_it->first) > Headers[i]) // 如果当前帧的时间戳大于Headers[i]
    {
      i++; // 索引递增
    }
    Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix(); // 计算初始旋转矩阵
    Vector3d P_inital = -R_inital * T[i];                    // 计算初始位置
    cv::eigen2cv(R_inital, tmp_r);                           // 将Eigen矩阵转换为OpenCV矩阵
    cv::Rodrigues(tmp_r, rvec);                              // 将旋转矩阵转换为旋转向量
    cv::eigen2cv(P_inital, t);                               // 将位置向量转换为OpenCV矩阵

    frame_it->second.is_key_frame = false;       // 设置为非关键帧
    vector<cv::Point3f> pts_3_vector;            // 3D点向量
    vector<cv::Point2f> pts_2_vector;            // 2D点向量
    for (auto &id_pts : frame_it->second.points) // 遍历当前帧的所有特征点
    {
      int feature_id = id_pts.first;  // 特征ID
      for (auto &i_p : id_pts.second) // 遍历特征的所有观测
      {
        it = sfm_tracked_points.find(feature_id); // 查找特征对应的3D点
        if (it != sfm_tracked_points.end())       // 如果找到了
        {
          Vector3d world_pts = it->second; // 获取世界坐标系下的3D点
          cv::Point3f pts_3(world_pts(0), world_pts(1),
                            world_pts(2));           // 转换为OpenCV的3D点
          pts_3_vector.push_back(pts_3);             // 添加到3D点向量
          Vector2d img_pts = i_p.second.head<2>();   // 获取图像坐标系下的2D点
          cv::Point2f pts_2(img_pts(0), img_pts(1)); // 转换为OpenCV的2D点
          pts_2_vector.push_back(pts_2);             // 添加到2D点向量
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0,
                 1);             // 单位相机内参矩阵，因为特征点已经归一化
    if (pts_3_vector.size() < 6) // 如果3D点不足6个，无法求解PnP
    {
      cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "Not enough points for solve pnp !");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t,
                      1)) // 使用OpenCV的PnP求解，1表示使用SOLVEPNP_ITERATIVE方法
    {
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "solve pnp fail!");
      return false;
    }
    cv::Rodrigues(rvec, r); // 将旋转向量转换为旋转矩阵
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);    // 将OpenCV矩阵转换为Eigen矩阵
    R_pnp = tmp_R_pnp.transpose(); // 转置得到相机到世界坐标系的旋转矩阵
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);                          // 将OpenCV矩阵转换为Eigen矩阵
    T_pnp = R_pnp * (-T_pnp);                        // 计算相机在世界坐标系下的位置
    frame_it->second.R = R_pnp * RIC[0].transpose(); // 计算IMU到世界坐标系的旋转矩阵
    frame_it->second.T = T_pnp;                      // 设置IMU在世界坐标系下的位置
  }
  if (visualInitialAlign()) // 调用视觉惯性对齐函数
    return true;            // 初始化成功
  else {
    RCLCPP_INFO(rclcpp::get_logger("estimator"),
                "misalign visual structure with IMU"); // 视觉结构与IMU不匹配
    return false;                                      // 初始化失败
  }
}

bool Estimator::visualInitialAlign() {
  TicToc t_g; // 计时器
  VectorXd x; // 用于存储求解结果
  // solve scale
  bool result =
      VisualIMUAlignment(all_image_frame, Bgs, g, x); // 调用视觉IMU对齐函数，求解尺度和重力方向
  if (!result)                                        // 如果求解失败
  {
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "solve g failed!"); // 输出调试信息
    return false;                                                     // 返回失败
  }

  // change state
  for (int i = 0; i <= frame_count; i++) // 遍历所有帧
  {
    Matrix3d Ri = all_image_frame[Headers[i]].R;     // 获取旋转矩阵
    Vector3d Pi = all_image_frame[Headers[i]].T;     // 获取位置向量
    Ps[i] = Pi;                                      // 设置滑动窗口中的位置
    Rs[i] = Ri;                                      // 设置滑动窗口中的旋转
    all_image_frame[Headers[i]].is_key_frame = true; // 将所有帧设为关键帧
  }

  double s = (x.tail<1>())(0);           // 获取尺度因子
  for (int i = 0; i <= WINDOW_SIZE; i++) // 遍历滑动窗口
  {
    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]); // 使用更新后的陀螺仪偏置重新积分IMU
  }
  for (int i = frame_count; i >= 0; i--) // 使用尺度因子更新所有位置
    Ps[i] = s * Ps[i] - Rs[i] * TIC[0] -
            (s * Ps[0] - Rs[0] * TIC[0]); // 将位置从相机坐标系转换到IMU坐标系，并应用尺度
  int kv = -1;                            // 速度索引初始化
  map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) // 遍历所有帧
  {
    if (frame_i->second.is_key_frame) // 如果是关键帧
    {
      kv++; // 速度索引递增
      Vs[kv] = frame_i->second.R *
               x.segment<3>(kv * 3); // 设置速度，将优化得到的全局速度转换到局部坐标系
    }
  }

  Matrix3d R0 = Utility::g2R(g); // 根据重力向量计算旋转矩阵，使z轴对齐重力
  double yaw = Utility::R2ypr(R0 * Rs[0]).x();           // 计算yaw角
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // 消除yaw角的影响，保持初始航向不变
  g = R0 * g;                                            // 更新重力向量
  // Matrix3d rot_diff = R0 * Rs[0].transpose();
  Matrix3d rot_diff = R0;                // 旋转差
  for (int i = 0; i <= frame_count; i++) // 遍历所有帧
  {
    Ps[i] = rot_diff * Ps[i]; // 旋转位置
    Rs[i] = rot_diff * Rs[i]; // 旋转姿态
    Vs[i] = rot_diff * Vs[i]; // 旋转速度
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"), "g0     " << g.transpose()); // 输出重力向量
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("estimator"),
                      "my R0  " << Utility::R2ypr(Rs[0]).transpose()); // 输出初始姿态的欧拉角表示

  f_manager.clearDepth(); // 清除所有特征点的深度信息
  f_manager.triangulate(frame_count, Ps, Rs, tic,
                        ric); // 使用新的位姿重新三角化所有特征点

  return true; // 初始化成功
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
  // find previous frame which contians enough correspondance and parallex with
  // newest frame 寻找与最新帧有足够匹配点且视差较大的历史帧
  for (int i = 0; i < WINDOW_SIZE; i++) // 遍历滑动窗口中的所有帧
  {
    vector<pair<Vector3d, Vector3d>> corres;             // 存储对应点对
    corres = f_manager.getCorresponding(i, WINDOW_SIZE); // 获取第i帧与最新帧之间的对应点
    if (corres.size() > 20)                              // 如果对应点数量大于20
    {
      double sum_parallax = 0;                     // 视差总和
      double average_parallax;                     // 平均视差
      for (int j = 0; j < int(corres.size()); j++) // 遍历所有对应点对
      {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1)); // 第i帧中的点
        Vector2d pts_1(corres[j].second(0),
                       corres[j].second(1));      // 最新帧中的点
        double parallax = (pts_0 - pts_1).norm(); // 计算视差（欧氏距离）
        sum_parallax = sum_parallax + parallax;   // 累加视差
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size()); // 计算平均视差
      if (average_parallax * 460 > 30 &&
          m_estimator.solveRelativeRT(corres, relative_R,
                                      relative_T)) // 如果平均视差大于阈值且成功求解相对位姿
      {
        l = i; // 记录选中的历史帧索引
        RCLCPP_DEBUG(rclcpp::get_logger("estimator"),
                     "average_parallax %f choose l %d and newest frame to "
                     "triangulate the whole structure",
                     average_parallax * 460, l);
        return true; // 返回成功
      }
    }
  }
  return false; // 如果没有找到合适的帧，返回失败
}

void Estimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) // 遍历滑动窗口中的所有帧
  {
    para_Pose[i][0] = Ps[i].x(); // 位置x
    para_Pose[i][1] = Ps[i].y(); // 位置y
    para_Pose[i][2] = Ps[i].z(); // 位置z
    Quaterniond q{Rs[i]};        // 旋转矩阵转四元数
    para_Pose[i][3] = q.x();     // 四元数x分量
    para_Pose[i][4] = q.y();     // 四元数y分量
    para_Pose[i][5] = q.z();     // 四元数z分量
    para_Pose[i][6] = q.w();     // 四元数w分量

    if (USE_IMU) // 如果使用IMU
    {
      para_SpeedBias[i][0] = Vs[i].x(); // 速度x
      para_SpeedBias[i][1] = Vs[i].y(); // 速度y
      para_SpeedBias[i][2] = Vs[i].z(); // 速度z

      para_SpeedBias[i][3] = Bas[i].x(); // 加速度计偏置x
      para_SpeedBias[i][4] = Bas[i].y(); // 加速度计偏置y
      para_SpeedBias[i][5] = Bas[i].z(); // 加速度计偏置z

      para_SpeedBias[i][6] = Bgs[i].x(); // 陀螺仪偏置x
      para_SpeedBias[i][7] = Bgs[i].y(); // 陀螺仪偏置y
      para_SpeedBias[i][8] = Bgs[i].z(); // 陀螺仪偏置z
    }
  }

  for (int i = 0; i < NUM_OF_CAM; i++) // 遍历所有相机
  {
    para_Ex_Pose[i][0] = tic[i].x(); // 相机相对IMU的位置x
    para_Ex_Pose[i][1] = tic[i].y(); // 相机相对IMU的位置y
    para_Ex_Pose[i][2] = tic[i].z(); // 相机相对IMU的位置z
    Quaterniond q{ric[i]};           // 相机相对IMU的旋转矩阵转四元数
    para_Ex_Pose[i][3] = q.x();      // 四元数x分量
    para_Ex_Pose[i][4] = q.y();      // 四元数y分量
    para_Ex_Pose[i][5] = q.z();      // 四元数z分量
    para_Ex_Pose[i][6] = q.w();      // 四元数w分量
  }

  VectorXd dep = f_manager.getDepthVector();            // 获取所有特征点的深度向量
  for (int i = 0; i < f_manager.getFeatureCount(); i++) // 遍历所有特征点
    para_Feature[i][0] = dep(i);                        // 设置特征点的深度参数

  para_Td[0][0] = td; // 设置时间偏差参数
}

void Estimator::double2vector() {
  Vector3d origin_R0 = Utility::R2ypr(Rs[0]); // 保存优化前的姿态（yaw-pitch-roll形式）
  Vector3d origin_P0 = Ps[0];                 // 保存优化前的位置

  if (failure_occur) // 如果发生了失败
  {
    origin_R0 = Utility::R2ypr(last_R0); // 使用上一次成功的姿态
    origin_P0 = last_P0;                 // 使用上一次成功的位置
    failure_occur = 0;                   // 重置失败标志
  }

  if (USE_IMU) // 如果使用IMU
  {
    Vector3d origin_R00 =
        Utility::R2ypr(Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4],
                                   para_Pose[0][5])
                           .toRotationMatrix()); // 优化后的初始姿态（yaw-pitch-roll形式）
    double y_diff = origin_R0.x() - origin_R00.x(); // 计算优化前后yaw角的差值
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0)); // 仅考虑yaw角差异的旋转矩阵
    if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
        abs(abs(origin_R00.y()) - 90) < 1.0) // 检查是否接近万向锁奇异点（pitch接近90度）
    {
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "euler singular point!"); // 输出调试信息
      rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4],
                                     para_Pose[0][5])
                             .toRotationMatrix()
                             .transpose(); // 使用矩阵乘法计算旋转差异，避免欧拉角表示的奇异性
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) // 遍历滑动窗口中的所有帧
    {

      Rs[i] =
          rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                         .normalized()
                         .toRotationMatrix(); // 应用旋转差异，更新旋转矩阵

      Ps[i] =
          rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1],
                              para_Pose[i][2] - para_Pose[0][2]) +
          origin_P0; // 应用旋转差异和位置偏移，更新位置

      Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1],
                                  para_SpeedBias[i][2]); // 应用旋转差异，更新速度

      Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4],
                        para_SpeedBias[i][5]); // 更新加速度计偏置

      Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7],
                        para_SpeedBias[i][8]); // 更新陀螺仪偏置
    }
  } else // 如果不使用IMU（纯视觉模式）
  {
    for (int i = 0; i <= WINDOW_SIZE; i++) // 遍历滑动窗口中的所有帧
    {
      Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                          para_Pose[i][5])
                  .normalized()
                  .toRotationMatrix(); // 直接从优化参数中恢复旋转矩阵

      Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1],
                       para_Pose[i][2]); // 直接从优化参数中恢复位置
    }
  }

  if (USE_IMU) // 如果使用IMU
  {
    for (int i = 0; i < NUM_OF_CAM; i++) // 遍历所有相机
    {
      tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1],
                        para_Ex_Pose[i][2]); // 更新相机相对IMU的位置
      ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4],
                           para_Ex_Pose[i][5])
                   .normalized()
                   .toRotationMatrix(); // 更新相机相对IMU的旋转矩阵
    }
  }

  VectorXd dep = f_manager.getDepthVector();            // 获取特征管理器中的深度向量
  for (int i = 0; i < f_manager.getFeatureCount(); i++) // 遍历所有特征点
    dep(i) = para_Feature[i][0];                        // 更新特征点的深度
  f_manager.setDepth(dep); // 将更新后的深度设置回特征管理器

  if (USE_IMU)          // 如果使用IMU
    td = para_Td[0][0]; // 更新时间偏差
}

bool Estimator::failureDetection() {
  return false;                     // 直接返回false，表示不检测失败
  if (f_manager.last_track_num < 2) // 如果跟踪的特征点数量太少
  {
    RCLCPP_INFO(rclcpp::get_logger("estimator"), " little feature %d",
                f_manager.last_track_num); // 输出信息
    // return true; // 注释掉的返回true
  }
  if (Bas[WINDOW_SIZE].norm() > 2.5) // 如果加速度计偏置的范数过大
  {
    RCLCPP_INFO(rclcpp::get_logger("estimator"), " big IMU acc bias estimation %f",
                Bas[WINDOW_SIZE].norm()); // 输出加速度计偏置过大的警告
    return true;                          // 检测到失败
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0) // 如果陀螺仪偏置的范数过大
  {
    RCLCPP_INFO(rclcpp::get_logger("estimator"), " big IMU gyr bias estimation %f",
                Bgs[WINDOW_SIZE].norm()); // 输出陀螺仪偏置过大的警告
    return true;                          // 检测到失败
  }
  /*
  if (tic(0) > 1) // 如果相机外参过大（被注释掉）
  {
      ROS_INFO(" big extri param estimation %d", tic(0) > 1);
      return true;
  }
  */
  Vector3d tmp_P = Ps[WINDOW_SIZE]; // 获取滑动窗口中最新帧的位置
  if ((tmp_P - last_P).norm() > 5)  // 如果位置变化过大（超过5米）
  {
    // ROS_INFO(" big translation"); // 输出位置变化过大的警告（已注释）
    // return true; // 返回失败（已注释）
  }
  if (abs(tmp_P.z() - last_P.z()) > 1) // 如果z方向位置变化过大（超过1米）
  {
    // ROS_INFO(" big z translation"); // 输出z方向位置变化过大的警告（已注释）
    // return true; // 返回失败（已注释）
  }
  Matrix3d tmp_R = Rs[WINDOW_SIZE];              // 获取滑动窗口中最新帧的旋转矩阵
  Matrix3d delta_R = tmp_R.transpose() * last_R; // 计算旋转变化
  Quaterniond delta_Q(delta_R);                  // 将旋转变化转换为四元数
  double delta_angle;                            // 用于存储旋转变化的角度
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0; // 计算旋转角度（转换为角度）
  if (delta_angle > 50)                                 // 如果旋转变化过大（超过50度）
  {
    RCLCPP_INFO(rclcpp::get_logger("estimator"), " big delta_angle "); // 输出旋转变化过大的警告
    // return true; // 返回失败（已注释）
  }
  return false; // 未检测到失败
}

void Estimator::optimization() {
  TicToc t_whole, t_prepare; // 创建计时器，分别用于整个优化和准备阶段
  vector2double();           // 将状态向量转换为优化变量

  ceres::Problem problem;             // 创建ceres优化问题
  ceres::LossFunction *loss_function; // 损失函数指针
  // loss_function = NULL; // 不使用鲁棒核函数（注释掉）
  loss_function = new ceres::HuberLoss(1.0); // 使用Huber损失函数，参数为1.0
  // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH); //
  // 使用Cauchy损失函数（注释掉） ceres::LossFunction* loss_function = new
  // ceres::HuberLoss(1.0); // 重复定义（注释掉）
  for (int i = 0; i < frame_count + 1; i++) // 遍历滑动窗口中的所有帧
  {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization(); // 创建位姿局部参数化对象（处理四元数等非欧空间）
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE,
                              local_parameterization); // 添加位姿参数块
    if (USE_IMU)                                       // 如果使用IMU
      problem.AddParameterBlock(para_SpeedBias[i],
                                SIZE_SPEEDBIAS); // 添加速度和偏置参数块
  }
  if (!USE_IMU)                                      // 如果不使用IMU（纯视觉模式）
    problem.SetParameterBlockConstant(para_Pose[0]); // 固定第一帧的位姿，作为参考坐标系

  for (int i = 0; i < NUM_OF_CAM; i++) // 遍历所有相机
  {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization(); // 创建位姿局部参数化对象
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization); // 添加相机外参参数块
    if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) ||
        openExEstimation) // 如果需要估计外参（满足条件或已开启外参估计）
    {
      // ROS_INFO("estimate extinsic param"); // 输出估计外参信息（注释掉）
      openExEstimation = 1; // 设置外参估计标志为开启
    } else                  // 否则不估计外参
    {
      // ROS_INFO("fix extinsic param"); // 输出固定外参信息（注释掉）
      problem.SetParameterBlockConstant(para_Ex_Pose[i]); // 固定外参参数块
    }
  }
  problem.AddParameterBlock(para_Td[0], 1); // 添加时间偏差参数块

  if (!ESTIMATE_TD || Vs[0].norm() < 0.2)          // 如果不估计时间偏差或者速度太小
    problem.SetParameterBlockConstant(para_Td[0]); // 固定时间偏差参数块

  if (last_marginalization_info &&
      last_marginalization_info->valid) // 如果有上一次的边缘化信息且有效
  {
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info); // 创建边缘化因子
    problem.AddResidualBlock(
        marginalization_factor, NULL,
        last_marginalization_parameter_blocks); // 添加边缘化残差块，不使用鲁棒核函数
  }
  if (USE_IMU) // 如果使用IMU
  {
    for (int i = 0; i < frame_count; i++) // 遍历滑动窗口中除最新帧外的所有帧
    {
      int j = i + 1;                          // 下一帧的索引
      if (pre_integrations[j]->sum_dt > 10.0) // 如果IMU预积分的时间间隔过大（大于10秒）
        continue;                             // 跳过该帧
      IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]); // 创建IMU因子
      problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                               para_SpeedBias[j]); // 添加IMU残差块，连接相邻两帧
    }
  }

  int f_m_cnt = 0;                          // 特征匹配计数器
  int feature_index = -1;                   // 特征索引
  for (auto &it_per_id : f_manager.feature) // 遍历所有特征
  {
    it_per_id.used_num = it_per_id.feature_per_frame.size(); // 计算特征被观测的次数
    if (it_per_id.used_num < 4)                              // 如果观测次数少于4次
      continue;                                              // 跳过该特征

    ++feature_index; // 特征索引递增

    int imu_i = it_per_id.start_frame,
        imu_j = imu_i - 1; // 特征首次观测到的帧和初始化计数器

    Vector3d pts_i = it_per_id.feature_per_frame[0].point; // 获取特征在起始帧中的归一化坐标

    for (auto &it_per_frame : it_per_id.feature_per_frame) // 遍历该特征的所有观测帧
    {
      imu_j++;            // 观测帧索引递增
      if (imu_i != imu_j) // 如果当前帧不是起始帧
      {
        Vector3d pts_j = it_per_frame.point; // 获取特征在当前帧中的归一化坐标
        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(
            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
            it_per_frame.velocity, // 创建两帧单目投影因子（带时间偏差）
            it_per_id.feature_per_frame[0].cur_td,
            it_per_frame.cur_td); // 传入特征点、特征点速度和时间偏差
        problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                 para_Ex_Pose[0], para_Feature[feature_index],
                                 para_Td[0]); // 添加残差块，连接起始帧和当前帧
      }

      if (STEREO && it_per_frame.is_stereo) // 如果是双目且当前帧有右目观测
      {
        Vector3d pts_j_right = it_per_frame.pointRight; // 特征在右目中的归一化坐标
        if (imu_i != imu_j)                             // 如果当前帧不是起始帧
        {
          ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td); // 创建两帧双目投影因子
          problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j],
                                   para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index],
                                   para_Td[0]); // 添加残差块，连接起始帧的左目和当前帧的右目
        } else // 如果当前帧是起始帧（同一帧内的左右目）
        {
          ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td); // 创建单帧双目投影因子
          problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index],
                                   para_Td[0]); // 添加残差块，连接同一帧的左右目
        }
      }
      f_m_cnt++; // 特征匹配计数器递增
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "visual measurement count: %d",
               f_m_cnt); // 输出视觉测量的数量
  // printf("prepare for ceres: %f \n", t_prepare.toc()); //
  // 输出准备时间（注释掉）

  ceres::Solver::Options options; // 创建ceres求解器选项

  options.linear_solver_type = ceres::DENSE_SCHUR; // 设置线性求解器类型为稠密Schur消元
  // options.num_threads = 2; // 线程数（注释掉）
  options.trust_region_strategy_type = ceres::DOGLEG; // 设置信赖域策略为DOGLEG
  options.max_num_iterations = NUM_ITERATIONS;        // 设置最大迭代次数
  // options.use_explicit_schur_complement = true; // 使用显式Schur补（注释掉）
  // options.minimizer_progress_to_stdout = true; //
  // 输出最小化进度到标准输出（注释掉） options.use_nonmonotonic_steps = true;
  // // 使用非单调步长（注释掉）
  if (marginalization_flag == MARGIN_OLD)                         // 如果边缘化旧帧
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0; // 设置最大求解时间为标准时间的4/5
  else                                                            // 如果边缘化次新帧
    options.max_solver_time_in_seconds = SOLVER_TIME; // 设置最大求解时间为标准时间
  TicToc t_solver;                                    // 求解计时器
  ceres::Solver::Summary summary;                     // 求解结果摘要
  ceres::Solve(options, &problem, &summary);          // 执行优化求解
  // cout << summary.BriefReport() << endl; // 输出摘要报告（注释掉）
  RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "Iterations : %d",
               static_cast<int>(summary.iterations.size())); // 输出迭代次数
  // printf("solver costs: %f \n", t_solver.toc()); // 输出求解时间（注释掉）

  double2vector(); // 将优化后的参数转换回状态向量
  // printf("frame_count: %d \n", frame_count); // 输出帧数（注释掉）

  if (frame_count < WINDOW_SIZE) // 如果滑动窗口未满
    return;                      // 返回，不执行边缘化

  TicToc t_whole_marginalization;         // 边缘化计时器
  if (marginalization_flag == MARGIN_OLD) // 如果边缘化旧帧
  {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo(); // 创建边缘化信息
    vector2double(); // 再次将状态向量转换为优化参数，用于边缘化

    if (last_marginalization_info &&
        last_marginalization_info->valid) // 如果存在上一次的边缘化信息且有效
    {
      vector<int> drop_set; // 需要丢弃的参数块索引集合
      for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++) // 遍历上一次的边缘化参数块
      {
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] ==
                para_SpeedBias[0]) // 如果参数块是最老帧的位姿或速度偏置
          drop_set.push_back(i);   // 将其添加到丢弃集合中
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor =
          new MarginalizationFactor(last_marginalization_info); // 创建新的边缘化因子
      ResidualBlockInfo *residual_block_info =
          new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks,
                                drop_set); // 创建残差块信息，包含参数块和丢弃集合
      marginalization_info->addResidualBlockInfo(
          residual_block_info); // 添加残差块信息到新的边缘化信息中
    }

    if (USE_IMU) {                              // 如果使用IMU
      if (pre_integrations[1]->sum_dt < 10.0) { // 如果第一帧与第二帧之间的IMU预积分时间间隔小于10秒
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]); // 创建IMU因子
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL, // 不使用鲁棒核函数
            vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                             para_SpeedBias[1]}, // 相关的参数块：最老帧和次老帧的位姿和速度偏置
            vector<int>{0, 1}); // 标记最老帧的位姿和速度偏置为要边缘化的变量
        marginalization_info->addResidualBlockInfo(
            residual_block_info); // 将IMU残差块添加到边缘化信息中
      }
    }

    {
      int feature_index = -1;                                    // 特征索引初始化
      for (auto &it_per_id : f_manager.feature) {                // 遍历所有特征
        it_per_id.used_num = it_per_id.feature_per_frame.size(); // 计算特征被观测的次数
        if (it_per_id.used_num < 4)                              // 如果观测次数少于4次
          continue;                                              // 跳过该特征

        ++feature_index; // 特征索引递增

        int imu_i = it_per_id.start_frame,
            imu_j = imu_i - 1; // 特征首次观测到的帧和初始化计数器
        if (imu_i != 0)        // 如果特征不是在最老帧首次观测到的
          continue;            // 跳过该特征，因为只对最老帧的特征进行边缘化

        Vector3d pts_i = it_per_id.feature_per_frame[0].point; // 特征在起始帧中的归一化坐标

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
          imu_j++;
          if (imu_i != imu_j) {                  // 如果当前帧不是起始帧
            Vector3d pts_j = it_per_frame.point; // 特征在当前帧中的归一化坐标
            ProjectionTwoFrameOneCamFactor *f_td =
                new ProjectionTwoFrameOneCamFactor( // 创建两帧单目投影因子（带时间偏差）
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td,
                    it_per_frame.cur_td); // 特征点坐标和速度以及时间偏差
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f_td, loss_function, // 使用鲁棒核函数
                vector<double *>{
                    para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                    para_Feature[feature_index],
                    para_Td
                        [0]}, // 相关的参数块：起始帧位姿、当前帧位姿、相机外参、特征点逆深度、时间偏差
                vector<int>{0, 3}); // 标记起始帧位姿和特征点逆深度为要边缘化的变量
            marginalization_info->addResidualBlockInfo(
                residual_block_info); // 将视觉残差块添加到边缘化信息中
          }
          if (STEREO && it_per_frame.is_stereo) { // 如果是双目且当前帧有右目观测
            Vector3d pts_j_right = it_per_frame.pointRight; // 特征在当前帧右目中的归一化坐标
            if (imu_i != imu_j) {                           // 如果当前帧不是起始帧
              ProjectionTwoFrameTwoCamFactor *f =
                  new ProjectionTwoFrameTwoCamFactor( // 创建两帧双目投影因子
                      pts_i, pts_j_right,             // 左目起始点和右目当前点
                      it_per_id.feature_per_frame[0].velocity,
                      it_per_frame.velocityRight, // 左目起始点速度和右目当前点速度
                      it_per_id.feature_per_frame[0].cur_td,
                      it_per_frame.cur_td); // 时间偏差
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                  f, loss_function, // 使用鲁棒核函数
                  vector<double *>{
                      para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                      para_Feature[feature_index],
                      para_Td
                          [0]}, // 相关的参数块：起始帧位姿、当前帧位姿、左目外参、右目外参、特征点逆深度、时间偏差
                  vector<int>{0, 4}); // 标记起始帧位姿和特征点逆深度为要边缘化的变量
              marginalization_info->addResidualBlockInfo(
                  residual_block_info); // 将视觉残差块添加到边缘化信息中
            } else { // 如果当前帧就是起始帧（同一帧内的左右目观测）
              ProjectionOneFrameTwoCamFactor *f =
                  new ProjectionOneFrameTwoCamFactor( // 创建单帧双目投影因子
                      pts_i, pts_j_right,             // 左目点和右目点
                      it_per_id.feature_per_frame[0].velocity,
                      it_per_frame.velocityRight, // 左目点速度和右目点速度
                      it_per_id.feature_per_frame[0].cur_td,
                      it_per_frame.cur_td); // 时间偏差
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                  f, loss_function, // 使用鲁棒核函数
                  vector<double *>{
                      para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index],
                      para_Td[0]}, // 相关的参数块：左目外参、右目外参、特征点逆深度、时间偏差
                  vector<int>{2}); // 标记特征点逆深度为要边缘化的变量
              marginalization_info->addResidualBlockInfo(
                  residual_block_info); // 将视觉残差块添加到边缘化信息中
            }
          }
        }
      }
    }

    TicToc t_pre_margin;                    // 预边缘化计时器
    marginalization_info->preMarginalize(); // 进行预边缘化，构建增量方程
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "pre marginalization %f ms",
                 t_pre_margin.toc()); // 输出预边缘化时间

    TicToc t_margin;                     // 边缘化计时器
    marginalization_info->marginalize(); // 执行边缘化，得到先验信息
    RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "marginalization %f ms",
                 t_margin.toc()); // 输出边缘化时间

    std::unordered_map<long, double *> addr_shift; // 地址映射表，用于更新参数块地址
    for (int i = 1; i <= WINDOW_SIZE; i++) { // 遍历滑动窗口中除最老帧外的所有帧
      addr_shift[reinterpret_cast<long>(para_Pose[i])] =
          para_Pose[i - 1]; // 位姿参数块地址前移，相当于将第i帧移动到第i-1帧的位置
      if (USE_IMU)
        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
            para_SpeedBias[i - 1]; // 速度偏置参数块地址前移
    }
    for (int i = 0; i < NUM_OF_CAM; i++)                                     // 遍历所有相机
      addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i]; // 相机外参不变

    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0]; // 时间偏差不变

    vector<double *> parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift); // 获取更新地址后的参数块列表

    if (last_marginalization_info)                    // 如果存在上一次的边缘化信息
      delete last_marginalization_info;               // 释放内存
    last_marginalization_info = marginalization_info; // 更新边缘化信息
    last_marginalization_parameter_blocks = parameter_blocks; // 更新边缘化参数块列表

  } else {                           // 如果边缘化次新帧
    if (last_marginalization_info && // 如果存在上一次的边缘化信息
        std::count(std::begin(last_marginalization_parameter_blocks),
                   std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1])) { // 如果上一次的边缘化参数块中包含次新帧的位姿

      MarginalizationInfo *marginalization_info = new MarginalizationInfo(); // 创建新的边缘化信息
      vector2double(); // 将状态向量转换为优化参数
      if (last_marginalization_info &&
          last_marginalization_info->valid) { // 如果上一次的边缘化信息有效
        vector<int> drop_set;
        for (int i = 0; // 遍历上一次的边缘化参数块
             i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
          assert(last_marginalization_parameter_blocks[i] !=
                 para_SpeedBias[WINDOW_SIZE - 1]); // 断言不包含次新帧的速度偏置，因为这会被整体丢弃
          if (last_marginalization_parameter_blocks[i] ==
              para_Pose[WINDOW_SIZE - 1]) // 如果是次新帧的位姿
            drop_set.push_back(i);        // 添加到丢弃集合中
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info); // 创建新的边缘化因子
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL, last_marginalization_parameter_blocks,
            drop_set); // 创建残差块信息，包含参数块和丢弃集合

        marginalization_info->addResidualBlockInfo(
            residual_block_info); // 将残差块信息添加到新的边缘化信息中
      }

      TicToc t_pre_margin; // 预边缘化计时器
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "begin marginalization"); // 开始边缘化
      marginalization_info->preMarginalize(); // 进行预边缘化，构建增量方程
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "end pre marginalization, %f ms",
                   t_pre_margin.toc()); // 输出预边缘化时间

      TicToc t_margin;                                                        // 边缘化计时器
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "begin marginalization"); // 开始边缘化
      marginalization_info->marginalize(); // 执行边缘化，得到先验信息
      RCLCPP_DEBUG(rclcpp::get_logger("estimator"), "end marginalization, %f ms",
                   t_margin.toc()); // 输出边缘化时间

      std::unordered_map<long, double *> addr_shift; // 地址映射表，用于更新参数块地址
      for (int i = 0; i <= WINDOW_SIZE; i++) {       // 遍历滑动窗口中的所有帧
        if (i == WINDOW_SIZE - 1)                    // 如果是次新帧
          continue;                                  // 跳过，因为它被边缘化了
        else if (i == WINDOW_SIZE) {                 // 如果是最新帧
          addr_shift[reinterpret_cast<long>(para_Pose[i])] =
              para_Pose[i - 1]; // 位姿参数块地址前移到次新帧位置
          if (USE_IMU)
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
                para_SpeedBias[i - 1]; // 速度偏置参数块地址前移
        } else {                       // 如果是其他帧
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i]; // 位姿参数块地址不变
          if (USE_IMU)
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] =
                para_SpeedBias[i]; // 速度偏置参数块地址不变
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++)                                     // 遍历所有相机
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i]; // 相机外参地址不变

      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0]; // 时间偏差地址不变

      vector<double *> parameter_blocks =
          marginalization_info->getParameterBlocks(addr_shift); // 获取更新地址后的参数块列表
      if (last_marginalization_info)                    // 如果存在上一次的边缘化信息
        delete last_marginalization_info;               // 释放内存
      last_marginalization_info = marginalization_info; // 更新边缘化信息
      last_marginalization_parameter_blocks = parameter_blocks; // 更新边缘化参数块列表
    }
  }
  // printf("whole marginalization costs: %f \n",
  // t_whole_marginalization.toc()); // 输出整个边缘化过程的时间（注释掉）
  // printf("whole time for ceres: %f \n",
  // t_whole.toc()); // 输出整个ceres优化的时间（注释掉）
}

void Estimator::slideWindow() {
  TicToc t_margin;                            // 滑动窗口计时器
  if (marginalization_flag == MARGIN_OLD) {   // 如果边缘化最老帧
    double t_0 = Headers[0];                  // 记录最老帧的时间戳
    back_R0 = Rs[0];                          // 备份最老帧的旋转矩阵
    back_P0 = Ps[0];                          // 备份最老帧的位置
    if (frame_count == WINDOW_SIZE) {         // 如果滑动窗口已满
      for (int i = 0; i < WINDOW_SIZE; i++) { // 遍历滑动窗口中除最新帧外的所有帧
        Headers[i] = Headers[i + 1];          // 时间戳前移
        Rs[i].swap(Rs[i + 1]);                // 旋转矩阵前移
        Ps[i].swap(Ps[i + 1]);                // 位置前移
        if (USE_IMU) {                        // 如果使用IMU
          std::swap(pre_integrations[i],
                    pre_integrations[i + 1]); // IMU预积分前移

          dt_buf[i].swap(dt_buf[i + 1]); // 时间间隔缓存前移
          linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]); // 线加速度缓存前移
          angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);       // 角速度缓存前移

          Vs[i].swap(Vs[i + 1]);   // 速度前移
          Bas[i].swap(Bas[i + 1]); // 加速度计偏置前移
          Bgs[i].swap(Bgs[i + 1]); // 陀螺仪偏置前移
        }
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1]; // 最新帧时间戳保持不变
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];           // 最新帧位置保持不变
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];           // 最新帧旋转矩阵保持不变

      if (USE_IMU) {                             // 如果使用IMU
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];   // 最新帧速度保持不变
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1]; // 最新帧加速度计偏置保持不变
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1]; // 最新帧陀螺仪偏置保持不变

        delete pre_integrations[WINDOW_SIZE]; // 释放最新帧的预积分
        pre_integrations[WINDOW_SIZE] =
            new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]}; // 创建新的预积分

        dt_buf[WINDOW_SIZE].clear();                  // 清空时间间隔缓存
        linear_acceleration_buf[WINDOW_SIZE].clear(); // 清空线加速度缓存
        angular_velocity_buf[WINDOW_SIZE].clear();    // 清空角速度缓存
      }

      if (true || solver_flag == INITIAL) { // 无论系统状态如何，都会清理所有图像帧
        map<double, ImageFrame>::iterator it_0; // 迭代器
        it_0 = all_image_frame.find(t_0);       // 找到最老帧
        delete it_0->second.pre_integration;    // 释放最老帧的预积分
        all_image_frame.erase(all_image_frame.begin(),
                              it_0); // 删除最老帧之前的所有帧
      }
      slideWindowOld(); // 调用特征管理器的滑动窗口函数（处理最老帧）
    }
  } else {                                             // 如果边缘化次新帧
    if (frame_count == WINDOW_SIZE) {                  // 如果滑动窗口已满
      Headers[frame_count - 1] = Headers[frame_count]; // 用最新帧的时间戳替换次新帧
      Ps[frame_count - 1] = Ps[frame_count];           // 用最新帧的位置替换次新帧
      Rs[frame_count - 1] = Rs[frame_count];           // 用最新帧的旋转矩阵替换次新帧

      if (USE_IMU) {
        for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) { // 遍历最新帧的IMU数据
          double tmp_dt = dt_buf[frame_count][i];                       // 时间间隔
          Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i]; // 线加速度
          Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];       // 角速度

          pre_integrations[frame_count - 1]->push_back(
              tmp_dt, tmp_linear_acceleration,
              tmp_angular_velocity); // 将最新帧的IMU数据添加到次新帧的预积分中

          dt_buf[frame_count - 1].push_back(tmp_dt); // 将最新帧的时间间隔添加到次新帧
          linear_acceleration_buf[frame_count - 1].push_back(
              tmp_linear_acceleration); // 将最新帧的线加速度添加到次新帧
          angular_velocity_buf[frame_count - 1].push_back(
              tmp_angular_velocity); // 将最新帧的角速度添加到次新帧
        }

        Vs[frame_count - 1] = Vs[frame_count];   // 用最新帧的速度替换次新帧
        Bas[frame_count - 1] = Bas[frame_count]; // 用最新帧的加速度计偏置替换次新帧
        Bgs[frame_count - 1] = Bgs[frame_count]; // 用最新帧的陀螺仪偏置替换次新帧

        delete pre_integrations[WINDOW_SIZE]; // 释放最新帧的预积分
        pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                            Bgs[WINDOW_SIZE]}; // 创建新的预积分

        dt_buf[WINDOW_SIZE].clear();                  // 清空最新帧的时间间隔缓存
        linear_acceleration_buf[WINDOW_SIZE].clear(); // 清空最新帧的线加速度缓存
        angular_velocity_buf[WINDOW_SIZE].clear();    // 清空最新帧的角速度缓存
      }
      slideWindowNew(); // 调用特征管理器的滑动窗口函数（处理次新帧）
    }
  }
}

void Estimator::slideWindowNew() {
  sum_of_front++;                     // 次新帧计数器递增
  f_manager.removeFront(frame_count); // 调用特征管理器的移除前端函数
}

void Estimator::slideWindowOld() {
  sum_of_back++; // 最老帧计数器递增

  bool shift_depth =
      solver_flag == NON_LINEAR ? true : false; // 如果是非线性优化状态，则需要移动深度
  if (shift_depth) {                            // 如果需要移动深度
    Matrix3d R0, R1;                            // 相机旋转矩阵
    Vector3d P0, P1;                            // 相机位置
    R0 = back_R0 * ric[0];                      // 最老帧备份的相机旋转矩阵
    R1 = Rs[0] * ric[0];                        // 滑动后的最老帧的相机旋转矩阵
    P0 = back_P0 + back_R0 * tic[0];            // 最老帧备份的相机位置
    P1 = Ps[0] + Rs[0] * tic[0];                // 滑动后的最老帧的相机位置
    f_manager.removeBackShiftDepth(R0, P0, R1,
                                   P1); // 移除最老帧并移动特征点的深度
  } else                    // 如果不需要移动深度（系统处于初始化状态）
    f_manager.removeBack(); // 直接移除最老帧的特征观测
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();       // 初始化为单位矩阵
  T.block<3, 3>(0, 0) = Rs[frame_count]; // 设置旋转矩阵部分为最新帧的旋转矩阵
  T.block<3, 1>(0, 3) = Ps[frame_count]; // 设置平移向量部分为最新帧的位置
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵
  T.block<3, 3>(0, 0) = Rs[index]; // 设置旋转矩阵部分为指定帧的旋转矩阵
  T.block<3, 1>(0, 3) = Ps[index]; // 设置平移向量部分为指定帧的位置
}

void Estimator::predictPtsInNextFrame() {
  // printf("predict pts in next frame\n");
  if (frame_count < 2) // 如果帧数小于2，无法预测
    return;
  // predict next pose. Assume constant velocity motion
  Eigen::Matrix4d curT, prevT, nextT; // 当前帧、前一帧和预测的下一帧变换矩阵
  getPoseInWorldFrame(curT);          // 获取当前帧在世界坐标系下的位姿
  getPoseInWorldFrame(frame_count - 1, prevT); // 获取前一帧在世界坐标系下的位姿
  nextT = curT * (prevT.inverse() * curT);     // 假设恒速运动，预测下一帧的位姿
  map<int, Eigen::Vector3d> predictPts; // 预测点的映射表，键为特征ID，值为预测的归一化坐标

  for (auto &it_per_id : f_manager.feature) { // 遍历所有特征
    if (it_per_id.estimated_depth > 0) {      // 如果特征有有效的深度估计
      int firstIndex = it_per_id.start_frame; // 特征首次观测到的帧的索引
      int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() -
                      1; // 特征最后观测到的帧的索引
      // printf("cur frame index  %d last frame index %d\n", frame_count,
      // lastIndex);
      if ((int)it_per_id.feature_per_frame.size() >= 2 && // 如果特征至少被观测到两次
          lastIndex == frame_count)                       // 且最后一次观测是在当前帧
      {
        double depth = it_per_id.estimated_depth; // 特征的估计深度
        Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) +
                         tic[0]; // 特征在首次观测帧的IMU坐标系下的坐标
        Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex]; // 特征在世界坐标系下的坐标
        Vector3d pts_local =
            nextT.block<3, 3>(0, 0).transpose() *
            (pts_w - nextT.block<3, 1>(0, 3)); // 特征在预测的下一帧相机坐标系下的坐标
        Vector3d pts_cam =
            ric[0].transpose() * (pts_local - tic[0]); // 特征在预测的下一帧相机坐标系下的归一化坐标
        int ptsIndex = it_per_id.feature_id; // 特征ID
        predictPts[ptsIndex] = pts_cam;      // 将预测的归一化坐标添加到映射表中
      }
    }
  }
  featureTracker.setPrediction(predictPts); // 设置特征追踪器的预测点
  // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj) {
  Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi; // 特征在世界坐标系下的坐标
  Vector3d pts_cj =
      ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj); // 特征在帧j的相机坐标系下的坐标
  Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>(); // 重投影误差
  double rx = residual.x();                                            // 误差的x分量
  double ry = residual.y();                                            // 误差的y分量
  return sqrt(rx * rx + ry * ry); // 返回误差的欧氏距离
}

void Estimator::outliersRejection(set<int> &removeIndex) {
  // return;
  int feature_index = -1;                                    // 特征索引初始化
  for (auto &it_per_id : f_manager.feature) {                // 遍历所有特征
    double err = 0;                                          // 误差和初始化
    int errCnt = 0;                                          // 误差计数初始化
    it_per_id.used_num = it_per_id.feature_per_frame.size(); // 更新特征被使用的次数
    if (it_per_id.used_num < 4)                              // 如果特征观测次数少于4次
      continue;                                              // 跳过该特征
    feature_index++;                                         // 特征索引递增
    int imu_i = it_per_id.start_frame,
        imu_j = imu_i - 1; // 特征首次观测到的帧和初始化计数器

    Vector3d pts_i = it_per_id.feature_per_frame[0].point; // 特征在首次观测帧中的归一化坐标
    double depth = it_per_id.estimated_depth;              // 特征的估计深度
    for (auto &it_per_frame : it_per_id.feature_per_frame) { // 遍历该特征的所有观测帧
      imu_j++;                                               // 观测帧索引递增
      if (imu_i != imu_j) {                                  // 如果当前帧不是起始帧
        Vector3d pts_j = it_per_frame.point; // 特征在当前帧中的归一化坐标
        double tmp_error =
            reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[0],
                              tic[0], depth, pts_i, pts_j); // 计算重投影误差
        err += tmp_error;                                   // 累加误差
        errCnt++;                                           // 误差计数递增
        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
      }
      // need to rewrite projecton factor.........
      if (STEREO && it_per_frame.is_stereo) { // 如果是双目且当前帧有右目观测

        Vector3d pts_j_right = it_per_frame.pointRight; // 特征在当前帧右目中的归一化坐标
        if (imu_i != imu_j) {                           // 如果当前帧不是起始帧
          double tmp_error =
              reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1],
                                tic[1], depth, pts_i,
                                pts_j_right); // 计算左目起始点到右目当前点的重投影误差
          err += tmp_error;                   // 累加误差
          errCnt++;                           // 误差计数递增
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        } else {                                // 如果当前帧是起始帧
          double tmp_error = reprojectionError( // 计算同一帧内左目到右目的重投影误差
              Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1], tic[1], depth,
              pts_i, pts_j_right); // 计算重投影误差
          err += tmp_error;        // 累加误差
          errCnt++;                // 误差计数递增
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        }
      }
    }
    double ave_err = err / errCnt;  // 计算平均误差
    if (ave_err * FOCAL_LENGTH > 3) // 如果平均误差乘以焦距大于阈值3（像素）
      removeIndex.insert(it_per_id.feature_id); // 将特征ID添加到移除集合中
  }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                               Eigen::Vector3d angular_velocity) {
  double dt = t - latest_time; // 计算时间间隔
  latest_time = t;             // 更新最新时间
  Eigen::Vector3d un_acc_0 =
      latest_Q * (latest_acc_0 - latest_Ba) - g; // 上一时刻的加速度去偏置并转换到世界坐标系
  Eigen::Vector3d un_gyr =
      0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg; // 两个时刻的平均角速度去偏置
  latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);      // 更新姿态（四元数）
  Eigen::Vector3d un_acc_1 =
      latest_Q * (linear_acceleration - latest_Ba) - g; // 当前时刻的加速度去偏置并转换到世界坐标系
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);         // 两个时刻的平均加速度
  latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc; // 更新位置（匀加速运动）
  latest_V = latest_V + dt * un_acc;                            // 更新速度（匀加速运动）
  latest_acc_0 = linear_acceleration;                           // 更新上一时刻的加速度
  latest_gyr_0 = angular_velocity;                              // 更新上一时刻的角速度
}

void Estimator::updateLatestStates() {
  mPropagate.lock();                       // 加锁，防止多线程冲突
  latest_time = Headers[frame_count] + td; // 更新最新时间（包含时间偏差）
  latest_P = Ps[frame_count];              // 更新最新位置
  latest_Q = Rs[frame_count];              // 更新最新姿态
  latest_V = Vs[frame_count];              // 更新最新速度
  latest_Ba = Bas[frame_count];            // 更新最新加速度计偏置
  latest_Bg = Bgs[frame_count];            // 更新最新陀螺仪偏置
  latest_acc_0 = acc_0;                    // 更新最新的加速度测量值
  latest_gyr_0 = gyr_0;                    // 更新最新的角速度测量值
  mBuf.lock();                             // 加锁缓冲区，防止多线程冲突
  queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf; // 复制加速度缓冲区
  queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf; // 复制角速度缓冲区
  mBuf.unlock();                                            // 解锁缓冲区
  while (!tmp_accBuf.empty()) {                             // 遍历所有IMU数据
    double t = tmp_accBuf.front().first;                    // 获取时间戳
    Eigen::Vector3d acc = tmp_accBuf.front().second;        // 获取加速度测量值
    Eigen::Vector3d gyr = tmp_gyrBuf.front().second;        // 获取角速度测量值
    fastPredictIMU(t, acc, gyr);                            // 使用IMU数据进行快速预测
    tmp_accBuf.pop();                                       // 移除处理过的加速度数据
    tmp_gyrBuf.pop();                                       // 移除处理过的角速度数据
  }
  mPropagate.unlock(); // 解锁
}

bool Estimator::isInitialized() const { return solver_flag == NON_LINEAR; }

// 根据像素位置计算目标到相机的NED坐标
Eigen::Vector3d Estimator::calculateTargetNED(double pixel_x, double pixel_y, int camera_id) {
  Eigen::Vector3d target_ned = Eigen::Vector3d::Zero();

  // 确保系统已经初始化
  if (solver_flag != NON_LINEAR) {
    RCLCPP_WARN(rclcpp::get_logger("estimator"),
                "System not initialized, cannot calculate target NED");
    return target_ned;
  }

  // 检查是否有足够的特征点
  if (f_manager.getFeatureCount() < 20) {
    RCLCPP_WARN(rclcpp::get_logger("estimator"), "Not enough features for target localization");
    return target_ned;
  }

  // 获取相机内参 - 使用featureTracker中的相机参数
  if (featureTracker.m_camera.empty() || camera_id >= (int)featureTracker.m_camera.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("estimator"), "Camera parameters not available");
    return target_ned;
  }

  auto camera = featureTracker.m_camera[camera_id];

  // 创建目标像素点
  Eigen::Vector2d target_pixel(pixel_x, pixel_y);

  // 找到距离目标像素最近的20个特征点
  vector<pair<double, const FeaturePerId *>> distance_features;

  const auto &feature_list = f_manager.getFeatureList();
  for (const auto &feature : feature_list) {
    if (feature.estimated_depth <= 0 || feature.solve_flag != 1) {
      continue; // 跳过没有有效深度的特征点
    }

    // 获取最新帧中的特征点位置
    if (!feature.feature_per_frame.empty()) {
      auto &last_frame = feature.feature_per_frame.back();
      Eigen::Vector2d feature_pixel(last_frame.uv.x(), last_frame.uv.y());

      // 计算像素距离
      double distance = (target_pixel - feature_pixel).norm();
      distance_features.push_back({distance, &feature});
    }
  }

  // 按距离排序，取最近的20个
  sort(distance_features.begin(), distance_features.end());
  int num_features = min(20, (int)distance_features.size());

  if (num_features == 0) {
    RCLCPP_WARN(rclcpp::get_logger("estimator"), "No valid features found for target localization");
    return target_ned;
  }

  vector<Eigen::Vector3d> target_positions;

  for (int i = 0; i < num_features; i++) {
    const FeaturePerId *feature = distance_features[i].second;

    // 获取特征点在相机坐标系下的3D位置
    int start_frame = feature->start_frame;
    if (start_frame < 0 || start_frame >= WINDOW_SIZE + 1) {
      continue;
    }

    // 获取特征点的世界坐标
    Eigen::Vector3d feature_world;
    int frame_idx = feature->start_frame;

    // 计算特征点在世界坐标系下的位置
    Eigen::Vector3d pts_i(0, 0, feature->estimated_depth);
    Eigen::Vector3d w_pts_i =
        Rs[frame_idx] * (ric[camera_id] * pts_i + tic[camera_id]) + Ps[frame_idx];

    // 获取当前相机在世界坐标系下的位置
    int current_frame = frame_count;
    Eigen::Vector3d camera_world_pos = Ps[current_frame] + Rs[current_frame] * tic[camera_id];

    // 计算特征点到相机的向量
    Eigen::Vector3d feature_to_camera = camera_world_pos - w_pts_i;

    // 获取特征点和目标的像素位置差
    auto &last_frame = feature->feature_per_frame.back();
    Eigen::Vector2d feature_pixel(last_frame.uv.x(), last_frame.uv.y());
    Eigen::Vector2d pixel_diff = target_pixel - feature_pixel;

    // 使用相机内参将像素差转换为角度差
    // 这里假设使用针孔相机模型
    double focal_length = FOCAL_LENGTH; // 从parameters.h中获取

    // 计算角度差 (简化模型)
    double angle_x = atan2(pixel_diff.x(), focal_length);
    double angle_y = atan2(pixel_diff.y(), focal_length);

    // 构建旋转矩阵来调整方向
    Eigen::Matrix3d rotation_adjustment = Eigen::Matrix3d::Identity();
    rotation_adjustment = Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitY()) *
                          rotation_adjustment;

    // 应用旋转调整到特征点向量
    Eigen::Vector3d adjusted_direction = rotation_adjustment * feature_to_camera.normalized();

    // 假设目标和特征点在同一高度，计算目标位置
    double feature_height = w_pts_i.z();
    double scale = abs(feature_height - camera_world_pos.z()) / abs(adjusted_direction.z());

    if (abs(adjusted_direction.z()) < 0.001) {
      continue; // 避免除零
    }

    Eigen::Vector3d target_world_pos = camera_world_pos + scale * adjusted_direction;

    // 转换为相机坐标系下的NED
    Eigen::Vector3d target_camera_ned = target_world_pos - camera_world_pos;

    target_positions.push_back(target_camera_ned);
  }

  if (target_positions.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("estimator"), "No valid target positions calculated");
    return target_ned;
  }

  // 去除异常点并计算平均值
  // 简单的方法：计算均值和标准差，去除偏离均值超过2个标准差的点
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto &pos : target_positions) {
    mean += pos;
  }
  mean /= target_positions.size();

  // 计算标准差
  Eigen::Vector3d variance = Eigen::Vector3d::Zero();
  for (const auto &pos : target_positions) {
    Eigen::Vector3d diff = pos - mean;
    variance += diff.cwiseProduct(diff);
  }
  variance /= target_positions.size();
  Eigen::Vector3d std_dev = variance.cwiseSqrt();

  // 去除异常点
  vector<Eigen::Vector3d> filtered_positions;
  for (const auto &pos : target_positions) {
    Eigen::Vector3d diff = (pos - mean).cwiseAbs();
    if (diff.x() < 2 * std_dev.x() && diff.y() < 2 * std_dev.y() && diff.z() < 2 * std_dev.z()) {
      filtered_positions.push_back(pos);
    }
  }

  // 计算最终平均值
  if (!filtered_positions.empty()) {
    target_ned = Eigen::Vector3d::Zero();
    for (const auto &pos : filtered_positions) {
      target_ned += pos;
    }
    target_ned /= filtered_positions.size();

    RCLCPP_INFO(rclcpp::get_logger("estimator"),
                "Target NED calculated: [%.3f, %.3f, %.3f] using %lu features", target_ned.x(),
                target_ned.y(), target_ned.z(), filtered_positions.size());
  } else {
    // 如果所有点都被过滤掉了，使用原始均值
    target_ned = mean;
    RCLCPP_WARN(rclcpp::get_logger("estimator"),
                "All positions were filtered out, using original mean");
  }

  return target_ned;
}
