#ifndef SLAM_NODE_H
#define SLAM_NODE_H

/**
 * @file slamNode.h
 * @author Yingqiao1998 (qiaodefined@gmail.com)
 * @brief 测试定位插件的节点
 * @date 2025-04-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <iostream>
#include <memory>
#include <string>

#include "coordinate.h"
#include "debug.h"

// 添加VINS-Fusion相关头文件
#include "../../src/estimator/estimator.h"
#include "opencv2/opencv.hpp"
#include "mavros_msgs/msg/gimbal_device_attitude_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/visibility_control.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sim_msgs/msg/target_info.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace slam_composition {
// 规定第一个vehicle的ENU坐标系为World坐标系
struct World {
  double east;
  double north;
  double up;
  double eastSpeed;
  double northSpeed;
  double upSpeed;
};

class SlamComponent : public rclcpp::Node {
public:
  RCLCPP_COMPONENTS_PUBLIC
  explicit SlamComponent(const rclcpp::NodeOptions &options);
  ~SlamComponent() = default;

private:
  // 无人机姿态和World位置订阅节点对象的回调函数
  void droneAttitudeWorldCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // 无人机经纬度订阅节点对象的回调函数
  void droneGeoCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  // 云台姿态订阅节点对象的回调函数
  void gimbalAttitudeCallback(const mavros_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg);
  // 目标真实经纬度订阅节点对象的回调函数
  void targetRealGeoCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  // 目标world位置订阅节点对象的回调函数
  void targetRealWorldCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // 目标World位置和经纬度定位回调函数，在此对象中执行目标定位
  void targetInfoCallback(const sim_msgs::msg::TargetInfo::SharedPtr msg);

  // VINS-Fusion相关回调函数
  void imuCallback(const mavros_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
  // 无人机姿态和World位置订阅节点对象
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _droneAttitudeWorldSub;
  // 无人机经纬度订阅节点对象
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _droneGeoSub;
  // 云台姿态订阅节点对象
  rclcpp::Subscription<mavros_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr _gimbalAttitudeSub;
  // 目标真实经纬度订阅节点对象
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _targetRealGeoSub;
  // 目标World位置订阅节点对象
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _targetRealWorldSub;
  // 目标识别跟踪数据订阅节点对象，在此对象中执行目标定位
  rclcpp::Subscription<sim_msgs::msg::TargetInfo>::SharedPtr _targetInfoSub;
  // 目标信息发布节点对象
  rclcpp::Publisher<sim_msgs::msg::TargetInfo>::SharedPtr _targetCoordinatePub;

  // VINS-Fusion相关订阅
  rclcpp::Subscription<mavros_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr _imuSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSub;

private:
  sim_msgs::msg::TargetInfo _targetInfo;

  // 无人机World坐标系位置
  World _droneWord{};
  // 云台的NED姿态
  algorithm::GeoAngle _cameraAngle{};
  // 无人机的NED姿态
  algorithm::GeoAngle _droneAngle{};
  // 无人机的经纬度和高度
  algorithm::GeoPosition _droneGeo{};
  // 目标的经纬度
  algorithm::GeoPosition _targetGeo{};
  // 目标的World坐标系位置
  World _targetWordReal{};

  std::shared_ptr<algorithm::Coordinate> _coordinate;
  std::shared_mutex _mutex;

  // VINS-Fusion estimator实例
  std::shared_ptr<Estimator> _estimator;
  uint debugImageCount = 0; // 用于调试图像计数
};
} // namespace slam_composition
#endif // SLAM_NODE_H