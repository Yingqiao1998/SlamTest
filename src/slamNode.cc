#include "slamNode.h"

#include "WGS84toCartesian.h"

#include "../estimator/parameters.h"
#include "mavros/frame_tf.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>
#include <thread>

namespace slam_composition {
SlamComponent::SlamComponent(const rclcpp::NodeOptions &options) : Node("slam", options) {
  // 定义各节点订阅的主题名称
  declare_parameter<std::string>("drone_attitude_world_sub", "/mavros/uas_2/global_position/local");
  declare_parameter<std::string>("drone_geo_sub", "/mavros/uas_2/global_position/global");
  declare_parameter<std::string>("gimbal_attitude_sub",
                                 "/mavros/uas_2/gimbal_control/device/attitude_status");
  declare_parameter<std::string>("target_real_geo_sub", "/mavros/uas_3/global_position/global");
  declare_parameter<std::string>("target_real_world_sub", "/mavros/uas_3/global_position/local");
  declare_parameter<std::string>("detect_sub", "/robot1/detect_objects");
  declare_parameter<std::string>("target_coordinate_pub", "target_info");

  // VINS-Fusion相关参数
  declare_parameter<std::string>("imu_topic",
                                 "/mavros/uas_2/gimbal_control/device/attitude_status");
  declare_parameter<std::string>("image_topic", "/robot1/camera/image_raw");

  // 创建订阅节点对象
  // 无人机姿态和World位置订阅节点对象
  std::string droneAttitudeWorldSubName;
  get_parameter("drone_attitude_world_sub", droneAttitudeWorldSubName);
  _droneAttitudeWorldSub = this->create_subscription<nav_msgs::msg::Odometry>(
      droneAttitudeWorldSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::droneAttitudeWorldCallback, this, std::placeholders::_1));
  // 无人机经纬度订阅节点对象
  std::string droneGeoSubName;
  get_parameter("drone_geo_sub", droneGeoSubName);
  _droneGeoSub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      droneGeoSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::droneGeoCallback, this, std::placeholders::_1));
  // 云台姿态订阅节点对象
  std::string gimbalAttitudeSubName;
  get_parameter("gimbal_attitude_sub", gimbalAttitudeSubName);
  _gimbalAttitudeSub = this->create_subscription<mavros_msgs::msg::GimbalDeviceAttitudeStatus>(
      gimbalAttitudeSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::gimbalAttitudeCallback, this, std::placeholders::_1));
  // 目标真实经纬度订阅节点对象
  std::string targetRealGeoSubName;
  get_parameter("target_real_geo_sub", targetRealGeoSubName);
  _targetRealGeoSub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      targetRealGeoSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::targetRealGeoCallback, this, std::placeholders::_1));
  // 目标真实World位置订阅节点对象
  std::string targetRealWorldSubName;
  get_parameter("target_real_world_sub", targetRealWorldSubName);
  _targetRealWorldSub = this->create_subscription<nav_msgs::msg::Odometry>(
      targetRealWorldSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::targetRealWorldCallback, this, std::placeholders::_1));
  // 目标识别跟踪数据订阅节点对象
  std::string detectSubName;
  get_parameter("detect_sub", detectSubName);
  _targetInfoSub = this->create_subscription<sim_msgs::msg::TargetInfo>(
      detectSubName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::targetInfoCallback, this, std::placeholders::_1));
  // 目标World位置和经纬度定位节点对象
  std::string targetCoordinatePubName;
  get_parameter("target_coordinate_pub", targetCoordinatePubName);
  _targetCoordinatePub = this->create_publisher<sim_msgs::msg::TargetInfo>(targetCoordinatePubName,
                                                                           rclcpp::SensorDataQoS());

  // 创建VINS-Fusion相关订阅
  std::string imuTopicName;
  get_parameter("imu_topic", imuTopicName);
  _imuSub = this->create_subscription<mavros_msgs::msg::GimbalDeviceAttitudeStatus>(
      imuTopicName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::imuCallback, this, std::placeholders::_1));

  std::string imageTopicName;
  get_parameter("image_topic", imageTopicName);
  _imageSub = this->create_subscription<sensor_msgs::msg::Image>(
      imageTopicName, rclcpp::SensorDataQoS(),
      std::bind(&SlamComponent::imageCallback, this, std::placeholders::_1));

  _coordinate = std::make_shared<algorithm::Coordinate>();

  std::string config_file = 
      "/home/sim/ros2-algorithm-sim/src/slam/cfg/vins_config.yaml";
  readParameters(config_file);

  // 初始化VINS-Fusion estimator
  _estimator = std::make_shared<Estimator>();
  // 重要：确保根据配置初始化参数（包括相机内参等）
  _estimator->setParameter();

  // 设置为单目相机+IMU模式
  _estimator->changeSensorType(1, 0); // use_imu=1, use_stereo=0

  algorithm::Coordinate::CoordinateConfig coordinateConfig{
      "/home/sim/ros2-algorithm-sim/src/coordinate/config/cameraParams.ini"};
  _coordinate->setConfig(coordinateConfig);

  // 暂停程序等待调试器附加
  // RCLCPP_WARN(this->get_logger(), "SLAM Component initialized, raising SIGSTOP for debugger attachment...");
  // raise(SIGSTOP);
  // std::this_thread::sleep_for(std::chrono::seconds(20));
}

// 无人机姿态和World位置订阅节点对象的回调函数实现
void SlamComponent::droneAttitudeWorldCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  // 取gzWorld位置
  _droneGeo.height = msg->pose.pose.position.z;
  _droneWord.east = msg->pose.pose.position.x;
  _droneWord.north = msg->pose.pose.position.y;
  _droneWord.up = msg->pose.pose.position.z;

  // RCLCPP_WARN (this->get_logger(), "Drone World: %f, %f, %f", _droneWord.east, _droneWord.north,
  //             _droneWord.up);

  // 取无人机姿态
  // 从消息中提取四元数
  // 预计算 ENU -> NED 坐标系转换的四元数
  tf2::Quaternion q_z_rot, q_x_rot;
  // 绕原 Z 轴旋转 +90°（yaw）
  q_z_rot.setRPY(0, 0, M_PI / 2);
  // 绕新 X 轴旋转 180°（roll）
  q_x_rot.setRPY(M_PI, 0, 0);
  // 组合得到坐标系变换四元数（ENU->NED）
  tf2::Quaternion q_enu_to_ned = q_z_rot * q_x_rot;

  // RCLCPP_WARN (this->get_logger(), "q_enu_to_ned: %f, %f, %f, %f", q_enu_to_ned.x(),
  //             q_enu_to_ned.y(), q_enu_to_ned.z(), q_enu_to_ned.w());

  tf2::Quaternion q_enu;
  tf2::fromMsg(msg->pose.pose.orientation, q_enu);

  tf2::Quaternion q_ned = q_enu_to_ned * q_enu;
  tf2::Matrix3x3 mat_ned(q_ned);
  double roll_rad, pitch_rad, yaw_rad;
  mat_ned.getRPY(roll_rad, pitch_rad, yaw_rad);
  roll_rad = 0;

  // 将角度转换为度
  _droneAngle.roll = roll_rad * 180 / M_PI;
  _droneAngle.pitch = pitch_rad * 180 / M_PI;
  _droneAngle.yaw = yaw_rad * 180 / M_PI;

  // RCLCPP_WARN (this->get_logger(), "Drone Angle: roll %f, pitch %f, yaw %f", _droneAngle.roll,
  //             _droneAngle.pitch, _droneAngle.yaw);
  // RCLCPP_WARN (this->get_logger(), "Drone Angle(rad): roll %f, pitch %f, yaw %f", roll_rad,
  //             pitch_rad, yaw_rad);
}

// 无人机经纬度订阅节点对象的回调函数实现
void SlamComponent::droneGeoCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  _droneGeo.lat = msg->latitude;
  _droneGeo.lon = msg->longitude;

  // RCLCPP_WARN (this->get_logger(), "Drone GPS: latitude %f, lontitude %f", _droneGeo.lat,
  //             _droneGeo.lon);
}

// 云台姿态订阅节点对象的回调函数实现
void SlamComponent::gimbalAttitudeCallback(
    const mavros_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg) {
  // 互斥锁，保护共享资源
  std::unique_lock<std::shared_mutex> lock(_mutex);

  // 1. 创建 tf2 四元数对象
  tf2::Quaternion quat(msg->q.x, msg->q.y, msg->q.z, msg->q.w);

  // 2. 将四元数转换为欧拉角（roll, pitch, yaw）,单位：弧度
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // 3. 更新云台姿态（将弧度转换为角度）
  _cameraAngle.roll = roll * 180.0 / M_PI;
  _cameraAngle.pitch = pitch * 180.0 / M_PI;
  _cameraAngle.yaw = yaw * 180.0 / M_PI;

  // 4. 打印日志，便于调试
  // RCLCPP_WARN (this->get_logger(), "Gimbal Angle: roll=%f, pitch=%f, yaw=%f", _cameraAngle.roll,
  //             _cameraAngle.pitch, _cameraAngle.yaw);
}

// 目标真实经纬度订阅节点对象的回调函数实现
void SlamComponent::targetRealGeoCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  _targetGeo.lat = msg->latitude;
  _targetGeo.lon = msg->longitude;

  // RCLCPP_WARN (this->get_logger(), "Target real GPS: latitude %f, lontitude %f", _targetGeo.lat,
  //             _targetGeo.lon);
}
// 目标真实World位置订阅节点对象的回调函数实现
void SlamComponent::targetRealWorldCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  _targetWordReal.east = msg->pose.pose.position.x;
  _targetWordReal.north = msg->pose.pose.position.y + 3;
  _targetWordReal.up = msg->pose.pose.position.z;

  _targetWordReal.eastSpeed = msg->twist.twist.linear.x;
  _targetWordReal.northSpeed = msg->twist.twist.linear.y;
  _targetWordReal.upSpeed = msg->twist.twist.linear.z;
  // RCLCPP_WARN (this->get_logger(), "Target World: %f, %f, %f", _targetWordReal.east,
  //             _targetWordReal.north, _targetWordReal.up);
  // RCLCPP_WARN(this->get_logger(), "Target World Speed: %f, %f", _targetWordReal.eastSpeed,
  //             _targetWordReal.northSpeed);
}

// 目标World位置和经纬度定位回调函数实现
void SlamComponent::targetInfoCallback(const sim_msgs::msg::TargetInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  _targetInfo = *msg;

  // RCLCPP_WARN _ONCE(get_logger(), "localization is running ...");

  std::vector<algorithm::Target> targetList{};

  // 解析目标识别跟踪数据
  for (unsigned int i = 0; i < _targetInfo.target_num; i++) {
    algorithm::Target tempTarget;
    tempTarget.clsId = _targetInfo.classes[i];
    tempTarget.bbox.cx = _targetInfo.bboxes[i * 4];
    tempTarget.bbox.cy = _targetInfo.bboxes[i * 4 + 1];
    tempTarget.bbox.w = _targetInfo.bboxes[i * 4 + 2];
    tempTarget.bbox.h = _targetInfo.bboxes[i * 4 + 3];
    // tempTarget.bbox.cy += std::ceil(tempTarget.bbox.h / 2.0);
    tempTarget.bbox.cy += tempTarget.bbox.h / 2.0;

    tempTarget.trackId = _targetInfo.track_id[i];
    targetList.emplace_back(tempTarget);
    // RCLCPP_WARN (this->get_logger(), "bbox.cx,bbox.cy: %d, %d",
    //             tempTarget.bbox.cx, tempTarget.bbox.cy);
  }

  // 使用VINS-Fusion进行目标定位
  // for (unsigned int i = 0; i < _targetInfo.target_num; i++) {
  //   // 获取目标的像素坐标（bbox中心）
  //   double pixel_x = _targetInfo.bboxes[i * 4];     // cx
  //   double pixel_y = _targetInfo.bboxes[i * 4 + 1]; // cy

  //   // if (!_estimator->isInitialized()) {
  //   //   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //   //                        "[estimator]: System not initialized, cannot calculate target NED");
  //   //   continue;
  //   // }
  //   // 使用VINS-Fusion计算目标到相机的NED坐标
  //   Eigen::Vector3d target_ned = _estimator->calculateTargetNED(pixel_x, pixel_y, 0);

  //   RCLCPP_WARN(this->get_logger(), "VINS Target %d NED: [%.3f, %.3f, %.3f]", i, target_ned.x(),
  //               target_ned.y(), target_ned.z());
  // }

  DebugInfo targetDebugInfo{};
  _coordinate->updateTarget(targetList, _cameraAngle, _droneAngle, _droneGeo, targetDebugInfo);
  // RCLCPP_WARN (this->get_logger(),
  //             "droneAngle: roll %f, pitch %f, yaw %f; cameraAngle: roll %f, "
  //             "pitch %f, yaw %f",
  //             _droneAngle.roll, _droneAngle.pitch, _droneAngle.yaw, _cameraAngle.roll,
  //             _cameraAngle.pitch, _cameraAngle.yaw);
  // 将目标定位结果写入目标信息
  for (unsigned int i = 0; i < _targetInfo.target_num; i++) {
    for (auto it = targetList.begin(); it != targetList.end(); it++) {
      if (it->trackId == _targetInfo.track_id[i]) {
        _targetInfo.locations.resize(_targetInfo.target_num * 5);
        _targetInfo.locations[i * 5] = it->geoPos.lon;
        _targetInfo.locations[i * 5 + 1] = it->geoPos.lat;
        _targetInfo.locations[i * 5 + 2] = it->geoPos.height;
        _targetInfo.locations[i * 5 + 3] = it->vlon;
        _targetInfo.locations[i * 5 + 4] = it->vlat;

        std::array<double, 2> targetPredictGeo{_targetInfo.locations[i * 5 + 1],
                                               _targetInfo.locations[i * 5]};
        std::array<double, 2> targetRealGeo{_targetGeo.lat, _targetGeo.lon};
        std::array<double, 2> errorDistance = wgs84::toCartesian(targetRealGeo, targetPredictGeo);
        std::array<double, 2> targetCalGeo{targetDebugInfo.calLat, targetDebugInfo.calLon};
        std::array<double, 2> calErrorDistance = wgs84::toCartesian(targetRealGeo, targetCalGeo);

        // RCLCPP_WARN (this->get_logger(), "Target real Geo: latitude %f, lontitude %f",
        //             targetRealGeo[0], targetRealGeo[1]);
        // RCLCPP_WARN (this->get_logger(), "Target predict Geo: latitude %f, lontitude %f",
        //             targetPredictGeo[0], targetPredictGeo[1]);

        // RCLCPP_WARN(this->get_logger(), "Target predict speed: %f, %f", it->vlon, it->vlat);

        double errorDistanceNorm =
            std::sqrt(errorDistance[0] * errorDistance[0] + errorDistance[1] * errorDistance[1]);
        double calErrorDistanceNorm = std::sqrt(calErrorDistance[0] * calErrorDistance[0] +
                                                calErrorDistance[1] * calErrorDistance[1]);

        // RCLCPP_WARN(this->get_logger(), "Track ID: %s, Error Distance: %f", it->trackId.c_str(),
        //             errorDistanceNorm);
        // RCLCPP_WARN(this->get_logger(), "Track ID: %s, Cal Error Distance: %f", it->trackId.c_str(),
        //             calErrorDistanceNorm);

        // 记录所有需要后期分析的数据到文件中,通过设置日志level来控制存储到文件中的数据
        // RCLCPP_WARN(
        //     this->get_logger(),
        //     "Track ID: %s, real speed: %f, %f, predict speed: %f, %f, read GEO: %f, %f, predict GEO: %f, %f, cal Geo: %f, %f, predict error distance: %f, cal error distance: %f",
        //     it->trackId.c_str(), _targetWordReal.eastSpeed, _targetWordReal.northSpeed, it->vlon,
        //     it->vlat, targetRealGeo[0], targetRealGeo[1], targetPredictGeo[0], targetPredictGeo[1],
        //     targetCalGeo[0], targetCalGeo[1], errorDistanceNorm, calErrorDistanceNorm);
      }
    }
  }

  _targetInfo.with_loc_info = true;
  _targetInfo.frame_type = sim_msgs::msg::TargetInfo::FRAME_GLOBAL_GEO;
  _targetCoordinatePub->publish(_targetInfo);
}
static inline float unpack_accel_1e3(uint16_t v) {
  return (static_cast<int32_t>(v) - 32768) / 1000.0f;
}
// IMU数据回调函数
void SlamComponent::imuCallback(const mavros_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(_mutex);
  const double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  const float ax = unpack_accel_1e3(msg->failure_flags);
  const float ay = msg->delta_yaw;
  const float az = msg->delta_yaw_velocity;

  Eigen::Vector3d linear_acceleration(ax, ay, az);
  Eigen::Vector3d angular_velocity(msg->angular_velocity_x, msg->angular_velocity_y,
                                   msg->angular_velocity_z);

  RCLCPP_WARN(
      this->get_logger(),
      "IMU ts: %.6f, acc[m/s^2]: [%.3f, %.3f, %.3f], gyro[rad/s]: [%.3f, %.3f, %.3f], raw_u16(ax)=0x%04X",
      timestamp, ax, ay, az, angular_velocity.x(), angular_velocity.y(), angular_velocity.z(),
      static_cast<unsigned>(msg->failure_flags));

  _estimator->inputIMU(timestamp, linear_acceleration, angular_velocity);
}

// 图像数据回调函数（cv_bridge 版）
void SlamComponent::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  const double timestamp = static_cast<double>(msg->header.stamp.sec) +
                           static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  if (msg->encoding.empty() || msg->width == 0 || msg->height == 0 || msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "Invalid frame data message!");
    return;
  }

  try {
    cv::Mat gray_frame;

    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      // 单通道：零拷贝共享
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      gray_frame = cv_ptr->image; // 已经是灰度
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      // BGR：零拷贝共享，再转灰度
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, gray_frame, cv::COLOR_BGR2GRAY);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      // RGB：用 cv_bridge 转成 BGR（复制/转换一次），再转灰度
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, gray_frame, cv::COLOR_BGR2GRAY);
    } else {
      RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    if (gray_frame.empty() || gray_frame.channels() != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid gray frame for VINS processing");
      return;
    }

    // 送入估计器
    _estimator->inputImage(timestamp, gray_frame);

    if (!_estimator->isInitialized()) {
      RCLCPP_WARN(this->get_logger(),
                  "[estimator]: System not initialized, cannot calculate target NED");
    } else {
      RCLCPP_WARN(this->get_logger(), "[estimator]: System initialized, can calculate target NED");
    }
  } catch (const cv_bridge::Exception &e) {
    // 建议专门捕获 cv_bridge 的异常
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion error: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Image processing error: %s", e.what());
  }
}

} // namespace slam_composition
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(slam_composition::SlamComponent)