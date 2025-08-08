#ifndef SIMPLECOORDINATE_H
#define SIMPLECOORDINATE_H

#include "cam_params.h"
#include "geo.h"
#include "location.h"
#include "locator.h"
#include "target.h"
#include "debug.h"


namespace algorithm {
class Coordinate {
public:
  /**
   * @brief 设置结构体
   *
   */
  struct CoordinateConfig {
    std::string cameraParamPath;
  };

public:
  Coordinate();
  int setConfig(const CoordinateConfig &coordinateConfig);

  int updateTarget(std::vector<Target> &targetList, const GeoAngle &cameraAngle,
                   const GeoAngle &droneAngle, const GeoPosition &droneGPS, DebugInfo &targetDebugInfo);

private:
  void calculateStandardDeviations(const std::array<Eigen::Vector3d, 5> &boxOutputs,
                                   float &standDevNE);

private:
  std::unordered_map<std::string, LocatedTarget> _locatedMap;
  // std::unordered_map<std::string, SegTarget> _buildLocatedMap;
  DroneObjlocation _nedLocation;
  float _deviation = 10; // 预测值和测量值的最大偏差，超过则认为是异常值
  int _cleanCout = 0;
  uint64_t timeStamp = 0;
};
} // namespace algorithm
#endif // SIMPLEGEOTRANS_H