#include "coordinate.h"

#include <iostream>
#include <unordered_set>

#include "WGS84toCartesian.h"
#include "cam_params_tools.hpp"
#include "image_point.h"

using namespace algorithm;

Coordinate::Coordinate() {
  _nedLocation = DroneObjlocation();
  _locatedMap = std::unordered_map<std::string, LocatedTarget>();
}

int Coordinate::setConfig(const CoordinateConfig &coordinateConfig) {
  CamParams camParams;
  int ret = CamParamsTools::parseParams(coordinateConfig.cameraParamPath, camParams);
  if (ret != 0) {
    return ret;
  }
  _nedLocation.set_parameter(camParams.width, camParams.height, camParams.fx, camParams.fy,
                             camParams.cx, camParams.cy);
  std::cout << "Camera parameters:" << "width: " << camParams.width
            << ", height: " << camParams.height << ", fx: " << camParams.fx
            << ", fy: " << camParams.fy << ", cx: " << camParams.cx << ", cy: " << camParams.cy
            << std::endl;
  ;
  return 0;
}

int Coordinate::updateTarget(std::vector<Target> &targetList, const GeoAngle &cameraAngle,
                             const GeoAngle &droneAngle, const GeoPosition &droneGPS, DebugInfo &targetDebugInfo) {
  double heightFix = std::max(0.1, droneGPS.height);
  for (auto &target : targetList) {
    Eigen::Vector3d output = _nedLocation.get_target_location(
        target.bbox.cx, target.bbox.cy + target.bbox.h / 2, heightFix,
        std::vector{(double)cameraAngle.roll, (double)cameraAngle.pitch, (double)cameraAngle.yaw},
        std::vector{(double)droneAngle.roll, (double)droneAngle.pitch, (double)droneAngle.yaw}, 0);

    // NOTE: 添加了框的标准差计算,en方向值分别赋给vx和vy
    ImagePoint leftTop{target.bbox.cx - target.bbox.w / 2, target.bbox.cy - target.bbox.h / 2};
    ImagePoint rightTop{target.bbox.cx + target.bbox.w / 2, target.bbox.cy - target.bbox.h / 2};
    ImagePoint leftBottom{target.bbox.cx - target.bbox.w / 2, target.bbox.cy + target.bbox.h / 2};
    ImagePoint rightBottom{target.bbox.cx + target.bbox.w / 2, target.bbox.cy + target.bbox.h / 2};

    std::vector<ImagePoint> corners = {leftTop, rightTop, leftBottom, rightBottom};
    Eigen::Vector3d leftTopOutput = _nedLocation.get_target_location(
        leftTop.x, leftTop.y, heightFix,
        std::vector{(double)cameraAngle.roll, (double)cameraAngle.pitch, (double)cameraAngle.yaw},
        std::vector{(double)droneAngle.roll, (double)droneAngle.pitch, (double)droneAngle.yaw}, 0);

    Eigen::Vector3d rightTopOutput = _nedLocation.get_target_location(
        rightTop.x, rightTop.y, heightFix,
        std::vector{(double)cameraAngle.roll, (double)cameraAngle.pitch, (double)cameraAngle.yaw},
        std::vector{(double)droneAngle.roll, (double)droneAngle.pitch, (double)droneAngle.yaw}, 0);

    Eigen::Vector3d leftBottomOutput = _nedLocation.get_target_location(
        leftBottom.x, leftBottom.y, heightFix,
        std::vector{(double)cameraAngle.roll, (double)cameraAngle.pitch, (double)cameraAngle.yaw},
        std::vector{(double)droneAngle.roll, (double)droneAngle.pitch, (double)droneAngle.yaw}, 0);

    Eigen::Vector3d rightBottomOutput = _nedLocation.get_target_location(
        rightBottom.x, rightBottom.y, heightFix,
        std::vector{(double)cameraAngle.roll, (double)cameraAngle.pitch, (double)cameraAngle.yaw},
        std::vector{(double)droneAngle.roll, (double)droneAngle.pitch, (double)droneAngle.yaw}, 0);
    std::array<Eigen::Vector3d, 5> boxOutput = {leftTopOutput, rightTopOutput, leftBottomOutput,
                                                rightBottomOutput, output};
    calculateStandardDeviations(boxOutput, target.vx);
    std::vector<double> nedPos = {output.x(), output.y()};

    if (_locatedMap.find(target.trackId) != _locatedMap.end()) {
      _locatedMap[target.trackId].update(
          target, nedPos, std::vector{(double)droneGPS.lat, (double)droneGPS.lon, heightFix},
          _deviation, targetDebugInfo);
    } else {
      LocatedTarget newTarget = LocatedTarget();
      newTarget.setStartGPS(std::vector{droneGPS.lat, droneGPS.lon});
      newTarget.update(target, nedPos,
                       std::vector{(double)droneGPS.lat, (double)droneGPS.lon, heightFix},
                       _deviation, targetDebugInfo);
      _locatedMap.insert(std::make_pair(target.trackId, newTarget));
    }
  }
  _cleanCout++;
  if (_cleanCout == 500) {
    std::unordered_set<std::string> targetSet;
    for (const auto &target : targetList) {
      targetSet.insert(target.trackId);
    }

    for (auto it = _locatedMap.begin(); it != _locatedMap.end();) {
      if (targetSet.find(it->first) == targetSet.end()) {
        it = _locatedMap.erase(it);
      } else {
        ++it;
      }
    }
    _cleanCout = 0;
  }
  return 0;
}

void Coordinate::calculateStandardDeviations(const std::array<Eigen::Vector3d, 5> &boxOutputs,
                                             float &standDevNE) {
  double varianceN = 0;
  double varianceE = 0;
  double meanN = 0;
  double meanE = 0;
  for (auto &point : boxOutputs) {
    meanN += point.x();
    meanE += point.y();
  }
  meanN /= boxOutputs.size();
  meanE /= boxOutputs.size();
  for (auto &point : boxOutputs) {
    varianceN += pow(point.x() - meanN, 2);
    varianceE += pow(point.y() - meanE, 2);
  }
  standDevNE = sqrt(pow(sqrt(varianceN / boxOutputs.size()), 2) +
                    pow(sqrt(varianceE / boxOutputs.size()), 2));
}