#include "locator.h"

// #include <iostream>

#include "WGS84toCartesian.h"

using namespace Eigen;

void LocatedTarget::nedToWGS84(const std::vector<double> nedPos, const std::vector<double> droneGPS,
                               std::vector<double> &targetGPS) {
  std::array<double, 2> basePointGPS = {droneGPS[0], droneGPS[1]};
  std::array<double, 2> en = {nedPos[1], nedPos[0]};
  std::array<double, 2> ans = wgs84::fromCartesian(basePointGPS, en);
  targetGPS[0] = ans[0];
  targetGPS[1] = ans[1];
}

void LocatedTarget::update(algorithm::Target &target, std::vector<double> nedPos,
                           std::vector<double> droneGPS, float deviation,
                           DebugInfo &targetDebugInfo) {
  std::array<double, 2> ENstartToNow =
      wgs84::toCartesian({_startGPS[0], _startGPS[1]}, {droneGPS[0], droneGPS[1]});

  MatrixXd tmpEst =
      _immEst.immSingleEstimator(nedPos[0] + ENstartToNow[1], nedPos[1] + ENstartToNow[0]);
  std::vector<double> targetNed;
  targetNed.resize(3);

  targetNed[0] = tmpEst(0);
  targetNed[1] = tmpEst(1);

  // target.vlat = tmpEst(2, 0);
  // target.vlon = tmpEst(3, 0);

  // target.vlat = std::sqrt(std::pow(tmpEst(2, 0), 2) + std::pow(tmpEst(3, 0), 2));
  // if (target.vlat > 2.5) {
  //   _targetSpeedCout++;
  //   if (_targetSpeedCout > 30) {
  //     target.vlon = 1;
  //   } else {
  //     target.vlon = 0;
  //   }
  // } else {
  //   if (_targetSpeedCout > 60) {
  //     _targetSpeedCout = 60;
  //   }
  //   if (_targetSpeedCout > 30) {
  //     target.vlon = 1;
  //     _targetSpeedCout--;
  //   } else {
  //     target.vlon = 0;
  //     _targetSpeedCout = 0;
  //   }
  // }
  // ==== 实时速度计算 ====
  target.vlat = std::hypot(tmpEst(2, 0), tmpEst(3, 0)); // 等价于 sqrt(x^2 + y^2)，更稳定

  // ==== 一阶低通滤波 ====
  vlatSmooth = SMOOTHING_ALPHA * target.vlat + (1.0f - SMOOTHING_ALPHA) * vlatSmooth;

  // ==== 迟滞 + 抗抖逻辑 ====
  if (vlatSmooth > SPEED_THRESHOLD_HIGH) {
    _targetSpeedCout = std::min(_targetSpeedCout + 1, MOTION_COUNT_MAX);
  } else if (vlatSmooth < SPEED_THRESHOLD_LOW) {
    _targetSpeedCout = std::max(_targetSpeedCout - 1, 0);
  }

  // ==== 状态输出 ====
  target.vlon = (_targetSpeedCout > MOTION_COUNT_THRESHOLD) ? 1 : 0;

  // targetNed[0] = nedPos[0];
  // targetNed[1] = nedPos[1];
  std::vector<double> targetCalNed = {nedPos[0], nedPos[1]};
  std::vector<double> tempGPS = {0.0, 0.0};

  nedToWGS84(targetNed, _startGPS, _targetGPS);
  nedToWGS84(targetCalNed, droneGPS, tempGPS);

  targetDebugInfo.calLat = tempGPS[0];
  targetDebugInfo.calLon = tempGPS[1];

  target.geoPos.lat = _targetGPS[0];
  target.geoPos.lon = _targetGPS[1];
  target.geoPos.height = 0;
  if (_targetPosHistory.size() > 100) {
    _targetPosHistory.erase(_targetPosHistory.begin());
  }
  _targetPosHistory.push_back(target.geoPos);
  // if (_targetPosHistory.size() < 5)
  // {
  //     target.vy = 0.0;
  //     return;
  // }

  algorithm::GeoPosition meanGeo;
  for (size_t i = 0; i < _targetPosHistory.size(); ++i) {
    meanGeo.lat += _targetPosHistory[i].lat;
    meanGeo.lon += _targetPosHistory[i].lon;
  }
  meanGeo.lat /= _targetPosHistory.size();
  meanGeo.lon /= _targetPosHistory.size();

  // std::vector<double> distVec;
  // double distMean;
  // for (int i = 0; i < _targetPosHistory.size(); i++) {
  //     auto tempEN = wgs84::toCartesian(
  //         {meanGeo.lat, meanGeo.lon},
  //         {_targetPosHistory[i].lat, _targetPosHistory[i].lon});
  //     // std::cout << "tmpEN: " << tempEN[0] << " " << tempEN[1] <<
  //     std::endl; double tmpDist = std::hypot(tempEN[0], tempEN[1]);
  //     // std::cout << "tempDist: " << tmpDist << std::endl;
  //     distMean += tmpDist;
  //     distVec.push_back(tmpDist);
  // }
  // distMean /= _targetPosHistory.size();
  // // std::cout << "distMean: " << distMean << std::endl;
  // double distVar = 0.0;
  // for (const auto &dist : distVec) {
  //     // std::cout << "dist: " << dist << std::endl;
  //     distVar += (dist - distMean) * (dist - distMean);
  // }
  // // std::cout << "distVar: " << distVar << std::endl;
  // distVar /= distVec.size();
  // target.vy = std::sqrt(distVar);

  // algorithm::GeoPosition meanGeo;
  // for (int i = 0; i < _targetPosHistory.size(); ++i)
  // {
  //     meanGeo.lat += _targetPosHistory[i].lat;
  //     meanGeo.lon += _targetPosHistory[i].lon;
  // }
  // meanGeo.lat /= _targetPosHistory.size();
  // meanGeo.lon /= _targetPosHistory.size();

  // std::vector<double> distVec;
  // double distMin = 1000.0;
  // double distMax = 0.0;
  // for (int i=0 ; i < _targetPosHistory.size(); i++) {
  //     auto tempEN = wgs84::toCartesian(
  //         {meanGeo.lat, meanGeo.lon}, {_targetPosHistory[i].lat,
  //         _targetPosHistory[i].lon});
  //     double tmpDist = std::hypot(tempEN[0], tempEN[1]);
  //     if (tmpDist > distMax)
  //     {
  //         distMax = tmpDist;
  //     }
  //     if (tmpDist < distMin)
  //     {
  //         distMin = tmpDist;
  //     }
  //     distVec.push_back(tmpDist);
  // }
  // double distSum = 0.0;
  // for (const auto &dist : distVec) {
  //     distSum += (dist - distMin) * (dist - distMin);
  // }
  // distSum /= distVec.size();
  // target.vy = std::pow(distMax-distMin, 2) / distSum;
}
