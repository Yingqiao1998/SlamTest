#ifndef LOCATOR_H
#define LOCATOR_H
#include <Eigen/Core>
#include <Eigen/Dense>

#include "imm.h"
#include "target.h"

#include "debug.h"

#define dimS 4 // 状态维数 - [x,y,vx,vy]
#define dimZ 2 // 量测维数 - [x,y]
#define dT 0.036f

// 定义定位目标类，储存目标当前ned坐标，同时提供融合定位更新函数
class LocatedTarget {
public:
  LocatedTarget() = default;
  void update(algorithm::Target &target, std::vector<double> nedPos, std::vector<double> droneGPS,
              float deviation, DebugInfo &targetDebugInfo);
  std::vector<double> getTargetGPS();

  void nedToWGS84(const std::vector<double> nedPos, const std::vector<double> droneGPS,
                  std::vector<double> &targetGPS);

  void setStartGPS(std::vector<double> startGPS) {
    _startGPS[0] = startGPS[0];
    _startGPS[1] = startGPS[1];
  }

private:
  std::vector<double> _targetGPS = {0.0, 0.0};
  immAlg _immEst = immAlg(dimS, dimZ, dT);
  // 起始点的GPS坐标
  std::vector<double> _startGPS = {0.0, 0.0};
  // std::array<algorithm::GeoPosition, 20> _targetPosHistory{};
  std::vector<algorithm::GeoPosition> _targetPosHistory;
  // int _historyIndex = 0;
  int _targetSpeedCout = 0;

private:
  // ==== 参数定义 ====
  const float SPEED_THRESHOLD_HIGH = 3.5f; // 判定为运动的上阈值
  const float SPEED_THRESHOLD_LOW = 2.5f;  // 判定为静止的下阈值
  const int MOTION_COUNT_THRESHOLD = 15;   // 超过该计数则认为是运动
  const int MOTION_COUNT_MAX = 30;         // 记分器最大值
  const float SMOOTHING_ALPHA = 0.2f;      // 平滑因子 (0.1~0.3为宜)

  // ==== 状态变量（建议声明为成员变量） ====
  float vlatSmooth = 0.0f;  // 平滑速度（初值设为0或第一帧速度）
};
#endif