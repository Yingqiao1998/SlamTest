#ifndef GEO_LOCATION_H
#define GEO_LOCATION_H

#include <Eigen/Core>
#include <Eigen/Dense>

class DroneObjlocation {
public:
  DroneObjlocation();
  DroneObjlocation(uint16_t img_width, uint16_t img_height, float fx, float fy, float cx, float cy);
  void set_parameter(uint16_t img_width, uint16_t img_height, float fx, float fy, float cx,
                     float cy);
  Eigen::Vector3d get_target_location(int cx, int cy, float distance,
                                      std::vector<double> euler_camera,
                                      std::vector<double> euler_drone, int distance_type);

private:
  uint16_t _img_width = 1920;
  uint16_t _img_height = 1080;

  float _fx = 2906.79562889912;
  float _fy = 2904.03101009034;
  float _cx = 901.980890988701;
  float _cy = 442.344583747795;

  Eigen::Matrix3d _K;
};

#endif //__GEO_LOCATION_H__