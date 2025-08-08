#include "location.h"
#include <iostream>

using namespace std;
using namespace Eigen;

DroneObjlocation::DroneObjlocation() {}

/**
 * @brief 设置相机参数，推荐对相机提前校准
 * @param img_width    相机图像宽度
 * @param img_height   相机图像高度
 * @param focal        一倍变焦时的焦距值，单位mm
 * @param fx   fx，像素焦距，如果没有校准，fx=focal/sigma，其中sigma为像元大小
 * @param fy   fy，如果没有校准则与fx一样
 * @param cx   cx，如果没有校准，cx=img_width/2
 * @param cy   cy，如果没有校准，cy=img_height/2
 */
DroneObjlocation::DroneObjlocation(uint16_t img_width, uint16_t img_height, float fx, float fy,
                                   float cx, float cy) {
  _img_width = img_width;
  _img_height = img_height;
  _fx = fx;
  _fy = fy;
  _cx = cx;
  _cy = cy;
  _K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
}

/**
 * @brief 设置相机参数
 * @param img_width    相机图像宽度
 * @param img_height   相机图像高度
 * @param focal        一倍变焦时的焦距值，单位mm
 * @param fx   fy，像素焦距
 * @param fy   fx，像素焦距
 * @param cx  cx
 * @param cy  cy
 */
void DroneObjlocation::set_parameter(uint16_t img_width, uint16_t img_height, float fx, float fy,
                                     float cx, float cy) {
  _img_width = img_width;
  _img_height = img_height;
  _fx = fx;
  _fy = fy;
  _cx = cx;
  _cy = cy;
  _K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
}

/**
 * @brief 根据目标相机坐标系下坐标计算目标在机体坐标系下的坐标
 * @param uv             目标所在图像中的像素坐标ux,vy
 * @param focal          相机焦距
 * @param distance       激光测距，即吊舱与图像中心对应的地面目标之间的距离
 * @param euler_camera   吊舱姿态角 yaw,pitch,roll
 * @param euler_drone    无人机姿态角 yaw,pitch,roll
 * @param distance_type
 * 0-distance代表无人机距离起飞点垂直高度，1-distance代表激光测距
 * @return 目标在相机、机体、站点NED坐标系下的坐标
 */
Vector3d DroneObjlocation::get_target_location(int cx, int cy, float distance,
                                               std::vector<double> euler_camera,
                                               std::vector<double> euler_drone, int distance_type) {
  /* 1-2. 准备转移矩阵 */
  // float roll_c = (euler_camera[0] - euler_drone[0]) * M_PI / 180;
  // float pitch_c = (euler_camera[1] - euler_drone[1]) * M_PI / 180;
  // float yaw_c = (euler_camera[2] - euler_drone[2]) * M_PI / 180;

  float roll_c = euler_camera[0] * M_PI / 180;
  float pitch_c = euler_camera[1] * M_PI / 180;
  float yaw_c = euler_camera[2] * M_PI / 180;

  float roll_b = euler_drone[0] * M_PI / 180;
  float pitch_b = euler_drone[1] * M_PI / 180;
  float yaw_b = euler_drone[2] * M_PI / 180;

  Matrix3d Rcb, Rbe;
  // Rcb = AngleAxisd(yaw_c, Vector3d::UnitZ()) * AngleAxisd(pitch_c, Vector3d::UnitY()) *
  //       AngleAxisd(roll_c, Vector3d::UnitX());
  Rcb.setIdentity();
  Rbe = AngleAxisd(yaw_c, Vector3d::UnitZ()) * AngleAxisd(pitch_c, Vector3d::UnitY()) *
        AngleAxisd(roll_c, Vector3d::UnitX());

  // std::cout << "rbe*rcb" << std::endl << Rbe * Rcb << std::endl;

  /* 1-3. 参考向量，在站点NED坐标系下与z轴同方向 */
  Vector3d pref_g(0, 0, 1);

  /* 2. 求无人机距离目标所在水平平面垂直高度
     * 若开启了激光测距，根据激光测距获取
     * 若没有开启激光测距，则假设大地水平，根据无人机距离起飞点高度作为该垂直高度的估计值
     */
  float H, L, theta_1;
  Vector3d p0_c, p0_e;
  switch (distance_type) {
  case 0:
    H = distance;
    break;
  case 1:
    // 激光测距距离，机无人机与图像中心对应的地面位置之间的距离为L=distance
    L = distance;
    // p0在相机坐标系下的坐标
    p0_c = Vector3d(L, 0, 0);
    // p0在站点NED坐标系下的坐标
    p0_e = Rbe * Rcb * p0_c;
    // 归一化
    p0_e.normalize();
    // 求与参考向量的夹角theta_1
    theta_1 = asin(p0_e.cross(pref_g).norm());
    // std::cout << "theta_1" << theta_1 << std::endl;
    // 得到距离目标所在水平平面垂直高度
    H = L * cos(theta_1);
    // cout << "theta_1, in rad=" << theta_1 << ", in deg=" << theta_1 *
    // 180 / M_PI << endl; cout << "H=" << H << endl;
    break;
  default:
    H = distance;
  }

  /* 3. 求相机到目标target的距离 */
  // 目标在图像坐标系下的坐标
  Vector3d pt_uv(cx, cy, 1);
  // 图像坐标系到归一化平面坐标系
  Vector3d pt_norm;
  pt_norm = _K.inverse() * pt_uv;
  // cout << "pt_norm=" << pt_norm << endl;
  // 归一化平面坐标系到相机坐标系
  Vector3d pt_c_norm(pt_norm.z(), pt_norm.x(), pt_norm.y());
  // 相机坐标系到站点NED坐标系
  Vector3d pt_e_norm = Rbe * Rcb * pt_c_norm;
  pt_e_norm.normalize();
  // 求与参考向量夹角theta_2
  float theta_2 = asin(pt_e_norm.cross(pref_g).norm());
  // 求距离
  float Lt = H / cos(theta_2);
  // std::cout << "theta_2: " << theta_2 << std::endl;
  // std::cout << "Lt: " << Lt << std::endl;
  // cout << "pt_c_norm=" << pt_c_norm << endl;

  /* 4-1. 求目标在相机坐标系下坐标 */
  Vector3d pt_c = pt_c_norm / pt_c_norm.norm() * Lt;
  // cout << "pt_c=" << pt_c << endl;

  /* 4-2. 求目标在机体坐标系下坐标 */
  Vector3d pt_b = Rcb * pt_c;
  // cout << "pt_b=" << pt_b << endl;

  /* 4-3. 求目标在站点NED坐标系下坐标 */
  Vector3d pt_e = Rbe * pt_b;
  // cout << "pt_e=" << pt_e << endl;

  return pt_e;
}

// Vector3d DroneObjlocation::get_target_location(int cx, int cy, float distance, std::vector<double> euler_camera,
//                                                std::vector<double> euler_drone, int distance_type) {
//     /* 1-2. 准备转移矩阵 */
//     float roll_c = euler_camera[2] * M_PI / 180;
//     float pitch_c = euler_camera[1] * M_PI / 180;
//     float yaw_c = (euler_camera[0] + euler_drone[0]) * M_PI / 180;
//     // float roll_c = euler_camera[2];
//     // float pitch_c = euler_camera[1];
//     // float yaw_c = euler_camera[0];
//     // float roll_b = euler_drone[2];
//     // float pitch_b = euler_drone[1];
//     // float yaw_b = euler_drone[0];
//     // std::cout << "roll_c=" << roll_c << ", pitch_c=" << pitch_c
//     //           << ", yaw_c=" << yaw_c << ", roll_b=" << roll_b
//     //           << ", pitch_b=" << pitch_b << ", yaw_b=" << yaw_b << std::endl;

//     Matrix3d Rce;
//     Rce = AngleAxisd(yaw_c, Vector3d::UnitZ()) * AngleAxisd(pitch_c, Vector3d::UnitY()) *
//           AngleAxisd(roll_c, Vector3d::UnitX());

//     std::cout << "Rce" << std::endl << Rce << std::endl;

//     /* 1-3. 参考向量，在站点NED坐标系下与z轴同方向 */
//     Vector3d pref_g(0, 0, 1);

//     /* 2. 求无人机距离目标所在水平平面垂直高度
//      * 若开启了激光测距，根据激光测距获取
//      * 若没有开启激光测距，则假设大地水平，根据无人机距离起飞点高度作为该垂直高度的估计值
//      */
//     float H, L, theta_1;
//     Vector3d p0_c, p0_e;
//     switch (distance_type) {
//         case 0:
//             H = distance;
//             break;
//         case 1:
//             // 激光测距距离，机无人机与图像中心对应的地面位置之间的距离为L=distance
//             L = distance;
//             // p0在相机坐标系下的坐标
//             p0_c = Vector3d(L, 0, 0);
//             // p0在站点NED坐标系下的坐标
//             p0_e = Rce * p0_c;
//             // 归一化
//             p0_e.normalize();
//             // 求与参考向量的夹角theta_1
//             theta_1 = asin(p0_e.cross(pref_g).norm());
//             // 得到距离目标所在水平平面垂直高度
//             H = L * cos(theta_1);
//             // cout << "theta_1, in rad=" << theta_1 << ", in deg=" << theta_1 *
//             // 180 / M_PI << endl; cout << "H=" << H << endl;
//             break;
//     }

//     /* 3. 求相机到目标target的距离 */
//     // 目标在图像坐标系下的坐标
//     Vector3d pt_uv(cx, cy, 1);
//     // 图像坐标系到归一化平面坐标系
//     Vector3d pt_norm;
//     pt_norm = _K.inverse() * pt_uv;
//     // cout << "pt_norm=" << pt_norm << endl;
//     // 归一化平面坐标系到相机坐标系
//     Vector3d pt_c_norm(pt_norm.z(), pt_norm.x(), pt_norm.y());
//     // 相机坐标系到站点NED坐标系
//     Vector3d pt_e_norm = Rce * pt_c_norm;
//     pt_e_norm.normalize();
//     // 求与参考向量夹角theta_2
//     float theta_2 = asin(pt_e_norm.cross(pref_g).norm());
//     // 求距离
//     float Lt = H / cos(theta_2);

//     std::cout << "theta_2: " << theta_2 << std::endl;
//     std::cout << "Lt: " << Lt << std::endl;
//     // cout << "pt_c_norm=" << pt_c_norm << endl;

//     /* 4-1. 求目标在相机坐标系下坐标 */
//     Vector3d pt_c = pt_c_norm / pt_c_norm.norm() * Lt;
//     // cout << "pt_c=" << pt_c << endl;
//     Vector3d pt_e = Rce * pt_c;
//     // cout << "pt_e=" << pt_e << endl;

//     return pt_e;
// }