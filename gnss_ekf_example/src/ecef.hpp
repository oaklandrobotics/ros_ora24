#pragma once
#include <eigen3/Eigen/Dense>
#include <cmath>

#define WGS84_A 6378137.0
#define WGS84_E2 6.6943799014e-3
static double N(double lat) {
  return WGS84_A / sqrt(1.0 - WGS84_E2 * sin(lat) * sin(lat));
}

static Eigen::Vector3d computeEcef(double lat, double lon, double alt) {
  Eigen::Vector3d output;
  double n_val = N(lat);
  output.x() = (n_val + alt) * cos(lat) * cos(lon);
  output.y() = (n_val + alt) * cos(lat) * sin(lon);
  output.z() = (n_val * (1 - WGS84_E2) + alt) * sin(lat);
  return output;
}

static Eigen::Matrix3d computeEnuRotMat(double ref_lat, double ref_lon, double ref_alt) {
  Eigen::Matrix3d enu_rot_mat;
  enu_rot_mat << -sin(ref_lon),                 cos(ref_lon),                0,
                 -sin(ref_lat) * cos(ref_lon), -sin(ref_lat) * sin(ref_lon), cos(ref_lat),
                  cos(ref_lat) * cos(ref_lon),  cos(ref_lat) * sin(ref_lon), sin(ref_lat);
  return enu_rot_mat;
}

static Eigen::Vector3d computeEnu(const Eigen::Vector3d& ref_ecef, const Eigen::Matrix3d& enu_rot_mat, double lat_deg, double lon_deg, double alt) {
  return enu_rot_mat * (computeEcef(lat_deg * M_PI / 180.0, lon_deg * M_PI / 180.0, alt) - ref_ecef);
}