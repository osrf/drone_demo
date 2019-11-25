#ifndef RVIZ_AERIAL_PLUGINS_GEODETIC_CONVERTER_HPP_
#define RVIZ_AERIAL_PLUGINS_GEODETIC_CONVERTER_HPP_

#include <math.h>
#include <eigen3/Eigen/Dense>

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;

// code originally from https://github.com/ethz-asl/geodetic_utils/blob/master/geodetic_utils/include/geodetic_utils/geodetic_conv.hpp
class GeodeticConverter
{
 public:
  GeodeticConverter(double latitude = 0, double longitude = 0, double altitude = 0);

  void getHome(double& latitude, double& longitude, double& altitude);

  void setHome(const double latitude, const double longitude, const double altitude);


  void geodetic2Ecef(const double latitude, const double longitude, const double altitude,
                    double& x, double& y, double& z);

  void ecef2Geodetic(const double x, const double y, const double z,
                    double& latitude, double& longitude, double& altitude);

  void ecef2Ned(const double x, const double y, const double z,
                double& north, double& east, double& down);

  void ned2Ecef(const double north, const double east, const double down,
                double& x, double& y, double& z);

  void geodetic2Ned(const double latitude, const double longitude, const double altitude,
                    double& north, double& east, double& down);


  void ned2Geodetic(const double north, const double east, const double down,
                    double& latitude, double& longitude, double& altitude);


  void geodetic2Enu(const double latitude, const double longitude, const double altitude,
                    double& east, double& north, double& up);

  void enu2Geodetic(const double east, const double north, const double up,
                    double& latitude, double& longitude, double& altitude);

private:
    // Geodetic system parameters
  double kSemimajorAxis;
  double kSemiminorAxis;
  double kFirstEccentricitySquared;
  double kSecondEccentricitySquared;
  double kFlattening;

  inline Matrix3x3d nRe(const double lat_radians, const double lon_radians);
  inline double rad2Deg(const double radians);
  inline double deg2Rad(const double degrees);

  double home_latitude_rad_, home_latitude_;
  double home_longitude_rad_, home_longitude_;
  double home_altitude_;

  double home_ecef_x_;
  double home_ecef_y_;
  double home_ecef_z_;

  Matrix3x3d ecef_to_ned_matrix_;
  Matrix3x3d ned_to_ecef_matrix_;

};
#endif  // RVIZ_AERIAL_PLUGINS_GEODETIC_CONVERTER_HPP_
