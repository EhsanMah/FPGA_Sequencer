#ifndef TYPES_H
#define TYPES_H

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/LaserScan.h"
#include <mutex>


struct LaserDataBuffer{
  sensor_msgs::LaserScan data;
  std::mutex mtx;
};

struct PoseDataBuffer
{
  //! Question: Given these elements come in two's (pose and time)
  //! Is there a better type of STL container rather than two seperate deques?
    geometry_msgs::Pose pose;
    std::mutex mtx;
};

struct OdomDataBuffer
{
  nav_msgs::Odometry odom;
  std::mutex mtx;
};

struct OgMapBuffer
{
    nav_msgs::OccupancyGrid grid;
    std::mutex mtx;
};

struct poseArrayBuffer
{
    geometry_msgs::PoseArray path;
    std::mutex mtx;
};
#endif // TYPES_H
