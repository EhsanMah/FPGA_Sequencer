#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include "types.h"
//! All the messages we need are here
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"

//! All the services .. RequestGoal is part of a5_setup, not a standard service
#include "a4_setup/RequestGoal.h"

#include <atomic>

//! The class we have developed included here in our node
#include "grid_processing.h"
#include "prmgenerator.h"

class PathPlanner{

public:
  /*! @brief PathPlanner constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PathPlanner(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PathPlanner();


  /*! @brief Request Goal Service
   *
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will start generating path when goal recieved.
   */
    bool requestGoalServiceCallback(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res);


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /*! @brief OccupancyGrid Callback
    *
    *  @param sensor_msgs::ImageConstPtr - The imageconst message
    *  @note This function and the declaration are ROS specific
    */
     void occupancyGridCallback(const nav_msgs::OccupancyGridPtr & msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
    void seperateThread(); 


private:
    ros::NodeHandle nh_;
    ros::Publisher viz_pub_;//! Command velocity publisher
    ros::Publisher pathPub_;
    ros::Subscriber sub1_,sub2_,sub3_;

    ros::ServiceServer service_;

    int count_;//! A counter to allow executing items on N iterations


    OgMapBuffer ogMapBuffer_;//! Container for image data
    PoseDataBuffer poseDataBuffer_;//! Container for pose data
     LaserDataBuffer laserData_;
    std::atomic<bool> goalReceived_;
    poseArrayBuffer robot_0_path_;
    PRMGenerator probRoadMap_;

};

