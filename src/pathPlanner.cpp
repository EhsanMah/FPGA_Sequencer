#include "graph.h"
#include "pathPlanner.h"
#include <iomanip>

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser and Grid)
 * - Respond to an incoming service call
 *
 */

PathPlanner::PathPlanner(ros::NodeHandle nh)
    : nh_(nh)
{
  //Subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &PathPlanner::odomCallback,this);
  
  //Subscribing to laser
  sub2_ = nh_.subscribe("robot_0/base_scan", 10, &PathPlanner::laserCallback,this);
  //Subscribing to occupnacy grid
  sub3_ = nh_.subscribe("local_map/local_map", 1, &PathPlanner::occupancyGridCallback,this);


  //Publishing markers
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/robot_0/prm",3,false);
  pathPub_= nh_.advertise<geometry_msgs::PoseArray>("/robot_0/path",3,false);


  ros::NodeHandle pn("~");
  double example;
  pn.param<double>("example", example, 0.1);
  ROS_INFO_STREAM("example:" << example);

  goalReceived_=false;
  service_=nh_.advertiseService("/robot0/request_goal",&PathPlanner::requestGoalServiceCallback,this);

}

PathPlanner::~PathPlanner()
{

}


bool PathPlanner::requestGoalServiceCallback(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request: [x,y]=[" << req.pose.x << "," << req.pose.y);
  a4_setup::RequestGoalResponsePtr a;

  //Let's get the Grid
  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;
  geometry_msgs::Pose goalPose;
  goalPose.position.x=req.pose.x;
  goalPose.position.y=req.pose.y;
  probRoadMap_.setGoalPose(goalPose);

   goalReceived_=true;
   return true; //We retrun true to indicate the service call sucseeded (your responce should indicate a value)
}

// A callback for odometry
void PathPlanner::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // We store a copy of the pose and lock a mutex when updating
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;
    probRoadMap_.setRobotOdom(*msg);


}


void PathPlanner::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;
  probRoadMap_.setGrid(*msg);


}

void PathPlanner::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

laserData_.mtx.lock();
laserData_.data=*msg;
laserData_.mtx.unlock();
}


void PathPlanner::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! What rate shoudl we run this at?
    ros::Rate rate_limiter(1.0);
    rate_limiter.sleep();
    while (ros::ok()) {

      if(goalReceived_) {

       probRoadMap_.create_PRM_Graph();

       robot_0_path_.mtx.lock();
       robot_0_path_.path.poses= probRoadMap_.getShortestPath();
       pathPub_.publish(robot_0_path_.path);
       robot_0_path_.mtx.unlock();
      }
      rate_limiter.sleep();
    }
}

