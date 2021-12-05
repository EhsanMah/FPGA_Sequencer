#ifndef PRMGENERATOR_H
#define PRMGENERATOR_H

#include <chrono>
#include<random>

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "graph.h"
#include "grid_processing.h"
#include "types.h"
#include <mutex>

#define MAX_DISTANCE 10.0;
#define MAX_NEIGBOUR_SIZE 5.0;
#define MAX_COORDINATE_RANGE 100.00;
#define MIN_COORDINATE_RANGE -100.00;
class PRMGenerator
{
public:
  /*! @brief PRMGenerator constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
  PRMGenerator(nav_msgs::OccupancyGrid grid,nav_msgs::Odometry robotPose, geometry_msgs::Pose goalPose);
  PRMGenerator();
  /*! @brief  setGrid Taken the parameter from occupancy grid and sets the grid
    *
    *  @param nav_msgs::OccupancyGrid
    *
    */
  void setGrid(nav_msgs::OccupancyGrid);
  /*! @brief setRobotOdom takes the robots obometry and set the values
    *
    *  @param  nav_msgs::Odometry
    *
    */
  void setRobotOdom(nav_msgs::Odometry);

  /*! @brief setStartPose Takes the start pose and sets the values
    *
    *  @param geometry_msgs::Pose
    *
    */
  void setStartPose(geometry_msgs::Pose);

  /*! @brief setGoalPose Takes the goal pose and sets the values
    *
    *  @param  geometry_msgs::Pose
    *  @
    */
  void setGoalPose(geometry_msgs::Pose);

  /*! @brief connectNeighbouringNodes Takes the neighbouring nodes and connects them
    *
    *  @param geometry_msgs::Pose
    *  @note
    */
  void connectNeighbouringNodes(geometry_msgs::Pose);
  /*! @brief transformGlobal takes the global coordinates and calculates them to transform into local coordinates
    *
    *  @param geometry_msgs::Pose origin,
    * @param geometry_msgs::Point ogPos
    *
    */
  geometry_msgs::Pose transformGlobal(geometry_msgs::Pose origin,geometry_msgs::Point ogPos);
  /*! @brief transformLocal takes the local coordinates and calculates them to transform into global coordinates
    *
    *  @param geometry_msgs::Pose localOrigin,
    * @param geometry_msgs::Point globalPos
    *
    */
  geometry_msgs::Point transformLocal(geometry_msgs::Pose localOrigin,geometry_msgs::Point globalPos);

  /*! @brief cellisfFree  checks if the sell is free and flags it
    *
    *  @param geometry_msgs::Pose cell
    *
    */
  bool cellisfFree(geometry_msgs::Pose cell);

  /*! @brief generateNode This function generates random nodes.
    *
    */
  geometry_msgs::Pose generateNode(void);

  /*! @brief create_PRM_Graph this function creates a PRM graph with random nodes and connects the graph
    *

    */
  void create_PRM_Graph(void);

  /*! @brief getDistance takes two points and calculates the distance between them
    *
    *  @param geometry_msgs::Point a
    * @param geometry_msgs::Point b
    *  @note
    */
  double getDistance(geometry_msgs::Point a,geometry_msgs::Point b);

  /*! @brief getShortestPath vector for storing the shortest calculated path
    *
    *  @param
    *  @note
    */
  std::vector<geometry_msgs::Pose>getShortestPath(void);

  /*! @brief calculateMinDistance Takes the distance and node and calculated the minimum distance
    *
    *  @param unsigned int dist[]
    * @param  bool visited[]
    *  @note
    */
  unsigned int calculateMinDistance(unsigned int dist[], bool visited[]);

  /*! @brief calculate shortest path for each npode from the starting point.using Dijkstras shortest Path Algorithm
    *
    *  @param takes the starting point

    *  @note
    */
  std::vector<unsigned int> dijkstrasShortestPath(geometry_msgs::Point src);
private:

  OgMapBuffer grid_;//! Container for pose data
  OdomDataBuffer robotPose_;//! Container for pose data
  PoseDataBuffer startPose_,goalPose_;//! Container for pose data

  double minRange_,maxRange_;//! Container for pose data
  Graph prmGraph_;//! Container for pose data
  unsigned int seed;//!< seed for Random Number generator
  std::default_random_engine generator;//!< Random Number generator to
 GridProcessing gridProcessor;//! Container for pose data
  std::vector<geometry_msgs::Pose> path_;//! Container for pose data

};

#endif // PRMGENERATOR_H
