#ifndef GRAPH_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define GRAPH_H

#include <string>
#include <vector>
#include <map>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <set>
#include <queue>
#include <list>


//A undirected, weighted graph
class Graph {
public:
 typedef std::string vertex;
 typedef unsigned int weight;

  //The '_t' notation is used to denote a new type
  typedef std::map<vertex, weight> edges_t;
  typedef std::map<vertex, edges_t> graph_t;

public:
  //Adds the passed in vertex to the graph (with no edges).
  void addVertex(geometry_msgs::Pose);
  //Checks if the vertex exists.
  bool hasVertex(geometry_msgs::Point);
  //Adds an edge between the two vertices with the given weight
  void addEdge(geometry_msgs::Point, geometry_msgs::Point, weight);
  //Returns a vector containing all the vertices.
  std::vector<geometry_msgs::Point> getVertices(void);
  //Returns a vector containing the neighbours of a given vertex
  std::vector<vertex> getNeighbours(geometry_msgs::Point p);
  std::vector<weight>getEdges(geometry_msgs::Point);

  vertex convertToVertex(geometry_msgs::Point);
  geometry_msgs::Point convertToPoint(vertex);
  bool endNodesConnected(geometry_msgs::Point start, geometry_msgs::Point goal);
  unsigned int getVertexID(geometry_msgs::Point p);

private:

  graph_t weightedGraph_; //The data structure storing our graph.
};

#endif // GRAPH_H
