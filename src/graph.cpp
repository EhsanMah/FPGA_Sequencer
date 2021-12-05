#include <set>
#include <queue>
#include "graph.h"
#include <iostream>
#include <iomanip>
#include <bits/stdc++.h>
#include <sstream>

void Graph::addVertex(geometry_msgs::Pose v) {
  //Insert a vertex with no edges
  geometry_msgs::Point vert;
  vert.x=v.position.x;
  vert.y=v.position.y;

   weightedGraph_.insert({convertToVertex(vert),{}});

}

bool Graph::hasVertex(geometry_msgs::Point vertex) {
  //Check if the vertex v, exists in the graph
  Graph::vertex v= convertToVertex(vertex);
  if (weightedGraph_.find(v) != weightedGraph_.end())
  {
    return true;
  }

  return false;
}

void Graph::addEdge(geometry_msgs::Point u, geometry_msgs::Point v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(convertToVertex(u)).insert({convertToVertex(v), w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(convertToVertex(v)).insert({convertToVertex(u), w}); //Inserting an edge between v and u, with weight w
}

std::vector<Graph::weight>Graph::getEdges(geometry_msgs::Point p){
  std::vector<Graph::weight>edges;
  for(auto adj = weightedGraph_[convertToVertex(p)].begin(); adj != weightedGraph_[convertToVertex(p)].end(); adj++) {
    edges.push_back(adj->second);

  }
  return edges;
}
std::vector<Graph::vertex> Graph::getNeighbours(geometry_msgs::Point p){
std::vector<Graph::vertex>neighbours;
for(auto adj = weightedGraph_[convertToVertex(p)].begin(); adj != weightedGraph_[convertToVertex(p)].end(); adj++) {
  neighbours.push_back(adj->first);

}

return neighbours;
}

std::vector<geometry_msgs::Point> Graph:: getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
std::vector<geometry_msgs::Point> vertices;

  for(auto &vert : weightedGraph_)
  {
    vertices.push_back(convertToPoint(vert.first));
  }

  return vertices;
}

Graph::vertex Graph::convertToVertex(geometry_msgs::Point p){
  Graph::vertex v;

  v="X:"+std::to_string(p.x)+","+std::to_string(p.y);
  return v;
}

geometry_msgs::Point Graph::convertToPoint(vertex v){
  geometry_msgs::Point p;
  std::string a= v;
  char string[v.length()+1];
  std::strcpy(string,v.c_str());
  std::stringstream str;
  str<<string[2]<<string[3]<<string[4]<<string[5]<<string[6];
  str>>p.x;
  std::stringstream str1;
   str1<<string[12]<<string[13]<<string[14]<<string[15]<<string[16];
  str1>>p.y;

  return p;
}

unsigned int Graph::getVertexID(geometry_msgs::Point p){
 std::vector<geometry_msgs::Point>::iterator it;
 unsigned int idx;
 std::vector<geometry_msgs::Point> vertices=getVertices();
 for( it = vertices.begin( ); it != vertices.end( ); ++it ) {
        if( it->x==p.x && it->y==p.y ) {

            idx = std::distance(vertices.begin(), it);
        }else {
            idx=0;
}
    }
 return idx;
}

bool Graph::endNodesConnected(geometry_msgs::Point start, geometry_msgs::Point goal){

std::vector<geometry_msgs::Point> vertices=getVertices();
  vertex _start=convertToVertex(start);
  vertex _goal=convertToVertex(goal);
  // Base case
      if (_start == _goal)
        return true;

      // Mark all the vertices as not visited
      bool *visited = new bool[vertices.size()];
      for (unsigned int i = 0; i < vertices.size(); i++)
          visited[i] = false;

      // Creating a queue for BFS
      std::list<vertex> queue;

      unsigned int indx = getVertexID(start);
      visited[indx] = true;
      queue.push_back(_start);
      std::map<vertex, weight>::iterator i;

      while (!queue.empty())
      {

          _start = queue.front();
          queue.pop_front();

          for (i = weightedGraph_[_start].begin(); i !=weightedGraph_[_start].end(); ++i)
          {

              if (i->first == _goal){
                  return true;
              }
              unsigned int idx =getVertexID(convertToPoint(i->first));
              // Else, continue to do BFS
              if (!visited[idx])
              {
                  visited[idx] = true;
                  queue.push_back(i->first);
              }
          }
      }

      return false;
}
