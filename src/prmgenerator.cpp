#include "prmgenerator.h"

PRMGenerator::PRMGenerator(nav_msgs::OccupancyGrid grid,nav_msgs::Odometry robotPose, geometry_msgs::Pose goalPose)
  {
  grid_.mtx.lock();
  grid_.grid=grid;
  gridProcessor.setGrid(grid);

  grid_.mtx.unlock();

  robotPose_.mtx.lock();
  robotPose_.odom=robotPose;
  robotPose_.mtx.unlock();

  startPose_.mtx.lock();
  startPose_.pose=robotPose.pose.pose;
   prmGraph_.addVertex(startPose_.pose);
  startPose_.mtx.unlock();

  goalPose_.mtx.lock();
  goalPose_.pose=goalPose;
  prmGraph_.addVertex(goalPose_.pose);
  goalPose_.mtx.unlock();

  minRange_=MIN_COORDINATE_RANGE;
  maxRange_=MAX_COORDINATE_RANGE;

  seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator.seed(seed);


}
PRMGenerator::PRMGenerator(){
  minRange_=MIN_COORDINATE_RANGE;
  maxRange_=MAX_COORDINATE_RANGE;

  seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator.seed(seed);
}
void PRMGenerator::setGrid(nav_msgs::OccupancyGrid grid){
  grid_.mtx.lock();
  grid_.grid=grid;

  gridProcessor.setGrid(grid);
    grid_.mtx.unlock();
}
void PRMGenerator::setRobotOdom(nav_msgs::Odometry odom){
  robotPose_.mtx.lock();
  robotPose_.odom=odom;
  this->setStartPose( robotPose_.odom.pose.pose);
    robotPose_.mtx.unlock();
}
void PRMGenerator::setStartPose(geometry_msgs::Pose start){
 startPose_.mtx.lock();
  startPose_.pose=start;
   startPose_.mtx.unlock();
}
void PRMGenerator::setGoalPose(geometry_msgs::Pose goal){
 goalPose_.mtx.lock();
  goalPose_.pose=goal;
 goalPose_.mtx.unlock();
}

geometry_msgs::Pose PRMGenerator::generateNode(void){
  geometry_msgs::Pose newNode;
  double randomX=0;
  double randomY=0;
  // if you face any problem in generating data come straight away to this
  std::normal_distribution<double>distribution(10,75.0);


            randomX = distribution(generator);
            randomY=distribution(generator);
           if(randomX<minRange_){
            randomX=minRange_;
           }
          else if(randomX>maxRange_){
              randomX=maxRange_;
             }else{
           }

           if(randomY<minRange_){
            randomY=minRange_;
           }
          else if(randomY>maxRange_){
              randomY=maxRange_;
             }else{
           }

        newNode.position.x=randomX;
        newNode.position.y=randomY;


  return newNode;
}



geometry_msgs::Pose PRMGenerator::transformGlobal(geometry_msgs::Pose origin,geometry_msgs::Point ogPos){
    geometry_msgs::Pose calculatedPoint;

    calculatedPoint.position.x=origin.position.x+ogPos.x;
    calculatedPoint.position.y=origin.position.y+ogPos.y;
    return calculatedPoint;
}

geometry_msgs::Point PRMGenerator::transformLocal(geometry_msgs::Pose localOrigin,geometry_msgs::Point globalPos){
  geometry_msgs::Point localPos;

  localPos.x=globalPos.x-localOrigin.position.x;
  localPos.y=globalPos.y-localOrigin.position.y;
  return  localPos;
}
bool PRMGenerator::cellisfFree(geometry_msgs::Pose cell){
 nav_msgs::OccupancyGrid gr;
  grid_.mtx.lock();
  gr = grid_.grid;
   grid_.mtx.unlock();

  unsigned int index = cell.position.y *gr.info.width + cell.position.x;
  if(index>4000){
    return false;
  }else {
    if(gr.data.at(index)<0){
      return false;
    }
    return (gr.data.at(index)<80)?true:false;
}

}


double PRMGenerator::getDistance(geometry_msgs::Point a,geometry_msgs::Point b){
  double distance =fabs(pow( pow((b.x-a.x),2)+pow((b.y-a.y),2),0.5));
  return distance;
}

void  PRMGenerator::connectNeighbouringNodes(geometry_msgs::Pose node){
std::vector<geometry_msgs::Point>neighbours;
auto vertices=prmGraph_.getVertices();

for (unsigned int v=0;v<vertices.size();v++) {
  if(getDistance(node.position,vertices.at(v))<10){
    neighbours.push_back(vertices.at(v));
  }

}

for(auto n: neighbours){
  geometry_msgs::Point neigbour;
   robotPose_.mtx.lock();
geometry_msgs::Pose orgPose=robotPose_.odom.pose.pose;
 robotPose_.mtx.unlock();
  unsigned int edge=static_cast<unsigned int>(getDistance(node.position,n));
  if(prmGraph_.getEdges(n).size()<5 && gridProcessor.checkConnectivity(transformLocal(orgPose,node.position),transformLocal(orgPose,n))==true){
    prmGraph_.addEdge(node.position,n,edge);
  }
  if(prmGraph_.getEdges(node.position).size()<=5){
    break;
    }
  }
}

void PRMGenerator::create_PRM_Graph(void){

  geometry_msgs::Pose start,destination;

   startPose_.mtx.lock();
  start=startPose_.pose;
   startPose_.mtx.unlock();
  goalPose_.mtx.lock();
  destination=goalPose_.pose;
  goalPose_.mtx.unlock();


  if(!prmGraph_.hasVertex(start.position)){
    prmGraph_.addVertex(start);
  }

  if(!prmGraph_.hasVertex(destination.position)){
    prmGraph_.addVertex(destination);
    connectNeighbouringNodes(destination);
  }
  while(!prmGraph_.endNodesConnected(start.position,destination.position)){

    geometry_msgs::Pose newNode=generateNode();

      if(cellisfFree(newNode)){
    geometry_msgs::Pose globalNewNode=transformGlobal(start,newNode.position);

    prmGraph_.addVertex(globalNewNode);
    connectNeighbouringNodes(globalNewNode);
        }else {
            }
       }
}

unsigned int PRMGenerator::calculateMinDistance(unsigned int dist[], bool visited[])
{
    // Initialize min value
    unsigned int min = INT_MAX;
    unsigned int min_index;
   unsigned int vertSize=prmGraph_.getVertices().size();
    for (unsigned int v = 0; v < vertSize; v++){
        if (visited[v] == false && dist[v] <= min){
            min = dist[v], min_index = v;
          }
    }
    return min_index;
}

std::vector<unsigned int> PRMGenerator::dijkstrasShortestPath(geometry_msgs::Point src)
{
  std::vector<geometry_msgs::Point> vertices = prmGraph_.getVertices();
  unsigned int V=vertices.size();
    unsigned int distance[V];

    bool visited[V];
    for (unsigned int i = 0; i < V; i++){
        distance[i] = INT_MAX;
        visited[i] = false;
        }

    distance[prmGraph_.getVertexID(src)] = 0;

    // Find shortest path for all vertices
    for (unsigned int count = 0; count < (V - 1); count++) {

        unsigned int u = calculateMinDistance(distance, visited);

        // Mark the picked vertex as processed
        visited[u] = true;


        for (unsigned int v = 0; v < V; v++){


          prmGraph_.getEdges(vertices.at(u)).at(prmGraph_.getVertexID(vertices.at(v)));
            if (!visited[v] && prmGraph_.getEdges(vertices.at(u)).at(prmGraph_.getVertexID(vertices.at(v))) && distance[u] != INT_MAX
                && distance[u] + prmGraph_.getEdges(vertices.at(u)).at(prmGraph_.getVertexID(vertices.at(v))) < distance[v]){
                distance[v] = distance[u] + prmGraph_.getEdges(vertices.at(u)).at(prmGraph_.getVertexID(vertices.at(v)));}

        }
    }

      std::vector<unsigned int> dist;
      dist.reserve(V);
      dist.assign(V,0);

      for (unsigned int a=0;a<V;a++) {
        dist.at(a)=distance[a];
      }

      return dist;
}

std::vector<geometry_msgs::Pose>PRMGenerator::getShortestPath(void){
   startPose_.mtx.lock();
  geometry_msgs::Point startNode=startPose_.pose.position;
   startPose_.mtx.unlock();
bool path_completed=0;
std::vector<unsigned int>minimumDistances=dijkstrasShortestPath(startNode);
std::vector<geometry_msgs::Point>vertices=prmGraph_.getVertices();
goalPose_.mtx.lock();
path_.push_back(goalPose_.pose);
geometry_msgs::Pose currentNode=goalPose_.pose;
goalPose_.mtx.unlock();

while(!path_completed){

  unsigned int minDist=INT_MAX;
  for(auto a:prmGraph_.getNeighbours(currentNode.position)){
    unsigned int currentNodeDist=minimumDistances.at(prmGraph_.getVertexID(prmGraph_.convertToPoint(a)));
    if(currentNodeDist<minDist){
      minDist=currentNodeDist;
      currentNode.position=prmGraph_.convertToPoint(a);
    }
}
    path_.push_back(currentNode);
  if(currentNode.position.x==startNode.x && currentNode.position.y==startNode.y){
    path_completed=true;
    break;
  }
}
  return path_;
}
