#include "ros/ros.h"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>

#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/MapMetaData.h>
using namespace std;
void grid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

  int x = msg->data.size();
  int z= msg-> info.width;
  int k= msg->info.height;
  int ox = msg->info.origin.position.x;
  int oy = msg->info.origin.position.y;


  int y[x];
  ofstream outData;
  outData.open("out1.csv", ios::app);

  for(int i=0;i<x;i++)
   {
    y[i] = msg->data[i];
    outData  << y[i] << endl;
   }
 outData.close();
float resolution = msg->info.resolution;

ROS_INFO("resolution:%f", resolution);
//cout<<resolution;
ROS_INFO("ox:%d", ox);
//cout<<z<<" "<<k;
ROS_INFO("oy:%d", oy);


//ROS_INFO("y:%d", y[10]);
//ROS_INFO("height:%d", k);
//ROS_INFO("width:%d", z);
//ROS_INFO("x:%d", x);


}

void dataa(const nav_msgs::MapMetaData::ConstPtr& orr)
{

  float z = orr->origin.position.x;
    ROS_INFO("z:%f",z);
   }
int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/map", 1000, grid);

  ros::spin();

  return 0;
}
