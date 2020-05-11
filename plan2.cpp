#include "ros/ros.h"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include "stdlib.h"
#include <std_msgs/String.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


//# define ROWS 544
# define ROWS 194
//# define COLUMNS 480
# define COLUMNS 231
# define LENGTH ROWS*COLUMNS

using namespace std;


 float init_x = -6.9;
 float init_y = -5.9;
//float init_x = -12.2;
//float init_y = -12.2;
float resolution = 0.05;



class get_data {
 
 public:

 
 int x[LENGTH];
 char mat[ROWS][COLUMNS];
 int node_list[][2]={};
 int *data();
 void reshape(int x[LENGTH]);
 int *get_grid_pos(double x, double y);
 int ** astar(int goal_x, int goal_y, int start_x, int start_y);
 int heuristics(int current_node_x,int current_node_y,int goal_x,int goal_y);
 void mark_path(double current_coord_x, double current_coord_y);
 void grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid);
};




int  * get_data::data()
  {
     
     cout<<"entered data function successfully"<<endl;

    int count = 0;
    ifstream myFile;
    myFile.open("/home/satyam/turtlebot/out2.csv");

    while (myFile.good() && (count<LENGTH) )
    {
      string y;
      getline(myFile, y);
      int num = stoi(y);
      x[count] = num;
      count ++;
    }
     
    cout  << "got data successfully"<<count<<endl;

    myFile.close();
    return x;

}


void get_data::reshape(int x[LENGTH])
{

  // cout<<" entered reshape successfully"<<endl;
int inc = 0;
for (int i= ROWS-1; i>=0; i--)
   {
    cout <<endl;
    for (int j=0; j<COLUMNS; j++)
    {
      //mat[i][j] = x[inc];
      if(x[inc] <0)
      {
        mat[i][j] = '&';
      }
      if(x[inc] == 0)
      {
        mat[i][j] = '0';
      }
      if(x[inc] == 100)
      {
        mat[i][j] = '*';
      }

      inc++;
      //cout << mat[i][j];
    }
    
   }
   // cout<< "reshaped successfully"<<endl;


  //  for (int i=0; i<ROWS; i++)
  //  {
  //    cout << endl;
  //   for (int j=0; j<COLUMNS; j++)
  //   {
       
  //     cout<<mat[i][j];
  //   }
  // }
}









int main(int argc, char **argv)
{

  ros::init(argc, argv, "plan");
  //ros::NodeHandle n;
  ros::NodeHandle mark;
  // ros::Rate r(10);


  // get_data g;

  //  g.data();
  

  //  g.reshape(g.x);





  // array bulding from pointers

  
  ros::Publisher vis_pub = mark.advertise <nav_msgs::Path>( "trajectory", 1 , true );

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  nav_msgs::Path path;
  //nav_msgs::Path path;
  path.header.stamp=current_time;
  path.header.frame_id="map";

  // ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &get_data::grid, &g);
// Detecting Obstacles



   double current_pos_x = 0.6;
   double current_pos_y = 0;

 // double current_pos_x;
 // double current_pos_y; 

//double current_pos_y = -5.9;

//std::vector<geometry_msgs::PoseStamped> poses;


  ros::Rate loop_rate(10);
  int i=0;
  int current_node_row = 0;
  int current_node_column = 0;


   while ((ros::ok())&&(i<20))
   {


   current_time = ros::Time::now();
 


  // current_node_row = my_node_list[i][0];
  // current_node_column = my_node_list[i][1];



   // current_pos_x = (current_node_column*resolution) + init_x;
   // current_pos_y = (current_node_row*resolution) + init_y;
  // cout << "x  " << current_pos_x << " y  " << current_pos_y<<" " << i <<endl;
  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = current_pos_x ;
  this_pose_stamped.pose.position.y = current_pos_y ;
  this_pose_stamped.header.stamp=current_time;
  this_pose_stamped.header.frame_id="map";
  path.poses.push_back(this_pose_stamped);

  // if(poses.empty())
  //    cout<<"yes";
  //  else
  //   cout<<"no";

  vis_pub.publish(path);
  
  i++;

// current_node_column = current_node_column + 1;
// current_node_row = current_node_row +1 ;
current_pos_x = current_pos_x+0.1;
current_pos_y = current_pos_y+0.0;
// g.mat[194-current_node_row][current_node_column]='*';
    //ros::Subscriber sub = n.subscribe("/obstacle", 1000, grid);

  ros::spinOnce();
  last_time = current_time;
  loop_rate.sleep();

 // if(poses.size())
 //     cout<<poses.size();
   }

 
  // for (int i=0; i<ROWS; i++)
  //  {
  //    cout << endl;
  //   for (int j=0; j<COLUMNS; j++)
  //   {
       
  //     cout<< g.mat[i][j];
  //   }
  // }


 return 0;
}
