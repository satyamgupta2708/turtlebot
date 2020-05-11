#include "ros/ros.h"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <bits/stdc++.h>
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
#include <limits.h>

//x_max = 4.65= -6.9+COLUMNS*0.05;  //y_max = 3.8= -5.9+ROWS*0.05;

# define ROWS 194
# define COLUMNS 231
# define LENGTH ROWS*COLUMNS
# define max numeric_limits<float>::infinity()

using namespace std;


typedef pair<int, int> pairs; 




 float init_x = -6.9;
 float init_y = -5.9;
 float resolution = 0.05;




class get_data {

private:
      
      float heuristics(int current_node_row,int current_node_col,int goal_row,int goal_col);
      void reshape(int x[LENGTH]);
      auto get_grid_pos(float x, float y);

public:

 int counter = 0;
 int x[LENGTH];
 int total_path_cost = 1;
 

 int start_row;
 int start_col;
 int goal_row; 
 int goal_col; 
 
 stack <pairs> path_taken ;
  
 void data();
 void astar();
 void get_destination(float x , float y);
 void get_origin(float x , float y);
 void path_traversed();

 struct Node
 {   
 	 float heur_cost;
 	 float move_cost ;
 	 float cost;
 	 int parent_node_row;
 	 int parent_node_col;
     int cell_status ;
     char cell_charc ;
     bool visited ; 
 } map[ROWS][COLUMNS];
 
 };




// function to get grid position corresponding to (x,y) coordinates

auto get_data::get_grid_pos(float x,float y)
{
   pairs position;

   int rows = (y-init_y)/resolution;     /// indicating Row 
   int col = (x-init_x)/resolution;     /// indicating Column
   
   position = make_pair((ROWS - rows),col);      //changes to be checked 

   return position; 

}


  void get_data::get_destination(float x , float y)
  {    
        pairs position;

        position = get_grid_pos(x,y);

         goal_row = position.first;
         goal_col  = position.second; 

         if (map[goal_row][goal_col].cell_status == 100)
         {

             cout<<"goal position is not reachable ---mission fail " <<endl;
             //exit(1);

         }

   }
  

   void get_data::get_origin(float x , float y)
  {    
        pairs position;

        position = get_grid_pos(x,y);

         start_row = position.first;
         start_col  = position.second; 

         if (map[start_row][start_col].cell_status == 100)
         {

             cout<<" start position is not reachable ---mission fail "<< endl ;
             //exit(1);

         }

   }




  // function to get data from csv file

 void get_data::data()
  
  {

    int count = 0;
    ifstream myFile;
    myFile.open("/home/satyam/turtlebot/out2.csv");

    while (myFile.good() && (count< LENGTH) )
    {
      string y;
      getline(myFile, y);
      int num = stoi(y);
      x[count] = num;
      count ++;
    }
     
    myFile.close();
    reshape(x);

  }



//function to shape arrays

void get_data::reshape(int x[LENGTH])
{

  cout<<" entered reshape successfully"<<endl;
  int inc = 0;
 

for (int i = ROWS-1; i>=0; i--)
   {
  
    for (int j=0; j<COLUMNS; j++)
    {
      
      if( x[inc] < 0 || x[inc]==100)
      {
        map[i][j].cell_status = 100;
        map[i][j].cell_charc = '*';
      }

      if( x[inc] == 0)
      {
        map[i][j].cell_status = 0;
        map[i][j].cell_charc = '0';
      }
      inc++;
    
     }
   }


  //  for (int i=0; i<ROWS; i++)
  //  {
  //    cout << endl;
  //   for (int j=0; j<COLUMNS; j++)
  //   {
       
  //     cout<< map[i][j].cell_charc;
  //   }
  // }
   cout<< "reshaped successfully"<<endl;

}

float  get_data::heuristics(int current_node_row,int current_node_col,int goal_row,int goal_col)
{
  
   float heur_distance = sqrt(pow((current_node_row-goal_row),2)+ pow((current_node_col-goal_col),2)) ;
   // cout<< heur_distance<<endl;
   return heur_distance ;
}







// function for astar main file
void get_data::astar()

{
  cout << "entering astar"<< endl;
  
  for(int i=0;i<ROWS;i++)
  {
      //cout<<endl;
      for(int j=0;j<COLUMNS;j++)
      {
        map[i][j].move_cost = 0;
        map[i][j].heur_cost = max;
        map[i][j].cost = max ;
        map[i][j].visited = false ;
        map[i][j].parent_node_row = -1;
        map[i][j].parent_node_col = -1;

      }
  }


  map[goal_row][goal_col].cell_status = 2;

  map[start_row][start_col].cell_status = 0;
  map[start_row][start_col].move_cost  = 0;
  map[start_row][start_col].heur_cost = heuristics(start_row, start_col, goal_row,  goal_col);
  map[start_row][start_col].parent_node_row = start_row;
  map[start_row][start_col].parent_node_col = start_col;
  map[start_row][start_col].cost = heuristics(start_row, start_col, goal_row,  goal_col) + 0 ;
  map[start_row][start_col].visited = true;
  
  int p;
  int q;
  
  int current_node_row = start_row;
  int current_node_col = start_col;
  float cost = 0 ;
  

  
  while(!((current_node_row == goal_row) && (current_node_col == goal_col)))
  {  
  	float min_value = max; 
        
     if(((current_node_row-1)>=0) && (map[current_node_row-1][current_node_col].cell_status!=100)
     	&&(map[current_node_row-1][current_node_col].visited == false))
   {

   	 
    cost = total_path_cost + heuristics(current_node_row-1,current_node_col,goal_row,goal_col);
       
       //cout << cost << endl;
       if(cost < map[current_node_row-1][current_node_col].cost)
          {
           
           cout<<"11"<<endl;
          map[current_node_row-1][current_node_col].move_cost = map[current_node_row][current_node_col].move_cost + 1;
          map[current_node_row-1][current_node_col].cost = cost;
          map[current_node_row-1][current_node_col].heur_cost = heuristics(current_node_row-1,current_node_col,goal_row,goal_col);
          map[current_node_row-1][current_node_col].parent_node_row = current_node_row;
          map[current_node_row-1][current_node_col].parent_node_col = current_node_col;
          

          }
             if (cost < min_value)
            {   
            	cout<<"12"<<endl;
            	min_value = cost;
                p = current_node_row-1;
                q = current_node_col;
             }
          
          
   }



          if(((current_node_row+1)<ROWS) && (map[current_node_row+1][current_node_col].cell_status!=100)
          	           &&(map[current_node_row+1][current_node_col].visited == false)) 
    {   

     
      cost =   total_path_cost + heuristics(current_node_row+1,current_node_col,goal_row,goal_col);
      //cout << cost << endl;
      
      if(cost < map[current_node_row +1][current_node_col].cost)
          {
             cout<<"21"<<endl;

          map[current_node_row+1][current_node_col].move_cost = map[current_node_row][current_node_col].move_cost + 1;
          map[current_node_row+1][current_node_col].cost = cost;
          map[current_node_row+1][current_node_col].heur_cost = heuristics(current_node_row+1,current_node_col,goal_row,goal_col);
          map[current_node_row+1][current_node_col].parent_node_row = current_node_row;
          map[current_node_row+1][current_node_col].parent_node_col = current_node_col;
          
          }


          if (cost < min_value)
            {   
            	 cout<<"22"<<endl;
            	min_value = cost;
                p = current_node_row+1;
                q = current_node_col;
             }
          
          
    }

   // Right Direction
  



   if(((current_node_col+1)<COLUMNS) && (map[current_node_row][current_node_col+1].cell_status!=100) 
   	                                     &&(map[current_node_row][current_node_col+1].visited == false))
   {   
         
   	

    cost = total_path_cost + heuristics(current_node_row,current_node_col+1,goal_row,goal_col);
   // cout << cost << endl;
     
    if(cost < map[current_node_row][current_node_col+1].cost)
          
          {
          
          cout<<"31"<<endl;
          map[current_node_row][current_node_col+1].move_cost = map[current_node_row][current_node_col].move_cost + 1;
          map[current_node_row][current_node_col+1].cost = cost;
          map[current_node_row][current_node_col+1].heur_cost = heuristics(current_node_row,current_node_col+1,goal_row,goal_col);
          map[current_node_row][current_node_col+1].parent_node_row = current_node_row;
          map[current_node_row][current_node_col+1].parent_node_col = current_node_col;
          
          }

          if (cost< min_value)
            {   

            	cout<<"32"<<endl;

            	min_value = cost;
                p = current_node_row;
                q = current_node_col+1;
             }
          
          
    }


  


    if(((current_node_col-1)>=0) && (map[current_node_row][current_node_col-1].cell_status!=100)
    	                              &&(map[current_node_row][current_node_col-1].visited == false))
   {

   	
    cost =   total_path_cost + heuristics(current_node_row,current_node_col-1,goal_row,goal_col);
       
        //cout << cost << endl;

    if(cost < map[current_node_row][current_node_col-1].cost)
          
          {
  cout<<"41"<<endl;

          map[current_node_row][current_node_col-1].move_cost = map[current_node_row][current_node_col].move_cost + 1;
          map[current_node_row][current_node_col-1].cost = cost;
          map[current_node_row][current_node_col-1].heur_cost = heuristics(current_node_row,current_node_col-1,goal_row,goal_col);
          map[current_node_row][current_node_col-1].parent_node_row = current_node_row;
          map[current_node_row][current_node_col-1].parent_node_col = current_node_col;
          }


          if (cost < min_value)
            {   
            	cout<<"42"<<endl;
            	min_value = cost;
                p = current_node_row;
                q = current_node_col-1;
             }
          
    }


        if(((current_node_row-1)>=0)&&((current_node_col+1)<COLUMNS) && (map[current_node_row-1][current_node_col+1].cell_status!=100)
        	                                &&(map[current_node_row-1][current_node_col+1].visited == false)) 
    {   
       
       cost =   total_path_cost + heuristics(current_node_row-1,current_node_col+1,goal_row,goal_col);
       //cout << cost << endl;
       
       if(cost < map[current_node_row-1][current_node_col+1].cost)
          
          {
          cout<<"51"<<endl;
           map[current_node_row-1][current_node_col+1].move_cost = map[current_node_row][current_node_col].move_cost + 1.414;
           map[current_node_row-1][current_node_col+1].cost = cost;
           map[current_node_row-1][current_node_col+1].heur_cost = heuristics(current_node_row-1,current_node_col+1,goal_row,goal_col);
           map[current_node_row-1][current_node_col+1].parent_node_row = current_node_row;
           map[current_node_row-1][current_node_col+1].parent_node_col = current_node_col;
           
           }
           
           if ( cost < min_value)
            {   
            	cout<<"52"<<endl;
            	min_value = cost;
                p = current_node_row -1;
                q = current_node_col +1;
             }
          
    }


  



        if(((current_node_row-1)>=0)&& ((current_node_col-1)>=0) && (map[current_node_row-1][current_node_col-1].cell_status!=100)
                                        &&(map[current_node_row-1][current_node_col-1].visited == false)) 
    {   
       
       cost =  total_path_cost + heuristics(current_node_row-1,current_node_col-1,goal_row,goal_col);
       //cout << cost << endl;

       if(cost < map[current_node_row-1][current_node_col-1].cost)
          
          {
          
cout<<"61"<<endl;
          map[current_node_row-1][current_node_col-1].move_cost = map[current_node_row][current_node_col].move_cost + 1.414;
          map[current_node_row-1][current_node_col-1].cost = cost;
          map[current_node_row-1][current_node_col-1].heur_cost = heuristics(current_node_row-1,current_node_col-1,goal_row,goal_col);
          map[current_node_row-1][current_node_col-1].parent_node_row = current_node_row;
          map[current_node_row-1][current_node_col-1].parent_node_col = current_node_col;
          
          }

          if (cost< min_value)
            {   
            	cout<<"62"<<endl;

            	min_value = cost;
                p = current_node_row -1;
                q = current_node_col -1;
             }
          
          
    }


  



      if(((current_node_row+1)<ROWS)&&((current_node_col+1)<COLUMNS) && (map[current_node_row+1][current_node_col+1].cell_status!=100)
      	                             &&(map[current_node_row +1][current_node_col+1].visited == false)) 
    {   
      
     
      cost =  total_path_cost + heuristics(current_node_row +1,current_node_col+1,goal_row,goal_col);
      
      //cout << cost << endl;

       if(cost < map[current_node_row +1][current_node_col+1].cost)
          
          {
           cout<<"71"<<endl;
          map[current_node_row +1][current_node_col+1].move_cost = map[current_node_row][current_node_col].move_cost + 1.414;
          map[current_node_row +1][current_node_col+1].cost = cost;
          map[current_node_row +1][current_node_col+1].heur_cost = heuristics(current_node_row+1,current_node_col+1,goal_row,goal_col);
          map[current_node_row +1][current_node_col+1].parent_node_row = current_node_row;
          map[current_node_row +1][current_node_col+1].parent_node_col = current_node_col;
          
          } 

         if (cost < min_value)
            {   cout<<"72"<<endl;
          
            	min_value =  cost;
                p = current_node_row +1;
                q = current_node_col +1;
             }
          
          

      }





       if(((current_node_row+1)<ROWS) && ((current_node_col-1)>=0)&&(map[current_node_row+1][current_node_col-1].cell_status!=100)
       	              &&(map[current_node_row+1][current_node_col-1].visited == false)) 
    {   
       
       
       cost = total_path_cost + heuristics(current_node_row+1,current_node_col-1,goal_row,goal_col);
       //cout << cost << endl;
       
       if(cost < map[current_node_row +1][current_node_col-1].cost)
          
          {
          
       cout<<"81"<<endl;
          map[current_node_row +1][current_node_col-1].move_cost = map[current_node_row][current_node_col].move_cost + 1.414;
          map[current_node_row +1][current_node_col-1].cost = cost;
          map[current_node_row +1][current_node_col-1].heur_cost = heuristics(current_node_row+1,current_node_col-1,goal_row,goal_col);
          map[current_node_row +1][current_node_col-1].parent_node_row = current_node_row;
          map[current_node_row +1][current_node_col-1].parent_node_col = current_node_col;
          
          }

          if (cost < min_value)
            {   
            	cout<<"82"<<endl;
            	min_value = cost;
                p = current_node_row +1;
                q = current_node_col -1;
             }
          
         

     }
   


      
      map[current_node_row][current_node_col].cost = max;
      map[current_node_row][current_node_col].visited = true;


      current_node_row = p;
      current_node_col = q;

      //cout << "total_path_cost ="<< total_path_cost<<endl;
      cout <<current_node_row<<" "<<current_node_col<<endl;

      cout << "parent_node_row="<<map[current_node_row][current_node_col].parent_node_row<<" "<<"parent_node_col="<< map[current_node_row][current_node_col].parent_node_col<<endl;
      

      total_path_cost += 1 ;

     
       map[current_node_row][current_node_col].cell_charc = '|';
      // current_node_row = p;
      // current_node_col = q;
      
    

}

         cout<<" exiting astar";

      path_traversed();

}

void get_data::path_traversed()
{
   int row = goal_row;
   int col = goal_col;

  while(!((row == map[row][col].parent_node_row)&&(col == map[row][col].parent_node_col)))
      
      {

            pairs x = make_pair(row,col);

            cout << x.first << " " << x.second<<" "<< counter <<endl;
            path_taken.push(x);

            map[row][col].cell_charc = '|';

            row = map[row][col].parent_node_row ;
            col = map[row][col].parent_node_col ;

            
            cout << map[row][col].visited << endl;

            counter = counter+1;
      }

      path_taken.push(make_pair(row,col));

      cout<< "exiting path traversed"<< endl;

}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "plan");
  ros::NodeHandle mark;


  get_data g;
  g.data();
   
 
  g.get_origin(1,0);
  g.get_destination(-1.7,0);

  cout << g.start_row << ' '<<g.start_col<< endl ;
  cout << g.goal_row << ' '<<g.goal_col << endl;
  cout<<max<<endl;



  g.astar();


cout << g.counter<<endl;


/// Marking Path


  ros::Publisher vis_pub = mark.advertise <nav_msgs::Path>( "trajectory", 100, true );

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();



   nav_msgs::Path path;
   path.header.stamp=current_time;
   path.header.frame_id="map";


   ros::Rate loop_rate(10);
  
  int i = 0;
  


  while (ros::ok()&&(!g.path_taken.empty()))                    
  {
  
	  current_time = ros::Time::now();
	  
	  int current_node_row;
	  int current_node_column;
	  
	  float current_pos_x;
	  float current_pos_y;

	  pairs p ;
	  p = g.path_taken.top();
	  g.path_taken.pop();
   
	current_node_row = p.first; 
	current_node_column = p.second; 
	//cout<<current_node_row <<", "<<   current_node_column << endl;

	current_pos_x = (current_node_column*resolution) + init_x;   
	current_pos_y = ((ROWS-current_node_row)*resolution) + init_y;

	  
    geometry_msgs::PoseStamped this_pose_stamped;

   this_pose_stamped.pose.position.x = current_pos_x ;
   this_pose_stamped.pose.position.y = current_pos_y ;
   this_pose_stamped.header.stamp = current_time;


    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);

      vis_pub.publish(path);

      i = i+1;
  
  


   ros::spinOnce();
   loop_rate.sleep();




 
}



// for (int i=0; i<ROWS; i++)
//    {
//      cout << endl;
//     for (int j=0; j<COLUMNS; j++)
//     {
       
//       cout<< g.map[i][j].cell_charc;
//     }
//   }

return 0;

}







    
