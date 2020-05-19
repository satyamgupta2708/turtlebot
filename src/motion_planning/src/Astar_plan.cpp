#include <ros/ros.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>



const int rows = 194;
const int columns = 231;
const int length = rows*columns;


const float x_init = -6.9;
const float y_init = -5.9;
const float resolution = 0.05;

const int obstacle = 100;
const int free_space = 0;
const int unknown = -1;

float max()
{

 return std::numeric_limits<float>::infinity();

}
 using namespace std;

class astar{

    void data();
	void reshape(std::vector<int>);
	float heuristics(int current_cell_row,int current_cell_col);
	auto get_grid_pos(float x,float y);
    void get_destination(float x , float y);
    void get_origin(float x , float y);

public:

      
      std::size_t counter;
       
       int start_row_n;
       int start_col_n;
       int goal_row_n;
       int goal_col_n;
       
        
       struct cell
       {
       	 
       	 float move_cost_n;
         float heur_cost_n;
         float total_cost_n;

         bool  visited_n;

         int parent_row_n;
         int parent_col_n;
         

        }cell_details_n[rows][columns];

        int map_n[rows][columns];

        
      std::set< std::pair<float,std::pair <int, int>> > open_list_n;
      //std::vector<std::vector<int> > map_n ( rows , std::vector<int> (columns));
      std::vector<std::vector<char> > mapc_n;//( rows , std::vector<int>(col));
      // std::vector<std::vector<cell> > cell_details_n(rows,std::vector<cell>(columns));
      std::stack<std::pair<int,int>> path_taken;
    
      std::vector< std::pair<std::pair <int, int>,std::pair <int, int>> >closed_list_n;
     
     astar(float src_x,float src_y,float dest_x,float dest_y);
     
     
     void astar_path();
     void update_cell_details(int , int , int , int , float , float , float);
     bool isvalid(int, int);
     void path_traversed();
     void print_closed_list();



};

//------------------------------------------------------------------------------------------------------------------------------------------------

astar::astar(float src_x,float src_y,float dest_x,float dest_y)
{
   std::cout << "constructor initialising the cell_details"<< std::endl;
    data();
    get_origin(src_x, src_y);
    get_destination(dest_x, dest_y);
    
  for(int i=0 ; i< rows; i++)
  {
      //cout<<endl;
      for(int j=0 ;j< columns ;j++)
      {
        cell_details_n[i][j].move_cost_n = 0;
        cell_details_n[i][j].heur_cost_n = max();
        cell_details_n[i][j].total_cost_n = max() ;
        cell_details_n[i][j].visited_n = false ;
        cell_details_n[i][j].parent_row_n = -1;
        cell_details_n[i][j].parent_col_n = -1;

      }
  }


//------------------------------------------------------------------------------------------------------------------------------------------------

   

}

auto astar::get_grid_pos(float x,float y)
{
   std::pair <int,int> position;

   int row = (y-y_init)/resolution;     /// indicating Row 
   int col = (x-x_init)/resolution;     /// indicating Column
   
   position = make_pair((rows - row),col);      //changes to be checked 

   return position; 

}

//------------------------------------------------------------------------------------------------------------------------------------------------


void astar::get_origin(float x , float y)
  {    
        std::pair <int,int> position;

        position = get_grid_pos(x,y);

         start_row_n = position.first;
         start_col_n = position.second; 

         if (map_n[start_row_n][start_col_n] == obstacle)
         {

             std::cout<<" start position is not reachable ---mission fail "<< std::endl ;
             //exit(1);

         }

   }

//------------------------------------------------------------------------------------------------------------------------------------------------


  void astar::get_destination(float x , float y)
  {    
        std::pair <int,int> position;

        position = get_grid_pos(x,y);

         goal_row_n= position.first;
         goal_col_n = position.second; 

         if (map_n[goal_row_n][goal_col_n] == obstacle)
         {

             std::cout<< "goal position is not reachable ---mission fail " << std::endl;
             //exit(1);

         }

   }
  
//------------------------------------------------------------------------------------------------------------------------------------------------

void astar::data()
  
  {

    std::size_t count = 0;
    std::vector<int> x;

    std::ifstream myFile;
    myFile.open("/home/satyam/turtlebot/out2.csv");

    while (myFile.good() && (count < length) )
    {
      std::string y;
      getline(myFile, y);
      int num = std::stoi(y);
      x.push_back(num);
      count ++;
    }
     
    myFile.close();
    reshape(x);

  }

//------------------------------------------------------------------------------------------------------------------------------------------------

  void astar::reshape(std::vector<int> x)
{

  std::cout<<" entered reshape successfully"<<std::endl;
  int inc = 0;
 
mapc_n.resize(rows);
for (int i =rows-1; i>=0; i--)
   {
      mapc_n[i].resize(columns);
    for (int j=0; j < columns; j++)
    {
      

      if( x[inc] == unknown || x[inc]== obstacle)
      {
        map_n[i][j] = obstacle;
        mapc_n[i][j] = '*';
      }

      else if( x[inc] == free_space)
      {
        map_n[i][j] = free_space;
        mapc_n[i][j]= '0';
      }
      inc++;
    
     }
   }


  //  for (int i=0; i<rows; i++)
  //  {
  //    cout << endl;
  //   for (int j=0; j<columns; j++)
  //   {
       
  //     cout<< mapc_n[i][j];
  //   }
  // }
   cout<< "reshaped successfully"<<endl;

}

//------------------------------------------------------------------------------------------------------------------------------------------------


float  astar::heuristics(int current_cell_row,int current_cell_col)
{
  
   float heur_distance = sqrt(pow((current_cell_row-goal_row_n),2)+ pow((current_cell_col-goal_col_n),2)) ;
   // cout<< heur_distance<<endl;
   return heur_distance ;
}

//------------------------------------------------------------------------------------------------------------------------------------------------


bool astar::isvalid(int row,int col)
{


   if (row >=0 && row < rows)
   {
      if (col >=0 && col < columns)
      {
      	   if (cell_details_n[row][col].visited_n == false && map_n[row][col] == free_space)///obstacle ka condition
      	   	return (true);
      	   else
      	   	return (false);

      }
      else
           return (false);
   }

   else
        return (false);

}

//------------------------------------------------------------------------------------------------------------------------------------------------


void astar::update_cell_details(int cell_row, int cell_col, int parent_row, int parent_col, float path_cost, float heur_cost, float total_cost)
{
        cell_details_n[cell_row][cell_col].move_cost_n = path_cost;
        cell_details_n[cell_row][cell_col].heur_cost_n =  heur_cost;
        cell_details_n[cell_row][cell_col].total_cost_n = total_cost;
        cell_details_n[cell_row][cell_col].parent_row_n = parent_row;
        cell_details_n[cell_row][cell_col].parent_col_n = parent_col;

}

//------------------------------------------------------------------------------------------------------------------------------------------------



/* 
        Generating all the 8 successor of this cell 
  
            N.W   N   N.E 
              \   |   / 
               \  |  / 
            W----Cell----E 
                 / | \ 
               /   |  \ 
            S.W    S   S.E 
  
        Cell-->Popped Cell (i, j) 
        N -->  North       (i-1, j) 
        S -->  South       (i+1, j) 
        E -->  East        (i, j+1) 
        W -->  West           (i, j-1) 
        N.E--> North-East  (i-1, j+1) 
        N.W--> North-West  (i-1, j-1) 
        S.E--> South-East  (i+1, j+1) 
        S.W--> South-West  (i+1, j-1)*/


void astar::astar_path()
{

  
  cell_details_n[start_row_n][start_col_n].move_cost_n  = 0;
  cell_details_n[start_row_n][start_col_n].heur_cost_n = 0;
  cell_details_n[start_row_n][start_col_n].parent_row_n = start_row_n;
  cell_details_n[start_row_n][start_col_n].parent_col_n = start_col_n;
  cell_details_n[start_row_n][start_col_n].total_cost_n = 0;
  cell_details_n[start_row_n][start_col_n].visited_n = true;


int current_row = start_row_n;
int current_col = start_col_n;

int i = current_row;
int j = current_col;


 

while(!((i == goal_row_n) && (j == goal_col_n)))
  {  
  	
        
     if(isvalid(i-1,j)==true)
   {
         
        float heur_cost = heuristics(i-1, j);
        float path_cost = cell_details_n[i][j].move_cost_n + 1;
       // cout<< path_cost<<endl;
        float total_cost = heur_cost + path_cost;
        
   	 
       
           if(total_cost< cell_details_n[i-1][j].total_cost_n  || cell_details_n[i-1][j].total_cost_n == max())
            {

                update_cell_details(i-1, j, i, j,path_cost, heur_cost, total_cost);
                
             }
        
       open_list_n.insert(make_pair(total_cost,make_pair(i-1,j)));

     }

      

       if(isvalid(i,j-1)==true)
          {
           
           float heur_cost = heuristics(i, j-1);
           float path_cost = cell_details_n[i][j].move_cost_n + 1;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost < cell_details_n[i][j-1].total_cost_n || cell_details_n[i][j-1].total_cost_n == max())
            {

                update_cell_details(i, j-1, i, j,path_cost, heur_cost, total_cost);
                
             }
        
            open_list_n.insert(make_pair(total_cost,make_pair(i,j-1)));

          
         }



          if(isvalid(i+1,j)==true)
    {   

     
           float heur_cost = heuristics(i+1, j);
           float path_cost = cell_details_n[i][j].move_cost_n + 1;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost< cell_details_n[i+1][j].total_cost_n  || cell_details_n[i+1][j].total_cost_n == max())
            {

                update_cell_details(i+1, j, i, j,path_cost, heur_cost, total_cost);
                
             }
        
       open_list_n.insert(make_pair(total_cost,make_pair(i+1,j)));
          
     }

   // Right Direction
  



   if(isvalid(i,j+1)==true)
   {   
         
   	
           float heur_cost = heuristics(i, j+1);
           float path_cost = cell_details_n[i][j].move_cost_n + 1;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost< cell_details_n[i][j+1].total_cost_n  || cell_details_n[i][j+1].total_cost_n == max())
            {

                update_cell_details(i, j+1, i, j, path_cost, heur_cost, total_cost);
                
             }
        
       open_list_n.insert(make_pair(total_cost,make_pair(i,j+1)));
          
     }

  

       if(isvalid(i-1, j+1)==true)
        {   
       
       
           float heur_cost = heuristics(i-1, j+1);
           float path_cost = cell_details_n[i][j].move_cost_n + 1.414;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost< cell_details_n[i-1][j+1].total_cost_n  || cell_details_n[i-1][j+1].total_cost_n == max())
            {

                update_cell_details(i-1, j+1, i, j, path_cost, heur_cost, total_cost);
                
             }
        
            open_list_n.insert(make_pair(total_cost,make_pair(i-1,j+1)));

        }

  



        if(isvalid(i-1,j-1)==true)
         { 
           

           float path_cost = cell_details_n[i][j].move_cost_n + 1.414;
           float heur_cost = heuristics(i-1, j-1);
           float total_cost = heur_cost + path_cost;
        
   	// cout<< path_cost<<endl;
       
           if(total_cost < cell_details_n[i-1][j-1].total_cost_n  || cell_details_n[i-1][j-1].total_cost_n == max())
            {

                update_cell_details(i-1, j-1, i, j, path_cost, heur_cost, total_cost);
                
             }


            open_list_n.insert(make_pair(total_cost,make_pair(i-1,j-1)));
        
        }

  



      if(isvalid(i+1,j+1)==true)
       {   
      
     
           float heur_cost = heuristics(i+1, j+1);
           float path_cost = cell_details_n[i][j].move_cost_n + 1.414;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost < cell_details_n[i+1][j+1].total_cost_n  || cell_details_n[i+1][j+1].total_cost_n == max())
            {

                update_cell_details(i+1, j+1, i, j, path_cost, heur_cost, total_cost);
                
             }


            open_list_n.insert(make_pair(total_cost,make_pair(i+1,j+1)));
        
        }





       if(isvalid(i+1,j-1)==true)

         {   
       
       
           float  heur_cost = heuristics(i+1, j-1);
           float path_cost = cell_details_n[i][j].move_cost_n + 1.414;
           float total_cost = heur_cost + path_cost;
        
   	 //cout<< path_cost<<endl;
       
           if(total_cost < cell_details_n[i+1][j-1].total_cost_n  || cell_details_n[i+1][j-1].total_cost_n == max())
            {

                update_cell_details(i+1, j-1, i, j, path_cost, heur_cost, total_cost);
                
             }
          

            open_list_n.insert(make_pair(total_cost,make_pair(i+1,j-1)));
        }

  
   cell_details_n[i][j].visited_n = true;
   closed_list_n.push_back(make_pair(make_pair(i,j),make_pair(cell_details_n[i][j].parent_row_n,cell_details_n[i][j].parent_col_n)));
   
    mapc_n[i][j] = '|';

   auto current_cell = *open_list_n.begin();

   i = current_cell.second.first;
   j = current_cell.second.second;

   open_list_n.erase(open_list_n.begin());
}
 
print_closed_list();
path_traversed();

}
//------------------------------------------------------------------------------------------------------------------------------------------------
void astar::print_closed_list()
{


cout << "\nThe vector elements are: "; 
    for(int i = 0; i < closed_list_n.size(); i++) 
        cout << closed_list_n[i].first.first << " "<<closed_list_n[i].first.second<< " "<<closed_list_n[i].second.first<< " "<<closed_list_n[i].second.second<<endl; 


}
//------------------------------------------------------------------------------------------------------------------------------------------------


// void astar::path_traversed()
// {
//    int row = goal_row_n;
//    int col = goal_col_n;

//    counter = 0;
//   while(!((row == cell_details_n[row][col].parent_row_n) && (col == cell_details_n[row][col].parent_col_n)))
      
//       {

//             std::pair <int,int> x = make_pair(row,col);

//             cout << x.first << " " << x.second<<" "<< counter <<endl;
//             path_taken.push(x);

//             mapc_n[row][col] = '|';

//             row = cell_details_n[row][col].parent_row_n ;
//             col = cell_details_n[row][col].parent_col_n ;

            
//             std::cout << cell_details_n[row][col].visited_n <<" "<<cell_details_n[row][col].total_cost_n <<std::endl;
             
//              counter ++;
            
//       }

//       path_taken.push(make_pair(row,col));

//       std::cout<< "exiting path traversed"<< std::endl;

// }

//------------------------------------------------------------------------------------------------------------------------------------------------
void astar::path_traversed()
{
   int row = goal_row_n;
   int col = goal_col_n;

   counter = 0;
  while(!((row == cell_details_n[row][col].parent_row_n) && (col == cell_details_n[row][col].parent_col_n)))
      
      {

            std::pair <int,int> x = make_pair(row,col);

            cout << x.first << " " << x.second<<" "<< counter <<endl;
            path_taken.push(x);

            mapc_n[row][col] = '|';

            row = cell_details_n[row][col].parent_row_n ;
            col = cell_details_n[row][col].parent_col_n ;

            
            std::cout << cell_details_n[row][col].visited_n <<" "<<cell_details_n[row][col].total_cost_n <<std::endl;
             
             counter ++;
            
      }

      path_taken.push(make_pair(row,col));

      std::cout<< "exiting path traversed"<< std::endl;

}
//------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char **argv)
{

  ros::init(argc, argv, "plan");
  ros::NodeHandle mark;

   // float src_x = -1.7;		
   // float src_y = 0;
   // float dest_x = 1.0;
   // float dest_y = 0;

   float src_x = 1.0;		
   float src_y = 0;
   float dest_x = -1.7;
   float dest_y = 0;


   // float src_x = -5;		
   // float src_y = -5;
   // float dest_x = 2;
   // float dest_y = 3;



  astar find_path(src_x,src_y,dest_x,dest_y);
  

  cout << find_path.start_row_n <<' '<<find_path.start_col_n<< endl;
  cout << find_path.goal_row_n << ' '<<find_path.goal_col_n << endl;
  



  find_path.astar_path();




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
  


  while (ros::ok()&&(!find_path.path_taken.empty()))                    
  {
  
	  current_time = ros::Time::now();
	  
	  int current_node_row;
	  int current_node_column;
	  
	  float current_pos_x;
	  float current_pos_y;

	  pair<int,int> p ;
	  p = find_path.path_taken.top();
	 
   
	current_node_row = p.first; 
	current_node_column = p.second; 
	//cout<<current_node_row <<", "<<   current_node_column << endl;

	current_pos_x = (current_node_column*resolution) + x_init;   
	current_pos_y = ((rows-current_node_row)*resolution) + y_init;
     
     find_path.path_taken.pop();
	  
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



for (int i=0; i<rows; i++)
   {
     cout << endl;
    for (int j=0; j< columns; j++)
    {
       
      cout<< find_path.mapc_n[i][j];
    }
  }

return 0;

}