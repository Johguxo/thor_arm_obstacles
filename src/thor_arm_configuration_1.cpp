#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <map>
#include<iostream>
#include<fstream>

const double tau = 2 * M_PI;
const double L = 0.7, W=0.4, H=0.54;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,std::vector<double> obstacles,int numberOfObstacles){
    //int numberAxis = 3;
    //int numberObstacles = numberOfObstacles/3;
    int numberObstacles = numberOfObstacles;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(numberObstacles);
    for (int i = 0;i <= numberObstacles-1; i++){
        collision_objects[i].id = "object" + std::to_string(i+1);
        collision_objects[i].header.frame_id = "base";

        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.05 + (rand()%30)/1000.0;//0.02 - 0.025
        collision_objects[i].primitives[0].dimensions[1] = 0.05 + (rand()%30)/1000.0;//0.02 - 0.025
        collision_objects[i].primitives[0].dimensions[2] = 0.4 + (rand()%10)/1000.0; //0.4 - 0.2

        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = obstacles[3*i];
        collision_objects[i].primitive_poses[0].position.y = obstacles[3*i + 1];
        collision_objects[i].primitive_poses[0].position.z = obstacles[3*i + 2];
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;
        collision_objects[i].operation = collision_objects[0].ADD;
    }
  planning_scene_interface.applyCollisionObjects(collision_objects);
}
std::vector< int > readFile(moveit_visual_tools::MoveItVisualTools& visual_tools){
  std::vector< int > positionsObstacles;
  std::fstream myReadFile;
  myReadFile.open("/home/johann/ws_moveit/src/thor_arm_pick_place/src/bottles_position.txt", std::ios::in);
  if (!myReadFile) {
		std::cout << "No such file";
    //visual_tools.prompt("nel");
	}
	else {
		char ch;
    double pos_x, pos_y, pos_z;
    //visual_tools.prompt("si");
		while (myReadFile >> pos_x >> pos_y>> pos_z) {
			
      //if (myReadFile.eof())
			//	break;

			std::cout << pos_x << " " << pos_y << " "<< pos_z << std::endl;
      positionsObstacles.push_back(pos_x);
      positionsObstacles.push_back(pos_y);
      positionsObstacles.push_back(pos_z);
		}
 
  }
  return positionsObstacles;
  //visual_tools.prompt("GA");
	myReadFile.close();
}
int main(int argc, char** argv){
  static const std::string NAME_NODE = "thor_arm_configuration_one";
  ros::init(argc, argv, NAME_NODE);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "thor_arm";
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  group.setPlanningTime(45.0);
  visual_tools.prompt("Presione 'next' en el RvizVisualToolsGui para empezar");
  std::vector< double > obstacles;
  std::vector<std::string> object_ids;
  std::string name;
  while(1){
    //Algorithm of re-scale the range of workspace of arm robotic
    double long_x = 69, long_y = 51, long_z = 15;
    double long_x_simu = 0.7;
    double long_y_simu = long_y*long_x_simu/long_x;
    double long_z_simu = long_z*long_x_simu/long_x;
    //std::cout << long_y_simu;
    //double obstacles [] = {0.5,-0.5,0.5,-0.7,0.5,0.3,0.1,-0.4,0.7};
    obstacles.clear();
    object_ids.clear();
    //int numberOfObstacles = sizeof(obstacles)/sizeof(obstacles[0]);
    std::vector< int > paramsAxis = readFile(visual_tools);
    int numberOfObstacles = (paramsAxis.size() + 1) / 3;
    for (int i = 0 ; i < paramsAxis.size() ; i++){
      obstacles.push_back(paramsAxis[i]*long_x_simu/long_x);
      //std::cout << "Param: " << i << std::endl;
    }
    for (int i = 0;i <= numberOfObstacles-1; i++){
      name = "object" + std::to_string(i+1);
      object_ids.push_back(name);
    }
    addCollisionObjects(planning_scene_interface,obstacles,numberOfObstacles);
    visual_tools.prompt("Presione 'next' en el RvizVisualToolsGui para seguir");
    planning_scene_interface.removeCollisionObjects(object_ids);
  }
  ros::waitForShutdown();
  return 0;
}
