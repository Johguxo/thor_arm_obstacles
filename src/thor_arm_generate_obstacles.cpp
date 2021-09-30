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

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,double* obstacles){
    int numberObstacles = 3;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(numberObstacles);
    for (int i = 0;i <= numberObstacles-1; i++){
        collision_objects[i].id = "object" + std::to_string(i+1);
        collision_objects[i].header.frame_id = "base";

        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.015 + (rand()%30)/1000.0;//0.015 - 0.025
        collision_objects[i].primitives[0].dimensions[1] = 0.015 + (rand()%30)/1000.0;//0.015 - 0.025
        collision_objects[i].primitives[0].dimensions[2] = 0.15 + (rand()%10)/1000.0; //0.15 - 0.2

        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = 0 + obstacles[3*i]/100.0;
        collision_objects[i].primitive_poses[0].position.y = -W/2 + obstacles[3*i + 1]/100.0;
        collision_objects[i].primitive_poses[0].position.z = H/2 + obstacles[3*i + 2]/100.0;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;
        collision_objects[i].operation = collision_objects[0].ADD;
    }
  planning_scene_interface.applyCollisionObjects(collision_objects);
}
void readFile(moveit_visual_tools::MoveItVisualTools& visual_tools){
  std::fstream myReadFile;
  myReadFile.open("/home/johann/ws_moveit/src/thor_arm_pick_place/src/text.txt", std::ios::in);
  if (!myReadFile) {
		std::cout << "No such file";
    visual_tools.prompt("nel");
	}
	else {
		char ch;
    double pos_x, pos_y, pos_z;
    visual_tools.prompt("si");
		while (myReadFile >> pos_x >> pos_y>> pos_z) {
			
      //if (myReadFile.eof())
			//	break;

			std::cout << pos_x << " " << pos_y << " "<< pos_z << std::endl;
     
		}
 
	}
  visual_tools.prompt("GA");
	myReadFile.close();
}
int main(int argc, char *argv[]){
  static const std::string NAME_NODE = "thor_arm_obstacle_generation";
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
  std::vector<std::string> object_ids;
  object_ids.push_back("object1");
  object_ids.push_back("object2");
  object_ids.push_back("object3");
  double obstacles [] = {5,-5,5,-7,5,3,10,-4,7};
  addCollisionObjects(planning_scene_interface,obstacles);
  visual_tools.prompt("Presione 'next' en el RvizVisualToolsGui para seguir");
  planning_scene_interface.removeCollisionObjects(object_ids);


  //readFile(visual_tools);

  obstacles[0] = 5;
  obstacles[1] = -4;
  obstacles[2] = 3;

  obstacles[3] = -7.2;
  obstacles[4] = 5.2;
  obstacles[5] = 2.5;
  
  obstacles[6] = 10.5;
  obstacles[7] = -4.6;
  obstacles[8] = 6;

  addCollisionObjects(planning_scene_interface,obstacles);
  visual_tools.prompt("Presione 'next' en el RvizVisualToolsGui para seguir");
  planning_scene_interface.removeCollisionObjects(object_ids);
  obstacles[0] = 5.2;
  obstacles[1] = -4.6;
  obstacles[2] = 3.6;

  obstacles[3] = -7;
  obstacles[4] = 5;
  obstacles[5] = 2.5;
  
  obstacles[6] = 9;
  obstacles[7] = -5;
  obstacles[8] = 6;

  addCollisionObjects(planning_scene_interface,obstacles);
  visual_tools.prompt("Presione 'next' en el RvizVisualToolsGui para seguir");
  planning_scene_interface.removeCollisionObjects(object_ids);
  readFile(visual_tools);
  //addCollisionObjects(planning_scene_interface);
  ros::waitForShutdown();
  return 0;
}
