#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <map>
#include <stdlib.h>

const double tau = 2 * M_PI;
const double L = 0.7, W=0.4, H=0.54;
/*
void firstConfiguration(std::vector<std::map<int,double>>& obstacles){
    std::map<int,double> map_obstacle;
    map_obstacle.insert(std::pair<int, double>(1, 5));
    map_obstacle.insert(std::pair<int, double>(2, -5));
    map_obstacle.insert(std::pair<int, double>(3, 5));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, -7));
    map_obstacle.insert(std::pair<int, double>(2, 5));
    map_obstacle.insert(std::pair<int, double>(3, 10));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, 6));
    map_obstacle.insert(std::pair<int, double>(2, -4));
    map_obstacle.insert(std::pair<int, double>(3, 7));
    obstacles.push_back(map_obstacle);
}
void secondConfiguration(std::vector<std::map<int,double>>& obstacles){
    std::map<int,double> map_obstacle;
    map_obstacle.insert(std::pair<int, double>(1, 5));
    map_obstacle.insert(std::pair<int, double>(2, -5));
    map_obstacle.insert(std::pair<int, double>(3, 5));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, -7));
    map_obstacle.insert(std::pair<int, double>(2, 5));
    map_obstacle.insert(std::pair<int, double>(3, 10));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, 6));
    map_obstacle.insert(std::pair<int, double>(2, -4));
    map_obstacle.insert(std::pair<int, double>(3, 7));
    obstacles.push_back(map_obstacle);
}
void thirdConfiguration(std::vector<std::map<int,double>>& obstacles){
    std::map<int,double> map_obstacle;
    map_obstacle.insert(std::pair<int, double>(1, 5));
    map_obstacle.insert(std::pair<int, double>(2, -5));
    map_obstacle.insert(std::pair<int, double>(3, 5));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, -7));
    map_obstacle.insert(std::pair<int, double>(2, 5));
    map_obstacle.insert(std::pair<int, double>(3, 10));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, 6));
    map_obstacle.insert(std::pair<int, double>(2, -4));
    map_obstacle.insert(std::pair<int, double>(3, 7));
    obstacles.push_back(map_obstacle);
}
void fourthConfiguration(std::vector<std::map<int,double>>& obstacles){
    std::map<int,double> map_obstacle;
    map_obstacle.insert(std::pair<int, double>(1, 5));
    map_obstacle.insert(std::pair<int, double>(2, -5));
    map_obstacle.insert(std::pair<int, double>(3, 5));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, -7));
    map_obstacle.insert(std::pair<int, double>(2, 5));
    map_obstacle.insert(std::pair<int, double>(3, 10));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, 6));
    map_obstacle.insert(std::pair<int, double>(2, -4));
    map_obstacle.insert(std::pair<int, double>(3, 7));
    obstacles.push_back(map_obstacle);
}*/
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    double numberObstacles;
    int n;
    std::vector<std::map<int,double>> obstacles;
    /*std::cout << "Que configuracion desea: 1 - 2 - 3 - 4: ";
    std::string inputString;
    std::getline(std::cin, inputString);
    if(inputString == "1"){
        std::cout << "gg";
    }
    if(inputString == "2"){

    }
    if(inputString == "3"){

    } 
    if(inputString == "4"){

    }*/
    std::map<int,double> map_obstacle;
    map_obstacle.insert(std::pair<int, double>(1, 5));
    map_obstacle.insert(std::pair<int, double>(2, -5));
    map_obstacle.insert(std::pair<int, double>(3, 5));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, -7));
    map_obstacle.insert(std::pair<int, double>(2, 5));
    map_obstacle.insert(std::pair<int, double>(3, 10));
    obstacles.push_back(map_obstacle);
    map_obstacle.clear();
    map_obstacle.insert(std::pair<int, double>(1, 6));
    map_obstacle.insert(std::pair<int, double>(2, -4));
    map_obstacle.insert(std::pair<int, double>(3, 7));
    obstacles.push_back(map_obstacle);
    numberObstacles = obstacles.size();
    //numberObstacles = 4
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(numberObstacles+1);
    for (int i = 0;i <= numberObstacles; i++){
        collision_objects[i].id = "object" + std::to_string(i);
        collision_objects[i].header.frame_id = "base";

        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.015 + (rand()%10)/1000;//0.015 - 0.025
        collision_objects[i].primitives[0].dimensions[1] = 0.015 + (rand()%10)/1000;//0.015 - 0.025
        collision_objects[i].primitives[0].dimensions[2] = 0.15 + (rand()%5)/1000; //0.15 - 0.2

        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = 0 + obstacles[i][1]/100;
        collision_objects[i].primitive_poses[0].position.y = -W/2 + obstacles[i][2]/100;
        collision_objects[i].primitive_poses[0].position.z = H/2 + obstacles[i][3]/100;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        collision_objects[i].operation = collision_objects[0].ADD;
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);
}
int main(int argc, char** argv){
  static int flag = 1;
  static const std::string NAME_NODE = "thor_arm_generate_obstacles";
  
  ros::init(argc, argv, NAME_NODE);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "thor_arm";
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  group.setPlanningTime(45.0);
  addCollisionObjects(planning_scene_interface);
  /*while(flag == 1){
    obstacles.clear();
    addCollisionObjects(planning_scene_interface,obstacles);
    std::cout << "Desea continuar con otra configuracion? 1:SI 2:NO --> ";
    std::cin >> flag;
    std::vector<std::string> object_ids;
    object_ids.push_back("object1");
    object_ids.push_back("object2");
    object_ids.push_back("object3");
    planning_scene_interface.removeCollisionObjects(object_ids);
  }*/
  ros::waitForShutdown();
  return 0;
}