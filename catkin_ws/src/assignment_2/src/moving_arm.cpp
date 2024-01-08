// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Message containing the pose of the pickable object
#include <apriltag_ros/AprilTagDetectionArray.h>

// Gazebo attacher
#include <gazebo_ros_link_attacher/Attach.h>

// Geometric shapes to enlarge the collision objects
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

void addCollisionObject(const std::string& frame_id,
                        const std::string& object_id,
                        const geometry_msgs::Pose& pose)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(8);
    
    // COLLISION OBJECT 0 : TABLE
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "map";

    // define the primitive and its dimensions
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.00;
    collision_objects[0].primitives[0].dimensions[1] = 1.00;
    collision_objects[0].primitives[0].dimensions[2] = 1.00;

    //define the pose of the table
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 7.77;
    collision_objects[0].primitive_poses[0].position.y = -2.94; 
    collision_objects[0].primitive_poses[0].position.z = 0.30; 
    collision_objects[0].operation = collision_objects[0].ADD;

    // Create a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;

    // Set the id of the collision object
    collision_object.id = object_id;

    // Create a generic shape
    shape_msgs::SolidPrimitive primitive;

    if(std::stoi(frame_id)==2)
    {
      primitive.type = shape_msgs::SolidPrimitive::CONE;
      primitive.dimensions.resize(2);
      primitive.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = 0.05;
      primitive.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = 0.1;
    }

    if(std::stoi(frame_id)==3)
    {
      primitive.type = shape_msgs::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
      primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
      primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
    }

    if(std::stoi(frame_id)==1 || std::stoi(frame_id)>3)
    {   
      primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.05;
      primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.3;
    }

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
    // Publish the collision object
    planning_scene_interface.addCollisionObjects(collision_objects);
}

void move_gripper(const geometry_msgs::Pose& pose)
{
  moveit::planning_interface::MoveGroupInterface move_group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::Pose right_pose = pose;
  right_pose.position.z -= 0.4; // To place the gripper just above the object to grasp
  move_group.setPoseTarget(right_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    // Execute the motion
    move_group.execute(my_plan);
  }
  else 
  {ROS_WARN("Failed to plan motion!");}
}

// void attach_object(const geometry_msgs::Pose& pose, const int id)
// {
//   if (id == 1)
//   {
//     addCollisionObject(std::to_string(msg->detections[i].id[0]), "4", 0.05, 0.3, msg->detections[i].pose.pose.pose);
//   }

// }

void messageCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    for (int i = 0; i < msg->detections.size(); i++)
    {
        ROS_INFO("The object is in: [%f] [%f] [%f]", msg->detections[i].pose.pose.pose.position.x, msg->detections[i].pose.pose.pose.position.y, msg->detections[i].pose.pose.pose.position.z);
        ROS_INFO("With orientation: [%f] [%f] [%f] [%f]", msg->detections[i].pose.pose.pose.orientation.x, msg->detections[i].pose.pose.pose.orientation.y, msg->detections[i].pose.pose.pose.orientation.z, msg->detections[i].pose.pose.pose.orientation.w);

        if(msg->detections[i].id[0] == 1)
        {
          ROS_INFO("Blue hexagon");
          addCollisionObject(std::to_string(msg->detections[i].id[0]), "blue hexagon", msg->detections[i].pose.pose.pose);
          move_gripper(msg->detections[i].pose.pose.pose);
          //attach_object(pose, msg->detections[i].id[0]);
        }
        if(msg->detections[i].id[0] == 2)
        {
          ROS_INFO("Green triangle");
          addCollisionObject(std::to_string(msg->detections[i].id[0]), "green triangle", msg->detections[i].pose.pose.pose);
          move_gripper(msg->detections[i].pose.pose.pose); 
          //attach_object(pose, msg->detections[i].id[0]); 
        }
        if(msg->detections[i].id[0] == 3)
        {
          ROS_INFO("Red cube");
          addCollisionObject(std::to_string(msg->detections[i].id[0]), "red cube", msg->detections[i].pose.pose.pose);
          move_gripper(msg->detections[i].pose.pose.pose);
          //attach_object(pose, msg->detections[i].id[0]);
        }
        if(msg->detections[i].id[0] > 3)
        {
          ROS_INFO("Gold obstacle to avoid");
          addCollisionObject(std::to_string(msg->detections[i].id[0]), "gold obstacle", msg->detections[i].pose.pose.pose);
        }
    }
}

int main(int argc, char** argv)
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);
    ros::spin();
    return 0;
  /*ros::init(argc, argv, "moving_arm");


  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = atof(argv[1]);
  goal_pose.pose.position.y = atof(argv[2]);
  goal_pose.pose.position.z = atof(argv[3]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroup group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  bool success = group_arm_torso.plan(my_plan);

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm_torso.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;*/
}