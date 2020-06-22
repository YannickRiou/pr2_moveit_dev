#ifndef PR2_RS_TEST_H
#define PR2_RS_TEST_H

#include <iostream>
#include <map>
#include <mutex>
#include <string>

#include <signal.h>

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

// Move it related include
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Gripper related include
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Head related include
#include <pr2_controllers_msgs/PointHeadAction.h>

// Torso related include
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

class PR2RsTest
{
public:
  explicit PR2RsTest(ros::NodeHandle nh);
  ~PR2RsTest();

  void clickCallback(const geometry_msgs::Pose& pose);
  void markerCallback(const visualization_msgs::MarkerConstPtr& marker, moveit::planning_interface::PlanningSceneInterface& plan_scene,std::vector<moveit_msgs::CollisionObject>& collision_object_vector);

  bool gripper_open(std::string gripper);
  bool gripper_close(std::string gripper, float effort);

  void move_head(std::string frame_id, double x, double y, double z);

  void pickplace(moveit::planning_interface::MoveGroupInterface& move_group,std::vector<moveit_msgs::CollisionObject>& collision_object_vector);

  moveit::planning_interface::MoveGroupInterface& getMoveGroupInterface(std::string interface);

  bool moveTo(const geometry_msgs::Pose& pose, std::string interface);

  void openGripper(trajectory_msgs::JointTrajectory& posture,std::string armUsed);
  void closedGripper(trajectory_msgs::JointTrajectory& posture,std::string armUsed);

  bool moveTorso(float position);

private:
  ros::NodeHandle nh_;

  const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
  const std::string LEFT_ARM_PLANNING_GROUP = "left_arm";
  const std::string RIGHT_GRIPPER_PLANNING_GROUP = "right_gripper";
  const std::string LEFT_GRIPPER_PLANNING_GROUP = "left_gripper";
  const std::string HEAD_PLANNING_GROUP = "head";

  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* right_arm_joint_model_group;
  const robot_state::JointModelGroup* left_arm_joint_model_group;
  const robot_state::JointModelGroup* right_gripper_joint_model_group;
  const robot_state::JointModelGroup* left_gripper_joint_model_group;

  const robot_state::JointModelGroup* head_joint_model_group;

  // Our Action interface type, provided as a typedef for convenience
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

  moveit::planning_interface::MoveGroupInterface right_arm_move_group;
  moveit::planning_interface::MoveGroupInterface left_arm_move_group;
  moveit::planning_interface::MoveGroupInterface right_gripper_move_group;
  moveit::planning_interface::MoveGroupInterface left_gripper_move_group;

  moveit::planning_interface::MoveGroupInterface head_move_group;


  // Our Action interface type, provided as a typedef for convenience
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

  GripperClient* left_gripper_client_;
  GripperClient* right_gripper_client_;

  PointHeadClient* point_head_client_;

  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

  TorsoClient* torso_client_;
};


#endif  // PR2_RS_TEST_H
