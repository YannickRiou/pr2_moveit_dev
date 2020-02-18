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

// Mesh related include
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// ActionLib related include
#include <actionlib/server/simple_action_server.h>
#include "pr2_moveit/pickAction.h"
#include "pr2_moveit/placeAction.h"
#include "pr2_moveit/moveAction.h"

// Toaster related include
#include <toaster_msgs/ObjectListStamped.h>

// Ontology related include
#include "ontologenius_query/OntologeniusQueryFullService.h"
#include "ontologenius_query/OntologeniusQueryResponse.h"

// Tf2 related include
 #include <tf2_ros/transform_listener.h>

// Various defines
#define ONTOLOGY_NS "robot"

class PR2manip
{
public:
  explicit PR2manip(ros::NodeHandle nh);
  ~PR2manip();

  bool gripper_open(std::string gripper);
  bool gripper_close(std::string gripper, float effort);

  void move_head(std::string frame_id, double x, double y, double z);

  void pickplace(moveit::planning_interface::MoveGroupInterface& move_group);

  moveit::planning_interface::MoveGroupInterface& getMoveGroupInterface(std::string interface);

  bool moveTo(const geometry_msgs::Pose& pose, std::string interface);

  void openGripper(trajectory_msgs::JointTrajectory& posture, std::string side);
  void closedGripper(trajectory_msgs::JointTrajectory& posture, std::string side);

  void pickObj(const pr2_moveit::pickGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_moveit::pickAction>* pickServer);
  void placeObj(const pr2_moveit::placeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_moveit::placeAction>* placeServer);
  void move(const pr2_moveit::moveGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_moveit::moveAction>* moveServer);

  std::vector<moveit_msgs::CollisionObject>& getCollisionObjVector();

  void objCallback (const toaster_msgs::ObjectListStamped::ConstPtr& msg);

  bool moveTorso(float position);

  void setToasterSub(ros::Subscriber sub);
  void setOntoClient(ros::ServiceClient srv);

private:
  ros::NodeHandle nh_;

  // Subscriber to toaster to get objects in the scene
  ros::Subscriber toaster_sub_;

  // Client to ask ontology about mesh and poses of objects in the scene
  ros::ServiceClient ontoClient_;

  // TF Listener to do transforms between two frame
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
  const std::string LEFT_ARM_PLANNING_GROUP = "left_arm";
  const std::string RIGHT_GRIPPER_PLANNING_GROUP = "right_gripper";
  const std::string LEFT_GRIPPER_PLANNING_GROUP = "left_gripper";

  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* right_arm_joint_model_group_;
  const robot_state::JointModelGroup* left_arm_joint_model_group_;
  const robot_state::JointModelGroup* right_gripper_joint_model_group_;
  const robot_state::JointModelGroup* left_gripper_joint_model_group_;

  // Our Action interface type, provided as a typedef for convenience
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

  moveit::planning_interface::MoveGroupInterface right_arm_move_group_;
  moveit::planning_interface::MoveGroupInterface left_arm_move_group_;
  moveit::planning_interface::MoveGroupInterface right_gripper_move_group_;
  moveit::planning_interface::MoveGroupInterface left_gripper_move_group_;


  // Our Action interface type, provided as a typedef for convenience
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

  GripperClient* left_gripper_client_;
  GripperClient* right_gripper_client_;

  PointHeadClient* point_head_client_;

  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

  TorsoClient* torso_client_;

  // Store the collision objects added to the scene
  std::vector<moveit_msgs::CollisionObject> collision_objects_vector_;

  // Remember object that was pick and also with which arm
  std::string objPickedId_;
  std::string armUsed_;

  typedef actionlib::SimpleActionServer<pr2_moveit::pickAction> pickActionServer;
  pr2_moveit::pickFeedback pickFeedback_;
  pr2_moveit::pickResult pickResult_;

  typedef actionlib::SimpleActionServer<pr2_moveit::placeAction> placeActionServer;
  pr2_moveit::placeFeedback placeFeedback_;
  pr2_moveit::placeResult placeResult_;

  typedef actionlib::SimpleActionServer<pr2_moveit::moveAction> moveActionServer;
  pr2_moveit::moveFeedback moveFeedback_;
  pr2_moveit::moveResult moveResult_;

};


#endif  // PR2_RS_TEST_H
