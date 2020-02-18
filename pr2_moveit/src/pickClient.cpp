#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "pr2_moveit/pickAction.h"
#include "pr2_moveit/placeAction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pickclient");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  actionlib::SimpleActionClient<pr2_moveit::pickAction> pick("/pr2_manip/pick", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  pick.waitForServer(); //will wait for infinite time

  ROS_INFO("PICK Action server started, sending goal.");
  // send a goal to the action
  pr2_moveit::pickGoal pickGoal;
  pickGoal.id = "milk_box_0";
  pick.sendGoal(pickGoal);

  //wait for the action to return
  bool finished_before_timeout_pick = pick.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout_pick)
  {
    actionlib::SimpleClientGoalState state_pick = pick.getState();
    ROS_INFO("PICK Action finished: %s",state_pick.toString().c_str());
  }
  else
    ROS_INFO("PICK Action did not finish before the time out.");


  actionlib::SimpleActionClient<pr2_moveit::placeAction> place("/pr2_manip/place", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  place.waitForServer(); //will wait for infinite time

  ROS_INFO("PLACE Action server started, sending goal.");
  // send a goal to the action
  pr2_moveit::placeGoal placeGoal;
  placeGoal.posePlace.position.x = 0.70;
  placeGoal.posePlace.position.y = 0.20;
  placeGoal.posePlace.position.z = 0.80;
  placeGoal.posePlace.orientation.x = 0.0;
  placeGoal.posePlace.orientation.y = 0.0;
  placeGoal.posePlace.orientation.z = 0.707;
  placeGoal.posePlace.orientation.w = 0.707;
  place.sendGoal(placeGoal);

  //wait for the action to return
  bool finished_before_timeout_place = place.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout_place)
  {
    actionlib::SimpleClientGoalState state_place = place.getState();
    ROS_INFO("PLACE Action finished: %s",state_place.toString().c_str());
  }
  else
    ROS_INFO("PLACE Action did not finish before the time out.");


  ros::waitForShutdown();
  return 0;
}
