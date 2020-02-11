
#include <pr2_rs_test/pr2_rs_test.h>

// Class constructor
PR2RsTest::PR2RsTest(ros::NodeHandle nh)
  : nh_(nh),
    right_arm_move_group(RIGHT_ARM_PLANNING_GROUP),
    left_arm_move_group(LEFT_ARM_PLANNING_GROUP),
    right_gripper_move_group(RIGHT_GRIPPER_PLANNING_GROUP),
    left_gripper_move_group(LEFT_GRIPPER_PLANNING_GROUP)
{
  
  // Pointer to JointModelGroup for improved performance.
  right_arm_joint_model_group =
    right_arm_move_group.getCurrentState()->getJointModelGroup(RIGHT_ARM_PLANNING_GROUP);
  left_arm_joint_model_group =
    left_arm_move_group.getCurrentState()->getJointModelGroup(LEFT_ARM_PLANNING_GROUP);
  right_gripper_joint_model_group =
    right_gripper_move_group.getCurrentState()->getJointModelGroup(RIGHT_GRIPPER_PLANNING_GROUP);
  left_gripper_joint_model_group =
    left_gripper_move_group.getCurrentState()->getJointModelGroup(LEFT_GRIPPER_PLANNING_GROUP);



  //***** Head controller *****//
  //Initialize the client for the Action interface to the head controller
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

  //wait for head controller action server to come up 
  while(!point_head_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }

  //***** Grippers controllers *****//
  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  left_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
  right_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true); 

  //wait for the left gripper action server to come up 
  while(!left_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
  }
    
  //wait for the right gripper action server to come up 
  while(!right_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }

  //***** Torso controller *****//

  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

  //wait for the action server to come up
  while(!torso_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the torso action server to come up");
  }

  // Set Arm to initial pose
  right_arm_move_group.setNamedTarget("RIGHT_ARM_INITIAL_POSE");
  left_arm_move_group.setNamedTarget("LEFT_ARM_INITIAL_POSE");

  // Move the arms to initial pose (away from robot vision)
  right_arm_move_group.move();
  left_arm_move_group.move();
}

// Class destructor
PR2RsTest::~PR2RsTest()
{
  delete left_gripper_client_;
  delete right_gripper_client_;
  delete point_head_client_;
  delete right_arm_joint_model_group;
  delete left_arm_joint_model_group;
  delete right_gripper_joint_model_group;
  delete left_gripper_joint_model_group;
  delete torso_client_;
}

bool PR2RsTest::moveTorso(float position)
{
  bool success;

  pr2_controllers_msgs::SingleJointPositionGoal move;
  move.position = 0.1;  //all the way up is 0.2
  move.min_duration = ros::Duration(2.0);
  move.max_velocity = 1.0;
  
  torso_client_->sendGoal(move);
  torso_client_->waitForResult();

  if(torso_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Torso command succeeded");
    success = true;
  }
  else
  {
    ROS_ERROR("Torso command failed.");
    success = false;
  }
  return success;
}

//Open the gripper
bool PR2RsTest::gripper_open(std::string gripper)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  bool success;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)
  
  if (gripper == "left_gripper")
  {
    ROS_INFO("Sending open goal");
    left_gripper_client_->sendGoal(open);
    left_gripper_client_->waitForResult();

    if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Left gripper opened!");
      success = true;
    }
    else
    {
      ROS_ERROR("Left gripper failed to open.");
      success = false;
    }
  }
  else if(gripper == "right_gripper")
  {
    right_gripper_client_->sendGoal(open);
    right_gripper_client_->waitForResult();

    if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Right gripper opened!");
      success = true;
    }
    else
    {
      ROS_ERROR("Right gripper failed to open.");
      success = false;
    }
  }

  return success;
}

//Close the gripper
bool PR2RsTest::gripper_close(std::string gripper, float effort)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;

  bool success;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = effort;  // Close gently
  
  if (gripper == "left_gripper")
  {
    ROS_INFO("Sending squeeze goal");
    left_gripper_client_->sendGoal(squeeze);
    left_gripper_client_->waitForResult();
    if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Left gripper closed!");
      success = true;
    }
    else
    {
      ROS_ERROR("Left gripper failed to close.");
       success = false;
    }
  } 
  else if (gripper == "right_gripper")
  {
    right_gripper_client_->sendGoal(squeeze);
    right_gripper_client_->waitForResult();
    if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Right gripper closed!");
       success = true;
    }
    else
    {
      ROS_ERROR("Right gripper failed to close.");
      success = false;
    }
  }
  return success;
}

void PR2RsTest::move_head(std::string frame_id, double x, double y, double z)
{
  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; 
  point.point.y = y; 
  point.point.z = z;
  goal.target = point;

  //we are pointing the high-def camera frame 
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "high_def_frame";
  goal.pointing_axis.x = 1;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);

  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  //send the goal
  point_head_client_->sendGoal(goal);

  //wait for it to get there (abort after 2 secs to prevent getting stuck)
  point_head_client_->waitForResult(ros::Duration(2));
}

void PR2RsTest::clickCallback(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose target_pose;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  target_pose.orientation.x = pose.orientation.x;
  target_pose.orientation.y= pose.orientation.y;
  target_pose.orientation.z = pose.orientation.z;
  target_pose.orientation.w = pose.orientation.w;
  target_pose.position.x = pose.position.x-0.3;
  target_pose.position.y = pose.position.x;
  target_pose.position.z = pose.position.x;
  right_arm_move_group.setPoseTarget(target_pose);

  right_arm_move_group.setPoseTarget(target_pose);
  right_arm_move_group.plan(my_plan);
  right_arm_move_group.move();
 
}

bool PR2RsTest::moveTo(const geometry_msgs::Pose& pose, std::string interface)
{
  geometry_msgs::Pose target_pose;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  target_pose.orientation.x = pose.orientation.x;
  target_pose.orientation.y= pose.orientation.y;
  target_pose.orientation.z = pose.orientation.z;
  target_pose.orientation.w = pose.orientation.w;
  target_pose.position.x = pose.position.x;
  target_pose.position.y = pose.position.y;
  target_pose.position.z = pose.position.z;

  if(interface == "right_arm_move_group")
  {
    right_arm_move_group.setPoseTarget(target_pose);
    success = (right_arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    right_arm_move_group.move();
  }
  else
  {
    left_arm_move_group.setPoseTarget(target_pose);
    success = (left_arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    left_arm_move_group.move();
  }
  return success;  
}


void PR2RsTest::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(6);
  posture.joint_names[0] = "l_gripper_joint";
  posture.joint_names[1] = "l_gripper_motor_screw_joint";
  posture.joint_names[2] = "l_gripper_l_finger_joint";
  posture.joint_names[3] = "l_gripper_r_finger_joint";
  posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.088;
  posture.points[0].positions[1] = 1;
  posture.points[0].positions[2] = 0.477;
  posture.points[0].positions[3] = 0.477;
  posture.points[0].positions[4] = 0.477;
  posture.points[0].positions[5] = 0.477;
  posture.points[0].time_from_start = ros::Duration(20);
}

void PR2RsTest::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(6);
  posture.joint_names[0] = "l_gripper_joint";
  posture.joint_names[1] = "l_gripper_motor_screw_joint";
  posture.joint_names[2] = "l_gripper_l_finger_joint";
  posture.joint_names[3] = "l_gripper_r_finger_joint";
  posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].positions[2] = 0.002;
  posture.points[0].positions[3] = 0.002;
  posture.points[0].positions[4] = 0.002;
  posture.points[0].positions[5] = 0.002;
  posture.points[0].time_from_start = ros::Duration(20);
}

void PR2RsTest::pickplace(moveit::planning_interface::MoveGroupInterface& move_group,std::vector<moveit_msgs::CollisionObject>& collision_object_vector)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(2);
  geometry_msgs::Pose target_pose;
  shape_msgs::SolidPrimitive target_dimension;
  std::string id;

  for(auto obj : collision_object_vector)
  {
    if(obj.id.find("obj_2") != std::string::npos)
    {
      target_pose = obj.primitive_poses[0];
      target_dimension = obj.primitives[0];
      id = obj.id; 
    }
  }

  std::cout << "***** Obj.id is :" << id << std::endl;

  /*********  PICK **********/

  // Setting grasp pose
  // ++++++++++++++++++++++
  grasps[0].grasp_pose.header.frame_id = "base_footprint";
  grasps[0].grasp_pose.pose.orientation.x = 0;
  grasps[0].grasp_pose.pose.orientation.y = 0;
  grasps[0].grasp_pose.pose.orientation.z = 0.2;
  grasps[0].grasp_pose.pose.orientation.w = 0;
  grasps[0].grasp_pose.pose.position.x = target_pose.position.x-(target_dimension.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/2.0)-0.15;
  grasps[0].grasp_pose.pose.position.y = target_pose.position.y;
  grasps[0].grasp_pose.pose.position.z = target_pose.position.z+(target_dimension.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/2.0)+0.05;

  std::cout << "Object position is :" << std::endl;
  std::cout << "X :" <<  target_pose.position.x << std::endl;
  std::cout << "Y :" <<  target_pose.position.y << std::endl;
  std::cout << "Z :" <<  target_pose.position.z << std::endl;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  // Defined with respect to frame_id 
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
  // Direction is set as positive x axis 
  grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.10;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  // Defined with respect to frame_id 
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
  // Direction is set as positive z axis 
  grasps[0].post_grasp_retreat.direction.vector.z = 1;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("tableLaas");

  // Call pick to pick up the object using the grasps given
  if(move_group.pick(id, grasps))
  {
    /*********  PLACE **********/

    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = "base_footprint";

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = target_pose.position.x-(target_dimension.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/2.0)-0.20;
    place_location[0].place_pose.pose.position.y = target_pose.position.y + 0.5;
    place_location[0].place_pose.pose.position.z = target_pose.position.z+(target_dimension.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/2.0)+0.05;


    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.03;
    place_location[0].pre_place_approach.desired_distance = 0.27;

    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);  

    // Call pick to pick up the object using the grasps given
    move_group.place(id, place_location);
  }
}

void PR2RsTest::markerCallback(const visualization_msgs::MarkerConstPtr& marker, moveit::planning_interface::PlanningSceneInterface& plan_scene,std::vector<moveit_msgs::CollisionObject>& collision_object_vector)
{
  // Add table as a fixed collision object. 
  moveit_msgs::CollisionObject collisionObj;
  collisionObj.id = marker->text;

  collisionObj.header.frame_id = "base_footprint";
  shape_msgs::SolidPrimitive collisionObj_primitive;

  // Check the type of the marker. We search for CUBE and CYCLINDER only
  if(marker->type == visualization_msgs::Marker::CUBE)
  {
    collisionObj_primitive.type = collisionObj_primitive.BOX;
    collisionObj_primitive.dimensions.resize(3);
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = marker->scale.x;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = marker->scale.y;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker->scale.z;
  }
  else if(marker->type == visualization_msgs::Marker::CYLINDER)
  {
    collisionObj_primitive.type = collisionObj_primitive.CYLINDER;
    collisionObj_primitive.dimensions.resize(2);
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = marker->scale.x/2.0;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = marker->scale.z; 


    std::cout << "Object Size is :" << std::endl;
    std::cout << "Radius :" << marker->scale.x/2.0 << std::endl;
    std::cout << "Height :" <<  marker->scale.z << std::endl;
  }
  else 
  {
    return;
  }

  // Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose collisionObj_pose;
  collisionObj_pose.position.x = marker->pose.position.x;
  collisionObj_pose.position.y = marker->pose.position.y;
  collisionObj_pose.position.z = marker->pose.position.z;

  collisionObj_pose.orientation.x = marker->pose.orientation.x;
  collisionObj_pose.orientation.y = marker->pose.orientation.y;
  collisionObj_pose.orientation.z = marker->pose.orientation.z;

  collisionObj.primitives.push_back(collisionObj_primitive);
  collisionObj.primitive_poses.push_back(collisionObj_pose);
  collisionObj.operation = collisionObj.ADD;

  collision_object_vector.push_back(collisionObj);

  // Now, let's add the collision object into the world
  // Little sleep necessary before adding it
  ros::Duration(0.2).sleep();
  // Add the remaining collision object 
  plan_scene.addCollisionObjects(collision_object_vector);
}

moveit::planning_interface::MoveGroupInterface& PR2RsTest::getMoveGroupInterface(std::string interface)
{
  if(interface == "right_arm_move_group")
  {
    return right_arm_move_group;
  }
  else if (interface == "left_arm_move_group")
  {
    return left_arm_move_group;
  }

}

void mySigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pr2_rs_test");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  // Store the planning scene where the collision object will be add
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Store the collision objects added to the scene
  std::vector<moveit_msgs::CollisionObject> collision_objects_vector;

  PR2RsTest pr2_rs(n);

  // Init pose
  pr2_rs.move_head("base_footprint",1.2,-0.2,0.36);
  pr2_rs.moveTorso(0.1);
  
  pr2_rs.gripper_close("left_gripper",25);
  pr2_rs.gripper_close("right_gripper",25);   

  // Wait for robosherlock to detect the object after moving the head
  ros::Duration(2).sleep(); 
  
  // Susbscribe to topic given by Rviz when "Publish Point" tool is used on an object
  ros::Subscriber click_sub = n.subscribe("/clicked_object_pose", 1000, &PR2RsTest::clickCallback,&pr2_rs);

  // Subscribe to topic giving the marker (RoboSherlock) to show the object as boxes in Rviz
  ros::Subscriber marker_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(&PR2RsTest::markerCallback,&pr2_rs,_1,boost::ref(planning_scene_interface),boost::ref(collision_objects_vector)));
     
  // Add table as a fixed collision object. 
  moveit_msgs::CollisionObject mainTable;
  mainTable.id = "tableLaas";
  mainTable.header.frame_id = "base_footprint";
  
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.85;
  primitive.dimensions[1] = 1.35;
  primitive.dimensions[2] = 0.87;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.position.x = 1.2;
  table_pose.position.y = 0;
  table_pose.position.z = 0.36;

  mainTable.primitives.push_back(primitive);
  mainTable.primitive_poses.push_back(table_pose);
  mainTable.operation = mainTable.ADD;

  collision_objects_vector.push_back(mainTable);

  // Now, let's add the collision object into the world
  ROS_INFO("Added tableLaas into the world");
  ros::Duration(0.5).sleep();
  planning_scene_interface.addCollisionObjects(collision_objects_vector);

  std::cout << "**** Trying Pick" << std::endl;
  ros::Duration(2).sleep();
  marker_sub.shutdown();
  pr2_rs.pickplace(pr2_rs.getMoveGroupInterface("left_arm_move_group"),collision_objects_vector);

  ros::waitForShutdown();
  return 0;
}
