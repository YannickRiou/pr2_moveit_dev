#include <pr2_manip/pr2_manip.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Class constructor
PR2manip::PR2manip(ros::NodeHandle nh)
  : nh_(nh),
    right_arm_move_group_(RIGHT_ARM_PLANNING_GROUP),
    left_arm_move_group_(LEFT_ARM_PLANNING_GROUP),
    right_gripper_move_group_(RIGHT_GRIPPER_PLANNING_GROUP),
    left_gripper_move_group_(LEFT_GRIPPER_PLANNING_GROUP),
    tfListener_(tfBuffer_)
{

  // Pointer to JointModelGroup for improved performance.
  right_arm_joint_model_group_ =
    right_arm_move_group_.getCurrentState()->getJointModelGroup(RIGHT_ARM_PLANNING_GROUP);
  left_arm_joint_model_group_ =
    left_arm_move_group_.getCurrentState()->getJointModelGroup(LEFT_ARM_PLANNING_GROUP);
  right_gripper_joint_model_group_ =
    right_gripper_move_group_.getCurrentState()->getJointModelGroup(RIGHT_GRIPPER_PLANNING_GROUP);
  left_gripper_joint_model_group_ =
    left_gripper_move_group_.getCurrentState()->getJointModelGroup(LEFT_GRIPPER_PLANNING_GROUP);



  //***** Head controller *****//
  //Initialize the client for the Action interface to the head controller
  /*point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

  //wait for head controller action server to come up
  while(!point_head_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }*/

  //***** Grippers controllers *****//
  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  /*left_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
  right_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

  //wait for the left gripper action server to come up
  while(!left_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
  }

  //wait for the right gripper action server to come up
  while(!right_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }*/

  //***** Torso controller *****//
/*
  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

  //wait for the action server to come up
  while(!torso_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the torso action server to come up");
  }
*/
  // Set Arm to initial pose
  right_arm_move_group_.setNamedTarget("RIGHT_ARM_INITIAL_POSE");
  left_arm_move_group_.setNamedTarget("LEFT_ARM_INITIAL_POSE");

  // Move the arms to initial pose (away from robot vision)
  right_arm_move_group_.move();
  left_arm_move_group_.move();
}

// Class destructor
PR2manip::~PR2manip()
{
  delete left_gripper_client_;
  delete right_gripper_client_;
  delete point_head_client_;
  delete right_arm_joint_model_group_;
  delete left_arm_joint_model_group_;
  delete right_gripper_joint_model_group_;
  delete left_gripper_joint_model_group_;
  delete torso_client_;
}

bool PR2manip::moveTorso(float position)
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
bool PR2manip::gripper_open(std::string gripper)
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
bool PR2manip::gripper_close(std::string gripper, float effort)
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

void PR2manip::move_head(std::string frame_id, double x, double y, double z)
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

bool PR2manip::moveTo(const geometry_msgs::Pose& pose, std::string interface)
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
    right_arm_move_group_.setPoseTarget(target_pose);
    success = (right_arm_move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    right_arm_move_group_.move();
  }
  else if (interface == "left_arm_move_group")
  {
    left_arm_move_group_.setPoseTarget(target_pose);
    success = (left_arm_move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    left_arm_move_group_.move();
  }
  return success;
}


void PR2manip::openGripper(trajectory_msgs::JointTrajectory& posture, std::string side)
{
  if(side == "left_arm")
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
    posture.points[0].time_from_start = ros::Duration(5);
  }
  else if(side == "right_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "r_gripper_joint";
    posture.joint_names[1] = "r_gripper_motor_screw_joint";
    posture.joint_names[2] = "r_gripper_l_finger_joint";
    posture.joint_names[3] = "r_gripper_r_finger_joint";
    posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.088;
    posture.points[0].positions[1] = 1;
    posture.points[0].positions[2] = 0.477;
    posture.points[0].positions[3] = 0.477;
    posture.points[0].positions[4] = 0.477;
    posture.points[0].positions[5] = 0.477;
    posture.points[0].time_from_start = ros::Duration(5);
  }
}

void PR2manip::closedGripper(trajectory_msgs::JointTrajectory& posture, std::string side)
{
  if (side == "left_arm")
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
    posture.points[0].time_from_start = ros::Duration(5);
  }
  else if (side == "right_arm")
  {
    posture.joint_names.resize(6);
    posture.joint_names[0] = "r_gripper_joint";
    posture.joint_names[1] = "r_gripper_motor_screw_joint";
    posture.joint_names[2] = "r_gripper_l_finger_joint";
    posture.joint_names[3] = "r_gripper_r_finger_joint";
    posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
    posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].positions[2] = 0.002;
    posture.points[0].positions[3] = 0.002;
    posture.points[0].positions[4] = 0.002;
    posture.points[0].positions[5] = 0.002;
    posture.points[0].time_from_start = ros::Duration(5);
  }
}


// One shot, se désabonner à la fin, à rappeler après chaque place
void PR2manip::objCallback (const toaster_msgs::ObjectListStamped::ConstPtr& msg)
{
  ontologenius_query::OntologeniusQueryFullService ontoQuery;

  char mesh_uri_c[150];
  int parseReturn = 0;
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::Mesh* m;

  std::vector<std::string> tempV;

  ontoQuery.request.ns = ONTOLOGY_NS;

  // Get ObjID, ObjPose and time from Toaster and put them in the main collision_objects_vector_
  for(auto toasterObj : msg->objectList)
  {
    moveit_msgs::CollisionObject cObj;

    // Fill in id, timestamp, and pose
    cObj.id = toasterObj.meEntity.id;
    geometry_msgs::PoseStamped old_pose, new_pose;
    if(msg->header.frame_id != "base_footprint")
    {
      try{
        old_pose.header.frame_id = msg->header.frame_id.substr(1,msg->header.frame_id.size());
        old_pose.header.stamp = msg->header.stamp;
        old_pose.pose = toasterObj.meEntity.pose;
        new_pose = tfBuffer_.transform(old_pose, "base_footprint");
        cObj.header.frame_id = new_pose.header.frame_id;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("Exception occured when trying to transform frames %s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }
    else
    {
      cObj.header.frame_id = msg->header.frame_id;
    }
    cObj.header.stamp.sec = msg->header.stamp.sec;
    cObj.mesh_poses.push_back(new_pose.pose);

    // Ask ontology for the mesh and fill it in the collision obj fields.
    ontoQuery.request.query = cObj.id + " hasMesh ?mesh";
    if (ontoClient_.call(ontoQuery))
    {
      for (int i=0; i < ontoQuery.response.results[0].names.size(); i++)
      {
        if (ontoQuery.response.results[0].names[i] == "mesh")
        {
          parseReturn = sscanf(ontoQuery.response.results[0].values[i].c_str(), "string#%s",mesh_uri_c);
          if(parseReturn == 1)
          {
            std::string mesh_uri(mesh_uri_c);
            m = shapes::createMeshFromResource(mesh_uri);
            shapes::constructMsgFromShape(m, mesh_msg);
            mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
            // Add the mesh to the Collision object message
            cObj.meshes.push_back(mesh);
          }
        }
        cObj.operation = cObj.ADD;
      }
    }
    else
    {
      ROS_ERROR("Failed to call Ontology for Mesh...");
      ROS_ERROR("Error : %s",ontoQuery.response.error.c_str());
      return;
    }

    // Add the current object to the collision object vector
    collision_objects_vector_.push_back(cObj);
  }

  // Now, let's add the collision object into the world
  // Little sleep necessary before adding it
  ros::Duration(0.2).sleep();
  // Add the remaining collision object
  planning_scene_interface_.addCollisionObjects(collision_objects_vector_);

  // Unsubscribe for now, will re-subscribe after doing a place
  toaster_sub_.shutdown();

  //TODO Verify timestamp and delete old objects.
}

moveit::planning_interface::MoveGroupInterface& PR2manip::getMoveGroupInterface(std::string interface)
{
  if(interface == "right_arm_move_group")
  {
    return right_arm_move_group_;
  }
  else if (interface == "left_arm_move_group")
  {
    return left_arm_move_group_;
  }
  toaster_sub_.shutdown();
}

void PR2manip::setToasterSub(ros::Subscriber sub)
{
  toaster_sub_ = sub;
}
void PR2manip::setOntoClient(ros::ServiceClient srv)
{
  ontoClient_ = srv;
}

void PR2manip::pickObj(const pr2_moveit::pickGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_moveit::pickAction>* pickServer)
{
  std::cout << "Goal id is " << goal->id << std::endl;

  //------  PICK ------//
  std::vector<moveit_msgs::Grasp> grasps;

  char frame_id_c[30];
  int parseReturn = 0;
  char vector;
  double vectorDir;
  double minDist;
  double desiredDist;

  // Memorize what object we want to pick
  objPickedId_ = goal->id;

  ontologenius_query::OntologeniusQueryFullService ontoQuery;

  // For now "robot" is assumed as default namespace
  ontoQuery.request.ns = ONTOLOGY_NS;

  for(auto obj : collision_objects_vector_)
  {
    if(obj.id.find(objPickedId_) != std::string::npos)
    {
      // Ask ontology for the mesh and fill it in the collision obj fields.
      ontoQuery.request.query = obj.id + " hasPickGraspPose ?pose, ?pose preGraspValue ?pre, ?pose graspValue ?grasp, ?pose postGraspValue ?post";
      if (ontoClient_.call(ontoQuery))
      {
        grasps.resize(ontoQuery.response.results.size());
        for (int i=0; i < ontoQuery.response.results.size(); i++)
        {
          for (int j=0; j < ontoQuery.response.results[i].names.size(); j++)
          {
            if (ontoQuery.response.results[i].names[j] == "pre")
            {
              parseReturn = sscanf(ontoQuery.response.results[i].values[j].c_str(), "approach#%[^,],%c,%lf,%lf,%lf",frame_id_c,&vector,&vectorDir,&minDist,&desiredDist);
              if(parseReturn == 5)
              {
                std::string frame_id(frame_id_c);
                // Pre-grasp approach
                grasps[i].pre_grasp_approach.direction.header.frame_id = frame_id;
                if(vector == 'x')
                  grasps[i].pre_grasp_approach.direction.vector.x = vectorDir;
                else if (vector == 'y')
                  grasps[i].pre_grasp_approach.direction.vector.y = vectorDir;
                else if (vector == 'z')
                  grasps[i].pre_grasp_approach.direction.vector.z = vectorDir;

                grasps[i].pre_grasp_approach.min_distance = minDist;
                grasps[i].pre_grasp_approach.desired_distance = desiredDist;
              }
              // TODO Return error if less values gets return by ontologenius
            }
            else if (ontoQuery.response.results[i].names[j] == "grasp")
            {
              geometry_msgs::Pose graspPose;

              // Parse csv response
              parseReturn = sscanf(ontoQuery.response.results[i].values[j].c_str(), "pose#%[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf",frame_id_c,&graspPose.position.x,&graspPose.position.y,&graspPose.position.z,&graspPose.orientation.x,&graspPose.orientation.y,&graspPose.orientation.z,&graspPose.orientation.w);
              if(parseReturn == 8)
              {
                std::string frame_id(frame_id_c);
                // Grasp pose
                grasps[i].grasp_pose.header.frame_id = frame_id_c;
                grasps[i].grasp_pose.pose.position.x = graspPose.position.x;
                grasps[i].grasp_pose.pose.position.y = graspPose.position.y;
                grasps[i].grasp_pose.pose.position.z = graspPose.position.z;
                grasps[i].grasp_pose.pose.orientation.x = graspPose.orientation.x;
                grasps[i].grasp_pose.pose.orientation.y = graspPose.orientation.y;
                grasps[i].grasp_pose.pose.orientation.z = graspPose.orientation.z;
                grasps[i].grasp_pose.pose.orientation.w = graspPose.orientation.w;

                if(obj.mesh_poses[0].position.y > 0)
                {
                  std::cout << objPickedId_ << "IS LEFT : " << obj.mesh_poses[0].position.y << std::endl;
                  armUsed_ = "left_arm";
                }
                else if(obj.mesh_poses[0].position.y < 0)
                {
                  std::cout << objPickedId_ << "IS RIGHT" << std::endl;
                  armUsed_ = "right_arm";
                }
                else if(obj.mesh_poses[0].position.y == 0)
                {
                  std::cout << objPickedId_ << "IS MIDDLE" << std::endl;
                  armUsed_ = "left_arm";
                }

              }
              // TODO Return error if less values gets return by ontologenius
            }
            else if (ontoQuery.response.results[i].names[j] == "post")
            {
              // Post-grasp retreat
              parseReturn = sscanf(ontoQuery.response.results[i].values[j].c_str(), "approach#%[^,],%c,%lf,%lf,%lf",frame_id_c,&vector,&vectorDir,&minDist,&desiredDist);
              if(parseReturn == 5)
              {
                std::string frame_id(frame_id_c);
                // Pre-grasp approach
                // TODO Have the three vectors sets to have precise direction
                grasps[i].post_grasp_retreat.direction.header.frame_id = frame_id_c;
                if(vector == 'x')
                  grasps[i].post_grasp_retreat.direction.vector.x = vectorDir;
                else if (vector == 'y')
                  grasps[i].post_grasp_retreat.direction.vector.y = vectorDir;
                else if (vector == 'z')
                  grasps[i].post_grasp_retreat.direction.vector.z = vectorDir;

                grasps[i].post_grasp_retreat.min_distance = minDist;
                grasps[i].post_grasp_retreat.desired_distance = desiredDist;
              }
              // TODO Return error if less values gets return by ontologenius
            }
          }
          // Pre-grasp end effector pose
          openGripper(grasps[i].pre_grasp_posture,armUsed_);

          // Grasp end effector pose
          closedGripper(grasps[i].grasp_posture,armUsed_);
        }
      }
      else
      {
        ROS_ERROR("Failed to call Ontology for grasp_pose...");
        ROS_ERROR("Error : %s",ontoQuery.response.error.c_str());
        return;
      }
      // Get object pose, verify the frame_id and remap it to base_footprint if necessary

    }

  }

  if(armUsed_ == "left_arm")
  {
    // TODO Modify support surface according to Toaster object return (table is assumed to have a tag)
    // Set support surface
    left_arm_move_group_.setSupportSurfaceName("tableLaas");
    // Call pick to pick up the object using the grasps given
    pickResult_.error_code = left_arm_move_group_.pick(goal->id, grasps);
  }
  else if(armUsed_ == "right_arm")
  {
    // TODO Modify support surface according to Toaster object return (table is assumed to have a tag)
    // Set support surface
    right_arm_move_group_.setSupportSurfaceName("tableLaas");
    // Call pick to pick up the object using the grasps given
    pickResult_.error_code = right_arm_move_group_.pick(goal->id, grasps);
  }


  pickServer->setSucceeded(pickResult_);
}

void PR2manip::placeObj(const pr2_moveit::placeGoalConstPtr& goal,  actionlib::SimpleActionServer<pr2_moveit::placeAction>* placeServer)
{
  //------  PLACE ------//
  std::vector<moveit_msgs::PlaceLocation> place_location;

  int parseReturn = 0;
  char vector;
  double vectorDir;
  double minDist;
  double desiredDist;

  ontologenius_query::OntologeniusQueryFullService ontoQuery;

  // For now "robot" is assumed as default namespace
  ontoQuery.request.ns = ONTOLOGY_NS;

  for(auto obj : collision_objects_vector_)
  {
    if(obj.id.find(objPickedId_) != std::string::npos)
    {
      // Ask ontology for the mesh and fill it in the collision obj fields.
      ontoQuery.request.query = obj.id + " hasPlaceGraspPose ?pose, ?pose preGraspValue ?pre, ?pose postGraspValue ?post";
      if (ontoClient_.call(ontoQuery))
      {
        place_location.resize(ontoQuery.response.results.size());
          for (int i=0; i < ontoQuery.response.results.size(); i++)
          {
            for (int j=0; j < ontoQuery.response.results[i].names.size(); j++)
            {
              if (ontoQuery.response.results[i].names[j] == "pre")
              {
                parseReturn = sscanf(ontoQuery.response.results[i].values[j].c_str(), "approach#base_footprint,%c,%lf,%lf,%lf",&vector,&vectorDir,&minDist,&desiredDist);
                if(parseReturn == 4)
                {
                  // Pre-place approach
                  place_location[i].pre_place_approach.direction.header.frame_id = "base_footprint";
                  if(vector == 'x')
                    place_location[i].pre_place_approach.direction.vector.x = vectorDir;
                  else if (vector == 'y')
                    place_location[i].pre_place_approach.direction.vector.y = vectorDir;
                  else if (vector == 'z')
                    place_location[i].pre_place_approach.direction.vector.z = vectorDir;

                  place_location[i].pre_place_approach.min_distance = minDist;
                  place_location[i].pre_place_approach.desired_distance = desiredDist;
                }
                // TODO Return error if less values gets return by ontologenius
              }
              else if (ontoQuery.response.results[i].names[j] == "post")
              {
                // Post-grasp retreat
                parseReturn = sscanf(ontoQuery.response.results[i].values[j].c_str(), "approach#base_footprint,%c,%lf,%lf,%lf",&vector,&vectorDir,&minDist,&desiredDist);
                if(parseReturn == 4)
                {
                  // Pre-place approach
                  // TODO Have the three vectors sets to have precise direction
                  place_location[i].post_place_retreat.direction.header.frame_id = "base_footprint";
                  if(vector == 'x')
                    place_location[i].post_place_retreat.direction.vector.x = vectorDir;
                  else if (vector == 'y')
                    place_location[i].post_place_retreat.direction.vector.y = vectorDir;
                  else if (vector == 'z')
                    place_location[i].post_place_retreat.direction.vector.z = vectorDir;

                  place_location[i].post_place_retreat.min_distance = minDist;
                  place_location[i].post_place_retreat.desired_distance = desiredDist;
                }
                // TODO Return error if less values gets return by ontologenius
              }
            }

            // Post-place end effector pose
            openGripper(place_location[i].post_place_posture,armUsed_);

          }
        }
        else
        {
          ROS_ERROR("Failed to call Ontology for place_pose...");
          ROS_ERROR("Error : %s",ontoQuery.response.error.c_str());
          return;
        }
      }
    }

  // Place pose
  place_location[0].place_pose.header.frame_id = "base_footprint";
  place_location[0].place_pose.pose.position.x = goal->posePlace.position.x;
  place_location[0].place_pose.pose.position.y = goal->posePlace.position.y;
  place_location[0].place_pose.pose.position.z = goal->posePlace.position.z;
  place_location[0].place_pose.pose.orientation.x = goal->posePlace.orientation.x;
  place_location[0].place_pose.pose.orientation.y = goal->posePlace.orientation.y;
  place_location[0].place_pose.pose.orientation.z = goal->posePlace.orientation.z;
  place_location[0].place_pose.pose.orientation.w = goal->posePlace.orientation.w;

  if(armUsed_ == "left_arm")
  {
    placeResult_.error_code = left_arm_move_group_.place(objPickedId_, place_location);
  }
  else if(armUsed_ == "right_arm")
  {
    placeResult_.error_code = right_arm_move_group_.place(objPickedId_, place_location);
  }



  placeServer->setSucceeded(placeResult_);

  // TODO Handle resubscribe
  //toaster_sub_ = nh_.subscribe<toaster_msgs::ObjectListStamped>("/pdg/objectList", 1000, boost::bind(&PR2manip::objCallback,this,_1));
}

void PR2manip::move(const pr2_moveit::moveGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_moveit::moveAction>* moveServer)
{


  moveServer->setSucceeded(moveResult_);
}

std::vector<moveit_msgs::CollisionObject>& PR2manip::getCollisionObjVector()
{
  return collision_objects_vector_;
}


void mySigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pr2_manip");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  PR2manip pr2(n);

/*
  // Init pose
  pr2.move_head("base_footprint",1.2,-0.2,0.36);
  pr2.moveTorso(0.1);

  pr2.gripper_close("left_gripper",25);
  pr2.gripper_close("right_gripper",25);
*/

  // Query client to get mesh and poses from ontology
  ros::ServiceClient ontoClient = n.serviceClient<ontologenius_query::OntologeniusQueryFullService>("ontologenius_query/fullQuery");
  pr2.setOntoClient(ontoClient);

  // Subscriber to toaster to get objects in the scene
  ros::Subscriber toasterSub = n.subscribe<toaster_msgs::ObjectListStamped>("/pdg/objectList", 1000, boost::bind(&PR2manip::objCallback,&pr2,_1));
  pr2.setToasterSub(toasterSub);

  // Action servers to pick place and move the robot
  actionlib::SimpleActionServer<pr2_moveit::pickAction> pickServer(pn, "pick", boost::bind(&PR2manip::pickObj, &pr2, _1, &pickServer), false);
  actionlib::SimpleActionServer<pr2_moveit::placeAction> placeServer(pn, "place", boost::bind(&PR2manip::placeObj, &pr2, _1, &placeServer), false);
  actionlib::SimpleActionServer<pr2_moveit::moveAction> moveServer(pn, "move", boost::bind(&PR2manip::move, &pr2, _1, &moveServer), false);

/*
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
  table_pose.position.x = 0.8;
  table_pose.position.y = 0;
  table_pose.position.z = 0.36;

  mainTable.primitives.push_back(primitive);
  mainTable.primitive_poses.push_back(table_pose);
  mainTable.operation = mainTable.ADD;

  pr2.getCollisionObjVector().push_back(mainTable);
  */

  ROS_INFO("Start Action servers");
  pickServer.start();
  placeServer.start();
  moveServer.start();

  /*
  std::cout << "**** Trying Pick" << std::endl;
  ros::Duration(2).sleep();
  marker_sub.shutdown();
  pr2.pickplace(pr2.getMoveGroupInterface("left_arm_move_group"));
  */
  ros::waitForShutdown();
  return 0;
}
