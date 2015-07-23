/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Testing program for humanoid/HRP2 functionality
*/

#include <ros/ros.h>
#include <ros/package.h> // for getting file path of package names

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

// OMPL
#include <moveit/ompl/model_based_planning_context.h>
#include <moveit/ompl/parameterization/model_based_state_space.h>
#include <moveit/ompl/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl/parameterization/joint_space/joint_model_state_space_factory.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl_thunder/Thunder.h>
#include <ompl_visual_tools/ompl_visual_tools.h>

// MoveIt msgs
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>

// Constraint sampler
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>

// Helper for Rviz
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// Random numbers
#include <random_numbers/random_numbers.h>

// Stability checker
#include <moveit_humanoid/humanoid_stability.h>

// ROS
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/pointer_cast.hpp>

// C++
#include <iostream>
#include <fstream>
#include <limits>       // std::numeric_limits

namespace hrp2jsknt_moveit_demos
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MARKER_TOPIC = "/hrp2_visual_markers";
static const std::string BASE_LINK = "/odom";

class HRP2Demos
{
private:

  ros::NodeHandle nh_;

  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  robot_state::RobotStatePtr goal_state_;
  robot_state::RobotStatePtr blank_state_;

  robot_model::JointModelGroup* joint_model_group_; // selected by user
  std::string planning_group_name_; // allow to change planning group from command line

  robot_model::JointModelGroup* whole_body_group_; // hard-coded
  std::string whole_body_group_name_; // hard-coded group for the whole robot including virtual joint

  planning_scene::PlanningScenePtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // The visual tools for interfacing with Rviz
  ompl_visual_tools::OmplVisualToolsPtr ompl_visual_tools_;

  // Optional monitor to communicate with Rviz
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Constraint sampler
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;

  // Tool for checking balance
  moveit_humanoid::HumanoidStabilityPtr humanoid_stability_;

  bool use_thunder_;

public:
  HRP2Demos(const std::string planning_group_name)
    : nh_("~")
    , robot_model_loader_(ROBOT_DESCRIPTION) // load the URDF
    , planning_group_name_(planning_group_name)
    , whole_body_group_name_("whole_body")
    , use_thunder_(false)
  {
    // Load the robot model
    robot_model_ = robot_model_loader_.getModel(); // Get a shared pointer to the robot

    // Create the planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    // Get the configuration for the joints in the group
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    whole_body_group_ = robot_model_->getJointModelGroup(whole_body_group_name_);

    // Create robot states
    robot_state_.reset(new robot_state::RobotState(robot_model_)); // TODO: load this robot state from planning_scene instead
    robot_state_->setToDefaultValues();
    goal_state_.reset(new robot_state::RobotState(*robot_state_));

    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
    visual_tools_->loadRobotStatePub("/hrp2_demos");
    visual_tools_->loadMarkerPub();
    //visual_tools_->loadTrajectoryPub();

    // Clear all old visual aspects from Rviz
    visual_tools_->deleteAllMarkers(); // clear all old markers

    // Allow time to startup
    ros::Duration(0.1).sleep();
  }

  void loadPlanningPipeline()
  {
    if (!planning_pipeline_)
    {
      // Setup planning pipeline
      planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));
    }
  }

  void loadHumanoidStabilityChecker(bool verbose)
  {
    // Configure stability checker
    if (!humanoid_stability_)
      humanoid_stability_.reset(new moveit_humanoid::HumanoidStability(verbose, *robot_state_, visual_tools_));
  }

  void setUseThunder(bool use_thunder)
  {
    use_thunder_ = use_thunder;
  }

  void printPlanningSceneObjects()
  {
    std::cout << std::endl;
    std::cout << "Objects in world: " << std::endl;
    for (std::size_t i = 0; i < planning_scene_->getWorld()->getObjectIds().size(); ++i)
    {
      std::cout << planning_scene_->getWorld()->getObjectIds()[i] << std::endl;
    }
    std::cout << std::endl;
  }

  // Whole body planning with MoveIt!
  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=1 verbose:=0 problems:=1 runs:=1 use_experience:=1 use_collisions:=0
  bool genRandWholeBodyPlans(int problems, bool verbose, bool use_experience, bool use_collisions, bool variable_obstacles, bool random_start)
  {

    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return false;

    if (!use_collisions)
    {
      ros::Duration(0.25).sleep();
      visual_tools_->removeAllCollisionObjectsPS(); // clear all old collision objects that might be visible in rviz
    }

    // Always prevent feet from going into floor
    //visual_tools_->publishCollisionFloor(0, "Floor"); // DOES NOT WORK?

    std::ofstream logging_file; // open to append
    if (use_thunder_ && use_experience)
      logging_file.open("/home/dave/ompl_storage/thunder_whole_body_logging.csv", std::ios::out | std::ios::app);
    else if (use_thunder_ && !use_experience)
      logging_file.open("/home/dave/ompl_storage/scratch_whole_body_logging.csv", std::ios::out | std::ios::app);
    else
      logging_file.open("/home/dave/ompl_storage/lightning_whole_body_logging.csv", std::ios::out | std::ios::app);

    // Move robot to specific place on plane
    fixRobotStateFoot(robot_state_, 1.5, 1.8);
    fixRobotStateFoot(goal_state_, 1.5, 1.8);

    // Set custom validity checker for balance constraints
    loadHumanoidStabilityChecker(verbose);
    planning_scene_->setStateFeasibilityPredicate(humanoid_stability_->getStateFeasibilityFn());

    // Load planning
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Loading planning pipeline");
    loadPlanningPipeline(); // always call before using generatePlan()

    // Remember the planning context even after solving is done
    planning_interface::PlanningContextPtr planning_context_handle;
    // Remember all planning context handles so we can save to file in the end
    std::set<planning_interface::PlanningContextPtr> planning_context_handles;

    ompl::tools::ExperienceSetupPtr experience_setup;
    ompl::ModelBasedStateSpacePtr model_state_space;

    // Loop through planning
    for (std::size_t i = 0; i < problems; ++i)
    {
      if (!ros::ok())
        break;

      // Show the lab as collision objects
      if (use_collisions)
      {
        if (variable_obstacles) // republish new coll environement every loop
        {
          visual_tools_->removeAllCollisionObjectsPS(); // clear all old collision objects that might be visible in rviz
          ros::Duration(0.25).sleep();

          int location = moveit_visual_tools::MoveItVisualTools::iRand(0,4);
          jskLabCollisionEnvironment(location);
        }
        else if (i == 0) // just publish one coll environment on first go araound
        {
          jskLabCollisionEnvironment(0);
        }
      }

      // Benchmark runtime
      ros::Time start_time;
      start_time = ros::Time::now();

      // Display
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "NEW PLAN STARTING (" << i+1 << " of " << problems << ")----------------------------------------------------- " << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;

      if (!genRandWholeBodyPlans(verbose, use_experience, random_start, planning_context_handle))
        return false;

      // Save all contexts to a set
      planning_context_handles.insert(planning_context_handle);
      if (planning_context_handles.size() > 1)
      {
        ROS_FATAL_STREAM_NAMED("hrp2_demos","Unexpected: more than 1 planning context now exists");
        exit(-1);
      }

      // Error check
      if (!planning_context_handle)
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","No planning context handle point available");
        continue; // skip the rest - this is undefined behavior but good for debugging
      }

      // Load ptrs on first pass
      if (i == 0 && (use_experience || true))
      {
        ompl::ModelBasedPlanningContextPtr mbpc = boost::dynamic_pointer_cast<ompl::ModelBasedPlanningContext>(planning_context_handle);
        experience_setup = boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());
        model_state_space = mbpc->getOMPLStateSpace();
      }

      // Debugging
      if (use_experience || true)
      {
        std::cout << std::endl;
        experience_setup->printLogs();
        experience_setup->saveDataLog(logging_file);
        logging_file.flush();
        if (verbose)
          ros::Duration(2).sleep(); // allow trajectory to play
      }

      // Benchmark runtime
      double duration = (ros::Time::now() - start_time).toSec();
      std::cout << "Total full planning loop (with debugging and benchmarking) took: " << duration << " seconds" << std::endl;

      // Save database periodically
      int save_every = std::min(i / 10 + 10.0, 100.0); // problems
      if ((i+1) % save_every == 0 && (use_experience || true))
      {
        ROS_WARN_STREAM_NAMED("hrp2_demos","Saving experience db...");
        experience_setup->saveIfChanged();
      }

    } // for all runs
    
    // Finish the logging file
    logging_file.close();

    // Save the solutions to file before shutting down
    if (use_experience || true)
    {
      ROS_WARN_STREAM_NAMED("hrp2_demos","Saving experience db...");
      experience_setup->saveIfChanged();


      // Display tips of robot from experience database
      //displayThunderDatabase(experience_setup, model_state_space);
    }

    return true;
  }

  bool genRandWholeBodyPlans(bool verbose, bool use_experience, bool random_start, planning_interface::PlanningContextPtr &planning_context_handle)
  {
    // Create a constraint sampler for random poses
    loadConstraintSampler(verbose); // TODO: this is inefficient, it would be better if we just did this once, but the problem is that changing plannning scenes do not work

    // Get a START state ------------------------------------------------------------
    int attempts = 10000;

    if (random_start)
    {
      ROS_DEBUG_STREAM_NAMED("hrp2_demos","Generating random start state");

      if (!constraint_sampler_->sample(*robot_state_, *robot_state_, attempts))
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Unable to find valid start state");
        return false;
      }
    }
    else
    {
      static const std::string state_name = "one_foot_transition"; //one_foot_start";
      ROS_DEBUG_STREAM_NAMED("hrp2_demos","Using for start state pose: " << state_name);
      setStateToGroupPose(robot_state_,  state_name, joint_model_group_);
    }

    // Update virtual joint transform to fake base
    robot_state_->updateWithDynamicRoot();

    // Testing
    visual_tools_->publishRobotState(robot_state_);

    // Error check
    bool check_verbose = true;
    if (!planning_scene_->isStateValid(*robot_state_, "", check_verbose)) // second argument is what planning group to collision check, "" is everything
    {
      ROS_FATAL_STREAM_NAMED("hrp2_demos","Start state is not valid!");
      return false;
    }

    // Visualize
    if (verbose)
    {
      visual_tools_->publishRobotState(robot_state_);
      std::cout << "Visualizing start state " << std::endl;
    }

    // Get a GOAL state ---------------------------------------------------
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Generating random goal state");

    if (!constraint_sampler_->sample(*goal_state_, *goal_state_, attempts))
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Unable to find valid goal state");
      return false;
    }
    // Update virtual joint transform to fake base
    goal_state_->updateWithDynamicRoot();

    // Error check
    if (!planning_scene_->isStateValid(*goal_state_, "", check_verbose)) // second argument is what planning group to collision check, "" is everything
    {
      ROS_FATAL_STREAM_NAMED("hrp2_demos","Goal state is not valid!");
      exit(0);
    }

    // Visualize
    if (verbose)
    {
      visual_tools_->publishRobotState(goal_state_);
      std::cout << "Visualizing goal state " << std::endl;
    }


    // Plan to pose
    genRandWholeBodyPlan(verbose, use_experience, planning_context_handle, robot_state_, goal_state_);
    return true;
  }

  void genRandWholeBodyPlan(bool verbose, bool use_experience, planning_interface::PlanningContextPtr &planning_context_handle,
                            robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
  {
    // Test start state
    if( !humanoid_stability_->isValid(*start_state, true) )
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Start not valid!!");

    // Test goal state
    if( !humanoid_stability_->isValid(*goal_state, true) )
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Goal not valid!!");


    // Create motion planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);

    // Goal constraint
    double tolerance_pose = 0.0001;
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(*goal_state, joint_model_group_,
                                                                                               tolerance_pose, tolerance_pose);
    req.goal_constraints.push_back(goal_constraint);

    // Other settings
    req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = planning_group_name_;
    req.num_planning_attempts = 1; // this must be one else it threads and doesn't use lightning/thunder correctly
    req.allowed_planning_time = 90; // second
    req.use_experience = use_experience;
    if (use_thunder_)
    {
      req.experience_method = "thunder";
    }
    else
    {
      req.experience_method = "lightning";
    }

    // Call pipeline
    std::vector<std::size_t> dummy;

    // SOLVE
    planning_pipeline_->generatePlan(planning_scene_, req, res, dummy, planning_context_handle);

    // Check that the planning was successful
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =======================================================");
      if (verbose)
        ROS_INFO_STREAM_NAMED("hrp2_demos","Attempting to visualize trajectory anyway...");
    }

    // Show the trajectory
    if (verbose)
    {
      // Create planning request
      moveit_msgs::MotionPlanResponse response;
      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      //std::cout << "Trajectory debug:\n " << response.trajectory << std::endl;

      // Optionally publish
      /*
        if (true)
        {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = response.trajectory.joint_trajectory;

        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > controller_action_client_;
        controller_action_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        ("hrp2jsknt/follow_joint_trajectory_action", true));

        ros::spinOnce();
        ros::Duration(2).sleep();
        ros::spinOnce();
        controller_action_client_->sendGoal(goal);
        }
      */

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //visual_tools_->hideRobot(); // hide the other robot so that we can see the trajectory TODO bug?
      bool wait_for_trajetory = false;
      visual_tools_->publishTrajectoryPath(response.trajectory, wait_for_trajetory);

    }
    else
    {
      ROS_INFO_STREAM_NAMED("hrp2_demos","Not visualizing because not in verbose mode");
    }


  }

  bool displayThunderDatabase(ompl::tools::ExperienceSetupPtr experience_setup,
                              const ompl::ModelBasedStateSpacePtr model_state_space)
  {
    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return false;
    jskLabCollisionEnvironment(0);

    // Get all of the paths in the database
    std::vector<ompl::base::PlannerDataPtr> graphs;
    experience_setup->getAllPlannerDatas(graphs);

    // Error check
    if (!graphs.size())
    {
      ROS_WARN_STREAM_NAMED("hrp2_demos","Unable to show first state of robot because graph is empty");
      return false;
    }

    // Load the OMPL visualizer
    if (!ompl_visual_tools_)
    {
      ompl_visual_tools_.reset(new ompl_visual_tools::OmplVisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
      ompl_visual_tools_->loadRobotStatePub("/hrp2_demos");
    }
    ompl_visual_tools_->setStateSpace(model_state_space);
    ompl_visual_tools_->deleteAllMarkers(); // clear all old markers

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    getNonFixedRobotTips(tips);

    ompl_visual_tools_->publishRobotGraph(graphs[0], tips);

    // Show HRP2 in some pose
    for (std::size_t i = 0; i < graphs[0]->numVertices() && ros::ok(); ++i)
    {
      ompl_visual_tools_->publishRobotState(graphs[0]->getVertex(i).getState());
      ros::Duration(0.1).sleep();
    }
    
  }

  void getNonFixedRobotTips(std::vector<const robot_model::LinkModel*> &tips)
  {
    // Remove the tip that is fixed to save display time
    joint_model_group_->getEndEffectorTips(tips);
    std::cout << "Found " << tips.size() << " end effector tips" << std::endl;
    std::size_t delete_index = tips.size();
    for (std::size_t i = 0; i < tips.size(); ++i)
    {
      if (tips[i] == robot_state_->getPrimaryFixedLink())
      {
        //std::cout << "Not displaying tip " << tips[i]->getName() << " because it is fixed" << std::endl;
        delete_index = i;
      }
    }
    if (delete_index < tips.size())
    {
      tips.erase( tips.begin() + delete_index );
    }
  }

  void displayDBPlans(int problems, bool verbose)
  {
    if (use_thunder_)
    {
      displayThunderPlans(problems, verbose);
    }
    else
    {
      displayLightningPlans(problems, verbose);
    }
  }

  // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=2 group:=left_arm verbose:=1
  void displayLightningPlans(int problems, bool verbose)
  {
    // All we care about is the trajectory robot
    visual_tools_->hideRobot();

    // Create a state space describing our robot's planning group
    ompl::ModelBasedStateSpaceSpecification model_ss_spec(robot_model_, joint_model_group_);
    const ompl::JointModelStateSpaceFactory factory;
    ompl::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();

    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Model Based State Space has dimensions: " << model_state_space->getDimension());

    // Load lightning and its database
    ompl::tools::Lightning lightning(model_state_space);
    lightning.setFile(joint_model_group_->getName());

    // Get all of the paths in the database
    std::vector<ompl::base::PlannerDataPtr> paths;
    lightning.getAllPlannerDatas(paths);

    ROS_INFO_STREAM_NAMED("hrp2_demos","Number of paths to publish: " << paths.size());

    // Load the OMPL visualizer
    ompl_visual_tools_.reset(new ompl_visual_tools::OmplVisualTools(BASE_LINK, MARKER_TOPIC, robot_model_));
    ompl_visual_tools_->loadRobotStatePub("/hrp2_demos");
    ompl_visual_tools_->setStateSpace(model_state_space);
    ompl_visual_tools_->deleteAllMarkers(); // clear all old markers

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    joint_model_group_->getEndEffectorTips(tips);
    std::cout << "Found " << tips.size() << " tips" << std::endl;

    bool show_trajectory_animated = verbose;

    // Loop through each path
    problems = !problems ? std::numeric_limits<int>::max() : problems; // if 0, show all problems
    std::cout << "Looping for " << problems << " problems" << std::endl;
    for (std::size_t path_id = 0; path_id < std::min(int(paths.size()), problems); ++path_id)
    {
      std::cout << "Processing path " << path_id << std::endl;
      ompl_visual_tools_->publishRobotPath(paths[path_id], joint_model_group_, tips, show_trajectory_animated);
    }

  } // function

    // roslaunch hrp2jsknt_moveit_demos hrp2_demos.launch mode:=2 group:=left_arm verbose:=1
  void displayThunderPlans(int problems, bool verbose)
  {
    // All we care about is the trajectory robot
    visual_tools_->hideRobot();

    // Create a state space describing our robot's planning group
    ompl::ModelBasedStateSpaceSpecification model_ss_spec(robot_model_, joint_model_group_);
    const ompl::JointModelStateSpaceFactory factory;
    ompl::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    // Setup the state space
    model_state_space->setup();

    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Model Based State Space has dimensions: " << model_state_space->getDimension());

    // Load thunder and its database
    ompl::tools::ThunderPtr thunder(new ompl::tools::Thunder(model_state_space));
    thunder->setFile(joint_model_group_->getName());
    thunder->setup(); // must be called before load

    // This is important for creating a fake problem definition
    ob::ScopedState<> start(model_state_space);
    ob::ScopedState<> goal(model_state_space);
    thunder->setStartAndGoalStates(start, goal);

    // Display tips of robot from experience database
    displayThunderDatabase(thunder, model_state_space);

  } // function

  void genRandMoveItPlan()
  {
    setStateToGroupPose(goal_state_,  "reset_whole_body_fixed", joint_model_group_);
    setStateToGroupPose(robot_state_, "reset_whole_body_fixed", joint_model_group_);

    // Generate random goal positions
    ros::Rate loop_rate(1);
    for (int counter=0; counter<1 && ros::ok(); counter++)
    {
      //ROS_WARN_STREAM_NAMED("hrp2_demos","RUN " << counter << " ------------------------------");

      // Reset
      visual_tools_->hideRobot();
      ros::Duration(1.0).sleep();

      // Make goal state crouching
      setStateCrouching(goal_state_);
      // Move virtual_joint in XY
      setStateComplex(goal_state_);

      // Visualize request first
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(1.0).sleep();

      visual_tools_->publishRobotState(goal_state_);
      ros::Duration(1.0).sleep();

      moveit_msgs::MotionPlanResponse response;

      // Create motion planning request
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;

      // Workspace
      req.workspace_parameters.header.frame_id = "/odom";
      // relative to center of robot
      double workspace_size = 5;
      req.workspace_parameters.min_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") - workspace_size;
      req.workspace_parameters.min_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") - workspace_size;
      req.workspace_parameters.min_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") - workspace_size;
      req.workspace_parameters.max_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") + workspace_size;
      req.workspace_parameters.max_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") + workspace_size;
      req.workspace_parameters.max_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") + workspace_size;

      // Start state
      moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

      // Path constraint - Keep the body link from rotating too much
      /*
        moveit_msgs::OrientationConstraint oc1;
        oc1.header.frame_id = "/odom";
        oc1.orientation.x = 0;
        oc1.orientation.y = 0;
        oc1.orientation.z = 0;
        oc1.orientation.w = 1;
        oc1.link_name = std::string("BODY");
        oc1.absolute_x_axis_tolerance = 0.1;
        oc1.absolute_y_axis_tolerance = 0.1;
        oc1.absolute_z_axis_tolerance = 0.1;
        oc1.weight = 1;
        req.path_constraints.orientation_constraints.push_back(oc1); // TODO: make this work
      */

      // Goal constraint
      double tolerance_pose = 0.0001;
      moveit_msgs::Constraints goal_constraint =
        kinematic_constraints::constructGoalConstraints(*goal_state_, joint_model_group_, tolerance_pose, tolerance_pose);
      req.goal_constraints.push_back(goal_constraint);

      // Other settings
      req.planner_id = "RRTConnectkConfigDefault";
      req.group_name = planning_group_name_;
      req.num_planning_attempts = 1;
      req.allowed_planning_time = 30;

      // Call pipeline
      loadPlanningPipeline(); // always call before using generatePlan()
      planning_pipeline_->generatePlan(planning_scene_, req, res);

      // Check that the planning was successful
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("Could not compute plan successfully =======================================================");
        ROS_INFO_STREAM_NAMED("hrp2_demos","Attempting to visualize trajectory anyway...");
      }

      response.trajectory = moveit_msgs::RobotTrajectory();
      res.getMessage(response);

      // Visualize the trajectory
      ROS_INFO("Visualizing the trajectory");
      //ROS_DEBUG_STREAM_NAMED("hrp2_demos","recieved trajectory: " << response.trajectory);

      visual_tools_->publishTrajectoryPath(response.trajectory, false);
    }
  }

  bool setRandomValidState(robot_state::RobotStatePtr &state, const robot_model::JointModelGroup* jmg)
  {
    // Loop until a collision free state is found
    static const int MAX_ATTEMPTS = 500;
    static int total_calls = 0;
    static int total_attempts = 0;
    total_calls++;

    int attempt_count = 0; // prevent loop for going forever
    while (attempt_count < MAX_ATTEMPTS)
    {
      // Generate random stat
      state->setToRandomPositions(jmg);

      state->updateWithDynamicRoot();
      //state->update(true); // prevent dirty transforms

      // Test for collision
      if (planning_scene_->isStateValid(*state, "", false))
      {
        total_attempts += attempt_count;
        std::cout << "This sample took " << attempt_count << " with overall average " << (total_attempts / total_calls) << std::endl;
        break;
      }

      attempt_count ++;
    }

    // Explain if no valid rand state found
    if (attempt_count >= MAX_ATTEMPTS)
    {
      ROS_WARN("Unable to find a random collision free configuration after %d attempts", MAX_ATTEMPTS);
      return false;
    }

    return true;
  }

  bool loadPlanningSceneMonitor()
  {
    // Allows us to sycronize to Rviz and also publish collision objects to ourselves
    ROS_DEBUG_STREAM_NAMED("hrp2_demos","Loading Planning Scene Monitor");
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, ROBOT_DESCRIPTION,
                                                                                   boost::shared_ptr<tf::Transformer>(), "hrp2_demos"));
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    if (planning_scene_monitor_->getPlanningScene())
    {
      // Optional monitors to start:
      bool use_octomap_monitor = false; // this prevents a /tf warning
      planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                         planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                         use_octomap_monitor);
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "test_planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("hrp2_demos","Planning scene not configured");
      return false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);

    return true;
  }

  void publishJskLabCollisionEnvironment(double x, double y)
  {
    // Load planning scene monitor so that we can publish a collision enviornment to rviz
    if (!loadPlanningSceneMonitor())
      return;

    ros::Duration(1).sleep();

    visual_tools_->deleteAllMarkers(); // clear all old markers
    visual_tools_->removeAllCollisionObjects(); // clear all old collision objects that might be visible in rviz

    ros::Duration(1).sleep();

    robot_state_->updateWithDynamicRoot();
    visual_tools_->publishRobotState(robot_state_);

    // Show the lab as collision objects
    jskLabCollisionEnvironment(x, y);
  }

  void jskLabCollisionEnvironment(int location)
  {
    double x_offset = 0;
    double y_offset = 0;
    ROS_INFO_STREAM_NAMED("temp", "JSK Lab location is number " << location);

    switch (location)
    {
      case 0: // In front of kitchen island
        break;
      case 1: // standing next to chair and TV
        x_offset = -2.0;
        y_offset = -0.3;
        break;
      case 2: // corner with bookshelves
        x_offset = -9.25;
        y_offset = -0.55;
        break;
      case 3: // back against wall
        x_offset = -4.9;
        y_offset = -0.55;
        break;
      case 4: // cart
        x_offset = -0.6;
        y_offset = 2.8;
        break;
    }
    jskLabCollisionEnvironment(x_offset, y_offset);
  }

  void jskLabCollisionEnvironment(double x_offset, double y_offset)
  {
    // Get image path based on package name
    std::string image_path = ros::package::getPath("hrp2jsknt_moveit_demos");
    if( image_path.empty() )
    {
      ROS_FATAL( "Unable to get hrp2jsknt_moveit_demos package path " );
      exit(99);
    }

    image_path.append("/resources/room73b2-without-floor.scene");
    visual_tools_->loadCollisionSceneFromFile(image_path, x_offset, y_offset);
  }

  // Convert a robot state to have a fixed foot
  void fixRobotStateFoot(robot_state::RobotStatePtr &robot_state, double x, double y)
  {
    robot_state->setToDefaultValues();

    // Enable the robot state to have a foot base
    const robot_model::LinkModel* foot = robot_model_->getLinkModel("RLEG_LINK5");
    Eigen::Affine3d default_foot_transform = robot_state->getGlobalLinkTransform(foot); // get the orientation
    //robot_state->printTransform(default_foot_transform, std::cout);

    // Change the translation transform component
    default_foot_transform.translation().x() = x;
    default_foot_transform.translation().y() = y;
    default_foot_transform.translation().z() = 0.1; //0.15 for floor obstacle
    //default_foot_transform.translation().z() = 0.0; //0.15 for floor obstacle

    // Set robot_state to maintain this location
    robot_state->addFixedLink(foot, default_foot_transform);
    robot_state->enableDynamicRoot();
  }

  // Set every joint in the group to the same joint value
  void setGroupToValue(robot_state::RobotStatePtr robot_state, const robot_model::JointModelGroup* jmg, double value)
  {
    double group_joints[jmg->getVariableCount()]; // TODO get joint, not variable
    for (std::size_t i = 0; i < jmg->getVariableCount(); ++i)
      group_joints[i] = value;
    robot_state->setJointGroupPositions(jmg, group_joints);
    robot_state->enforceBounds();
  }

  void genIKRequestsSweep(int runs, int problems, std::size_t seed)
  {
    // Whether to do parameter sweeping
    if (false)
    {
      // Save to file
      std::ofstream istream("/home/dave/ros/ik_data/times.dat");

      double test_values[7] = { 0.0, 0.0001, 0.001, 0.05, 0.01, 0.05, 0.1};

      // Vary alphas
      //for (double alpha = 0.0; alpha < 1.0; alpha += 0.05)
      for (std::size_t i = 0; i < 7; ++i)
      {
        double alpha = test_values[i];
        ROS_WARN_STREAM_NAMED("hrp2_demos","Testing alpha with value " << alpha);

        // Set the alpha on the ROS param server

        //double alpha = 0.6;
        nh_.setParam("alpha", alpha);
        ros::Duration(0.1).sleep();

        double duration = genIKRequestsBenchmark(runs, problems, seed);

        // Save
        istream << alpha << "," << duration << std::endl;
      }

      istream.close();
    }
    else
    {
      genIKRequestsBenchmark(runs, problems, seed);
    }
  }

  double genIKRequestsBenchmark(int runs, int problems, std::size_t seed)
  {
    // Benchmark time
    ros::Time start_time;
    start_time = ros::Time::now();

    for (std::size_t i = 0; i < runs; ++i)
    {
      // Run test
      if (!genIKRequests(problems, seed))
      {
        ROS_ERROR_STREAM_NAMED("hrp2_demos","Test failed");
        return 1000; // a large number
      }
    }

    // Benchmark time
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds");

    return duration;
  }

  // Get the starting pose that corresponds to selected planning group
  std::string getStartPose(const std::string& planning_group)
  {
    if (planning_group == "left_arm")
    {
      return "left_arm_ik_default";
    }
    else if (planning_group == "whole_body_fixed")
    {
      return "reset_whole_body_fixed";
    }
    else if (planning_group == "whole_body")
    {
      return "reset_whole_body";
    }
    else if (planning_group == "upper_body")
    {
      return "upper_body_ik_default";
    }

    ROS_FATAL_STREAM_NAMED("hrp2_demos","Unknown planning group, no start pose found.");
    exit(-1);
  }

  bool genIKRequests(int problems, std::size_t seed)
  {
    // Get the starting pose that corresponds to selected planning group
    std::string start_pose = getStartPose(planning_group_name_);

    // Random number generator -----------------------------------------------------
    random_numbers::RandomNumberGenerator *rng;

    // Create random number generator that is stochastic
    if (seed == 0) // use random seed
    {
      rng = new random_numbers::RandomNumberGenerator();
      ROS_INFO_STREAM_NAMED("genSimpleArmIKRequets","Used random seed: " << rng->getFirstSeed() );
    }
    else
    {
      rng = new random_numbers::RandomNumberGenerator(seed);
    }

    // Get tip links for this setup
    std::vector<const robot_model::LinkModel*> tips;
    joint_model_group_->getEndEffectorTips(tips);

    std::vector<std::string> tip_names;
    joint_model_group_->getEndEffectorTips(tip_names);

    // Choose random end effector goal positions for left and right arm ------------------

    // Skip x number random numbers (for debugging)
    int skip_rands = 0; // set this to the id of the run you want to test
    problems += skip_rands; // offset problems by where we start from
    for (std::size_t i = 0; i < skip_rands; ++i)
    {
      ROS_WARN_STREAM_NAMED("hrp2_demos","Skipping position " << i);
      robot_state_->setToRandomPositions(joint_model_group_, *rng);
    }

    for (std::size_t problem_id = skip_rands; problem_id < problems; ++problem_id)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("hrp2_demos","Testing number " << problem_id+1 << " of " << problems << " ======================================");

      robot_state_->setToDefaultValues();
      goal_state_->setToDefaultValues();

      // Stochastically random goal positions
      robot_state_->setToRandomPositions(joint_model_group_, *rng);

      // hack to skip run 2 that is bad
      if (problem_id == 1 && false)
      {
        ROS_WARN_STREAM_NAMED("hrp2_demos","using skip run 2 hack");
        continue;
      }

      // Set the seed value from SRDF
      setStateToGroupPose(goal_state_, start_pose, joint_model_group_);

      // Check that the new state is valid
      robot_state_->enforceBounds();
      if (!robot_state_->satisfiesBounds(joint_model_group_))
      {
        ROS_FATAL_STREAM_NAMED("setGroupToValue","New joint values do not satisfy bounds for group " << joint_model_group_->getName());
        exit(-1);
      }

      // Debug seed joint values:
      if (true)
      {
        std::vector<double> joints(joint_model_group_->getVariableCount());
        robot_state_->copyJointGroupPositions(joint_model_group_, joints);

        if (true)
        {
          // Show desired joint values
          for (std::size_t i = 0; i < joints.size(); ++i)
          {
            std::cout << joints[i] << ", ";
          }
          std::cout << std::endl;
        }

        double epsilon = 0.05;
        for (std::size_t j = 0; j < joints.size(); ++j)
        {
          if (joints[j] < joint_model_group_->getJointModels()[j]->getVariableBounds()[0].min_position_ + epsilon)
            std::cout << " LOW " << joint_model_group_->getJointModels()[j]->getVariableBounds()[0].min_position_ << std::endl;

          if (joints[j] > joint_model_group_->getJointModels()[j]->getVariableBounds()[0].max_position_ - epsilon)
            std::cout << " HIGH " << joint_model_group_->getJointModels()[j]->getVariableBounds()[0].max_position_ << std::endl;
        }
      }

      // Visualize
      visual_tools_->publishRobotState(robot_state_);

      // Get the end effector poses
      EigenSTL::vector_Affine3d poses;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        poses.push_back( robot_state_->getGlobalLinkTransform(tips[i]) );
      }

      // Use an IK solver to find the same solution
      unsigned int attempts = 1;
      double timeout = 0; // unset

      // IK Solver
      if (!goal_state_->setFromIK(joint_model_group_, poses, tip_names, attempts, timeout))
      {
        ROS_ERROR_STREAM_NAMED("demos","setFromIK failed");
        return false;
      }

      // Error check that the values are the same
      EigenSTL::vector_Affine3d poses_new;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        poses_new.push_back( goal_state_->getGlobalLinkTransform(tips[i]) );
      }

      bool passed = true;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        if (!poseIsSimilar(poses[i], poses_new[i]))
        {
          ROS_ERROR_STREAM_NAMED("hrp2_demos","Pose not similar: " << tips[i]->getName());
          passed = false;
        }
      }

      if (!passed)
      {
        std::cout << "=========================================================== " << std::endl;
        ROS_ERROR_STREAM_NAMED("hrp2_demos","POSES ARE NOT SIMILAR, BENCHMARK FAILED on test " << problem_id);
        std::cout << "=========================================================== " << std::endl;
        return false;
      }
      else
      {
        // Show the new robot state
        //ros::Duration(0.25).sleep();
        //visual_tools_->publishRobotState(goal_state_);
      }

    } // for problems

    return true;
  }

  bool poseIsSimilar(const Eigen::Affine3d &pose1, const Eigen::Affine3d &pose2)
  {
    Eigen::Quaterniond quat1 = Eigen::Quaterniond(pose1.rotation());
    Eigen::Quaterniond quat2 = Eigen::Quaterniond(pose2.rotation());

    geometry_msgs::Pose p1 = moveit_visual_tools::MoveItVisualTools::convertPose(pose1);
    geometry_msgs::Pose p2 = moveit_visual_tools::MoveItVisualTools::convertPose(pose2);

    //double similarity_threshold = 0.01;
    if (
        abs(p1.position.x - p2.position.x) > std::numeric_limits<double>::epsilon() ||
        abs(p1.position.y - p2.position.y) > std::numeric_limits<double>::epsilon() ||
        abs(p1.position.z - p2.position.z) > std::numeric_limits<double>::epsilon() ||
        quat1.angularDistance(quat2) > 3 //std::numeric_limits<double>::epsilon()
        //      quat1.dot(quat2) < 1 - 100*std::numeric_limits<double>::epsilon() //using "dot" avoids a trigonometric function.
        )
    {
      if (abs(p1.position.x - p2.position.x) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff x: " << std::setprecision(12) << abs(p1.position.x - p2.position.x)  << std::endl;
      if (abs(p1.position.y - p2.position.y) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff y: " << std::setprecision(12) << abs(p1.position.y - p2.position.y)  << std::endl;
      if (abs(p1.position.z - p2.position.z) > std::numeric_limits<double>::epsilon())
        std::cout << "Diff z: " << std::setprecision(12) << abs(p1.position.z - p2.position.z)  << std::endl;

      if (quat1.dot(quat2) < 1 - std::numeric_limits<double>::epsilon()) //using "dot" avoids a trigonometric function.
      {
        std::cout << "Difference in angle is greater than epsilon: " << std::setprecision(12) << quat1.dot(quat2) << std::endl;
      }
      if (quat1.angularDistance(quat2) > 3 )
      {
        std::cout << "Angular Distance is too large: " << std::setprecision(12) << quat1.angularDistance(quat2) << std::endl;
      }

      std::cout << "pose1 \n" << p1 <<std::endl;
      std::cout << "pose2 \n" << p2 <<std::endl;

      return false;
    }
    return true;
  }

  void setStateXYTheta(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    x += moveit_visual_tools::MoveItVisualTools::dRand(0,0.25);
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    y += moveit_visual_tools::MoveItVisualTools::dRand(0,0.25);
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    // Rotation
    Eigen::Quaternion<float>q(
                              goal_state->getVariablePosition("virtual_joint/rot_w"),
                              goal_state->getVariablePosition("virtual_joint/rot_x"),
                              goal_state->getVariablePosition("virtual_joint/rot_y"),
                              goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::MoveItVisualTools::dRand(-20,20) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
  }

  void setStateComplex(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    //x += moveit_visual_tools::MoveItVisualTools::dRand(0,0.25);
    x = 1;
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    //y += moveit_visual_tools::MoveItVisualTools::dRand(0,0.25);
    y = 0.5;
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    /*
    // Rotation
    Eigen::Quaternion<float>q(
    goal_state->getVariablePosition("virtual_joint/rot_w"),
    goal_state->getVariablePosition("virtual_joint/rot_x"),
    goal_state->getVariablePosition("virtual_joint/rot_y"),
    goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::MoveItVisualTools::dRand(-20,20) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
    */
  }

  void setStateToGroupPose(robot_state::RobotStatePtr &state, const std::string& pose_name, const robot_model::JointModelGroup* jmg)
  {
    if (!state->setToDefaultValues(jmg, pose_name))
    {
      ROS_ERROR_STREAM_NAMED("demo","Failed to set pose '" << pose_name << "' for planning group '" << jmg->getName() << "'");
    }
  }

  void setStateInPlace(robot_state::RobotStatePtr &goal_state)
  {
    double x = goal_state->getVariablePosition("virtual_joint/trans_x");
    x = moveit_visual_tools::MoveItVisualTools::dRand(-1.0,1.0);
    goal_state->setVariablePosition("virtual_joint/trans_x",x);

    double y = goal_state->getVariablePosition("virtual_joint/trans_y");
    y = moveit_visual_tools::MoveItVisualTools::dRand(-1.0,1.0);
    goal_state->setVariablePosition("virtual_joint/trans_y",y);

    double z = goal_state->getVariablePosition("virtual_joint/trans_z");
    z = moveit_visual_tools::MoveItVisualTools::dRand(-0.38,0.0);
    goal_state->setVariablePosition("virtual_joint/trans_z",z);

    // Rotation
    Eigen::Quaternion<float>q(
                              goal_state->getVariablePosition("virtual_joint/rot_w"),
                              goal_state->getVariablePosition("virtual_joint/rot_x"),
                              goal_state->getVariablePosition("virtual_joint/rot_y"),
                              goal_state->getVariablePosition("virtual_joint/rot_z"));
    Eigen::Quaternion<float> rotate(Eigen::AngleAxis<float>(moveit_visual_tools::MoveItVisualTools::dRand(-15,15) * M_PI / 180, Eigen::Vector3f::UnitZ()));
    q = q * rotate;

    goal_state->setVariablePosition("virtual_joint/rot_x",q.x());
    goal_state->setVariablePosition("virtual_joint/rot_y",q.y());
    goal_state->setVariablePosition("virtual_joint/rot_z",q.z());
    goal_state->setVariablePosition("virtual_joint/rot_w",q.w());
  }

  void setStateCrouching(robot_state::RobotStatePtr &state)
  {
    std::vector<std::string> name(46);
    name[0] = "RLEG_JOINT0";
    name[1] = "RLEG_JOINT1";
    name[2] = "RLEG_JOINT2";
    name[3] = "RLEG_JOINT3";
    name[4] = "RLEG_JOINT4";
    name[5] = "RLEG_JOINT5";
    name[6] = "RLEG_JOINT6";
    name[7] = "LLEG_JOINT0";
    name[8] = "LLEG_JOINT1";
    name[9] = "LLEG_JOINT2";
    name[10] = "LLEG_JOINT3";
    name[11] = "LLEG_JOINT4";
    name[12] = "LLEG_JOINT5";
    name[13] = "LLEG_JOINT6";
    name[14] = "CHEST_JOINT0";
    name[15] = "CHEST_JOINT1";
    name[16] = "HEAD_JOINT0";
    name[17] = "HEAD_JOINT1";
    name[18] = "RARM_JOINT0";
    name[19] = "RARM_JOINT1";
    name[20] = "RARM_JOINT2";
    name[21] = "RARM_JOINT3";
    name[22] = "RARM_JOINT4";
    name[23] = "RARM_JOINT5";
    name[24] = "RARM_JOINT6";
    name[25] = "RARM_JOINT7";
    name[26] = "LARM_JOINT0";
    name[27] = "LARM_JOINT1";
    name[28] = "LARM_JOINT2";
    name[29] = "LARM_JOINT3";
    name[30] = "LARM_JOINT4";
    name[31] = "LARM_JOINT5";
    name[32] = "LARM_JOINT6";
    name[33] = "LARM_JOINT7";
    name[34] = "R_THUMBCM_Y";
    name[35] = "R_THUMBCM_P";
    name[36] = "R_INDEXMP_R";
    name[37] = "R_INDEXMP_P";
    name[38] = "R_INDEXPIP_R";
    name[39] = "R_MIDDLEPIP_R";
    name[40] = "L_THUMBCM_Y";
    name[41] = "L_THUMBCM_P";
    name[42] = "L_INDEXMP_R";
    name[43] = "L_INDEXMP_P";
    name[44] = "L_INDEXPIP_R";
    name[45] = "L_MIDDLEPIP_R";
    std::vector<double> position(46);
    position[0] = -0.00946599;
    position[1] = -0.303998;
    position[2] = -1.40749;
    position[3] = 2.30053;
    position[4] = -0.907706;
    position[5] = 0.169391;
    position[6] = 0;
    position[7] = -0.00928688;
    position[8] = -0.292467;
    position[9] = -1.3977;
    position[10] = 2.28464;
    position[11] = -0.901551;
    position[12] = 0.157853;
    position[13] = 0;
    position[14] = -0.0752511;
    position[15] = 0.952245;
    position[16] = 0;
    position[17] = 0;
    position[18] = -0.662891;
    position[19] = -0.227223;
    position[20] = 0.413513;
    position[21] = -0.41508;
    position[22] = -0.386773;
    position[23] = -0.0795726;
    position[24] = 0.0724259;
    position[25] = 0;
    position[26] = 0;
    position[27] = 0;
    position[28] = 0;
    position[29] = 0;
    position[30] = 0;
    position[31] = 0;
    position[32] = 0;
    position[33] = 0;
    position[34] = 0;
    position[35] = 0;
    position[36] = 0;
    position[37] = 0;
    position[38] = 0;
    position[39] = 0;
    position[40] = 0;
    position[41] = 0;
    position[42] = 0;
    position[43] = 0;
    position[44] = 0;
    position[45] = 0;

    // Save to goal state
    state->setVariablePositions(name,position);

    // Multi-dof joints:
    state->setVariablePosition("virtual_joint/trans_x", -0.0596537);
    state->setVariablePosition("virtual_joint/trans_y", 0.0392288);
    state->setVariablePosition("virtual_joint/trans_z", -0.363454);
    state->setVariablePosition("virtual_joint/rot_x", 0.0671162);
    state->setVariablePosition("virtual_joint/rot_y", 0.00640489);
    state->setVariablePosition("virtual_joint/rot_z", 0.00298931);
    state->setVariablePosition("virtual_joint/rot_w", 0.99772);
  }

  bool createPlanCrouch(moveit_msgs::MotionPlanResponse &response)
  {
    // Create motion planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // Workspace
    req.workspace_parameters.header.frame_id = "/odom";
    // relative to center of robot
    req.workspace_parameters.min_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") - 1;
    req.workspace_parameters.min_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") - 1;
    req.workspace_parameters.min_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") - 1;
    req.workspace_parameters.max_corner.x = robot_state_->getVariablePosition("virtual_joint/trans_x") + 1;
    req.workspace_parameters.max_corner.y = robot_state_->getVariablePosition("virtual_joint/trans_y") + 1;
    req.workspace_parameters.max_corner.z = robot_state_->getVariablePosition("virtual_joint/trans_z") + 1;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

    // Path constraint - Keep the body link from rotating too much
    /*
      moveit_msgs::PositionConstraint pc1;
      pc1.link_name = "LLEG_LINK5";
      shape_msgs::SolidPrimitive primitive;
      primitive.type = shape_msgs::SolidPrimitive::BOX;
      primitive.dimensions.push_back(1); // x
      primitive.dimensions.push_back(1); // y
      primitive.dimensions.push_back(0.1); // z
      moveit_msgs::BoundingVolume bv1;
      bv1.primitives.push_back(primitive);
      geometry_msgs::Pose origin;
      bv1.primitive_poses.push_back(origin);
      pc1.constraint_region = bv1;
      pc1.weight = 1;
      req.path_constraints.position_constraints.push_back(pc1);
    */

    // Goal constraint
    double tolerance_pose = 0.0001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal_state_, joint_model_group_, tolerance_pose, tolerance_pose);
    req.goal_constraints.push_back(goal_constraint);

    // Other settings
    //req.planner_id = "RRTkConfigDefault"; // Regular RRT
    req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = planning_group_name_;
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 30;

    // Call pipeline
    loadPlanningPipeline(); // always call before using generatePlan()
    planning_pipeline_->generatePlan(planning_scene_, req, res);

    // Check that the planning was successful
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =======================================================");
      return false;
    }

    response.trajectory = moveit_msgs::RobotTrajectory();
    res.getMessage(response);
    return true;
  }

  void printVirtualJointPosition(const robot_state::RobotStatePtr &robot_state)
  {
    ROS_INFO_STREAM_NAMED("hrp2_demos","Virtual Joint Positions:");
    const double* positions = robot_state->getJointPositions("virtual_joint");
    std::cout << "Position: " << std::endl;
    std::cout << "X: " << positions[0] << std::endl;
    std::cout << "Y: " << positions[1] << std::endl;
    std::cout << "Z: " << positions[2] << std::endl;
    std::cout << "Quaternion: " << std::endl;
    std::cout << "X: " << positions[3] << std::endl;
    std::cout << "Y: " << positions[4] << std::endl;
    std::cout << "Z: " << positions[5] << std::endl;
    std::cout << "W: " << positions[6] << std::endl;
  }

  bool loadConstraintSampler(bool verbose)
  {
    // Create a constraint sampler for random poses
    moveit_msgs::Constraints constr;
    constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
    constraint_sampler_manager_loader_.reset(new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader());
    constraint_samplers::ConstraintSamplerManagerPtr csm = constraint_sampler_manager_loader_->getConstraintSamplerManager();
    constraint_sampler_ = csm->selectSampler(planning_scene_, planning_group_name_, constr);
    constraint_sampler_->setVerbose(verbose);

    if (!constraint_sampler_)
    {
      ROS_FATAL_STREAM_NAMED("hrp2_demos","No constraint sampler loaded");
      return false;
    }
    ROS_INFO_STREAM_NAMED("hrp2_demos","Chosen constraint sampler: " << constraint_sampler_->getName() );

    return true;
  }

}; // class

} // namespace

int main(int argc, char **argv)
{
  // initialize random seed:
  //srand (time(NULL));

  ros::init (argc, argv, "hrp2_demos");
  ROS_INFO_STREAM_NAMED("main","Starting HRP2 Demos");

  // Needed for ROS_INFO commands to work
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Parse command line arguments
  int mode = 1;
  int runs = 1; // how many times to run the same problem
  int problems = 1; // how many problems to solve
  bool verbose = false;
  bool use_experience = true;
  bool use_collisions = false;
  bool use_thunder = true;
  bool variable_obstacles = false;
  bool random_start = false;
  std::string planning_group_name = "whole_body";
  std::size_t seed = 0;
  double x = 0;
  double y = 0;

  for (std::size_t i = 0; i < argc; ++i)
  {
    if( std::string(argv[i]).compare("--mode") == 0 )
    {
      ++i;
      mode = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","In mode " << mode);
    }

    if( std::string(argv[i]).compare("--verbose") == 0 )
    {
      ++i;
      verbose = atoi(argv[i]); // converts to int
      if (verbose)
        ROS_INFO_STREAM_NAMED("main","Verbose is true");
    }

    if( std::string(argv[i]).compare("--runs") == 0 )
    {
      ++i;
      runs = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Performing " << runs << " runs");
    }

    if( std::string(argv[i]).compare("--problems") == 0 )
    {
      ++i;
      problems = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Performing " << problems << " problems");
    }

    if( std::string(argv[i]).compare("--seed") == 0 )
    {
      ++i;
      seed = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using seed " << seed);
    }

    if( std::string(argv[i]).compare("--group") == 0 )
    {
      ++i;
      planning_group_name = argv[i];
      ROS_INFO_STREAM_NAMED("main","Using planning group " << planning_group_name);
    }

    if( std::string(argv[i]).compare("--use_experience") == 0 )
    {
      ++i;
      use_experience = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using experience: " << use_experience);
    }

    if( std::string(argv[i]).compare("--use_collisions") == 0 )
    {
      ++i;
      use_collisions = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using collisions: " << use_collisions);
    }

    if( std::string(argv[i]).compare("--use_thunder") == 0 )
    {
      ++i;
      use_thunder = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using Thunder (vs. Lightning): " << use_thunder);
    }

    if( std::string(argv[i]).compare("--random_start") == 0 )
    {
      ++i;
      random_start = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using random start state: " << random_start);
    }

    if( std::string(argv[i]).compare("--x") == 0 )
    {
      ++i;
      x = atof(argv[i]);
      ROS_INFO_STREAM_NAMED("main","x value " << x);
    }

    if( std::string(argv[i]).compare("--y") == 0 )
    {
      ++i;
      y = atof(argv[i]);
      ROS_INFO_STREAM_NAMED("main","y value " << y);
    }

    if( std::string(argv[i]).compare("--variable_obstacles") == 0 )
    {
      ++i;
      variable_obstacles = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","variable_obstacles: " << variable_obstacles);
    }
  }
  hrp2jsknt_moveit_demos::HRP2Demos client(planning_group_name);
  client.setUseThunder(use_thunder);


  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("hrp2_demos","---------------------------------------------------------------------------------------");
  switch (mode)
  {
    case 1:
      ROS_INFO_STREAM_NAMED("hrp2_demos","1 - Whole body planning with MoveIt!");
      client.genRandWholeBodyPlans(problems, verbose, use_experience, use_collisions, variable_obstacles, random_start);
      break;
    case 2:
      ROS_INFO_STREAM_NAMED("hrp2_demos","2 - Show the experience database visually in Rviz");
      client.displayDBPlans(problems, verbose);
      break;
    case 3:
      ROS_ERROR_STREAM_NAMED("temp","No option");
      break;
    case 4:
      ROS_ERROR_STREAM_NAMED("temp","No option");
      break;
    case 5:
      ROS_ERROR_STREAM_NAMED("temp","No option");
      break;
    case 6:
      ROS_INFO_STREAM_NAMED("hrp2_demos","6 - Generate random positions and plan to them with MoveIt");
      client.genRandMoveItPlan();
      break;
    case 7:
      ROS_ERROR_STREAM_NAMED("temp","No option");
      break;
    case 8:
      ROS_INFO_STREAM_NAMED("hrp2_demos","8 - Test single arm planning on HRP2 using MoveIt Whole Body IK solver");
      client.genIKRequestsSweep(runs, problems, seed);
      break;
    default:
      ROS_WARN_STREAM_NAMED("hrp2_demos","Unkown mode: " << mode);
  }


  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();
  return 0;
}
