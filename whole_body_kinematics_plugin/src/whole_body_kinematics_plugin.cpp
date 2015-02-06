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
 */


#include <moveit/whole_body_kinematics_plugin/whole_body_kinematics_plugin.h>
#include <class_loader/class_loader.h>

#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/macros/console_colors.h>

#include <boost/format.hpp>

#include <moveit/whole_body_kinematics_plugin/kdl/utilities/svd_HH.hpp> // temp

//register as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(whole_body_kinematics_plugin::WholeBodyKinematicsPlugin, kinematics::KinematicsBase)

namespace whole_body_kinematics_plugin
{

WholeBodyKinematicsPlugin::WholeBodyKinematicsPlugin()
  : verbose_(false), // set this via rosparam
    debug_mode_(false), // set this via rosparam
    nh_("~"),
    joint_limit_offset_(0.001)
{}

void WholeBodyKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  robot_state_->setToRandomPositions(joint_model_group_, *rng_);
  robot_state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);

  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array(i) = jnt_array_vector[i];
  }
}

void WholeBodyKinematicsPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
                                                       const std::vector<double> &consistency_limits,
                                                       KDL::JntArray &jnt_array) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0 ; i < dimension_; ++i)
    near[i] = seed_state(i);

  joint_model_group_->getVariableRandomPositionsNearBy(robot_state_->getRandomNumberGenerator(), values, near, consistency_limits);

  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array(i) = values[i];
  }

  ROS_ERROR_STREAM_NAMED("temp","getRandomConfiguration with consistency limits called!");
}

bool WholeBodyKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                                 const std::vector<double> &consistency_limits,
                                                 const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool WholeBodyKinematicsPlugin::initialize(const std::string &robot_description,
                                           const std::string& group_name,
                                           const std::string& base_frame,
                                           const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  // Call KinematicsBase parent class
  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  //bool base_frame_temp = base_frame;

  // Get solver parameters from param server
  //ROS_DEBUG_STREAM_NAMED("initialize","Looking for IK settings on rosparam server at: " << nh_.getNamespace() << "/" << group_name_ << "/");
  nh_.param(group_name_ + "/kinematics_solver_max_solver_iterations", max_solver_iterations_, 500);
  nh_.param(group_name_ + "/kinematics_solver_epsilon", epsilon_, 1e-5);
  nh_.param(group_name_ + "/kinematics_solver_null_space_epsilon", null_space_epsilon_, 1e-3);
  nh_.param(group_name_ + "/kinematics_solver_ee_pos_vel_limit", ee_pos_vel_limit_, 0.1);
  nh_.param(group_name_ + "/kinematics_solver_ee_rot_vel_limit", ee_rot_vel_limit_, 1.5);
  nh_.param(group_name_ + "/kinematics_solver_null_space_vel_gain", null_space_vel_gain_, 0.001);
  nh_.param(group_name_ + "/kinematics_solver_joint_velocity_max_ratio", joint_velocity_max_ratio_, 0.1);

  nh_.param(group_name_ + "/kinematics_solver_verbose", verbose_, false);
  nh_.param(group_name_ + "/kinematics_solver_debug_mode", debug_mode_, false);
  nh_.param(group_name_ + "/kinematics_solver_visualize_search", visualize_search_, false);
  //nh_.param(group_name_ + "/kinematics_solver_base_frame", base_frame_temp, false);


  // Load URDF and SRDF --------------------------------------------------------------------
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("whole_body_ik","URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }
  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  // Load the joint model group ------------------------------------------------------------------------
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  // If jmg is a chain, it can only have one tip
  if(joint_model_group_->isChain() && tip_frames.size() > 1)
  {
    ROS_ERROR_STREAM_NAMED("temp","The joint model group specified is a chain but " << tip_frames.size() << " tip frames and poses were passed in");
    return false;
  }
  if(!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("whole_body_ik","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  // Get the num of dimensions
  ROS_DEBUG_STREAM_NAMED("whole_body_ik","Found " << joint_model_group_->getActiveJointModels().size() << " active joints and "
                        << joint_model_group_->getMimicJointModels().size() << " mimic joints");

  dimension_ = joint_model_group_->getActiveJointModels().size();

  // Debug joints
  if (verbose_)
  {
    std::cout << std::endl << "Joint Model Variable Names: ------------------------------------------- " << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Random number generator
  if (true)
  {
    ROS_WARN_STREAM_NAMED("whole_body_ik","Using stochastic random joint selection");
    rng_ = new random_numbers::RandomNumberGenerator(27349872); // stoachastic behavior
  }
  else
  {
    rng_ = new random_numbers::RandomNumberGenerator();
  }

  // View tip frames
  if (verbose_)
  {
    std::cout << std::endl << "Tip Link Names: ------------------------------------------- " << std::endl;
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Copy joint names and limits
  for (std::size_t i=0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    // Filter out non revolute or prismatic joint
    if(joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
       joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
      ik_group_info_.limits.insert(ik_group_info_.limits.end(), jvec.begin(), jvec.end());
    }
    else
    {
      
      ROS_ERROR_STREAM_NAMED("whole_body_ik","This is the Whole Body kinematics plugin, and it does not support mimic/redundant/non-revolute or prismatic joints. Bad join name: " << joint_model_group_->getJointModelNames()[i];);
    }
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if(!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("srv","Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(), group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Populate the joint limits
  joint_min_.resize(ik_group_info_.limits.size());
  joint_max_.resize(ik_group_info_.limits.size());
  joint_vel_max_.resize(ik_group_info_.limits.size());
  for(unsigned int i=0; i < ik_group_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_group_info_.limits[i].min_position;
    joint_max_(i) = ik_group_info_.limits[i].max_position;
    // Used for limiting a joint's velocity
    joint_vel_max_(i) = fabs(joint_max_(i) - joint_min_(i)) * joint_velocity_max_ratio_;
  }

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // Load the Robot Viz Tools for publishing to Rviz
  if (visualize_search_)
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom","/hrp2_visual_markers", robot_model_));
    visual_tools_->loadRobotStatePub("/moveit_whole_body_ik");
  }

  // Load the datastructures into memory needed for solving
  ctj_data_.reset(new CartesionToJointData(dimension_, tip_frames.size()));

  // the weights applied in the joint space
  // i think this decides how much power the 'opt_positions' (above) has on the overall velocity.
  KDL::JntArray weights(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    weights(i) = 0.45;

  // if a singular value is below this value, its inverse is set to zero, default: 0.00001
  double eps=0.00001;
  // maximum iterations for the svd calculation, default: 150
  int maxiter=150;

  // Load the jacobian generator
  jacobian_generator_.reset(new JacobianGenerator(verbose_));
  if (!jacobian_generator_->initialize(urdf_model, robot_model_, tip_frames_, joint_model_group_))
  {
    ROS_ERROR_STREAM_NAMED("whole_body_ik","Failed to convert URDF to KDL Chains and setup jacobians");
    return false;
  }

  // inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity
  ik_solver_vel_.reset(new IkSolverPinverse(tip_frames.size(), dimension_, joint_min_, joint_max_,
                                            weights, ctj_data_->jacobian_, eps, maxiter, verbose_));

  ROS_DEBUG_NAMED("whole_body_ik","MoveIt! Whole Body IK solver initialized");
  return true;
}

int WholeBodyKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_group_info_.joint_names.size(); i++) {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int WholeBodyKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  /*
    int i=0;
    while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
    return i+1;
    }
    i++;
    }
    return -1;
  */
}

bool WholeBodyKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool WholeBodyKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options,
                                                 const moveit::core::RobotState* context_state) const
{
  //ROS_INFO_STREAM_NAMED("searchPositionIK","Starting MoveIt Whole Body IK Solver --------------------------------------");

  ros::WallTime n1 = ros::WallTime::now();

  // Check if seed state correct
  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("searchPositionIK","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check consistency limits
  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("searchPositionIK","Consistency limits be empty or must have size " << dimension_
                           << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("searchPositionIK","Mismatched number of pose requests (" << ik_poses.size()
                           << ") to tip frames (" << tip_frames_.size() << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (verbose_)
  {
    for (std::size_t i = 0; i < ik_poses.size(); ++i)
    {
      std::cout << "Pose " << i << ": \n" << ik_poses[i] << std::endl;
    }
  }

  // Joint arrays
  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);

  // Setup solution vector
  solution.resize(dimension_);

  // Convert format of pose
  std::vector<KDL::Frame> kdl_poses;
  for (std::size_t i = 0; i < ik_poses.size(); ++i)
  {
    KDL::Frame temp;
    tf::poseMsgToKDL(ik_poses[i], temp);
    kdl_poses.push_back(temp);
  }

  // Copy the seed state
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_seed_state(i) = ik_seed_state[i];
  }
  if (verbose_)
  {
    std::cout << "Seed state: ";
    std::copy(ik_seed_state.begin(), ik_seed_state.end(), std::ostream_iterator<double>(std::cout, ", "));
    std::cout << std::endl;
  }

  jnt_pos_in = jnt_seed_state;

  // Loop until solution is within epsilon
  unsigned int counter(0);
  std::size_t total_loops = 0; // track number of jacobians generated and qdot_outs calculated
  static int total_iterations = 0;

  while(true)
  {
    if (verbose_ || true)
      ROS_DEBUG_NAMED("searchPositionIK","Iteration: %d, time: %f, Timeout: %f", int(total_loops), (ros::WallTime::now()-n1).toSec(),timeout);
    counter++;

    // Check if timed out
    if(timedOut(n1,timeout))
    {
      ROS_DEBUG_NAMED("searchPositionIK","IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;
    }

    // Update random values unless we are on the first iteration
    if (counter > 1)
    {
      if(!consistency_limits.empty())
      {
        // Look for random joint values within consistency limits of the seed state
        getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_in);
      }
      else
      {
        // Look for any random joint values
        getRandomConfiguration(jnt_pos_in);
      }
      if (verbose_)
      {
        ROS_DEBUG_NAMED("searchPositionIK","New random configuration");
        for(unsigned int j=0; j < dimension_; j++)
          ROS_DEBUG_NAMED("searchPositionIK","%d %f", j, jnt_pos_in(j));
      }
    }

    // Solve
    int ik_valid = newtonRaphsonIterator(jnt_pos_in, kdl_poses, jnt_pos_out, total_loops);

    // Check if ROS is ok
    if (ik_valid == -5)
    {
      ROS_ERROR_STREAM_NAMED("searchPositionIK","ROS requested shutdown");
      return -1;
    }

    // Check solution
    if( !consistency_limits.empty() &&
        ( (ik_valid < 0 && !options.return_approximate_solution) || !checkConsistency(jnt_seed_state, consistency_limits, jnt_pos_out)) )
    {
      if (verbose_)
        ROS_DEBUG_NAMED("searchPositionIK","Could not find IK solution: does not match consistency limits");
      continue;
    }
    else if(ik_valid < 0 && !options.return_approximate_solution)
    {
      if (verbose_)
        ROS_DEBUG_NAMED("searchPositionIK","Could not find IK solution");
      continue;
    }

    // Convert solution to MoveIt format
    for(unsigned int j=0; j < dimension_; j++)
      solution[j] = jnt_pos_out(j);

    // Optionally check again with custom callback
    if(!solution_callback.empty())
    {
      geometry_msgs::Pose dummy; // do emphasize that this pose is not used
      solution_callback(dummy, solution, error_code);
    }
    else
      error_code.val = error_code.SUCCESS;

    total_iterations += total_loops;

    // Check if callback was sucessful
    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("searchPositionIK","Solved after " << total_loops << " iterations, with total iterations for all IK calls: " << total_iterations);

      if (visualize_search_)
      {
        robot_state_->setJointGroupPositions(joint_model_group_, solution);
        visual_tools_->publishRobotState(robot_state_);
        ros::Duration(0.1).sleep();
      }

      return true;
    }

  }
  // should never get here
}

/* \brief Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the
 *  position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account. */
int WholeBodyKinematicsPlugin::newtonRaphsonIterator(const KDL::JntArray& q_init, const std::vector<KDL::Frame>& kdl_poses, KDL::JntArray& q_out, std::size_t &total_loops) const
{
  // First solution guess is the seed state
  q_out = q_init;

  // TEMP VARS:
  bool debug_would_have_stopped = false; // used for testing if we stopped to soon

  // Actualy requried vars
  bool all_poses_valid; // track if any pose is still not within epsilon distance to its goal
  bool all_poses_almost_valid; // track if our poses are close enought that we can remove the null space component

  // Set the performance criterion back to zero
  // TODO is this a valid assumption?
  for (std::size_t i = 0; i <   ctj_data_->prev_H_.rows(); ++i)
  {
    ctj_data_->prev_H_(i) = 0.0;
  }

  // Make a writeable copy so that we can reduce this gain as iterations continue
  double null_space_vel_gain = null_space_vel_gain_;

  // Start main loop
  std::size_t solver_iteration; // Iterate on approximate guess of joint values 'q_out'
  for (  solver_iteration = 0; solver_iteration < max_solver_iterations_; solver_iteration++ )
  {
    total_loops ++;

    // To match euslisp's debug mode
    if (debug_mode_)
    {
      std::cout << "------------------------------------------------------------" << std::endl;
      std::cout << "loop:   " << solver_iteration << " of " << max_solver_iterations_ << std::endl;
    }

    if (!ros::ok())
    {
      ROS_ERROR_STREAM_NAMED("newtonRaphsonIterator","ROS requested shutdown");
      return -5;
    }

    all_poses_valid = true;
    all_poses_almost_valid = true;

    // Send to MoveIt's robot_state
    robot_state_->setJointGroupPositions(joint_model_group_, q_out.data);

    // Visualize progress
    if (visualize_search_)
    {
      // Publish
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(2.1).sleep();
    }

    // For each end effector
    for (std::size_t pose_id = 0; pose_id < kdl_poses.size(); ++pose_id)
    {
      // Do forward kinematics to get new EE pose location
      Eigen::Affine3d eef_pose = robot_state_->getGlobalLinkTransform(tip_frames_[pose_id]);

      // Bring the pose to the frame of the IK solver TODO this isn't necessary if base frame is same as global link transform frame
      robot_state_->setToIKSolverFrame( eef_pose, getBaseFrame() );

      // Convert Eigen::Affine3d to KDL::Frame
      poseEigenToKDL(eef_pose, ctj_data_->current_pose_);

      // Calculate the difference between our desired pose and current pose
      ctj_data_->delta_twist_ = diff(ctj_data_->current_pose_, kdl_poses[pose_id]);   // v_in = actual - target

      // Check if the difference between our desired pose and current pose is within epsilon tolerance
      if (!Equal(ctj_data_->delta_twist_, KDL::Twist::Zero(), epsilon_))
        all_poses_valid = false;

      // Check if the difference between our desired pose and current pose is within our secondary tolerance
      if (!Equal(ctj_data_->delta_twist_, KDL::Twist::Zero(), null_space_epsilon_))
        all_poses_almost_valid = false;

      // Limit the end effector positional velocity
      KDL::Vector &pos_vel = ctj_data_->delta_twist_.vel;

      // Debug record
      if (debug_mode_)
      {
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 0 ) = pos_vel.x();
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 1 ) = pos_vel.y();
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 2 ) = pos_vel.z();
      }

      if (true)
      {
        //pos_vel.print();
        if (pos_vel.x() > ee_pos_vel_limit_ || pos_vel.y() > ee_pos_vel_limit_ || pos_vel.z() > ee_pos_vel_limit_)
        {
          // Normalize the vector
          //std::cout << "Normalizing: " << std::endl;
          pos_vel.Normalize();
          //pos_vel.print();
          pos_vel = pos_vel * ee_pos_vel_limit_;

        }
        //pos_vel.print();
      }

      // Limit the end effector rotational velocity
      KDL::Vector &rot_vel = ctj_data_->delta_twist_.rot;

      // Debug record
      if (debug_mode_)
      {
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 3 ) = rot_vel.x();
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 4 ) = rot_vel.y();
        ctj_data_->delta_twists_debug_( TWIST_SIZE * pose_id + 5 ) = rot_vel.z();
      }

      if (true)
      {
        //rot_vel.print();
        if (rot_vel.x() > ee_rot_vel_limit_ || rot_vel.y() > ee_rot_vel_limit_ || rot_vel.z() > ee_rot_vel_limit_)
        {
          // Normalize the vector
          //std::cout << "Normalizing: " << std::endl;
          rot_vel.Normalize();
          //rot_vel.print();
          rot_vel = rot_vel * ee_rot_vel_limit_;
        }
        //rot_vel.print();
      }

      // Add this twist to our large twist vector from multiple end effectors
      for (std::size_t twist_index = 0; twist_index < TWIST_SIZE; ++twist_index)
      {
        //std::cout << " >>> Twist value is: " << ctj_data_->delta_twist_( twist_index ) << " being placed in index "
        // << TWIST_SIZE * pose_id + twist_index << std::endl;
        ctj_data_->delta_twists_( TWIST_SIZE * pose_id + twist_index ) = ctj_data_->delta_twist_( twist_index );
      }
    }

    // See twist delta
    if (debug_mode_)
    {
      // Show End effector name
      std::cout << "eef     :      ";
      for (std::size_t pose_id = 0; pose_id < kdl_poses.size(); ++pose_id)
      {
        std::cout << boost::format("%-60s") % tip_frames_[pose_id];
      }
      std::cout << std::endl;

      // Input Velocity Limit Usagae
      std::cout << "xdot_orig: " << MOVEIT_CONSOLE_COLOR_RED;
      for (std::size_t i = 0; i < ctj_data_->delta_twists_.rows(); ++i)
      {
        if (ctj_data_->delta_twists_debug_(i) != ctj_data_->delta_twists_(i))
          std::cout << boost::format("%10.4f") % ctj_data_->delta_twists_debug_(i);
        else
          std::cout << boost::format("%+10s") % " ";
      }
      std::cout << std::endl << MOVEIT_CONSOLE_COLOR_RESET;

      // Input Velocity
      std::cout << "xdot_in  : ";
      for (std::size_t i = 0; i < ctj_data_->delta_twists_.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % ctj_data_->delta_twists_(i);
      }
      std::cout << std::endl;

      // Joint names
      std::cout << "joint    :     ";
      for (std::size_t i = 0; i < dimension_; i+=2)
      {
        std::cout << boost::format("%-20s") % joint_model_group_->getVariableNames()[i];
      }
      std::cout << std::endl;

      std::cout << "                         ";
      for (std::size_t i = 1; i < dimension_; i+=2)
      {
        std::cout << boost::format("%-20s") % joint_model_group_->getVariableNames()[i];
      }
      std::cout << std::endl;

      // Min Joint Value
      std::cout << "  jnt min: " ;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % (joint_min_(i)); // * 57.2957795);
      }
      //std::cout << "   (degrees)" << std::endl;
      std::cout << std::endl;

      // Angle
      std::cout << "jnt angle: " ;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % (q_out(i)); // * 57.2957795);
      }
      //std::cout << "   (degrees)" << std::endl;
      std::cout << std::endl;

      // Max Joint Value
      std::cout << "  jnt max: " ;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % (joint_max_(i)); // * 57.2957795);
      }
      //std::cout << "   (degrees)" << std::endl;
      std::cout << std::endl;

      // Percent close to limits
      std::cout << "limit %  : " ;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        double value = fabs(q_out(i) - joint_min_(i)) / fabs(joint_max_(i) - joint_min_(i)) * 100;

        // Show numbers red if too close to joint limits
        if (value < 0.2 || value > 99.8)
          std::cout << MOVEIT_CONSOLE_COLOR_RED;
        else
          std::cout << MOVEIT_CONSOLE_COLOR_GREEN;

        std::cout << boost::format("%10.4f") % value;

        std::cout << MOVEIT_CONSOLE_COLOR_RESET;
      }
      std::cout << "   (percent)" << std::endl;

      double desired[16] = {-0.100539, 0.122703, -3.03299, 1.5326, 0.159486, -0.0918993, -0.239314, -0.0489768, -0.255731, -1.79893, -1.05269, -1.10977, -1.89462, 0.735812, 0.383027, -1.22049};

      // Desired percent
      std::cout << "desired %: " ;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        double value = fabs(desired[i] - joint_min_(i)) / fabs(joint_max_(i) - joint_min_(i)) * 100;

        // Show numbers red if too close to joint limits
        if (value < 0.2 || value > 99.8)
          std::cout << MOVEIT_CONSOLE_COLOR_RED;
        else
          std::cout << MOVEIT_CONSOLE_COLOR_GREEN;

        std::cout << boost::format("%10.4f") % value;

        std::cout << MOVEIT_CONSOLE_COLOR_RESET;
      }
      std::cout << "   (percent)" << std::endl;

      // User Weight
      //std::cout << "usrwei:    " << std::endl;
    }

    // Check if we are done
    if (all_poses_valid)
    {
      if (verbose_)
        ROS_DEBUG_STREAM_NAMED("newtonRaphsonIterator","All of our end effectors are withing epsilon tolerance of goal location");

      if (debug_would_have_stopped)
      {
        ROS_ERROR_STREAM_NAMED("newtonRaphsonIterator","FOUND AN INSTANCE WERE WE WOULD HAVE GIVEN UP TOO EARLY!");
        exit(-1);
      }

      if (verbose_)
      {
        std::cout << "FINAL SOLUTIONS: ";
        for (std::size_t i = 0; i < q_out.rows(); ++i)
          std::cout << q_out(i) << ", ";
        std::cout << std::endl;
      }

      return 1;
    }

    // Check if we can reduce the influence of the null space
    if (all_poses_almost_valid)
    {
      if (debug_mode_)
        ROS_ERROR_STREAM_NAMED("temp","Within null space epsilon!");
      null_space_vel_gain *= 0.05;
    }


    //Calculate the jacobian "jac" the current joint positions in robot_state
    if( !jacobian_generator_->generateJacobian(robot_state_, ctj_data_->jacobian_) )
    {
      ROS_ERROR_STREAM_NAMED("whole_body_ik","Failed to generate jacobian");
      return -3;
    }

    // Run velocity solver - qdot is returned as the joint velocities (delta q)
    // (change in joint value guess)
    if (ik_solver_vel_->cartesianToJoint(q_out, ctj_data_->delta_twists_, ctj_data_->jacobian_,
                                         ctj_data_->qdot_, ctj_data_->prev_H_, debug_mode_, solver_iteration == 0,
                                         null_space_vel_gain) != 1)
    {
      ROS_ERROR_STREAM_NAMED("newtonRaphsonIterator","Error in ik solver pinverse");
      return -1;
    }

    // Check for stagnation in qdot every 4 iterations
    if (solver_iteration % 4 == 0 && debug_would_have_stopped == false)
    {
      bool has_change = false;
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        // Store in temp so we only have to do absolute value once
        double temp = ctj_data_->qdot_(i);
        if (temp == 0) // take absolute value of zero only
          temp = fabs(temp);

        // Only check if we haven't already found a non
        //        double eps = 1e-10;
        if (!has_change && // stop checking if we've already proved it has changed
            (temp > ctj_data_->qdot_cache_(i) + epsilon_ * 0.01 ||
             temp < ctj_data_->qdot_cache_(i) - epsilon_ * 0.01 ))
          has_change = true;

        ctj_data_->qdot_cache_(i) = temp;
      }

      if (!has_change)
      {
        if (debug_mode_)
        {
          ROS_ERROR_STREAM_NAMED("newtonRaphsonIterator","Giving up because no change detected");
        }
        debug_would_have_stopped = true;
        break; // disable this to keep going
      }
    }

    // Error Check - TODO remove this check
    if (true)
    {
      for (std::size_t i = 0; i < ctj_data_->qdot_.rows(); ++i)
      {
        if (isnan(ctj_data_->qdot_(i)))
        {
          std::cout << "FOUND NAN " << std::endl;
          ctj_data_->qdot_.print();
          break;
        }
      }
    }

    // Show joint velocity limit
    if (debug_mode_)
    {
      std::cout << "qdot_limt: ";
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % joint_vel_max_(i);
      }
      std::cout << std::endl;
    }

    // Limit the joint velocity qdot_out
    double qdot_diff_;
    double qdot_diff_max_ = 0;
    for (std::size_t i = 0; i < q_out.rows(); ++i)
    {
      if (fabs(ctj_data_->qdot_(i) / joint_vel_max_(i)) > qdot_diff_max_)
      {
        qdot_diff_max_ = fabs(ctj_data_->qdot_(i) / joint_vel_max_(i));
      }
    }

    if (debug_mode_)
      std::cout << "qdot_new : " << MOVEIT_CONSOLE_COLOR_RED;
    if (qdot_diff_max_ >= 1) // only apply limit if at least one joint is over its limit
    {
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        ctj_data_->qdot_(i) /= qdot_diff_max_;
        if (debug_mode_)
          std::cout << boost::format("%10.4f") % ctj_data_->qdot_(i);
      }
    }
    if (debug_mode_)
      std::cout << std::endl << MOVEIT_CONSOLE_COLOR_RESET;



    // Add current guess 'q_out' with our new change in guess qdot (delta q)
    // q_out = q_out + q_dot
    // Add(q_out, ctj_data_->qdot_, q_out);
    q_out.data = q_out.data + ctj_data_->qdot_.data;



    // Enforce joint limits
    if (debug_mode_)
      std::cout << "limit enf: " << MOVEIT_CONSOLE_COLOR_RED;
    for (unsigned int j = 0; j < q_out.rows(); j++)
    {
      if (q_out(j) < joint_min_(j))
      {
        if (debug_mode_)
          std::cout << boost::format("%+10s") % "min";

        q_out(j) = joint_min_(j) + joint_limit_offset_;
      }
      else if (q_out(j) > joint_max_(j))
      {
        if (debug_mode_)
          std::cout << boost::format("%+10s") % "max";
        q_out(j) = joint_max_(j) - joint_limit_offset_;
      }
      else if (debug_mode_)
        std::cout << boost::format("%+10s") % " ";
    }
    if (debug_mode_)
      std::cout << std::endl << MOVEIT_CONSOLE_COLOR_RESET;


    // TEMP TODO
    //if (solver_iteration > 2 )
    //  exit(0);
  }

  // We never found a close enough solution
  return -3;
}

bool WholeBodyKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                              const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::Pose> &poses) const
{
  /*
    ros::WallTime n1 = ros::WallTime::now();

    poses.resize(link_names.size());
    if(joint_angles.size() != dimension_)
    {
    ROS_ERROR_NAMED("whole_body_ik","Joint angles vector must have size: %d",dimension_);
    return false;
    }

    KDL::Frame f_out;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    KDL::JntArray jnt_pos_in(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
    {
    jnt_pos_in(i) = joint_angles[i];
    }

  */
  ROS_ERROR_STREAM_NAMED("temp","Currently not implemented");
  return false;
}

const bool WholeBodyKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
                                                    std::string* error_text_out) const
{
  return true;
}

// Convert Eigen::Affine3d to KDL::Frame
void WholeBodyKinematicsPlugin::poseEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k) const
{
  // Set position
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e.matrix()(i,3);

  // Set orientation
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e.matrix()(i/3,i%3);
}



} // namespace

