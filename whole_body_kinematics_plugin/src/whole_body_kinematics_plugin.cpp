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

#include <boost/format.hpp>

#include <moveit/whole_body_kinematics_plugin/kdl/utilities/svd_HH.hpp> // temp

//register as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(whole_body_kinematics_plugin::WholeBodyKinematicsPlugin, kinematics::KinematicsBase)

namespace whole_body_kinematics_plugin
{

WholeBodyKinematicsPlugin::WholeBodyKinematicsPlugin()
  : verbose_(false),
    nh_("~")
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
  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

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

  if(joint_model_group_->isChain())
  {
    ROS_ERROR_NAMED("whole_body_ik","Group '%s' is a chain, which this plugin might not support. I've only tested it with non-chains", group_name.c_str());
    return false;
  }
  if(!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("whole_body_ik","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  // Get the num of dimensions
  ROS_INFO_STREAM_NAMED("temp","Found " << joint_model_group_->getActiveJointModels().size() << " active joints and "
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

    std::cout << "Used to Expect: ----------------" << std::endl;
    std::cout << " - LARM_LINK6 " << std::endl;
    std::cout << " - RARM_LINK6 " << std::endl;
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
      ROS_ERROR_STREAM_NAMED("whole_body_ik","This is the Whole Body kinematics plugin, and it does not support mimic/redundant/non-revolute or prismatic joints");
    }
  }

  // Copy data to FK version
  fk_group_info_.joint_names = ik_group_info_.joint_names;
  fk_group_info_.limits = ik_group_info_.limits;

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
  fk_group_info_.link_names = joint_model_group_->getLinkModelNames();

  // DEBUG
  std::cout << "Total link names: " << std::endl;
  std::copy(ik_group_info_.link_names.begin(), ik_group_info_.link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Populate the joint limits
  joint_min_.resize(ik_group_info_.limits.size());
  joint_max_.resize(ik_group_info_.limits.size());
  for(unsigned int i=0; i < ik_group_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_group_info_.limits[i].min_position;
    joint_max_(i) = ik_group_info_.limits[i].max_position;
  }

  // Get Solver Parameters from param server
  nh_.param("max_solver_iterations", max_solver_iterations_, 500);
  nh_.param("epsilon", epsilon_, 1e-5);

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new moveit_visual_tools::VisualTools("/odom","/hrp2_visual_markers", robot_model_));
  visual_tools_->loadRobotStatePub("/moveit_whole_body_ik");

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
  // alpha the null-space velocity gain
  double alpha = 0.45;

  // Load the jacobian generator
  jacobian_generator_.reset(new JacobianGenerator(verbose_));
  if (!jacobian_generator_->initialize(urdf_model, robot_model_, tip_frames_, joint_model_group_))
  {
    ROS_ERROR_STREAM_NAMED("whole_body_ik","Failed to convert URDF to KDL Chains and setup jacobians");
    return false;
  }

  // inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity
  ik_solver_vel_.reset(new KDL::IkSolverVel_pinv_nso(tip_frames.size(), dimension_, joint_min_, joint_max_,
      weights, ctj_data_->jacobian_, eps, maxiter, alpha, verbose_));

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

  // Check if ROS is ok
  if (!ros::ok())
  {
    ROS_ERROR_STREAM_NAMED("cartesianToJoint","ROS requested shutdown");
    return -1;
  }

  // Optimization debug functionality
  if (false)
  {
    double alpha;
    nh_.param("alpha", alpha, 0.6);
    ROS_INFO_STREAM_NAMED("temp","Read new alpha from param server of value: " << alpha << " from namespace " << nh_.getNamespace());
    ik_solver_vel_->setAlpha(alpha);
  }

  // Check if seed state correct
  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("srv","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check consistency limits
  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("whole_body_ik","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("srv","Mismatched number of pose requests (" << ik_poses.size()
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
  while(true)
  {
    if (verbose_)
      ROS_DEBUG_NAMED("whole_body_ik","Outer most iteration: %d, time: %f, Timeout: %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
    counter++;

    // Check if timed out
    if(timedOut(n1,timeout))
    {
      ROS_DEBUG_NAMED("whole_body_ik","IK timed out");
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
        ROS_DEBUG_NAMED("whole_body_ik","New random configuration");
        for(unsigned int j=0; j < dimension_; j++)
          ROS_DEBUG_NAMED("whole_body_ik","%d %f", j, jnt_pos_in(j));
      }
    }

    // Solve
    int ik_valid = cartesionToJoint(jnt_pos_in, kdl_poses, jnt_pos_out);

    // Check solution
    if( !consistency_limits.empty() &&
      ( (ik_valid < 0 && !options.return_approximate_solution) || !checkConsistency(jnt_seed_state, consistency_limits, jnt_pos_out)) )
    {
      if (verbose_)
        ROS_DEBUG_NAMED("whole_body_ik","Could not find IK solution: does not match consistency limits");
      continue;
    }
    else if(ik_valid < 0 && !options.return_approximate_solution)
    {
      if (verbose_)
        ROS_DEBUG_NAMED("whole_body_ik","Could not find IK solution");
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

    // Check if callback was sucessful
    if(error_code.val == error_code.SUCCESS)
    {
      static int total_iterations = 0;
      total_iterations += counter;
      ROS_DEBUG_STREAM_NAMED("whole_body_ik","Solved after " << counter << " iterations, total iterations for all IK calls: " << total_iterations);

      if (false || verbose_)
      {
        robot_state_->setJointGroupPositions(joint_model_group_, solution);
        visual_tools_->publishRobotState(robot_state_);
        std::cout << "publishing! " << std::endl;
        ros::Duration(0.1).sleep();
      }

      return true;
    }

  }
  // should never get here
}

// Convert Eigen::Affine3d to KDL::Frame
void poseEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  // Set position
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e.matrix()(i,3);

  // Set orientation
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e.matrix()(i/3,i%3);

  //k.M.data[i] = t.getBasis()[i/3][i%3];
}

/* \brief Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the
 *  position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account. */
int WholeBodyKinematicsPlugin::cartesionToJoint(const KDL::JntArray& q_init, const std::vector<KDL::Frame>& kdl_poses, KDL::JntArray& q_out) const
{
  // First solution guess is the seed state
  q_out = q_init;

  // TEMP VARS:
  int dimension_of_subgroup = dimension_/2; // TODO remove this hack and replace with robot_state/robot_model maybe
  KDL::JntArray q_out_subgroup(dimension_of_subgroup); // TODO this is two-arm specific and is a terrible hack
  bool debug_would_have_stopped = false; // used for testing if we stopped to soon
  double offset = 0.0001; // amount to move the joint away from the limit when the limit is hit

  // Actualy requried vars
  bool all_poses_valid; // track if any pose is still not within epsilon distance to its goal
  
  // Set the performance criterion back to zero
  for (std::size_t i = 0; i <   ctj_data_->prev_H_.rows(); ++i)
  {
    ctj_data_->prev_H_(i) = 0.0;    
  }

  // Start main loop
  std::size_t solver_iteration; // Iterate on approximate guess of joint values 'q_out'
  for (  solver_iteration = 0; solver_iteration < max_solver_iterations_; solver_iteration++ )
  {

    if (verbose_)
      ROS_WARN_STREAM_NAMED("cartesionToJoint","Starting solver iteration " << solver_iteration << " of " << max_solver_iterations_ << " =================");

    if (!ros::ok())
    {
      ROS_ERROR_STREAM_NAMED("cartesianToJoint","ROS requested shutdown");
      return -1;
    }

    all_poses_valid = true;

    // Convert to vector of doubles
    for (std::size_t i = 0; i < q_out.rows(); ++i)
      ctj_data_->current_joint_values_[i] = q_out(i);

    if (verbose_)
    {
      std::cout << "Curr JValues: " ;
      //std::copy(ctj_data_->current_joint_values_.begin(), ctj_data_->current_joint_values_.end(), std::ostream_iterator<double>(std::cout, ", "));
      for (std::size_t i = 0; i < ctj_data_->current_joint_values_.size(); ++i)
      {
        std::cout << boost::format("%10.4f") % ctj_data_->current_joint_values_[i];
      }
      std::cout << std::endl;
    }
    robot_state_->setJointGroupPositions(joint_model_group_, ctj_data_->current_joint_values_);

    // Visualize progress
    if (true && solver_iteration % 1 == 0 || verbose_ && solver_iteration % 100 == 0)
    {
      // Publish
      visual_tools_->publishRobotState(robot_state_);
      ros::Duration(0.1).sleep();
    }

    // For each end effector
    for (std::size_t pose_id = 0; pose_id < kdl_poses.size(); ++pose_id)
    {
      // Do forward kinematics to get new EE pose location
      Eigen::Affine3d eef_pose = robot_state_->getGlobalLinkTransform(tip_frames_[pose_id]);

      // Bring the pose to the frame of the IK solver
      robot_state_->setToIKSolverFrame( eef_pose, getBaseFrame() );

      // Convert Eigen::Affine3d to KDL::Frame
      poseEigenToKDL(eef_pose, ctj_data_->current_pose_);

      // Calculate the difference between our desired pose and current pose
      ctj_data_->delta_twist_ = diff(ctj_data_->current_pose_, kdl_poses[pose_id]);   // v_in = actual - target

      // Check if the difference between our desired pose and current pose is within epsilon tolerance
      if (!Equal(ctj_data_->delta_twist_, KDL::Twist::Zero(), epsilon_))
        all_poses_valid = false;

      // Add this twist to our large twist vector from multiple end effectors
      for (std::size_t twist_index = 0; twist_index < TWIST_SIZE; ++twist_index)
      {
        //std::cout << " >>> Twist value is: " << ctj_data_->delta_twist_( twist_index ) << " being placed in index " << TWIST_SIZE * pose_id + twist_index << std::endl;
        ctj_data_->delta_twists_( TWIST_SIZE * pose_id + twist_index ) = ctj_data_->delta_twist_( twist_index );
      }
    }

    // See twist delta
    if (verbose_)
    {
      std::cout << "Twist:   ";
      for (std::size_t i = 0; i < ctj_data_->delta_twists_.rows(); ++i)
      {
        std::cout << boost::format("%12.5f") % ctj_data_->delta_twists_(i);
      }
      std::cout << std::endl;
    }

    // Check if we are donep
    if (all_poses_valid)
    {
      if (verbose_)
        ROS_DEBUG_STREAM_NAMED("cartesionToJoint","All of our end effectors are withing epsilon tolerance of goal location");

      if (debug_would_have_stopped)
      {
        ROS_ERROR_STREAM_NAMED("temp","FOUND AN INSTANCE WERE WE WOULD HAVE GIVEN UP TOO EARLY!");
        exit(-1);
      }

      if (verbose_)
      {
        std::cout << "FINAL SOLUTIONS: ";
        for (std::size_t i = 0; i < q_out.rows(); ++i)
          std::cout << q_out(i) << ", ";
        std::cout << std::endl;
      }

      return 0;
    }

    //Calculate the jacobian "jac" the current joint positions in robot_state
    if( !jacobian_generator_->generateJacobian(robot_state_, ctj_data_->jacobian_) )
    {
      ROS_ERROR_STREAM_NAMED("whole_body_ik","Failed to generate jacobian");
      return -3;
    }

    // Run velocity solver - qdot is returned as the joint velocities (delta q)
    // (change in joint value guess)
    ik_solver_vel_->CartToJnt(q_out, ctj_data_->delta_twists_, ctj_data_->jacobian_, ctj_data_->qdot_, ctj_data_->prev_H_);

    // See velocities
    if (verbose_)
    {
      std::cout << "Qdot: ";
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%11.7f") % ctj_data_->qdot_(i);
        //std::cout << boost::format("%20.15f") % ctj_data_->qdot_(i);
      }
      std::cout << std::endl;
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
        if (verbose_)
        {
          ROS_ERROR_STREAM_NAMED("temp","Giving up because no change detected");
        }
        debug_would_have_stopped = true;
        break; // disable this to keep going
      }
    }

    // Check
    for (std::size_t i = 0; i < ctj_data_->qdot_.rows(); ++i)
    {
      if (isnan(ctj_data_->qdot_(i)))
      {
        std::cout << "FOUND NAN " << std::endl;
        ctj_data_->qdot_.print();
        break;
      }
    }


    // Add current guess 'q_out' with our new change in guess qdot (delta q)
    // q_out = q_out + q_dot
    Add(q_out, ctj_data_->qdot_, q_out);

    // Enforce joint limits
    for (unsigned int j = 0; j < q_out.rows(); j++)
    {
      if (q_out(j) < joint_min_(j))
      {
        if (verbose_)
          ROS_ERROR_STREAM_NAMED("cartesionToJoint","Min joint limit hit for joint " << j);
        q_out(j) = joint_min_(j) + offset;
      }
      else if (q_out(j) > joint_max_(j))
      {
        if (verbose_)
          ROS_ERROR_STREAM_NAMED("cartesionToJoint","Max joint limit hit for joint " << j);
        q_out(j) = joint_max_(j) - offset;
      }
    }

    // Loop
    //ROS_INFO_STREAM_NAMED("temp","temp quit early");
    //exit(-1);
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
  if (jmg->isChain())
  {
    if(error_text_out)
    {
      *error_text_out = "This plugin only supports joint groups which are not chains";
    }
    return false;
  }

  return true;
}


} // namespace

//PLUGINLIB_EXPORT_CLASS(whole_body_kinematics_plugin::WholeBodyKinematicsPlugin,
//  kinematics::KinematicsBase);
