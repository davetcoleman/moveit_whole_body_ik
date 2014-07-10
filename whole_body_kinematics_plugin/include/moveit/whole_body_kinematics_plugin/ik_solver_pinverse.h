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
   Based on ik solver code from Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
*/

#ifndef MOVEIT_WHOLE_BODY_IK__IK_SOLVER_PINVERSE
#define MOVEIT_WHOLE_BODY_IK__IK_SOLVER_PINVERSE

#include <stdlib.h>
#include "kdl/utilities/svd_HH.hpp"
#include "kdl/jacobian.hpp"

namespace whole_body_kinematics_plugin
{

using namespace KDL;
/**
 * Implementation of a inverse velocity kinematics algorithm based
 * on the generalize pseudo inverse to calculate the velocity
 * transformation from Cartesian to joint space of a general
 * KDL::Chain. It uses a svd-calculation based on householders
 * rotations.
 *
 * **This uses the Gradient Projection Method to produce a gradient that causes self-motion
 *   within the null space of the redudant kinematics (dof > 6).
 *
 */
class IkSolverPinverse
{
public:
  /**
   * Constructor of the solver
   *
   * @param chain the chain to calculate the inverse velocity
   * kinematics for
   * @param opt_pos the desired positions of the chain used by to resolve the redundancy
   * @param weights the weights applied in the joint space
   * @param eps if a singular value is below this value, its
   * inverse is set to zero, default: 0.00001
   * @param maxiter maximum iterations for the svd calculation,
   * default: 150
   * @param alpha the null-space velocity gain
   *
   */
  IkSolverPinverse(int num_tips, int num_joints, JntArray joint_min, JntArray joint_max, JntArray weights, const Jacobian2d& jacobian,
    double eps=0.00001,int maxiter=150, double alpha = 0.25, bool verbose = false);

  ~IkSolverPinverse() {};

  int cartesianToJoint(const JntArray& q_in, const JntArray& xdot_in, Jacobian2d& jacobian, JntArray& qdot_out, JntArray& prev_H);

  bool weightedLeastNorm(const JntArray& q_in, Jacobian2d& jacobian, JntArray& prev_H);

  /**
   *Set joint weights for optimization criterion
   *
   *@param weights the joint weights
   *
   */
  int setWeights(const JntArray &weights);

  int setAllWeights(const double &weight);

  /**
   *Set null psace velocity gain
   *
   *@param alpha NUllspace velocity cgain
   *
   */
  int setAlpha(const double alpha);

  void print(Eigen::MatrixXd &data) const;
  void print(Eigen::VectorXd &data) const;

private:
  //  Jacobian2d jacobian;
  SVD_HH svd;
  std::vector<JntArray> U;
  JntArray S;
  std::vector<JntArray> V;
  JntArray H; // performance criterion
  JntArray tmp;
  JntArray tmp2;
  double eps;
  int maxiter;

  double alpha;
  JntArray weights;
  JntArray W; // weighting matrix
  JntArray joint_min;
  JntArray joint_max;
  JntArray joint_mid;
  JntArray joint_constant1; // pre-compute some of the necessary values
  JntArray joint_constant2; // pre-compute some of the necessary values
  JntArray joint_constant3; // pre-compute some of the necessary values
  int num_tips; // number of end effectors to solve for
  bool verbose; // to show output debug info or not

  Eigen::MatrixXd pinverse_; // psuedo inverse matrix
  Eigen::MatrixXd tmp3_; // psuedo inverse matrix
  Eigen::MatrixXd tmp4_;
  Eigen::MatrixXd identity_; // reusuable identity matrix
};

typedef boost::shared_ptr<whole_body_kinematics_plugin::IkSolverPinverse> IkSolverPinversePtr;

} // namespace
#endif

