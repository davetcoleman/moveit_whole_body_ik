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

#include <moveit/whole_body_kinematics_plugin/ik_solver_pinverse.h>
#include <moveit/whole_body_kinematics_plugin/MatrixSolvers.h> // SVD solver
#include <iostream> // TODO remove
#include <boost/format.hpp>

namespace whole_body_kinematics_plugin
{

IkSolverPinverse::IkSolverPinverse(int _num_tips, int _num_joints, JntArray _joint_min, JntArray _joint_max,
  JntArray weights, const Jacobian2d& jacobian, double _eps, int _maxiter, double _alpha, bool verbose)
  :
  // Load the jacobian to have #joint ROWS x #tips COLS
  //  jacobian(_num_joints, _num_tips*6),
  svd_(jacobian),
  U(_num_tips*6,JntArray(_num_joints)),
  S(_num_joints),
  V(_num_joints, JntArray(_num_joints)),
  H(_num_joints),
  tmp(_num_joints),
  tmp2(_num_joints-6*_num_tips),
  eps(_eps),
  maxiter(_maxiter),
  alpha_(_alpha),
  num_tips(_num_tips),
  weights_(weights),
  W(_num_joints),
  // Properties of joint
  joint_min(_joint_min),
  joint_max(_joint_max),
  joint_mid(_joint_min.rows()),
  joint_constant1(_joint_min.rows()),
  joint_constant2(_joint_min.rows()),
  joint_constant3(_joint_min.rows()),
  // Debugging
  verbose_(verbose)
{
  for (std::size_t i = 0; i < joint_min.rows(); ++i)
  {
    // Calculate midpoint of all joints
    joint_mid(i) = (joint_max(i) + joint_min(i)) / 2.0;

    // Calculate a component of the Zghal performance criterion
    joint_constant1(i) = (joint_max(i) - joint_min(i)) * (joint_max(i) - joint_min(i));
    // Calculate components of the other performance criterion (from JSK - name unknown)
    joint_constant2(i) = (joint_max(i) + joint_min(i)) / 2.0;
    joint_constant3(i) = (joint_max(i) - joint_min(i)) / 2.0;
  }

  // Initialize the matrix
  pinverse_.resize(_num_joints, _num_tips*6);
  tmp3_.resize    (_num_joints, _num_joints);
  tmp4_.resize    (_num_joints, _num_tips*6);
  identity_.resize(_num_joints, _num_joints);

  identity_.setIdentity();
}

/**
 * \param q_in - current joint location, (?? = this allows us to linearize the jacobian around its current state)
 * \param xdot_in - the difference between desired pose and current pose
 * \param jacobian
 * \param qdot_out - velocity (delta q) - change in joint values
 * \param prev_H - contains the previous performance criterion
 7 */
int IkSolverPinverse::cartesianToJoint(const JntArray& q_in, const JntArray& xdot_in, Jacobian2d& jacobian, JntArray& qdot_out, JntArray& prev_H)
{
  // weights:
  bool use_wln = false;
  // inverse methods:
  bool use_psm = false;
  bool use_kdl = false;
  bool use_kdl2 = true;
  // null space:
  bool use_gpm = true; // not with use_kdl

  unsigned int i,j;
  double sum;

  // TODO: choose best ratio
  double sv_ratio=1e-300; // default: 1.0e-3

  // Find the Weighted Least Norm Jacobian
  if (use_wln)
    weightedLeastNorm(q_in, jacobian, prev_H);

  // Method 1: Calculate the entire pseudo inverse as found in HRP --------------------------
  if (use_psm)
  {
    // Calculate pseudo inverse
    // pinv(A) = V*S^(-1)*U^(T)
    hrp::calcPseudoInverse(jacobian.data, pinverse_, sv_ratio);

    //print(pinverse_);

    // Apply pinverse to the velocity vector
    for (i = 0; i < jacobian.columns(); ++i) // row of pinverse,
    {
      sum = 0.0;
      for (j = 0; j < jacobian.rows(); ++j) // column of pinverse
      {
        sum += pinverse_(i,j) * xdot_in(j);
      }

      if (use_wln)
        qdot_out(i) = W(i) *  sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Method 2: Do SVD faster using the KDL method ---------------------------------------------
  if (use_kdl)
  {
    //Do a singular value decomposition of "jacobian" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jacobian = U*S*Vt
    svd_.calculate(jacobian,U,S,V,maxiter);

    // We have to calculate qdot_out = jac_pinv*xdot_in
    // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    // qdot_out = V*S_pinv*Ut*xdot_in

    //first we calculate S_pinv*Ut*xdot_in
    for (i = 0;i < jacobian.columns();i++)
    {
      sum = 0.0;
      for (j = 0;j < jacobian.rows();j++)
      {
        sum += U[j](i) * xdot_in(j);
      }
      //If the singular value is too small (<eps), don't invert it but
      //set the inverted singular value to zero (truncated svd)
      if ( fabs(S(i)) < eps )
      {
        tmp(i) = 0.0 ;
      }
      else
      {
        tmp(i) = sum/S(i) ;
      }
    }

    //tmp is now: tmp=S_pinv*Ut*xdot_in, we still have to premultiply
    //it with V to get qdot_out
    for (i = 0;i < jacobian.columns();i++)
    {
      sum = 0.0;
      for (j = 0;j < jacobian.columns();j++)
      {
        sum += V[i](j) * tmp(j);
      }

      //Put the result in qdot_out
      if (use_wln)
        qdot_out(i) = W(i) * sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Method 3: Do SVD faster using the KDL method but directly calc pseudo invers---------------------------------------------
  if (use_kdl2)
  {
    //Do a singular value decomposition of "jacobian" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jacobian = U*S*Vt
    svd_.calculate(jacobian,U,S,V,maxiter);

    if (verbose_)
    {
      std::cout << "U ------------------- " << std::endl;
      std::cout << "rows " << U.size() << std::endl;
      std::cout << "col " << U[0].rows() << std::endl;
      for (i = 0; i < U.size(); ++i)
      {
        for (j = 0; j < U[i].rows(); ++j)
        {
          std::cout << U[i](j) << ", ";
        }
        std::cout << std::endl;
      }

      std::cout << "S " << std::endl;
      S.print();
    }

    //J^(+) = V*S^(-1)*U^(T)

    int m = jacobian.columns(); // joints
    int n = jacobian.rows(); // eefs

    // We have to calculate qdot_out = jac_pinv*xdot_in
    // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    // qdot_out = V*S_pinv*Ut*xdot_in

    //first we calculate S_pinv*Ut
    for (i = 0;i < n; i++) // eefs / rows
    {
      for (j = 0;j < m; j++) // joints / columns
      {
        //If the singular value is too small (<eps), don't invert it but
        //set the inverted singular value to zero (truncated svd)
        if ( fabs(S(j)) < eps )
        {
          tmp4_(j,i) = 0.0 ;
        }
        else
        {
          tmp4_(j,i) = U[i](j) / S(j) ;
        }
      }
    }

    if (verbose_)
    {
      std::cout << "tmp4 " << std::endl;
      print(tmp4_);
    }

    // tmp4 = S_pinv*Ut
    // we still have to premultiply it with V
    for (i = 0; i < n; i++) // eefs / rows
    {
      for (j = 0; j < m; j++) // joints / columns
      {
        sum = 0.0;
        for (std::size_t k = 0; k < m; ++k)
        {
          sum += V[j](k) * tmp4_(k,i);
        }
        pinverse_(j, i) = sum;
      }
    }

    if (verbose_)
    {
      std::cout << "pinverse " << std::endl;
      print(pinverse_);
    }

    // Apply pinverse to the velocity vector
    for (i = 0; i < jacobian.columns(); ++i) // row of pinverse,
    {
      sum = 0.0;
      for (j = 0; j < jacobian.rows(); ++j) // column of pinverse
      {
        sum += pinverse_(i,j) * xdot_in(j);
      }

      if (use_wln)
        qdot_out(i) = W(i) *  sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Gradient Projection Method using Null Space ----------------------------------------------

  if (use_gpm)
  {
    // H
    for(i = 0; i < jacobian.columns(); i++) // joints
    {
      //H(i) = 0.25 * joint_constant1(i) / (  (joint_max(i) - q_in(i)) * (q_in(i) - joint_min(i)) );

      //H(i) = ( joint_constant2(i) - q_in(i) ) / joint_constant3(i);

      // Calculate the change in joint location relative to its limits

      H(i) =
        pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) )
        /
        ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) );

      /*
        H(i) = 0;
        if (i == 3)
        H(3) = 1;
      */
    }

    bool this_verbose = true;
    if (this_verbose)
    {
      std::cout << std::endl  << "Identity " << std::endl;
      print(identity_);
      std::cout << std::endl  << "pinverse: " << std::endl;
      print(pinverse_);
      std::cout << std::endl  << "Jacobian: " << std::endl;
      jacobian.print();
      std::cout << std::endl  << "H criterion: " << std::endl;
      H.print();
      std::cout << std::endl  << "Alpha: " << alpha_ << std::endl;
    }

    std::cout << "J^+ * J" << std::endl;
    tmp3_ = (pinverse_ * jacobian.data);
    print(tmp3_);

    std::cout << "I - J^+ * J" << std::endl;
    tmp3_ = (identity_ - pinverse_ * jacobian.data);
    print(tmp3_);

    // temp H
    /*
      Eigen::VectorXd tempH;
      tempH.resize(H.data.rows(),1);
      tempH[3] = 1;
      std::cout << "H tmp " << std::endl;
      print(tempH);
    */

    std::cout << "* H " << std::endl;
    tmp3_ = (identity_ - pinverse_ * jacobian.data) * H.data;
    print(tmp3_);

    // qdot += k(I - J^(+)*J)
    tmp3_ = alpha_ * (identity_ - pinverse_ * jacobian.data) * H.data;  // TODO is this jacobian already weighted?

    if (this_verbose)
    {
      std::cout << std::endl  << "QDot before null space componet:" << std::endl;
      qdot_out.print();

      std::cout << std::endl  << "Null space component: " << std::endl;
      print(tmp3_);
    }

    qdot_out.data += tmp3_;

  }
  exit(0);

  return 1;
}

bool IkSolverPinverse::weightedLeastNorm(const JntArray& q_in, Jacobian2d& jacobian, JntArray& prev_H)
{
  double gradientH;

  // Find the Weighted Least Norm Jacobian
  for (std::size_t i = 0; i < W.rows(); ++i)
  {
    // Calculate the change in joint location relative to its limits
    // | gradient H(theta) |
    gradientH = abs(
      pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) )
      /
      ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) )
    );

    // If joint is moveing away from limit do not change its weight (leave as 1)
    if (gradientH - prev_H(i) >= 0) // change in performance criterion is positive
    {
      W(i) = 1 + gradientH;
    }
    else
    {
      W(i) = 1;
    }

    // TODO: remove this safety check
    /*
      if ( isnan(gradientH) || isnan(W(i)) || isnan(-1/sqrt(W(i))))
      {
      std::cout << "Is nan! " << std::endl;
      std::cout << "  gradH:  " << gradientH << std::endl;
      std::cout << "  first:  " << pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) ) << std::endl;
      std::cout << "  second: " << ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) ) << std::endl;
      std::cout << "  max:    " << joint_max(i) << "\n";
      std::cout << "  min:    " << joint_min(i) << std::endl;
      std::cout << "  q_in:   " << q_in(i) << "\n";
      std::cout << "  i: " << i << std::endl;
      std::cout << "  W(i): " << W(i) << std::endl;
      std::cout << "  W(i)^(-1/2): " << 1/sqrt(W(i)) << std::endl;
      q_in.print();
      std::cout << "---------------------------- " << std::endl;
      }
    */

    // Prepare the weight for multiplication by the jacobian
    // W = W^(-1/2)
    W(i) = 1/sqrt(W(i));

    // Save gradient for next iteration to find delta
    prev_H(i) = gradientH;
  }

  if (verbose_ || false)
  {
    std::cout << "Weighing Matrix: " << std::endl;
    W.print();
  }

  // Apply weighting matrix to jacobian
  // J_w = J*W^(-1/2)
  for (std::size_t i = 0; i < jacobian.rows(); ++i)
  {
    for (std::size_t j = 0; j < jacobian.columns(); ++j)
    {
      jacobian(i,j) *= W(j);
    }
  }

  return true;
}

int IkSolverPinverse::setWeights(const JntArray & weights)
{
  weights_ = weights;
  return 0;
}

int IkSolverPinverse::setAllWeights(const double &weight)
{
  for(unsigned int i=0; i < weights_.rows(); i++)
    weights_(i) = weight;

  return 0;
}

int IkSolverPinverse::setAlpha(const double alpha)
{
  alpha_ = alpha;
  return 0;
}

void IkSolverPinverse::print(Eigen::MatrixXd &data) const
{
  std::cout << "------------ " << data.rows() << " rows by " << data.cols() << " cols --------------- " << std::endl;
  std::cout << "[" << std::endl;
  for (std::size_t i = 0; i < data.rows(); ++i)
  {
    std::cout << "[";
    for (std::size_t j = 0; j < data.cols(); ++j)
    {
      // Hide zeros
      /*
        if ( data(i,j) <= std::numeric_limits<double>::epsilon() )
        std::cout << boost::format("%6s") % "-";
        else
      */
      std::cout << boost::format("%6.10f") % data(i,j);

      if (j < data.cols() - 1)
        std::cout << ",";
    }
    std::cout << "]";

    // close the whole matrix
    if (i == data.rows() - 1)
      std::cout << "]";

    std::cout << std::endl;
  }
}


void IkSolverPinverse::print(Eigen::VectorXd &data) const
{
  std::cout << "========= " << data.rows() << "x Joint Array =======" << std::endl;
  for (std::size_t i = 0; i < data.rows(); ++i)
  {
    std::cout << data(i) << std::endl;
  }
  std::cout << "======================= " << std::endl;
}


} // namespace
