// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <moveit/whole_body_kinematics_plugin/kdl/ik_solver_vel_pinv_nso.hpp> // customized ik generalize pseudo inverse
#include <iostream> // TODO remove

namespace KDL
{

IkSolverVel_pinv_nso::IkSolverVel_pinv_nso(int _num_tips, int _num_joints, JntArray _joint_min, JntArray _joint_max,
  JntArray _weights, const Jacobian2d& jacobian, double _eps, int _maxiter, double _alpha, bool _verbose)
  :
  // Load the jacobian to have #joint ROWS x #tips COLS
  //  jacobian(_num_joints, _num_tips*6),
  svd(jacobian),
  U(_num_tips*6,JntArray(_num_joints)),
  S(_num_joints),
  V(_num_joints, JntArray(_num_joints)),
  H(_num_joints),
  tmp(_num_joints),
  tmp2(_num_joints-6*_num_tips),
  eps(_eps),
  maxiter(_maxiter),
  alpha(_alpha),
  num_tips(_num_tips),
  weights(_weights),
  W(_num_joints),
  // Properties of joint
  joint_min(_joint_min),
  joint_max(_joint_max),
  joint_mid(_joint_min.rows()),
  joint_constant(_joint_min.rows()),
  // Debugging
  verbose(_verbose)
{
  for (std::size_t i = 0; i < joint_min.rows(); ++i)
  {
    // Calculate midpoint of all joints
    joint_mid(i) = (joint_max(i) + joint_min(i)) / 2.0;

    // Calculate a component of the Zghal performance criterion
    // const = (theta_max - theta_min)^2
    joint_constant(i) = (joint_max(i) - joint_min(i)) * (joint_max(i) - joint_min(i));
  }
}

/**
 * \param q_in - current joint location, (?? = this allows us to linearize the jacobian around its current state)
 * \param xdot_in - the difference between desired pose and current pose
 * \param jacobian
 * \param qdot_out - velocity (delta q) - change in joint values
 * \param prev_H - contains the previous performance criterion
 */
int IkSolverVel_pinv_nso::CartToJnt(const JntArray& q_in, const JntArray& xdot_in, Jacobian2d& jacobian, JntArray& qdot_out, JntArray& prev_H)
{
  if (verbose && false)
  {
    std::cout << "Resulting Combined Jacobian:" << std::endl;
    jacobian.print();
  }

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

    // Prepare the weight for multiplication by the jacobian
    // W = W^(-1/2)
    W(i) = 1/sqrt(W(i));

    // Save gradient for next iteration to find delta
    prev_H(i) = gradientH;
  }

  if (verbose)
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

  //Do a singular value decomposition of "jacobian" with maximum
  //iterations "maxiter", put the results in "U", "S" and "V"
  //jacobian = U*S*Vt
  if (verbose)
    std::cout << "Singular value decomposition: " << std::endl;

  int ret = svd.calculate(jacobian,U,S,V,maxiter);

  double sum, component;
  unsigned int i,j;

  // We have to calculate qdot_out = jac_pinv*xdot_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*xdot_in

  if (verbose)
    std::cout << "First we calculate Ut*xdot_in " << std::endl;

  //first we calculate S_pinv*Ut*xdot_in
  for (i=0;i<jacobian.columns();i++)
  {
    sum = 0.0;
    for (j=0;j<jacobian.rows();j++)
    {
      sum += U[j](i) * xdot_in(j);
    }
    //If the singular value is too small (<eps), don't invert it but
    //set the inverted singular value to zero (truncated svd)
    if ( fabs(S(i))<eps )
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
  for (i=0;i<jacobian.columns();i++)
  {
    sum = 0.0;
    for (j=0;j<jacobian.columns();j++)
    {
      sum+=V[i](j)*tmp(j);
    }
    //Put the result in qdot_out
    qdot_out(i) = W(i) * sum;
  }

  // Now onto NULL space ==========================================================
  /*
  // Create weighted position error vector
  for(i = 0; i < jacobian.columns(); i++)
  {
  // Original Criterion:
  // A. Liegeois. Automatic supervisory control of the configuration and behavior of multibody mechnisms
  if (false)
  {
  H(i) = (1/jacobian.columns()) * weights(i)*(joint_mid(i) - q_in(i));
  }
  // Liegeois paper on GPM
  // the first one from A Weighted Least-Norm Solution Based Scheme for Avoiding Joint Limits for Redundant Joint Manipilators
  else if (false)
  {
  // Calculate H(q)
  component = ( q_in(i) - joint_mid(i) ) / ( joint_mid(i) - joint_max(i) );
  H(i) = (component*component) / jacobian.columns();
  }
  // Zghal et. all performance criterion
  // the better one from A Weighted Least-Norm Solution Based Scheme for Avoiding Joint Limits for Redundant Joint Manipilators
  else if (true)
  {
  // H(q) = 1/4 * (max - min)^2 / [ (max - theta)(theta-min) ]
  H(i) = 0.25 * joint_constant(i) / (  (joint_max(i) - q_in(i)) * (q_in(i) - joint_min(i)) );
  }
  else
  {


  }
  }

  //Vtn*H
  // temp2 is a vector the length of our redudant dofs (length = num_joints - 6*num_eefs)
  // temp2 = V * H
  for (i = jacobian.rows() + 1; i < jacobian.columns(); i++)
  {
  tmp2(i-(jacobian.rows()+1)) = 0.0;
  for (j = 0; j < jacobian.columns(); j++)
  {
  tmp2(i-(jacobian.rows()+1)) += V[j](i) * H(j);
  }
  }

  // Add the velocity of the null space redudancy to our qdot_out
  // qdot_out = qdot_out + alpha*temp2
  bool show_null_space = false || verbose;
  if (show_null_space)
  std::cout << "Null space velocity: ";

  for (i = 0; i < jacobian.columns(); i++)
  {
  sum = 0.0;
  for (j = jacobian.rows() + 1; j < jacobian.columns(); j++)
  {
  sum += V[i](j)*tmp2(j);
  }

  if (show_null_space)
  std::cout << alpha*sum << "  ";

  // Apply to velocity output
  qdot_out(i) += alpha*sum;
  }
  if (show_null_space)
  std::cout << std::endl;


  // Debug
  if (verbose)
  {
  std::cout << "Joint Velocity: " << std::endl;
  for (std::size_t i = 0; i < qdot_out.rows(); ++i)
  {
  std::cout << "Joint " << i << ": " << qdot_out(i) << std::endl;
  }
  }
  */

  //return the return value of the svd decomposition
  return ret;
}

/*
  int IkSolverVel_pinv_nso::CartToJnt(const JntArray& q_in, const JntArray& xdot_in, const Jacobian2d& jacobian, JntArray& qdot_out)
  {

  if (verbose && false)
  {
  std::cout << "Resulting Combined Jacobian:" << std::endl;
  jacobian.print();
  }

  //Do a singular value decomposition of "jacobian" with maximum
  //iterations "maxiter", put the results in "U", "S" and "V"
  //jacobian = U*S*Vt
  if (verbose)
  std::cout << "Singular value decomposition: " << std::endl;

  int ret = svd.calculate(jacobian,U,S,V,maxiter);

  double sum, component;
  unsigned int i,j;

  // We have to calculate qdot_out = jac_pinv*xdot_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*xdot_in

  if (verbose)
  std::cout << "First we calculate Ut*xdot_in " << std::endl;

  //first we calculate S_pinv*Ut*xdot_in
  for (i=0;i<jacobian.columns();i++)
  {
  sum = 0.0;
  for (j=0;j<jacobian.rows();j++)
  {
  sum += U[j](i) * xdot_in(j);
  }
  //If the singular value is too small (<eps), don't invert it but
  //set the inverted singular value to zero (truncated svd)
  if ( fabs(S(i))<eps )
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
  for (i=0;i<jacobian.columns();i++)
  {
  sum = 0.0;
  for (j=0;j<jacobian.columns();j++)
  {
  sum+=V[i](j)*tmp(j);
  }
  //Put the result in qdot_out
  qdot_out(i)=sum;
  }

  // Now onto NULL space ==========================================================

  // Create weighted position error vector
  bool show_criterions = false;
  if (show_criterions)
  std::cout << "Criterions:  ";
  for(i = 0; i < jacobian.columns(); i++)
  {
  // Original:
  if (false)
  {
  tmp(i) = weights(i)*(joint_mid(i) - q_in(i));
  }
  // Liegeois paper on GPM ----------------------------------
  else if (true)
  {
  // Calculate H(q)
  component = ( q_in(i) - joint_mid(i) ) / ( joint_mid(i) - joint_max(i) );
  tmp(i) = (component*component) / jacobian.columns();
  }
  else
  {
  // Performance criterion
  tmp(i) = 0.25 * joint_constant(i) / (  (joint_max(i) - q_in(i)) * (q_in(i) - joint_min(i)) );

  }
  if (show_criterions)
  std::cout << tmp(i) << " | ";
  }
  if (show_criterions)
  std::cout << std::endl;

  //Vtn*tmp
  // temp2 is a vector the length of our redudant dofs (joints n - 6)
  // temp2 = V * tmp
  for (i = jacobian.rows()+1;i<jacobian.columns();i++)
  {
  tmp2(i-(jacobian.rows()+1)) = 0.0;
  for (j = 0;j<jacobian.columns();j++)
  {
  tmp2(i-(jacobian.rows()+1)) += V[j](i)*tmp(j);
  }
  }

  // Add the velocity of the null space redudancy to our qdot_out
  // qdot_out = qdot_out + alpha*temp2
  if (verbose)
  std::cout << "Null space: ";
  for (i = 0;i<jacobian.columns();i++)
  {
  sum = 0.0;
  for (j = jacobian.rows()+1;j<jacobian.columns();j++)
  {
  sum += V[i](j)*tmp2(j);
  }

  if (verbose)
  std::cout << alpha*sum << "  ";
  qdot_out(i) += alpha*sum;
  }
  if (verbose)
  std::cout << std::endl;


  // Debug
  if (verbose)
  {
  std::cout << "Joint Velocity: " << std::endl;
  for (std::size_t i = 0; i < qdot_out.rows(); ++i)
  {
  std::cout << "Joint " << i << ": " << qdot_out(i) << std::endl;
  }
  }

  //return the return value of the svd decomposition
  return ret;
  }
*/

int IkSolverVel_pinv_nso::setWeights(const JntArray & _weights)
{
  weights = _weights;
  return 0;
}

int IkSolverVel_pinv_nso::setAllWeights(const double &weight)
{
  for(unsigned int i=0; i < weights.rows(); i++)
    weights(i) = weight;

  return 0;
}

int IkSolverVel_pinv_nso::setAlpha(const double _alpha)
{
  alpha = _alpha;
  return 0;
}

}
