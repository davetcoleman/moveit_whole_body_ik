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

#include <moveit/simple_kdl_kinematics_plugin/kdl/ik_solver_vel_pinv_nso.hpp> // customized ik generalize pseudo inverse
#include <iostream> // TODO remove

namespace KDL
{

IkSolverVel_pinv_nso::IkSolverVel_pinv_nso(const std::vector<Chain>& _chains, JntArray _opt_pos, JntArray _weights, 
  double _eps, int _maxiter, double _alpha, int _num_tips):
  chains(_chains),
  jnt2jac(chains, 2), // TODO change 2
  // Load the jacobian to have #joint ROWS x #tips COLS
  jacobian(_opt_pos.rows(), _num_tips*6),
  svd(jacobian),
  U(6,JntArray(_opt_pos.rows())),
  S(_opt_pos.rows()),
  V(_opt_pos.rows(), JntArray(_opt_pos.rows())),
  tmp(_opt_pos.rows()),
  tmp2(_opt_pos.rows()-6),
  eps(_eps),
  maxiter(_maxiter),
  alpha(_alpha),
  num_tips(_num_tips),
  weights(_weights),
  opt_pos(_opt_pos)
{
}

void IkSolverVel_pinv_nso::printJacobian(const Jacobian &jacobian)
{
  std::cout << "--------------------------- " << std::endl;
  for (std::size_t i = 0; i < jacobian.rows(); ++i)
  {
    for (std::size_t j = 0; j < jacobian.columns(); ++j)
    {
      std::cout << jacobian(i,j) << ", ";
    }
    std::cout << std::endl;
  }
}


int IkSolverVel_pinv_nso::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
  //Let the ChainJntToJacSolver calculate the jacobian "jac" for
  //the current joint positions "q_in"
  jnt2jac.JntToJac(q_in,jacobian);

  printJacobian(jacobian);

  //Do a singular value decomposition of "jacobian" with maximum
  //iterations "maxiter", put the results in "U", "S" and "V"
  //jacobian = U*S*Vt
  int ret = svd.calculate(jacobian,U,S,V,maxiter);

  double sum;
  unsigned int i,j;

  // We have to calculate qdot_out = jac_pinv*v_in
  // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
  // qdot_out = V*S_pinv*Ut*v_in

  //first we calculate Ut*v_in
  for (i=0;i<jacobian.columns();i++) {
    sum = 0.0;
    for (j=0;j<jacobian.rows();j++) {
      sum+= U[j](i)*v_in(j);
    }
    //If the singular value is too small (<eps), don't invert it but
    //set the inverted singular value to zero (truncated svd)
    tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
  }
  //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
  //it with V to get qdot_out
  for (i=0;i<jacobian.columns();i++) {
    sum = 0.0;
    for (j=0;j<jacobian.columns();j++) {
      sum+=V[i](j)*tmp(j);
    }
    //Put the result in qdot_out
    qdot_out(i)=sum;
  }

  //Now onto NULL space

  for(i = 0; i < jacobian.columns(); i++)
    tmp(i) = weights(i)*(opt_pos(i) - q_in(i));

  //Vtn*tmp
  for (i=jacobian.rows()+1;i<jacobian.columns();i++) {
    tmp2(i-(jacobian.rows()+1)) = 0.0;
    for (j=0;j<jacobian.columns();j++) {
      tmp2(i-(jacobian.rows()+1)) +=V[j](i)*tmp(j);
    }
  }

  for (i=0;i<jacobian.columns();i++) {
    sum = 0.0;
    for (j=jacobian.rows()+1;j<jacobian.columns();j++) {
      sum +=V[i](j)*tmp2(j);
    }
    qdot_out(i) += alpha*sum;
  }

  //return the return value of the svd decomposition
  return ret;
}

int IkSolverVel_pinv_nso::setWeights(const JntArray & _weights)
{
  weights = _weights;
  return 0;
}

int IkSolverVel_pinv_nso::setOptPos(const JntArray & _opt_pos)
{
  opt_pos = _opt_pos;
  return 0;
}

int IkSolverVel_pinv_nso::setAlpha(const double _alpha)
{
  alpha = _alpha;
  return 0;
}

}
