//
// Copyright (c) 2019, Wolfgang Merkt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_pinocchio_dynamics_solver/pinocchio_dynamics_solver.h>

REGISTER_DYNAMICS_SOLVER_TYPE("PinocchioDynamicsSolver", exotica::PinocchioDynamicsSolver)

namespace exotica
{
void PinocchioDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const bool verbose = false;
    if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    }
    else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelPlanar(), model_, verbose);
    }
    else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FLOATING)
    {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelFreeFlyer(), model_, verbose);
    }
    else
    {
        ThrowPretty("This condition should never happen. Unknown BaseType.");
    }

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    num_controls_ = model_.nv;
}

Eigen::VectorXd PinocchioDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    // TODO: THIS DOES NOT WORK FOR A FLOATING BASE YET!!
    pinocchio::Data data(model_);
    pinocchio::aba(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u);
    Eigen::VectorXd x_dot(num_positions_ + num_velocities_);
    x_dot.head(num_positions_) = x.tail(num_positions_);
    x_dot.tail(num_velocities_) = data.ddq;
    return x_dot;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    const int NQ = num_positions_;
    const int NV = num_velocities_;
    const int NX = NQ + NV;
    const int NU = num_controls_;

    pinocchio::Data data(model_);
    // pinocchio::aba(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u);
    pinocchio::computeABADerivatives(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

    Eigen::MatrixXd fx_symb = Eigen::MatrixXd::Zero(NX, NX);
    fx_symb.topRightCorner(NV, NV) = Eigen::MatrixXd::Identity(NV, NV);
    fx_symb.bottomLeftCorner(NQ, NV) = data.ddq_dq;









    // // Finite differences
    // constexpr double eps = 1e-6;
    // const int NX_ = num_positions_ + num_velocities_;

    // Eigen::MatrixXd fx_fd(NX_, NX_);

    // for (int i = 0; i < NX_; ++i)
    // {
    //     Eigen::VectorXd x_low = x;
    //     Eigen::VectorXd x_high = x;
    //     x_low(i) -= eps / 2.0;
    //     x_high(i) += eps / 2.0;

    //     fx_fd.col(i) = (f(x_high, u) - f(x_low, u)) / eps;
    // }


    // HIGHLIGHT_NAMED("fx_pin_size", fx_symb.rows() << " x " << fx_symb.cols());
    // HIGHLIGHT_NAMED("fx_fd_size", fx_fd.rows() << " x " << fx_fd.cols());
    // HIGHLIGHT_NAMED("fx_pin", fx_symb);
    // HIGHLIGHT_NAMED("fx_fd", fx_fd);



    // return fx_fd;
    return fx_symb;
}

Eigen::MatrixXd PinocchioDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    const int NQ = num_positions_;
    const int NV = num_velocities_;
    const int NX = NQ + NV;
    const int NU = num_controls_;

    pinocchio::Data data(model_);
    // pinocchio::aba(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u);
    pinocchio::computeABADerivatives(model_, data, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

    Eigen::MatrixXd fu_symb = Eigen::MatrixXd::Zero(NX, NU);
    fu_symb.bottomRightCorner(NV, NU) = data.Minv;











    // constexpr double eps = 1e-6;
    // const int NX_ = num_positions_ + num_velocities_;
    // const int NU_ = num_controls_;

    // Eigen::MatrixXd fu_fd(NX_, NU_);

    // for (int i = 0; i < NU_; ++i)
    // {
    //     Eigen::VectorXd u_low = u;
    //     Eigen::VectorXd u_high = u;
    //     u_low(i) -= eps / 2.0;
    //     u_high(i) += eps / 2.0;
        
    //     fu_fd.col(i) = (f(x, u_high) - f(x, u_low)) / eps;
    // }

    // HIGHLIGHT_NAMED("fu_pin_size", fu_symb.rows() << " x " << fu_symb.cols());
    // HIGHLIGHT_NAMED("fu_fd_size", fu_fd.rows() << " x " << fu_fd.cols());
    // HIGHLIGHT_NAMED("fu_pin", fu_symb);
    // HIGHLIGHT_NAMED("fu_fd", fu_fd);




    return fu_symb;
    // return fu_fd;
}


// Eigen::VectorXd PinocchioDynamicsSolver::Integrate(const StateVector& x, const ControlVector& u)
// {
//     switch (integrator_)
//     {
//         // Forward Euler (RK1)
//         case Integrator::RK1:
//         {
//             ControlVector u_limited(u.rows(), 1);
//             if (control_limits_.size() > 0)
//             {
//                 for (int i = 0; i < num_controls_; ++ i)
//                     u_limited(i, 1) = std::min(
//                         std::max(u(i, 1), -control_limits_(i)),
//                         control_limits_(i)
//                     );
//             }

//             StateVector xdot = f(x, u);
//             Eigen::VectorXd x_new = x + dt_ * xdot;

//             // enforce joint limits
//             for (int i = 0; i < num_positions_; ++ i) 
//             {
//                 if (
//                     x_new(i) <= model_.lowerPositionLimit(i) ||
//                     x_new(i) >= model_.upperPositionLimit(i)
//                 ) {
//                     x_new(i) = std::min(
//                         std::max(x_new(i), model_.lowerPositionLimit(i)),
//                         model_.upperPositionLimit(i)
//                     );

//                     // clamp velocities to 0
//                     x_new(num_positions_ + i) = 0;
//                 }

//             }
//             // x_new.head(num_positions_) = x_new.head(num_positions_).cwiseMax(
//             //     model_.lowerPositionLimit
//             // ).cwiseMin(
//             //     model_.upperPositionLimit
//             // );

//             return x_new;
//         }
//     }

//     ThrowPretty("PinocchioDynamicsSolver only supports RK1!");
// }

Eigen::VectorXd PinocchioDynamicsSolver::inverse_dynamics(const StateVector& x)
{
    pinocchio::Data data(model_);

    // compute dynamic drift -- Coriolis, centrifugal, gravity
    // Assume 0 acceleration, i.e. no control
    // Eigen::VectorXd b = pinocchio::rnea(model_, data,
    Eigen::VectorXd rnea_res = pinocchio::rnea(model_, data,
        x.head(num_positions_).eval(), x.tail(num_velocities_).eval(),
        Eigen::VectorXd::Zero(num_velocities_).eval()
    );
    // pinocchio::crba(model_, data,
    //     x.head(num_positions_).eval()
    // );

    // compute mass matrix M
    // Eigen::VectorXd u = data.C * x.tail(num_velocities_) + data.g;

    // HIGHLIGHT_NAMED("x", x);
    // HIGHLIGHT_NAMED("u", u);
    // HIGHLIGHT_NAMED("M", data.M);
    // HIGHLIGHT_NAMED("C", data.C);
    // HIGHLIGHT_NAMED("g", data.g);
    // HIGHLIGHT_NAMED("rnea_res", rnea_res);
    // HIGHLIGHT("=====================================");

    return rnea_res;
}


}  // namespace exotica
