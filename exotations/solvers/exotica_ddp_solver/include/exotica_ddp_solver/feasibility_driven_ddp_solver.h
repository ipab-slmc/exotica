//
// Copyright (c) 2019-2020, LAAS-CNRS, University of Edinburgh, University of Oxford
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

#ifndef EXOTICA_DDP_SOLVER_FEASIBILITY_DRIVEN_DDP_SOLVER_H_
#define EXOTICA_DDP_SOLVER_FEASIBILITY_DRIVEN_DDP_SOLVER_H_

#include <exotica_ddp_solver/abstract_ddp_solver.h>
#include <exotica_ddp_solver/feasibility_driven_ddp_solver_initializer.h>

namespace exotica
{
// Feasibility-driven DDP solver
// Based on the implementation in Crocoddyl: https://github.com/loco-3d/crocoddyl.git
// Cf. https://arxiv.org/abs/1909.04947
class AbstractFeasibilityDrivenDDPSolver : public AbstractDDPSolver
{
public:
    void Solve(Eigen::MatrixXd& solution) override;
    void SpecifyProblem(PlanningProblemPtr pointer) override;

    const std::vector<Eigen::VectorXd>& get_fs() const { return fs_; };
    const std::vector<Eigen::VectorXd>& get_xs() const { return xs_; };
    const std::vector<Eigen::VectorXd>& get_us() const { return us_; };

protected:
    int NDX_;
    int last_T_ = -1;

    void IncreaseRegularization() override;
    void DecreaseRegularization() override;
    const Eigen::Vector2d& ExpectedImprovement();
    void UpdateExpectedImprovement();
    inline bool IsNaN(const double value)
    {
        if (std::isnan(value) || std::isinf(value) || value >= 1e30)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    void SetCandidate(const std::vector<Eigen::VectorXd>& xs_warm, const std::vector<Eigen::VectorXd>& us_warm, const bool is_feasible);
    double CheckStoppingCriteria();

    double CalcDiff();
    bool ComputeDirection(const bool recalcDiff);
    virtual bool BackwardPassFDDP();
    void BackwardPass() override { return (void)BackwardPassFDDP(); }
    virtual void ComputeGains(const int t);
    void ForwardPass(const double steplength);
    double TryStep(const double steplength);

    virtual void AllocateData();

    Eigen::MatrixXd control_limits_;
    double initial_regularization_rate_ = 1e-9;             // Set from parameters on Instantiate
    bool clamp_to_control_limits_in_forward_pass_ = false;  // Set from parameters on Instantiate

    double steplength_;                  //!< Current applied step-length
    Eigen::Vector2d d_;                  //!< LQ approximation of the expected improvement
    double dV_;                          //!< Cost reduction obtained by TryStep
    double dVexp_;                       //!< Expected cost reduction
    double th_acceptstep_ = 0.1;         //!< Threshold used for accepting step
    double th_stop_ = 1e-9;              //!< Tolerance for stopping the algorithm
    double th_gradient_tolerance_ = 0.;  //!< Gradient tolerance
    double stop_;                        //!< Value computed by CheckStoppingCriteria

    double dg_ = 0.;
    double dq_ = 0.;
    double dv_ = 0.;
    double th_acceptnegstep_ = 2.;  //!< Threshold used for accepting step along ascent direction
    std::vector<Eigen::VectorXd> us_;
    std::vector<Eigen::VectorXd> xs_;
    bool is_feasible_ = false;
    double xreg_ = 1e-9;      //!< State regularization
    double ureg_ = 1e-9;      //!< Control regularization
    double regmin_ = 1e-9;    //!< Minimum regularization (will not decrease lower)
    double regmax_ = 1e9;     //!< Maximum regularization (to exit by divergence)
    double regfactor_ = 10.;  //!< Factor by which the regularization gets increased/decreased

    std::vector<Eigen::VectorXd> xs_try_;  //!< State trajectory computed by line-search procedure
    std::vector<Eigen::VectorXd> us_try_;  //!< Control trajectory computed by line-search procedure
    std::vector<Eigen::VectorXd> dx_;

    // allocate data
    std::vector<Eigen::VectorXd> fs_;  //!< Gaps/defects between shooting nodes

    Eigen::VectorXd xnext_;
    Eigen::MatrixXd FxTVxx_p_;
    std::vector<Eigen::MatrixXd> FuTVxx_p_;
    std::vector<Eigen::MatrixXd> Qxu_;
    Eigen::VectorXd fTVxx_p_;
    std::vector<Eigen::LDLT<Eigen::MatrixXd> > Quu_ldlt_;
    std::vector<Eigen::VectorXd> Quuk_;
    double th_grad_ = 1e-12;     //!< Tolerance of the expected gradient used for testing the step
    double th_stepdec_ = 0.5;    //!< Step-length threshold used to decrease regularization
    double th_stepinc_ = 0.01;   //!< Step-length threshold used to increase regularization
    bool was_feasible_ = false;  //!< Label that indicates in the previous iterate was feasible
};

class FeasibilityDrivenDDPSolver : public AbstractFeasibilityDrivenDDPSolver, public Instantiable<FeasibilityDrivenDDPSolverInitializer>
{
public:
    void Instantiate(const FeasibilityDrivenDDPSolverInitializer& init) override;
};

}  // namespace exotica

#endif  // EXOTICA_DDP_SOLVER_FEASIBILITY_DRIVEN_DDP_SOLVER_H_
