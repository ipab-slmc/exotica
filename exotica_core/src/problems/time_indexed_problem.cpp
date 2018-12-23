/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <exotica_core/problems/time_indexed_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedProblem", exotica::TimeIndexedProblem)

namespace exotica
{
TimeIndexedProblem::TimeIndexedProblem()
    : T(0), tau(0), W_rate(0)
{
    Flags = KIN_FK | KIN_J;
}

TimeIndexedProblem::~TimeIndexedProblem() = default;

Eigen::MatrixXd TimeIndexedProblem::getBounds() const
{
    return scene_->getKinematicTree().getJointLimits();
}

void TimeIndexedProblem::Instantiate(TimeIndexedProblemInitializer& init)
{
    init_ = init;

    N = scene_->getKinematicTree().getNumControlledJoints();

    W_rate = init_.Wrate;
    W = Eigen::MatrixXd::Identity(N, N) * W_rate;
    if (init_.W.rows() > 0)
    {
        if (init_.W.rows() == N)
        {
            W.diagonal() = init_.W * W_rate;
        }
        else
        {
            throw_named("W dimension mismatch! Expected " << N << ", got " << init_.W.rows());
        }
    }

    if (init.LowerBound.rows() == N)
    {
        scene_->getKinematicTree().setJointLimitsLower(init.LowerBound);
    }
    else if (init.LowerBound.rows() != 0)
    {
        throw_named("Lower bound size incorrect! Expected " << N << " got " << init.LowerBound.rows());
    }
    if (init.UpperBound.rows() == N)
    {
        scene_->getKinematicTree().setJointLimitsUpper(init.UpperBound);
    }
    else if (init.UpperBound.rows() != 0)
    {
        throw_named("Lower bound size incorrect! Expected " << N << " got " << init.UpperBound.rows());
    }

    useBounds = init.UseBounds;

    Cost.initialize(init_.Cost, shared_from_this(), CostPhi);
    Inequality.initialize(init_.Inequality, shared_from_this(), InequalityPhi);
    Equality.initialize(init_.Equality, shared_from_this(), EqualityPhi);

    T = init_.T;
    qdot_max = init_.JointVelocityLimit;
    applyStartState(false);
    reinitializeVariables();
}

void TimeIndexedProblem::reinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem", "Initialize problem with T=" << T);

    setTau(init_.Tau);

    NumTasks = Tasks.size();
    PhiN = 0;
    JN = 0;
    TaskSpaceVector yref;
    for (int i = 0; i < NumTasks; i++)
    {
        appendVector(yref.map, Tasks[i]->getLieGroupIndices());
        PhiN += Tasks[i]->Length;
        JN += Tasks[i]->LengthJ;
    }

    yref.setZero(PhiN);
    Phi.assign(T, yref);

    if (Flags & KIN_J) J.assign(T, Eigen::MatrixXd(JN, N));
    x.assign(T, Eigen::VectorXd::Zero(N));
    xdiff.assign(T, Eigen::VectorXd::Zero(N));
    if (Flags & KIN_J_DOT)
    {
        Hessian Htmp;
        Htmp.setConstant(JN, Eigen::MatrixXd::Zero(N, N));
        H.assign(T, Htmp);
    }

    // Set initial trajectory
    InitialTrajectory.resize(T, scene_->getControlledState());

    Cost.reinitializeVariables(T, shared_from_this(), CostPhi);
    Inequality.reinitializeVariables(T, shared_from_this(), InequalityPhi);
    Equality.reinitializeVariables(T, shared_from_this(), EqualityPhi);
    preupdate();
}

void TimeIndexedProblem::setT(const int& T_in)
{
    if (T_in <= 2)
    {
        throw_named("Invalid number of timesteps: " << T_in);
    }
    T = T_in;
    reinitializeVariables();
}

void TimeIndexedProblem::setTau(const double& tau_in)
{
    if (tau_in <= 0.) throw_pretty("tau is expected to be greater than 0. (tau=" << tau_in << ")");
    tau = tau_in;
    ct = 1.0 / tau / T;
    xdiff_max = qdot_max * tau;
}

void TimeIndexedProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Cost.updateS();
    Inequality.updateS();
    Equality.updateS();

    // Create a new set of kinematic solutions with the size of the trajectory
    // based on the lastest KinematicResponse in order to reflect model state
    // updates etc.
    KinematicSolutions.clear();
    KinematicSolutions.resize(T);
    for (int i = 0; i < T; i++) KinematicSolutions[i] = std::make_shared<KinematicResponse>(*scene_->getKinematicTree().getKinematicResponse());
}

void TimeIndexedProblem::setInitialTrajectory(const std::vector<Eigen::VectorXd>& q_init_in)
{
    if (q_init_in.size() != T)
        throw_pretty("Expected initial trajectory of length "
                     << T << " but got " << q_init_in.size());
    if (q_init_in[0].rows() != N)
        throw_pretty("Expected states to have " << N << " rows but got "
                                                << q_init_in[0].rows());

    InitialTrajectory = q_init_in;
    setStartState(q_init_in[0]);
}

std::vector<Eigen::VectorXd> TimeIndexedProblem::getInitialTrajectory()
{
    return InitialTrajectory;
}

double TimeIndexedProblem::getDuration()
{
    return tau * static_cast<double>(T);
}

void TimeIndexedProblem::Update(Eigen::VectorXdRefConst x_in, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }

    x[t] = x_in;

    // Set the corresponding KinematicResponse for KinematicTree in order to
    // have Kinematics elements updated based in x_in.
    scene_->getKinematicTree().setKinematicResponse(KinematicSolutions[t]);

    // Pass the corresponding number of relevant task kinematics to the TaskMaps
    // via the PlanningProblem::updateMultipleTaskKinematics method. For now we
    // support passing _two_ timesteps - this can be easily changed later on.
    std::vector<std::shared_ptr<KinematicResponse>> kinematics_solutions{KinematicSolutions[t]};

    // If the current timestep is 0, pass the 0th timestep's response twice.
    // Otherwise pass the (t-1)th response.
    kinematics_solutions.emplace_back((t == 0) ? KinematicSolutions[t] : KinematicSolutions[t - 1]);

    // Actually update the tasks' kinematics mappings.
    PlanningProblem::updateMultipleTaskKinematics(kinematics_solutions);

    scene_->Update(x_in, static_cast<double>(t) * tau);
    Phi[t].setZero(PhiN);
    if (Flags & KIN_J) J[t].setZero();
    if (Flags & KIN_J_DOT)
        for (int i = 0; i < JN; i++) H[t](i).setZero();
    for (int i = 0; i < NumTasks; i++)
    {
        // Only update TaskMap if Rho is not 0
        if (Tasks[i]->isUsed)
        {
            if (Flags & KIN_J_DOT)
            {
                Tasks[i]->update(x[t], Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length), J[t].middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ), H[t].segment(Tasks[i]->Start, Tasks[i]->Length));
            }
            else if (Flags & KIN_J)
            {
                Tasks[i]->update(x[t], Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length), J[t].middleRows(Tasks[i]->StartJ, Tasks[i]->LengthJ));
            }
            else
            {
                Tasks[i]->update(x[t], Phi[t].data.segment(Tasks[i]->Start, Tasks[i]->Length));
            }
        }
    }
    if (Flags & KIN_J_DOT)
    {
        Cost.update(Phi[t], J[t], H[t], t);
        Inequality.update(Phi[t], J[t], H[t], t);
        Equality.update(Phi[t], J[t], H[t], t);
    }
    else if (Flags & KIN_J)
    {
        Cost.update(Phi[t], J[t], t);
        Inequality.update(Phi[t], J[t], t);
        Equality.update(Phi[t], J[t], t);
    }
    else
    {
        Cost.update(Phi[t], t);
        Inequality.update(Phi[t], t);
        Equality.update(Phi[t], t);
    }
    if (t > 0) xdiff[t] = x[t] - x[t - 1];
    numberOfProblemUpdates++;
}

double TimeIndexedProblem::getScalarTaskCost(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return ct * Cost.ydiff[t].transpose() * Cost.S[t] * Cost.ydiff[t];
}

Eigen::VectorXd TimeIndexedProblem::getScalarTaskJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Cost.J[t].transpose() * Cost.S[t] * Cost.ydiff[t] * 2.0 * ct;
}

double TimeIndexedProblem::getScalarTransitionCost(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return ct * xdiff[t].transpose() * W * xdiff[t];
}

Eigen::VectorXd TimeIndexedProblem::getScalarTransitionJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return 2.0 * ct * W * xdiff[t];
}

Eigen::VectorXd TimeIndexedProblem::getEquality(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Equality.S[t] * Equality.ydiff[t];
}

Eigen::MatrixXd TimeIndexedProblem::getEqualityJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Equality.S[t] * Equality.J[t];
}

Eigen::VectorXd TimeIndexedProblem::getInequality(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Inequality.S[t] * Inequality.ydiff[t];
}

Eigen::MatrixXd TimeIndexedProblem::getInequalityJacobian(int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    return Inequality.S[t] * Inequality.J[t];
}

void TimeIndexedProblem::setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            if (goal.rows() != Cost.Indexing[i].Length) throw_pretty("Expected length of " << Cost.Indexing[i].Length << " and got " << goal.rows());
            Cost.y[t].data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRho(const std::string& task_name, const double rho, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            Cost.Rho[t](Cost.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoal(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.y[t].data.segment(Cost.Indexing[i].Start, Cost.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRho(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Cost.Indexing.size(); i++)
    {
        if (Cost.Tasks[i]->getObjectName() == task_name)
        {
            return Cost.Rho[t](Cost.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            if (goal.rows() != Equality.Indexing[i].Length) throw_pretty("Expected length of " << Equality.Indexing[i].Length << " and got " << goal.rows());
            Equality.y[t].data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRhoEQ(const std::string& task_name, const double rho, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            Equality.Rho[t](Equality.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoalEQ(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.y[t].data.segment(Equality.Indexing[i].Start, Equality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRhoEQ(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Equality.Indexing.size(); i++)
    {
        if (Equality.Tasks[i]->getObjectName() == task_name)
        {
            return Equality.Rho[t](Equality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            if (goal.rows() != Inequality.Indexing[i].Length) throw_pretty("Expected length of " << Inequality.Indexing[i].Length << " and got " << goal.rows());
            Inequality.y[t].data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length) = goal;
            return;
        }
    }
    throw_pretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedProblem::setRhoNEQ(const std::string& task_name, const double rho, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            Inequality.Rho[t](Inequality.Indexing[i].Id) = rho;
            preupdate();
            return;
        }
    }
    throw_pretty("Cannot set Rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedProblem::getGoalNEQ(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.y[t].data.segment(Inequality.Indexing[i].Start, Inequality.Indexing[i].Length);
        }
    }
    throw_pretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedProblem::getRhoNEQ(const std::string& task_name, int t)
{
    if (t >= T || t < -1)
    {
        throw_pretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t == -1)
    {
        t = T - 1;
    }
    for (int i = 0; i < Inequality.Indexing.size(); i++)
    {
        if (Inequality.Tasks[i]->getObjectName() == task_name)
        {
            return Inequality.Rho[t](Inequality.Indexing[i].Id);
        }
    }
    throw_pretty("Cannot get Rho. Task map '" << task_name << "' does not exist.");
}

bool TimeIndexedProblem::isValid()
{
    bool succeeded = true;
    auto bounds = scene_->getKinematicTree().getJointLimits();

    // Check for every state
    for (unsigned int t = 0; t < T; t++)
    {
        // Check joint limits
        for (unsigned int i = 0; i < N; i++)
        {
            if (x[t](i) < bounds(i, 0) || x[t](i) > bounds(i, 1))
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::isValid", "State at timestep " << t << " is out of bounds");
                succeeded = false;
            }
        }

        // Check inequality constraints
        if (getInequality(t).rows() > 0)
        {
            if (getInequality(t).maxCoeff() > init_.InequalityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::isValid", "Violated inequality constraints at timestep " << t << ": " << getInequality(t).transpose());
                succeeded = false;
            }
        }

        // Check equality constraints
        if (getEquality(t).rows() > 0)
        {
            if (getEquality(t).norm() > init_.EqualityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::isValid", "Violated equality constraints at timestep " << t << ": " << getEquality(t).norm());
                succeeded = false;
            }
        }
    }

    return succeeded;
}
}
