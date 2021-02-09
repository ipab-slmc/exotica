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

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

namespace exotica
{
template <typename T, int NX, int NU>
AbstractDynamicsSolver<T, NX, NU>::AbstractDynamicsSolver() = default;

template <typename T, int NX, int NU>
AbstractDynamicsSolver<T, NX, NU>::~AbstractDynamicsSolver() = default;

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::InstantiateBase(const Initializer& init)
{
    Object::InstantiateObject(init);
    DynamicsSolverInitializer dynamics_solver_initializer = DynamicsSolverInitializer(init);
    this->SetDt(dynamics_solver_initializer.dt);
    this->SetIntegrator(dynamics_solver_initializer.Integrator);

    // Just store the control limits supplied.
    //  They will need to be reshaped to the correct size when num_velocities_ becomes known.
    raw_control_limits_low_ = dynamics_solver_initializer.ControlLimitsLow;
    raw_control_limits_high_ = dynamics_solver_initializer.ControlLimitsHigh;

    if (debug_) INFO_NAMED(object_name_, "Initialized DynamicsSolver of type " << GetObjectName() << " with dt=" << dynamics_solver_initializer.dt << " and integrator=" << dynamics_solver_initializer.Integrator);
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::AssignScene(std::shared_ptr<Scene> scene_in)
{
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, 1> AbstractDynamicsSolver<T, NX, NU>::F(const StateVector& x, const ControlVector& u)
{
    // TODO: Switch to Integrate here...
    return SimulateOneStep(x, u);  // ToDo: Fold SimulateOneStep into here
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::SetDt(double dt_in)
{
    if (dt_in < 0.0001) ThrowPretty("dt needs to be strictly greater than 0. Provided: " << dt_in);
    dt_ = dt_in;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, 1> AbstractDynamicsSolver<T, NX, NU>::SimulateOneStep(const StateVector& x, const ControlVector& u)
{
    switch (integrator_)
    {
        // Forward Euler (RK1), symplectic Euler
        case Integrator::RK1:
        case Integrator::SymplecticEuler:
        {
            StateVector xdot = f(x, u);
            StateVector xout(get_num_state());
            Integrate(x, xdot, dt_, xout);
            return xout;
        }
        // NB: RK2 and RK4 are currently deactivated as we do not yet have correct derivatives for state transitions.
        /*// Explicit trapezoid rule (RK2)
        case Integrator::RK2:
        {
            assert(num_positions_ == num_velocities_);  // If this is not true, we should have specialised methods.

            StateVector xdot0 = f(x, u);
            StateVector x1est = x + dt_ * xdot0;  // explicit Euler step
            StateVector xdot1 = f(x1est, u);

            // 2nd order result: x = x0 + dt (xd0+xd1)/2.
            return x + (dt_ / 2.) * (xdot0 + xdot1);
        }
        // Runge-Kutta 4
        case Integrator::RK4:
        {
            assert(num_positions_ == num_velocities_);  // If this is not true, we should have specialised methods.

            StateVector k1 = dt_ * f(x, u);
            StateVector k2 = dt_ * f(x + 0.5 * k1, u);
            StateVector k3 = dt_ * f(x + 0.5 * k2, u);
            StateVector k4 = dt_ * f(x + k3, u);
            StateVector dx = (k1 + k4) / 6. + (k2 + k3) / 3.;

            return x + dx;
        }*/
        default:
            ThrowPretty("Not implemented!");
    };
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::Integrate(const StateVector& x, const StateVector& dx, const double dt, StateVector& xout)
{
    assert(num_positions_ == num_velocities_);  // Integration on manifolds needs to be handled using function overloads in specific dynamics solvers.
    assert(x.size() == get_num_state());
    assert(dx.size() == get_num_state_derivative());
    assert(xout.size() == get_num_state());
    if (dt < 1e-6) ThrowPretty("dt needs to be positive!");

    switch (integrator_)
    {
        // Forward Euler (RK1)
        case Integrator::RK1:
        {
            xout.noalias() = x + dt * dx;
        }
        break;

        // Semi-implicit Euler
        case Integrator::SymplecticEuler:
        {
            StateVector dx_new(get_num_state_derivative());
            dx_new.head(num_positions_) = dt * x.tail(num_velocities_) + (dt * dt) * dx.tail(num_velocities_);
            dx_new.tail(num_velocities_) = dt * dx.tail(num_velocities_);
            xout.noalias() = x + dx_new;

            // xout.tail(num_velocities_).noalias() = x.tail(num_velocities_) + dt * dx.tail(num_velocities_);  // Integrate acceleration to velocity
            // xout.head(num_positions_).noalias() = x.head(num_positions_) + dt * xout.tail(num_velocities_);  // Integrate position with new velocity
        }
        break;

        default:
            ThrowPretty("Not implemented!");
            // TODO implement the other solvers, but how to get dx updated?!
    };
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, 1> AbstractDynamicsSolver<T, NX, NU>::Simulate(const StateVector& x, const ControlVector& u, T t)
{
    const int num_timesteps = static_cast<int>(t / dt_);
    StateVector x_t_plus_1 = x;
    for (int i = 0; i < num_timesteps; ++i)
    {
        x_t_plus_1 = SimulateOneStep(x_t_plus_1, u);
    }
    return x_t_plus_1;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, Eigen::Dynamic, 1> AbstractDynamicsSolver<T, NX, NU>::GetPosition(Eigen::VectorXdRefConst x_in)
{
    assert(x_in.size() == get_num_state());
    return x_in.head(num_positions_).eval();
}

template <typename T, int NX, int NU>
int AbstractDynamicsSolver<T, NX, NU>::get_num_controls() const
{
    return num_controls_;
}

template <typename T, int NX, int NU>
int AbstractDynamicsSolver<T, NX, NU>::get_num_positions() const
{
    return num_positions_;
}

template <typename T, int NX, int NU>
int AbstractDynamicsSolver<T, NX, NU>::get_num_velocities() const
{
    return num_velocities_;
}

template <typename T, int NX, int NU>
int AbstractDynamicsSolver<T, NX, NU>::get_num_state() const
{
    if (num_state_ == -1)
        return num_positions_ + num_velocities_;
    else
        return num_state_;
}

template <typename T, int NX, int NU>
int AbstractDynamicsSolver<T, NX, NU>::get_num_state_derivative() const
{
    if (num_state_derivative_ == -1)
        return 2 * num_velocities_;
    else
        return num_state_derivative_;
}

template <typename T, int NX, int NU>
T AbstractDynamicsSolver<T, NX, NU>::get_dt() const
{
    return dt_;
}

template <typename T, int NX, int NU>
Integrator AbstractDynamicsSolver<T, NX, NU>::get_integrator() const
{
    return integrator_;
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::set_integrator(Integrator integrator_in)
{
    integrator_ = integrator_in;
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::SetIntegrator(const std::string& integrator_in)
{
    if (integrator_in == "RK1")
        integrator_ = Integrator::RK1;
    else if (integrator_in == "SymplecticEuler")
        integrator_ = Integrator::SymplecticEuler;
    else if (integrator_in == "RK2")
        integrator_ = Integrator::RK2;
    else if (integrator_in == "RK4")
        integrator_ = Integrator::RK4;
    else
        ThrowPretty("Unknown integrator: " << integrator_in);
}

template <typename T, int NX, int NU>
const Eigen::MatrixXd& AbstractDynamicsSolver<T, NX, NU>::get_control_limits()
{
    if (!control_limits_initialized_)
        set_control_limits(raw_control_limits_low_, raw_control_limits_high_);
    return control_limits_;
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::set_control_limits(Eigen::VectorXdRefConst control_limits_low, Eigen::VectorXdRefConst control_limits_high)
{
    if (num_controls_ == -1)
        ThrowPretty("Attempting to set control limits before num_controls is set.");

    control_limits_initialized_ = true;
    control_limits_ = Eigen::MatrixXd(num_controls_, 2);

    if (control_limits_low.size() == num_controls_)
        control_limits_.col(0) = control_limits_low;
    else if (control_limits_low.size() == 1)
        control_limits_.col(0) = Eigen::VectorXd::Constant(num_controls_, control_limits_low(0));
    else
        ThrowPretty("Wrong control limits (low) size. Should either be 1 or " << num_controls_);

    if (control_limits_high.size() == num_controls_)
        control_limits_.col(1) = control_limits_high;
    else if (control_limits_high.size() == 1)
        control_limits_.col(1) = Eigen::VectorXd::Constant(num_controls_, control_limits_high(0));
    else
        ThrowPretty("Wrong control limits (high) size. Should either be 1 or " << num_controls_);
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::ClampToStateLimits(Eigen::Ref<Eigen::VectorXd> state_in)
{
    if (!has_state_limits_) ThrowPretty("No StateLimits!");
    if (state_in.size() != get_num_state()) ThrowPretty("Wrong size state passed in!");
    assert(state_in.size() == state_limits_lower_.size());
    assert(state_limits_lower_.size() == state_limits_upper_.size());

    state_in = state_in.cwiseMax(state_limits_lower_).cwiseMin(state_limits_upper_);
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::InitializeSecondOrderDerivatives()
{
    if (second_order_derivatives_initialized_)
        return;

    const int ndx = get_num_state_derivative();

    fxx_default_ = Eigen::Tensor<T, 3>(ndx, ndx, ndx);
    fxx_default_.setZero();

    fuu_default_ = Eigen::Tensor<T, 3>(ndx, num_controls_, num_controls_);
    fuu_default_.setZero();

    fxu_default_ = Eigen::Tensor<T, 3>(ndx, ndx, num_controls_);
    fxu_default_.setZero();

    second_order_derivatives_initialized_ = true;
}

template <typename T, int NX, int NU>
Eigen::Tensor<T, 3> AbstractDynamicsSolver<T, NX, NU>::fxx(const StateVector& x, const ControlVector& u)
{
    if (!second_order_derivatives_initialized_) InitializeSecondOrderDerivatives();
    return fxx_default_;
}

template <typename T, int NX, int NU>
Eigen::Tensor<T, 3> AbstractDynamicsSolver<T, NX, NU>::fuu(const StateVector& x, const ControlVector& u)
{
    if (!second_order_derivatives_initialized_) InitializeSecondOrderDerivatives();
    return fuu_default_;
}

template <typename T, int NX, int NU>
Eigen::Tensor<T, 3> AbstractDynamicsSolver<T, NX, NU>::fxu(const StateVector& x, const ControlVector& u)
{
    if (!second_order_derivatives_initialized_) InitializeSecondOrderDerivatives();
    return fxu_default_;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NU, 1> AbstractDynamicsSolver<T, NX, NU>::InverseDynamics(const StateVector& state)
{
    ThrowPretty("This dynamics solver does not support inverse dynamics!");
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, NX> AbstractDynamicsSolver<T, NX, NU>::fx_fd(const StateVector& x, const ControlVector& u)
{
    const int nx = get_num_state();
    const int ndx = get_num_state_derivative();

    // This finite differencing only works with RK1 due to Integrate(x, u, dt)
    // We thus store the previous Integrator, set it to RK1, then set it back
    // afterwards.
    Integrator previous_integrator = integrator_;
    integrator_ = Integrator::RK1;  // Note, this by-passes potentially overriden virtual set_integrator callbacks

    // Finite differences
    Eigen::MatrixXd fx_fd(ndx, ndx);
    constexpr double eps = 1e-6;
    Eigen::VectorXd x_low(nx), x_high(nx), xdiff(ndx);
    for (int i = 0; i < ndx; ++i)
    {
        xdiff.setZero();
        xdiff(i) = eps / 2.0;

        Integrate(x, -xdiff, 1., x_low);
        Integrate(x, xdiff, 1., x_high);

        fx_fd.col(i) = (f(x_high, u) - f(x_low, u)) / eps;
    }

    // Reset integrator
    integrator_ = previous_integrator;

    return fx_fd;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, NX> AbstractDynamicsSolver<T, NX, NU>::fx(const StateVector& x, const ControlVector& u)
{
    return fx_fd(x, u);
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, NU> AbstractDynamicsSolver<T, NX, NU>::fu_fd(const StateVector& x, const ControlVector& u)
{
    const int ndx = get_num_state_derivative();

    // Finite differences
    constexpr double eps = 1e-6;
    Eigen::MatrixXd fu_fd(ndx, num_controls_);
    Eigen::VectorXd u_low(num_controls_), u_high(num_controls_);
    for (int i = 0; i < num_controls_; ++i)
    {
        u_low = u;
        u_high = u;
        u_low(i) -= eps / 2.0;
        u_high(i) += eps / 2.0;

        fu_fd.col(i) = (f(x, u_high) - f(x, u_low)) / eps;
    }

    return fu_fd;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, NU> AbstractDynamicsSolver<T, NX, NU>::fu(const StateVector& x, const ControlVector& u)
{
    return fu_fd(x, u);
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::ComputeDerivatives(const StateVector& x, const ControlVector& u)
{
    // Compute derivatives of differential dynamics
    fx_ = fx(x, u);
    fu_ = fu(x, u);

    // Compute derivatives of state transition function
    // NB: In our "f" order, we have both velocity and acceleration. We only need the acceleration derivative part:
    Eigen::Block<Eigen::MatrixXd> da_dx = fx_.block(num_velocities_, 0, num_velocities_, get_num_state_derivative());
    Eigen::Block<Eigen::MatrixXd> da_du = fu_.block(num_velocities_, 0, num_velocities_, num_controls_);

    // TODO: Do this only once...
    Fx_.setZero(get_num_state_derivative(), get_num_state_derivative());
    Fu_.setZero(get_num_state_derivative(), get_num_controls());

    switch (integrator_)
    {
        // Forward Euler (RK1)
        case Integrator::RK1:
        {
            Fx_.topRightCorner(num_velocities_, num_velocities_).diagonal().array() = dt_;
            Fx_.bottomRows(num_velocities_).noalias() = dt_ * da_dx;
            Fx_.diagonal().array() += 1.0;

            Fu_.bottomRows(num_velocities_).noalias() = da_du * dt_;
        }
        break;
        // Semi-implicit Euler
        case Integrator::SymplecticEuler:
        {
            Fx_.topRows(num_velocities_).noalias() = da_dx * dt_ * dt_;
            Fx_.bottomRows(num_velocities_).noalias() = da_dx * dt_;
            Fx_.topRightCorner(num_velocities_, num_velocities_).diagonal().array() += dt_;
            Fx_.diagonal().array() += 1.0;

            Fu_.topRows(num_velocities_).noalias() = da_du * dt_ * dt_;  // Semi-implicit: configuration changes with acceleration in same step
            Fu_.bottomRows(num_velocities_).noalias() = da_du * dt_;
        }
        break;
        default:
            ThrowPretty("Not implemented!");
    };
}

template <typename T, int NX, int NU>
const Eigen::Matrix<T, NX, NX>& AbstractDynamicsSolver<T, NX, NU>::get_fx() const
{
    return fx_;
}

template <typename T, int NX, int NU>
const Eigen::Matrix<T, NX, NU>& AbstractDynamicsSolver<T, NX, NU>::get_fu() const
{
    return fu_;
}

template <typename T, int NX, int NU>
const Eigen::Matrix<T, NX, NX>& AbstractDynamicsSolver<T, NX, NU>::get_Fx() const
{
    return Fx_;
}

template <typename T, int NX, int NU>
const Eigen::Matrix<T, NX, NU>& AbstractDynamicsSolver<T, NX, NU>::get_Fu() const
{
    return Fu_;
}

template class AbstractDynamicsSolver<double, Eigen::Dynamic, Eigen::Dynamic>;
}  // namespace exotica
