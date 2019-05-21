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
template class AbstractDynamicsSolver<double, Eigen::Dynamic, Eigen::Dynamic>;

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

    if (debug_) INFO_NAMED(object_name_, "Initialized DynamicsSolver of type " << GetObjectName() << " with dt=" << dynamics_solver_initializer.dt << " and integrator=" << dynamics_solver_initializer.Integrator);
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::AssignScene(std::shared_ptr<Scene> scene_in)
{
}

template <typename T, int NX, int NU>
void AbstractDynamicsSolver<T, NX, NU>::SetDt(double dt_in)
{
    if (dt_in < 0.0001) ThrowPretty("dt needs to be strictly greater than 0. Provided: " << dt_in);
    dt_ = dt_in;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, 1> AbstractDynamicsSolver<T, NX, NU>::Integrate(const StateVector& x, const ControlVector& u)
{
    switch (integrator_)
    {
        // Forward Euler (RK1)
        case Integrator::RK1:
        {
            StateVector xdot = f(x, u);
            return x + dt_ * xdot;
        }
        // Explicit trapezoid rule (RK2)
        case Integrator::RK2:
        {
            StateVector xdot0 = f(x, u);
            StateVector x1est = x + dt_ * xdot0;  // explicit Euler step
            StateVector xdot1 = f(x1est, u);

            // 2nd order result: x = x0 + dt (xd0+xd1)/2.
            return x + (dt_ / 2.) * (xdot0 + xdot1);
        }
        // Runge-Kutta 4
        case Integrator::RK4:
        {
            StateVector k1 = dt_ * f(x, u);
            StateVector k2 = dt_ * f(x + 0.5 * k1, u);
            StateVector k3 = dt_ * f(x + 0.5 * k2, u);
            StateVector k4 = dt_ * f(x + k3, u);
            StateVector dx = (k1 + k4) / 6. + (k2 + k3) / 3.;

            return x + dx;
        }
    };
    ThrowPretty("Not implemented!");
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, NX, 1> AbstractDynamicsSolver<T, NX, NU>::Simulate(const StateVector& x, const ControlVector& u, T t)
{
    const int num_timesteps = static_cast<int>(t / dt_);
    StateVector x_t_plus_1 = x;
    for (int i = 0; i < num_timesteps; ++i)
    {
        x_t_plus_1 = Integrate(x_t_plus_1, u);
    }
    return x_t_plus_1;
}

template <typename T, int NX, int NU>
Eigen::Matrix<T, Eigen::Dynamic, 1> AbstractDynamicsSolver<T, NX, NU>::GetPosition(Eigen::VectorXdRefConst x_in)
{
    assert(x_in.size() == (num_positions_ + num_velocities_));
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
void AbstractDynamicsSolver<T, NX, NU>::SetIntegrator(std::string integrator_in)
{
    if (integrator_in == "RK1")
        integrator_ = Integrator::RK1;
    else if (integrator_in == "RK2")
        integrator_ = Integrator::RK2;
    else if (integrator_in == "RK4")
        integrator_ = Integrator::RK4;
    else
        ThrowPretty("Unknown integrator: " << integrator_in);
}

}  // namespace exotica
