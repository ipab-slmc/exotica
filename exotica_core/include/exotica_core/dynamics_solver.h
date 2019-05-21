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

#ifndef EXOTICA_CORE_DYNAMICS_SOLVER_H_
#define EXOTICA_CORE_DYNAMICS_SOLVER_H_

#include <exotica_core/factory.h>
#include <exotica_core/object.h>
#include <exotica_core/property.h>

#include <exotica_core/dynamics_solver_initializer.h>

#define REGISTER_DYNAMICS_SOLVER_TYPE(TYPE, DERIV) EXOTICA_CORE_REGISTER(exotica::DynamicsSolver, TYPE, DERIV)

namespace exotica
{
class Scene;

enum Integrator
{
    RK1 = 0,  ///< Forward Euler
    RK2,      ///< Explicit trapezoid rule
    RK4,      ///< Runge-Kutta 4
    // RK45
};

template <typename T, int NX, int NU>
class AbstractDynamicsSolver : public Object, Uncopyable, public virtual InstantiableBase
{
public:
    typedef Eigen::Matrix<T, NX, 1> StateVector;         ///< Convenience definition for a StateVector containing both position and velocity (dimension NX x 1)
    typedef Eigen::Matrix<T, NU, 1> ControlVector;       ///< Convenience definition for a ControlVector (dimension NU x 1)
    typedef Eigen::Matrix<T, NX, NX> StateDerivative;    ///< Convenience definition for a StateDerivative
    typedef Eigen::Matrix<T, NX, NU> ControlDerivative;  ///< Convenience definition for a ControlDerivative

    AbstractDynamicsSolver();
    virtual ~AbstractDynamicsSolver();

    /// \brief Instantiates the base properties of the DynamicsSolver
    virtual void InstantiateBase(const Initializer& init);

    /// \brief Passes the Scene of the PlanningProblem to the DynamicsSolver
    ///
    ///  Called immediately after creation of the DynamicsSolver plug-in using a pointer to the Scene of the PlanningProblem. This can be used to extract required information from the Scene, e.g., URDF, dimensionality, etc.
    virtual void AssignScene(std::shared_ptr<Scene> scene_in);

    /// \brief Sets the timestep dt to be used for integration.
    virtual void SetDt(double dt_in);

    /// \brief Forward dynamics
    virtual StateVector f(const StateVector& x, const ControlVector& u) = 0;

    /// \brief Derivative of the forward dynamics w.r.t. the state
    virtual StateDerivative fx(const StateVector& x, const ControlVector& u) = 0;

    /// \brief Derivative of the forward dynamics w.r.t. the control
    virtual ControlDerivative fu(const StateVector& x, const ControlVector& u) = 0;

    // TODO: 2nd-order derivatives to be implemented
    // virtual StateVector fxx(const StateVector& x, const ControlVector& u);
    // virtual StateVector fuu(const StateVector& x, const ControlVector& u);
    // virtual StateVector fxu(const StateVector& x, const ControlVector& u);

    /// \brief Simulates the dynamic system from starting state x using control u for t seconds
    ///
    /// Simulates the system and steps the simulation by timesteps dt for a total time of t using the specified integration scheme starting from state x and with controls u.
    StateVector Simulate(const StateVector& x, const ControlVector& u, T t);

    /// \brief Return the difference of two state vectors.
    ///     Used when e.g. angle differences need to be wrapped from [-pi; pi]
    virtual StateVector StateDelta(const StateVector& x_1, const StateVector& x_2);

    /// \brief Returns the position-part of the state vector to update the scene.
    /// For types including SE(3) and rotation, convert to the appropriate representation here by overriding this method.
    virtual Eigen::Matrix<T, Eigen::Dynamic, 1> GetPosition(Eigen::VectorXdRefConst x_in);

    /// \brief Returns number of controls
    int get_num_controls() const;

    /// \brief Returns number of positions
    int get_num_positions() const;

    /// \brief Returns number of velocities
    int get_num_velocities() const;

    /// \brief Returns integration timestep dt
    T get_dt() const;

    /// \brief Returns used integration scheme
    Integrator get_integrator() const;

    /// \brief Sets integrator type
    void set_integrator(Integrator integrator_in);

    /// \brief Sets integrator type based on request string
    void SetIntegrator(std::string integrator_in);

protected:
    int num_controls_;    ///< Number of controls in the dynamic system.
    int num_positions_;   ///< Number of positions in the dynamic system.
    int num_velocities_;  ///< Number of velocities in the dynamic system.

    T dt_ = 0.01;                              ///< Internal timestep used for integration. Defaults to 10ms.
    Integrator integrator_ = Integrator::RK1;  ///< Chosen integrator. Defaults to Euler (RK1).

    /// \brief Integrates the dynamic system from state x with controls u applied for one timestep dt using the selected integrator.
    inline StateVector Integrate(const StateVector& x, const ControlVector& u);
};

typedef AbstractDynamicsSolver<double, Eigen::Dynamic, Eigen::Dynamic> DynamicsSolver;

typedef std::shared_ptr<exotica::DynamicsSolver> DynamicsSolverPtr;
}

#endif  // EXOTICA_CORE_DYNAMICS_SOLVER_H_
