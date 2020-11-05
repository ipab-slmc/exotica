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
#include <exotica_core/tools.h>

#include <exotica_core/dynamics_solver_initializer.h>

#define REGISTER_DYNAMICS_SOLVER_TYPE(TYPE, DERIV) EXOTICA_CORE_REGISTER(exotica::DynamicsSolver, TYPE, DERIV)

namespace exotica
{
class Scene;

enum Integrator
{
    RK1 = 0,          ///< Forward Euler (explicit)
    SymplecticEuler,  ///< Semi-Implicit Euler
    RK2,              ///< Explicit trapezoid rule
    RK4,              ///< Runge-Kutta 4
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

    /// \brief Forward dynamics. This computes the differential dynamics.
    virtual StateVector f(const StateVector& x, const ControlVector& u) = 0;

    /// \brief State transition function. This internally computes the differential dynamics and applies the chosen integration scheme.
    virtual StateVector F(const StateVector& x, const ControlVector& u);

    /// \brief Computes derivatives fx, fu, Fx, Fu [single call for efficiency, derivatives can be retrieved with get_fx, get_fu, get_Fx, get_Fu]
    virtual void ComputeDerivatives(const StateVector& x, const ControlVector& u);

    /// \brief Returns derivative Fx computed by ComputeDerivatives
    const StateDerivative& get_Fx() const;

    /// \brief Returns derivative Fu computed by ComputeDerivatives
    const ControlDerivative& get_Fu() const;

    /// \brief Returns derivative fx computed by ComputeDerivatives
    const StateDerivative& get_fx() const;

    /// \brief Returns derivative fu computed by ComputeDerivatives
    const ControlDerivative& get_fu() const;

    /// \brief Derivative of the forward dynamics w.r.t. the state
    virtual StateDerivative fx(const StateVector& x, const ControlVector& u);

    /// \brief Derivative of the forward dynamics w.r.t. the control
    virtual ControlDerivative fu(const StateVector& x, const ControlVector& u);

    /// \brief Derivative of the forward dynamics w.r.t. the state [finite differencing]
    StateDerivative fx_fd(const StateVector& x, const ControlVector& u);

    /// \brief Derivative of the forward dynamics w.r.t. the control [finite differencing]
    ControlDerivative fu_fd(const StateVector& x, const ControlVector& u);

    /// \brief Returns whether second-order derivatives are available
    const bool& get_has_second_order_derivatives() const
    {
        return has_second_order_derivatives_;
    }

    // NOTE: Second order derivatives a 3D matrices, i.e. tensors
    //  We use the numerator convention (see https://en.wikipedia.org/wiki/Matrix_calculus)
    // X_i,j,k = d(X_i,j)/d x_k
    //
    // Additionally, the first subscript is the *second* partial derivative.
    //  I.e. f_xu = (f_u)_x
    // TODO: Eigen::Tensor to be replaced with exotica::Hessian
    virtual Eigen::Tensor<T, 3> fxx(const StateVector& x, const ControlVector& u);
    virtual Eigen::Tensor<T, 3> fuu(const StateVector& x, const ControlVector& u);
    virtual Eigen::Tensor<T, 3> fxu(const StateVector& x, const ControlVector& u);

    /// \brief Simulates the dynamic system from starting state x using control u for t seconds
    ///
    /// Simulates the system and steps the simulation by timesteps dt for a total time of t using the specified integration scheme starting from state x and with controls u.
    // TODO: To be deprecated - or at least remove its use - as it's difficult to get partial derivatives.
    StateVector Simulate(const StateVector& x, const ControlVector& u, T t);

    /// \brief Return the difference of two state vectors.
    ///     Used when e.g. angle differences need to be wrapped from [-pi; pi]
    ///     Returns x_1-x_2
    virtual StateVector StateDelta(const StateVector& x_1, const StateVector& x_2)
    {
        assert(x_1.size() == x_2.size());
        return x_1 - x_2;
    }

    /// \brief Return the difference of the StateDelta operation between two state vectors.
    ///     The ArgumentPosition argument can be used to select whether to take derivative w.r.t. x_1 or x_2.
    virtual Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> dStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second)
    {
        assert(x_1.size() == x_2.size());
        assert(first_or_second == ArgumentPosition::ARG0 || first_or_second == ArgumentPosition::ARG1);

        if (first_or_second == ArgumentPosition::ARG0)
            return Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(get_num_state_derivative(), get_num_state_derivative());
        else
            return -1.0 * Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(get_num_state_derivative(), get_num_state_derivative());
    }

    virtual Hessian ddStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second)
    {
        assert(x_1.size() == x_2.size());

        // In Euclidean spaces, this is zero.
        Hessian ddStateDelta;
        ddStateDelta.setConstant(get_num_state_derivative(), Eigen::MatrixXd::Zero(get_num_state_derivative(), get_num_state_derivative()));
        return ddStateDelta;
    }

    /// \brief Returns the position-part of the state vector to update the scene.
    /// For types including SE(3) and rotation, convert to the appropriate representation here by overriding this method.
    virtual Eigen::Matrix<T, Eigen::Dynamic, 1> GetPosition(Eigen::VectorXdRefConst x_in);

    /// \brief Returns number of controls
    int get_num_controls() const;

    /// \brief Returns number of positions
    int get_num_positions() const;

    /// \brief Returns number of velocities
    int get_num_velocities() const;

    /// \brief Returns size of state space (nx)
    int get_num_state() const;

    /// \brief Returns size of derivative vector of state space (ndx)
    int get_num_state_derivative() const;

    /// \brief Returns integration timestep dt
    T get_dt() const;

    /// \brief Returns used integration scheme
    Integrator get_integrator() const;

    /// \brief Sets integrator type
    void set_integrator(Integrator integrator_in);

    /// \brief Sets integrator type based on request string
    void SetIntegrator(const std::string& integrator_in);

    /// \brief Returns the control limits vector.
    //  returns: Two-column matrix, first column contains low control limits,
    //      second - the high control limits
    const Eigen::MatrixXd& get_control_limits();

    /// \brief Sets the control limits
    void set_control_limits(Eigen::VectorXdRefConst control_limits_low, Eigen::VectorXdRefConst control_limits_high);

    /// \brief Returns whether state limits are available
    const bool& get_has_state_limits() const
    {
        return has_state_limits_;
    }

    /// \brief Clamps the passed in state to the state limits
    void ClampToStateLimits(Eigen::Ref<Eigen::VectorXd> state_in);

    /// \brief Returns a control vector corresponding to the state vector assuming zero acceleration
    virtual ControlVector InverseDynamics(const StateVector& state);

    /// \brief Integrates without performing dynamics.
    virtual void Integrate(const StateVector& x, const StateVector& dx, const double dt, StateVector& xout);

private:
    bool control_limits_initialized_ = false;
    Eigen::VectorXd raw_control_limits_low_, raw_control_limits_high_;

protected:
    int num_controls_ = -1;          ///< Number of controls in the dynamic system.
    int num_positions_ = -1;         ///< Number of positions in the dynamic system.
    int num_velocities_ = -1;        ///< Number of velocities in the dynamic system.
    int num_state_ = -1;             ///< Size of state space (num_positions + num_velocities)
    int num_state_derivative_ = -1;  ///< Size of the tangent vector to the state space (2 * num_velocities)

    bool has_second_order_derivatives_ = false;          ///< Whether this solver provides second order derivatives. If false (default), assumed to be all zeros.
    bool second_order_derivatives_initialized_ = false;  ///< Whether fxx, fxu and fuu have been initialized to 0.

    bool has_state_limits_ = false;       ///< Whether the solver specifies state limits
    Eigen::VectorXd state_limits_lower_;  ///< Lower state limits (configuration and velocity)
    Eigen::VectorXd state_limits_upper_;  ///< Upper state limits (configuration and velocity)

    T dt_ = 0.01;                              ///< Internal timestep used for integration. Defaults to 10ms.
    Integrator integrator_ = Integrator::RK1;  ///< Chosen integrator. Defaults to Euler (RK1).
    // TODO: Need to enforce control limits.
    // First column is the low limits, second is the high limits.
    Eigen::MatrixXd control_limits_;  ///< ControlLimits. Default is empty vector.

    /// \brief Integrates the dynamic system from state x with controls u applied for one timestep dt using the selected integrator.
    // TODO: To be deprecated in favour of explicit call to Integrate in Simulate
    virtual StateVector SimulateOneStep(const StateVector& x, const ControlVector& u);

    void InitializeSecondOrderDerivatives();
    Eigen::Tensor<T, 3> fxx_default_, fuu_default_, fxu_default_;

    Eigen::MatrixXd fx_;  ///< Internal storage of differential dynamics partial derivative fx computed by ComputeDerivatives
    Eigen::MatrixXd fu_;  ///< Internal storage of differential dynamics partial derivative fu computed by ComputeDerivatives
    Eigen::MatrixXd Fx_;  ///< Internal storage of state transition partial derivative Fx computed by ComputeDerivatives
    Eigen::MatrixXd Fu_;  ///< Internal storage of state transition partial derivative Fu computed by ComputeDerivatives
};

typedef AbstractDynamicsSolver<double, Eigen::Dynamic, Eigen::Dynamic> DynamicsSolver;

typedef std::shared_ptr<exotica::DynamicsSolver> DynamicsSolverPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_DYNAMICS_SOLVER_H_
