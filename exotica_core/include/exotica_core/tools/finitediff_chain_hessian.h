// Copyright (C) 2018
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Modified from unsupported/Eigen/src/AutoDiff/AutoDiffJacobian.h
// and unsupported/Eigen/src/NumericalDiff/NumericalDiff.h

#ifndef EIGEN_FINITEDIFF_CHAIN_HESSIAN_H_
#define EIGEN_FINITEDIFF_CHAIN_HESSIAN_H_

#include <functional>

#include <exotica_core/tools/finitediff_chain_jacobian.h>
#include <exotica_core/tools/finitediff_common.h>
#include <exotica_core/tools/functor.h>

namespace Eigen
{
template <typename Functor, NumericalDiffMode mode = Forward>
class FiniteDiffChainHessian : public Functor
{
public:
    typedef typename Functor::InputType InputType;
    typedef typename Functor::ValueType ValueType;
    typedef typename ValueType::Scalar Scalar;

    enum
    {
        InputsAtCompileTime = InputType::RowsAtCompileTime,
        ValuesAtCompileTime = ValueType::RowsAtCompileTime,
        JacobianInputsAtCompileTime = Functor::JacobianColsAtCompileTime  // JacobianInputsAtCompileTime no longer have to match InputsAtCompileTime
    };

    typedef Matrix<Scalar, ValuesAtCompileTime, JacobianInputsAtCompileTime> JacobianType;
    typedef Matrix<Scalar, JacobianInputsAtCompileTime, 1> InputJacobianRowType;
    typedef Array<Matrix<Scalar, JacobianInputsAtCompileTime, JacobianInputsAtCompileTime>, ValuesAtCompileTime, 1> HessianType;
    typedef typename JacobianType::Index Index;

    typedef std::function<void(const InputJacobianRowType &, InputType &)> UpdateFunctionCallbackType;

    UpdateFunctionCallbackType update_ = [](const InputJacobianRowType &jx, InputType &x) { x = jx; };
    Scalar epsfcn_;

    FiniteDiffChainHessian(Scalar epsfcn = 0.) : Functor(), epsfcn_(epsfcn) {}
    FiniteDiffChainHessian(const Functor &f, Scalar epsfcn = 0.) : Functor(f), epsfcn_(epsfcn) {}
    FiniteDiffChainHessian(const Functor &f, UpdateFunctionCallbackType update, Scalar epsfcn = 0.) : Functor(f), update_(update), epsfcn_(epsfcn) {}
// forward constructors
#if EIGEN_HAS_VARIADIC_TEMPLATES
    template <typename... T>
    FiniteDiffChainHessian(Scalar epsfcn = 0., const T &... Values) : Functor(Values...), epsfcn_(epsfcn)
    {
    }
    template <typename... T>
    FiniteDiffChainHessian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T &... Values) : Functor(Values...), update_(update), epsfcn_(epsfcn)
    {
    }
#else
    template <typename T0>
    FiniteDiffChainHessian(const T0 &a0, Scalar epsfcn = 0.) : Functor(a0), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1>
    FiniteDiffChainHessian(const T0 &a0, const T1 &a1, Scalar epsfcn = 0.) : Functor(a0, a1), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1, typename T2>
    FiniteDiffChainHessian(const T0 &a0, const T1 &a1, const T2 &a2, Scalar epsfcn = 0.) : Functor(a0, a1, a2), epsfcn_(epsfcn)
    {
    }

    template <typename T0>
    FiniteDiffChainHessian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0) : Functor(a0), update_(update), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1>
    FiniteDiffChainHessian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0, const T1 &a1) : Functor(a0, a1), update_(update), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1, typename T2>
    FiniteDiffChainHessian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0, const T1 &a1, const T2 &a2) : Functor(a0, a1, a2), update_(update), epsfcn_(epsfcn)
    {
    }
#endif

#if EIGEN_HAS_VARIADIC_TEMPLATES
    // Some compilers don't accept variadic parameters after a default parameter,
    // i.e., we can't just write _jac=0 but we need to overload operator():
    EIGEN_STRONG_INLINE
    int operator()(const InputJacobianRowType &_jx, ValueType &v) const
    {
        InputType x;
        update_(_jx, x);
        this->operator()(x, v);
        return 1;
    }

    template <typename... ParamsType>
    int operator()(const InputJacobianRowType &_jx, ValueType &v, const ParamsType &... Params) const
    {
        InputType x;
        update_(_jx, x);
        this->operator()(x, v, Params...);
        return 1;
    }

    template <typename... ParamsType>
    int operator()(const InputJacobianRowType &_jx, ValueType &v, JacobianType &jac, const ParamsType &... Params) const
    {
        FiniteDiffChainJacobian<Functor, mode> autoj(*static_cast<const Functor *>(this), update_, epsfcn_);
        return autoj(_jx, v, jac, Params...);
    }

    template <typename... ParamsType>
    int operator()(const InputJacobianRowType &_jx, ValueType &v, JacobianType &jac, HessianType &hess, const ParamsType &... Params) const
#else
    EIGEN_STRONG_INLINE
    int operator()(const InputJacobianRowType &_jx, ValueType &v) const
    {
        InputType x;
        update_(_jx, x);
        this->operator()(x, v);
        return 1;
    }

    int operator()(const InputJacobianRowType &_jx, ValueType &v, JacobianType &jac) const
    {
        FiniteDiffChainJacobian<Functor, mode> autoj(*static_cast<const Functor *>(this), update_, epsfcn_);
        return autoj(_jx, v, jac);
    }

    int operator()(const InputJacobianRowType &_jx, ValueType &v, JacobianType &jac, HessianType &hess) const
#endif
    {
        using std::sqrt;
        using std::abs;
        // Local variables
        FiniteDiffChainJacobian<Functor, mode> autoj(*static_cast<const Functor *>(this), update_, epsfcn_);
        Scalar h;
        int nfev = 0;
        const typename InputJacobianRowType::Index n = _jx.size();
        const typename ValueType::Index m = jac.rows();
        const Scalar eps = sqrt(((std::max)(epsfcn_, NumTraits<Scalar>::epsilon())));
        JacobianType jac1(jac.rows(), jac.cols()), jac2(jac.rows(), jac.cols());
        InputJacobianRowType jx = _jx;
        ValueType _v = v;
        Index cols = jac.cols();
        if (JacobianInputsAtCompileTime == Dynamic)
        {
            hess.resize(m);
            for (Index i = 0; i < m; ++i)
            {
                hess[i].resize(cols, cols);
                hess[i].setZero();
            }
        }

#if EIGEN_HAS_VARIADIC_TEMPLATES
        nfev += autoj(_jx, v, jac, Params...);
#else
        nfev += autoj(_jx, v, jac);
#endif

        switch (mode)
        {
            case Forward:
                // copy J(x)
                jac1 = jac;
                break;
            case Central:
                // do nothing
                break;
            default:
                eigen_assert(false);
        };

        // Function Body
        for (int j = 0; j < n; ++j)
        {
            h = eps * abs(jx[j]);
            if (h == 0.)
            {
                h = eps;
            }
            h = sqrt(sqrt(h));
            switch (mode)
            {
                case Forward:
                    jx[j] += h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    nfev += autoj(jx, _v, jac2, Params...);
#else
                    nfev += autoj(jx, _v, jac2);
#endif
                    jx[j] = _jx[j];
                    for (int l = 0; l < m; ++l)
                    {
                        hess[l].row(j) = (jac2.row(l) - jac1.row(l)) / h;
                    }
                    break;
                case Central:
                    jx[j] += h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    nfev += autoj(jx, _v, jac2, Params...);
#else
                    nfev += autoj(jx, _v, jac2);
#endif
                    jx[j] -= 2 * h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    nfev += autoj(jx, _v, jac1, Params...);
#else
                    nfev += autoj(jx, _v, jac1);
#endif
                    jx[j] = _jx[j];
                    for (int l = 0; l < m; ++l)
                    {
                        hess[l].col(j) = (jac2.row(l) - jac1.row(l)) / (2.0 * h);
                    }
                    break;
                default:
                    eigen_assert(false);
            };
        }
        return nfev;
    }
};
}

#endif  // EIGEN_FINITEDIFF_CHAIN_HESSIAN_H_
