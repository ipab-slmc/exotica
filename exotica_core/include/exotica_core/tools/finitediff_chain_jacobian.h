// Copyright (C) 2018
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Modified from unsupported/Eigen/src/AutoDiff/AutoDiffJacobian.h
// and unsupported/Eigen/src/NumericalDiff/NumericalDiff.h

#ifndef EIGEN_FINITEDIFF_CHAIN_JACOBIAN_H_
#define EIGEN_FINITEDIFF_CHAIN_JACOBIAN_H_

#include <functional>

#include <exotica_core/tools/finitediff_common.h>
#include <exotica_core/tools/functor.h>

namespace Eigen
{
template <typename Functor, NumericalDiffMode mode = Forward>
class FiniteDiffChainJacobian : public Functor
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
    typedef typename JacobianType::Index Index;

    typedef std::function<void(const InputJacobianRowType &, InputType &)> UpdateFunctionCallbackType;

    UpdateFunctionCallbackType update_ = [](const InputJacobianRowType &jx, InputType &x) { x = jx; };
    Scalar epsfcn_;

    FiniteDiffChainJacobian(Scalar epsfcn = 0.) : Functor(), epsfcn_(epsfcn) {}
    FiniteDiffChainJacobian(const Functor &f, Scalar epsfcn = 0.) : Functor(f), epsfcn_(epsfcn) {}
    FiniteDiffChainJacobian(const Functor &f, UpdateFunctionCallbackType update, Scalar epsfcn = 0.) : Functor(f), update_(update), epsfcn_(epsfcn) {}
// forward constructors
#if EIGEN_HAS_VARIADIC_TEMPLATES
    template <typename... T>
    FiniteDiffChainJacobian(Scalar epsfcn = 0., const T &... Values) : Functor(Values...), epsfcn_(epsfcn)
    {
    }
    template <typename... T>
    FiniteDiffChainJacobian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T &... Values) : Functor(Values...), update_(update), epsfcn_(epsfcn)
    {
    }
#else
    template <typename T0>
    FiniteDiffChainJacobian(const T0 &a0, Scalar epsfcn = 0.) : Functor(a0), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1>
    FiniteDiffChainJacobian(const T0 &a0, const T1 &a1, Scalar epsfcn = 0.) : Functor(a0, a1), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1, typename T2>
    FiniteDiffChainJacobian(const T0 &a0, const T1 &a1, const T2 &a2, Scalar epsfcn = 0.) : Functor(a0, a1, a2), epsfcn_(epsfcn)
    {
    }

    template <typename T0>
    FiniteDiffChainJacobian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0) : Functor(a0), update_(update), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1>
    FiniteDiffChainJacobian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0, const T1 &a1) : Functor(a0, a1), update_(update), epsfcn_(epsfcn)
    {
    }
    template <typename T0, typename T1, typename T2>
    FiniteDiffChainJacobian(UpdateFunctionCallbackType update, Scalar epsfcn = 0., const T0 &a0, const T1 &a1, const T2 &a2) : Functor(a0, a1, a2), update_(update), epsfcn_(epsfcn)
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
#endif
    {
        using std::abs;
        using std::sqrt;
        // Local variables
        Scalar h;
        int nfev = 0;
        const typename InputJacobianRowType::Index n = _jx.size();
        const Scalar eps = sqrt(((std::max)(epsfcn_, NumTraits<Scalar>::epsilon())));
        ValueType val1, val2;
        InputJacobianRowType jx = _jx;
        InputType x;
        if (ValuesAtCompileTime == Dynamic)
        {
            val1.resize(v.rows());
            val2.resize(v.rows());
        }

#if EIGEN_HAS_VARIADIC_TEMPLATES
        update_(jx, x);
        Functor::operator()(x, v, Params...);
        ++nfev;
#else
        update_(jx, x);
        Functor::operator()(x, v);
        ++nfev;
#endif

        switch (mode)
        {
            case Forward:
                // copy f(x)
                val1 = v;
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
            switch (mode)
            {
                case Forward:
                    jx[j] += h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    update_(jx, x);
                    Functor::operator()(x, val2, Params...);
                    ++nfev;
#else
                    update_(jx, x);
                    Functor::operator()(x, val2);
                    ++nfev;
#endif
                    jx[j] = _jx[j];
                    jac.col(j) = (val2 - val1) / h;
                    break;
                case Central:
                    jx[j] += h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    update_(jx, x);
                    Functor::operator()(x, val2, Params...);
                    ++nfev;
#else
                    update_(jx, x);
                    Functor::operator()(x, val2);
                    ++nfev;
#endif
                    jx[j] -= 2 * h;
#if EIGEN_HAS_VARIADIC_TEMPLATES
                    update_(jx, x);
                    Functor::operator()(x, val1, Params...);
                    ++nfev;
#else
                    update_(jx, x);
                    Functor::operator()(x, val1);
                    ++nfev;
#endif
                    jx[j] = _jx[j];
                    jac.col(j) = (val2 - val1) / (2 * h);
                    break;
                default:
                    eigen_assert(false);
            };
        }
        return nfev;
    }
};
}  // namespace Eigen

#endif  // EIGEN_FINITEDIFF_CHAIN_JACOBIAN_H_
