// Copyright (C) 2018
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Modified from unsupported/Eigen/src/AutoDiff/AutoDiffJacobian.h

#ifndef EIGEN_AUTODIFF_CHAIN_HESSIAN_H_
#define EIGEN_AUTODIFF_CHAIN_HESSIAN_H_

#include <exotica_core/tools/autodiff_chain_jacobian.h>
#include <exotica_core/tools/autodiff_scalar.h>
#include <exotica_core/tools/functor.h>

namespace Eigen
{
template <typename Functor>
class AutoDiffChainHessian : public Functor
{
public:
    AutoDiffChainHessian() : Functor() {}
    AutoDiffChainHessian(const Functor &f) : Functor(f) {}
// forward constructors
#if EIGEN_HAS_VARIADIC_TEMPLATES
    template <typename... T>
    AutoDiffChainHessian(const T &... Values) : Functor(Values...)
    {
    }
#else
    template <typename T0>
    AutoDiffChainHessian(const T0 &a0) : Functor(a0)
    {
    }
    template <typename T0, typename T1>
    AutoDiffChainHessian(const T0 &a0, const T1 &a1) : Functor(a0, a1)
    {
    }
    template <typename T0, typename T1, typename T2>
    AutoDiffChainHessian(const T0 &a0, const T1 &a1, const T2 &a2) : Functor(a0, a1, a2)
    {
    }
#endif

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

    typedef Matrix<Scalar, InputsAtCompileTime, JacobianInputsAtCompileTime> InputJacobianType;  // Jacobian.cols() matches InputJacobian.cols()
    typedef Array<Matrix<Scalar, JacobianInputsAtCompileTime, JacobianInputsAtCompileTime>, ValuesAtCompileTime, 1> HessianType;
    typedef Array<Matrix<Scalar, JacobianInputsAtCompileTime, JacobianInputsAtCompileTime>, InputsAtCompileTime, 1> InputHessianType;
    typedef typename JacobianType::Index Index;

    typedef Matrix<Scalar, JacobianInputsAtCompileTime, 1> InnerDerivativeType;  // Derivative rows() matches InputJacobian.cols()
    typedef AutoDiffScalar<InnerDerivativeType> InnerActiveScalar;
    typedef Matrix<InnerActiveScalar, JacobianInputsAtCompileTime, 1> OuterDerivativeType;
    typedef AutoDiffScalar<OuterDerivativeType> OuterActiveScalar;

    typedef Matrix<OuterActiveScalar, InputsAtCompileTime, 1> ActiveInput;
    typedef Matrix<OuterActiveScalar, ValuesAtCompileTime, 1> ActiveValue;

#if EIGEN_HAS_VARIADIC_TEMPLATES
    // Some compilers don't accept variadic parameters after a default parameter,
    // i.e., we can't just write _jac=0 but we need to overload operator():
    EIGEN_STRONG_INLINE
    void operator()(const InputType &x, ValueType &v) const
    {
        this->operator()(x, v);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, const ParamsType &... Params) const
    {
        this->operator()(x, v, Params...);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const ParamsType &... Params) const
    {
        AutoDiffChainJacobian<Functor> autoj(*static_cast<const Functor *>(this));
        autoj(x, v, jac, Params...);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const InputJacobianType &ijac,
                    const ParamsType &... Params) const
    {
        AutoDiffChainJacobian<Functor> autoj(*static_cast<const Functor *>(this));
        autoj(x, v, jac, ijac, Params...);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, HessianType &hess, const ParamsType &... Params) const
    {
        this->operator()(x, v, jac, hess, nullptr, nullptr, Params...);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, HessianType &hess, const InputJacobianType &ijac, const InputHessianType &ihess,
                    const ParamsType &... Params) const
    {
        this->operator()(x, v, jac, hess, &ijac, &ihess, Params...);
    }

    // Optional parameter InputJacobian (_ijac)
    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, HessianType &hess, const InputJacobianType *_ijac = 0, const InputHessianType *_ihess = 0,
                    const ParamsType &... Params) const
#else
    EIGEN_STRONG_INLINE
    void operator()(const InputType &x, ValueType &v) const
    {
        this->operator()(x, v);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac) const
    {
        AutoDiffChainJacobian<Functor> autoj(*static_cast<const Functor *>(this));
        autoj(x, v, jac);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const InputJacobianType &ijac) const
    {
        AutoDiffChainJacobian<Functor> autoj(*static_cast<const Functor *>(this));
        autoj(x, v, jac, ijac);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac, HessianType &hess) const
    {
        this->operator()(x, v, jac, hess, nullptr, nullptr);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac, HessianType &hess, const InputJacobianType &ijac, const InputHessianType &ihess) const
    {
        this->operator()(x, v, jac, hess, &ijac, &ihess);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac = 0, HessianType &hess, const InputJacobianType *_ijac = 0, const InputHessianType *_ihess = 0) const
#endif
    {
        ActiveInput ax = x.template cast<OuterActiveScalar>();
        ActiveValue av(jac.rows());

        // Provide either both input jacobian and hessian, or none
        eigen_assert((_ijac && _ihess) || (!_ijac && !_ihess));

        if (!_ijac)
        {
            eigen_assert(InputsAtCompileTime == JacobianInputsAtCompileTime);

            if (InputsAtCompileTime == Dynamic)
                for (Index j = 0; j < jac.rows(); ++j)
                {
                    av[j].derivatives().resize(x.rows());
                    for (Index k = 0; k < x.rows(); ++k)
                        av[j].derivatives()[k].derivatives().resize(x.rows());
                }

            for (Index i = 0; i < x.rows(); ++i)
            {
                ax[i].derivatives() = InnerDerivativeType::Unit(x.rows(), i);
                ax[i].value().derivatives() = InnerDerivativeType::Unit(x.rows(), i);
                for (Index k = 0; k < x.rows(); ++k)
                {
                    ax[i].derivatives()(k).derivatives() = InnerDerivativeType::Zero(x.rows());
                }
            }
        }
        else
        {
            // If specified, copy derivatives from InputJacobian
            const InputJacobianType &ijac = *_ijac;
            const InputHessianType &ihess = *_ihess;

            eigen_assert(x.rows() == ihess.rows());
            eigen_assert(ijac.cols() == ihess[0].rows() && ijac.cols() == ihess[0].cols());

            if (InputsAtCompileTime == Dynamic)
                for (Index j = 0; j < jac.rows(); ++j)
                {
                    av[j].derivatives().resize(ijac.cols());
                    for (Index k = 0; k < ijac.cols(); ++k)
                        av[j].derivatives()[k].derivatives().resize(ijac.cols());
                }

            for (Index i = 0; i < x.rows(); ++i)
            {
                ax[i].derivatives() = ijac.row(i);
                ax[i].value().derivatives() = ijac.row(i);
                for (Index k = 0; k < ijac.cols(); ++k)
                {
                    ax[i].derivatives()(k).derivatives() = ihess[i].row(k);
                }
            }
        }

#if EIGEN_HAS_VARIADIC_TEMPLATES
        Functor::operator()(ax, av, Params...);
#else
        Functor::operator()(ax, av);
#endif

        Index cols = _ijac ? _ijac->cols() : x.rows();
        if (JacobianInputsAtCompileTime == Dynamic)
        {
            hess.resize(jac.rows());
            for (Index i = 0; i < jac.rows(); ++i)
                hess[i].resize(cols, cols);
        }

        for (Index i = 0; i < jac.rows(); ++i)
        {
            v[i] = av[i].value().value();
            jac.row(i) = av[i].value().derivatives();
            for (Index j = 0; j < cols; ++j)
                hess[i].row(j) = av[i].derivatives()[j].derivatives();
        }
    }
};

}  // namespace Eigen

#endif  // EIGEN_AUTODIFF_CHAIN_HESSIAN_H_
