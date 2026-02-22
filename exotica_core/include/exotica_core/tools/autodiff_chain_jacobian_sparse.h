// Copyright (C) 2018
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Modified from unsupported/Eigen/src/AutoDiff/AutoDiffJacobian.h

#ifndef EIGEN_AUTODIFF_CHAIN_JACOBIAN_SPARSE_H_
#define EIGEN_AUTODIFF_CHAIN_JACOBIAN_SPARSE_H_

#include <exotica_core/tools/autodiff_scalar.h>
#include <exotica_core/tools/functor.h>

namespace Eigen
{
template <typename Functor>
class AutoDiffChainJacobianSparse : public Functor
{
public:
    AutoDiffChainJacobianSparse() : Functor() {}
    AutoDiffChainJacobianSparse(const Functor &f) : Functor(f) {}
// forward constructors
#if EIGEN_HAS_VARIADIC_TEMPLATES
    template <typename... T>
    AutoDiffChainJacobianSparse(const T &... Values) : Functor(Values...)
    {
    }
#else
    template <typename T0>
    AutoDiffChainJacobianSparse(const T0 &a0) : Functor(a0)
    {
    }
    template <typename T0, typename T1>
    AutoDiffChainJacobianSparse(const T0 &a0, const T1 &a1) : Functor(a0, a1)
    {
    }
    template <typename T0, typename T1, typename T2>
    AutoDiffChainJacobianSparse(const T0 &a0, const T1 &a1, const T2 &a2) : Functor(a0, a1, a2)
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

    typedef SparseMatrix<Scalar> JacobianType;

    typedef SparseMatrix<Scalar> InputJacobianType;  // Jacobian.cols() matches InputJacobian.cols()
    typedef typename JacobianType::Index Index;

    typedef SparseVector<Scalar> DerivativeType;  // Derivative rows() matches InputJacobian.cols()
    typedef AutoDiffScalar<DerivativeType> ActiveScalar;
    typedef typename DerivativeType::InnerIterator JacobianInnerIteratorType;

    typedef Matrix<ActiveScalar, InputsAtCompileTime, 1> ActiveInput;
    typedef Matrix<ActiveScalar, ValuesAtCompileTime, 1> ActiveValue;

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
        this->operator()(x, v, jac, nullptr, Params...);
    }

    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const InputJacobianType &ijac,
                    const ParamsType &... Params) const
    {
        this->operator()(x, v, jac, &ijac, Params...);
    }

    // Optional parameter InputJacobian (_ijac)
    template <typename... ParamsType>
    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const InputJacobianType *_ijac = 0,
                    const ParamsType &... Params) const
#else
    EIGEN_STRONG_INLINE
    void operator()(const InputType &x, ValueType &v) const
    {
        this->operator()(x, v);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac) const
    {
        this->operator()(x, v, jac, nullptr);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac, const InputJacobianType &ijac) const
    {
        this->operator()(x, v, jac, &ijac);
    }

    void operator()(const InputType &x, ValueType &v, JacobianType &jac = 0, const InputJacobianType *_ijac = 0) const
#endif
    {
        ActiveInput ax = x.template cast<ActiveScalar>();
        ActiveValue av(jac.rows());

        if (!_ijac)
        {
            eigen_assert(x.rows() == jac.cols());
            for (Index j = 0; j < jac.rows(); ++j)
                av[j].derivatives().resize(x.rows());

            for (Index i = 0; i < x.rows(); ++i)
            {
                ax[i].derivatives().resize(x.rows());
                ax[i].derivatives().insert(i) = 1.0;
            }
        }
        else
        {
            // If specified, copy derivatives from InputJacobian
            const InputJacobianType &ijac = *_ijac;

            for (Index j = 0; j < jac.rows(); ++j)
                av[j].derivatives().resize(ijac.cols());

            for (Index i = 0; i < x.rows(); ++i)
            {
                ax[i].derivatives().resize(ijac.cols());
                ax[i].derivatives() = ijac.row(i);
            }
        }

#if EIGEN_HAS_VARIADIC_TEMPLATES
        Functor::operator()(ax, av, Params...);
#else
        Functor::operator()(ax, av);
#endif

        for (int i = 0; i < jac.rows(); ++i)
        {
            v[i] = av[i].value();
            for (JacobianInnerIteratorType it(av[i].derivatives(), 0); it; ++it)
            {
                jac.insert(i, it.row()) = av[i].derivatives().coeffRef(it.row());
            }
        }
    }
};

}  // namespace Eigen

#endif  // EIGEN_AUTODIFF_CHAIN_JACOBIAN_SPARSE_H_
