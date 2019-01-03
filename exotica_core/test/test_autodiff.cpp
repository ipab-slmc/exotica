//
// Copyright (c) 2018, University of Edinburgh
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

#include <gtest/gtest.h>
#include <Eigen/Dense>

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
// Autodiff is only supported with Eigen 3.3.0 or higher

#include <exotica_core/tools/autodiff_chain_hessian.h>
#include <exotica_core/tools/autodiff_chain_hessian_sparse.h>
#include <exotica_core/tools/autodiff_chain_jacobian.h>
#include <exotica_core/tools/autodiff_chain_jacobian_sparse.h>
#include <exotica_core/tools/finitediff_chain_hessian.h>
#include <exotica_core/tools/finitediff_chain_jacobian.h>

#include <iostream>

using namespace exotica;

constexpr int N = 1000;
constexpr double THRESHOLD_ANALYTIC = 1e-5;
constexpr double THRESHOLD_FINITE_DIFFERENCES = 1e-3;
constexpr double THRESHOLD_FINITE_DIFFERENCES_HESSIAN = 5e-1;
constexpr double INPUT_VECTOR_SCALE = 2.0;

// This function combines Function3(Function2(x)).
// The result will be used for comparison with passing the derivative of Function2 into the AutoDiff.
template <class FunctorType>
struct Function1 : public FunctorType
{
    enum
    {
        Inputs = 4,
        Values = 4,
        JacobianCols = 4
    };

    template <typename T>
    void operator()(const Eigen::Matrix<T, FunctorType::InputType::RowsAtCompileTime, 1> &x, Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> &y) const
    {
        Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> tmp(y.rows());
        // Always cast known scalar type matrices/vectors into the templated type <T>.
        // This is required for AutoDiff to work properly.
        for (int i = 0; i < 4; ++i)
        {
            y(i, 0) = (Eigen::AngleAxis<T>(x(i, 0), Eigen::Vector3d::UnitZ().cast<T>()).toRotationMatrix() * Eigen::Vector3d::UnitX().cast<T>()).dot(Eigen::Vector3d::UnitX().cast<T>());
            tmp(i, 0) = y(i, 0);
            if (i > 0) y(i, 0) += tmp(i - 1, 0);
        }
    }
};

// Function2 rotates a UnitX vector around Z axis.
// This is a helper function that will provide input for Function3.
template <class FunctorType>
struct Function2 : public FunctorType
{
    enum
    {
        Inputs = 4,
        Values = 12,
        JacobianCols = 4
    };

    template <typename T>
    void operator()(const Eigen::Matrix<T, FunctorType::InputType::RowsAtCompileTime, 1> &x, Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> &y) const
    {
        Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> tmp(y.rows());
        for (int i = 0; i < 4; ++i)
        {
            y.block(i * 3, 0, 3, 1) = Eigen::AngleAxis<T>(x(i, 0), Eigen::Vector3d::UnitZ().cast<T>()).toRotationMatrix() * Eigen::Vector3d::UnitX().cast<T>();
        }
    }
};

// This function computes dot product between the input vector and UnitX.
// The input vector will be taken from the output of Function2 (including the derivatives).
template <class FunctorType>
struct Function3 : public FunctorType
{
    enum
    {
        Inputs = 12,
        Values = 4,
        JacobianCols = 4
    };

    template <typename T>
    void operator()(const Eigen::Matrix<T, FunctorType::InputType::RowsAtCompileTime, 1> &x, Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> &y) const
    {
        Eigen::Matrix<T, FunctorType::ValueType::RowsAtCompileTime, 1> tmp(y.rows());
        for (int i = 0; i < 4; ++i)
        {
            y(i, 0) = Eigen::Vector3d::UnitX().cast<T>().dot(x.block(i * 3, 0, 3, 1));
            tmp(i, 0) = y(i, 0);
            if (i > 0) y(i, 0) += tmp(i - 1, 0);
        }
    }
};

// Typedefs for convenience
template <template <typename> class DiffType, typename _FunctorType, template <typename> class FunctionType>
struct TestingBase
{
    typedef _FunctorType FunctorType;
    typedef FunctionType<FunctorType> Function;
    typedef DiffType<Function> Diff;
    typedef typename Diff::InputType InputType;
    typedef typename Diff::ValueType ValueType;
    typedef typename Diff::JacobianType JacobianType;
};

// Compute Jacobians using AutoDiff
template <class T>
struct JacobianFull : public T
{
    void operator()(const typename T::InputType &x, typename T::ValueType &y,
                    typename T::JacobianType &j)
    {
        typename T::Function f;
        typename T::Diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
            y.resize(T::Function::Values, 1);
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
            j.resize(T::Function::Values, T::Function::JacobianCols);

        // Compute full Jacobian
        autoj(x, y, j);
    }
};

// Compute Jacobians using AutoDiff given the Jacobian of the input variables
template <class T>
struct JacobianCompound : public T
{
    void operator()(const typename T::InputType &x, typename T::ValueType &y,
                    typename T::JacobianType &j, const typename T::Diff::InputJacobianType &ij)
    {
        typename T::Function f;
        typename T::Diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
            y.resize(T::Function::Values, 1);
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
            j.resize(T::Function::Values, T::Function::JacobianCols);

        // Compute compund Jacobian
        autoj(x, y, j, ij);
    }
};

// Compute Jacobians using FiniteDiff
// Requires separate method because of additional template parameters
template <class T, Eigen::NumericalDiffMode mode>
struct JacobianFullFinite : public T
{
    typedef Eigen::FiniteDiffChainJacobian<typename T::Function, mode> diff;

    int operator()(const typename T::InputType &x, typename T::ValueType &y,
                   typename T::JacobianType &j)
    {
        typename T::Function f;
        diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
            y.resize(T::Function::Values, 1);
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
            j.resize(T::Function::Values, T::Function::JacobianCols);

        // Compute full Jacobian
        return autoj(x, y, j);
    }
};

// Compute Jacobians using FiniteDiff given the Jacobian of the input variables
// Requires separate method because of additional template parameters
// compute_intermediate is used to provide update of the inputs
template <class T, class T2, Eigen::NumericalDiffMode mode>
struct JacobianCompoundFinite : public T
{
    typedef Eigen::FiniteDiffChainJacobian<typename T::Function, mode> diff;
    typedef typename T2::Function F2;

    int operator()(const typename diff::InputJacobianRowType &x, typename T::ValueType &y,
                   typename T::JacobianType &j)
    {
        typename T::Function f;

        auto compute_intermediate = [](const typename diff::InputJacobianRowType &jx, typename T::InputType &_x) {
            if (T::InputType::RowsAtCompileTime == Eigen::Dynamic) _x.resize(F2::Values);
            F2 f;
            // This is where the chaining of the functions happens
            f(jx, _x);
        };
        diff autoj(f, compute_intermediate);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
            y.resize(T::Function::Values, 1);
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
            j.resize(T::Function::Values, T::Function::JacobianCols);

        // Compute full Jacobian
        return autoj(x, y, j);
    }
};

// Compute Hessians using AutoDiff
template <class T>
struct HessianFull : public T
{
    void operator()(const typename T::InputType &x, typename T::ValueType &y,
                    typename T::JacobianType &j, typename T::Diff::HessianType &hess)
    {
        typename T::Function f;
        typename T::Diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
        {
            y.resize(T::Function::Values, 1);
            hess.resize(T::Function::Values);
        }
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
        {
            j.resize(T::Function::Values, T::Function::JacobianCols);
            for (int i = 0; i < hess.rows(); ++i)
            {
                hess[i].resize(T::Function::JacobianCols, T::Function::JacobianCols);
            }
        }

        // Compute full Hessian
        autoj(x, y, j, hess);
    }
};

// Compute Hessians using AutoDiff given the Jacobian of the input variables
template <class T>
struct HessianCompound : public T
{
    void operator()(const typename T::InputType &x, typename T::ValueType &y,
                    typename T::JacobianType &j, typename T::Diff::HessianType &hess,
                    const typename T::Diff::InputJacobianType &ij, const typename T::Diff::InputHessianType &ihess)
    {
        typename T::Function f;
        typename T::Diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
        {
            y.resize(T::Function::Values, 1);
            hess.resize(T::Function::Values);
        }
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
        {
            j.resize(T::Function::Values, T::Function::JacobianCols);
            for (int i = 0; i < hess.rows(); ++i)
            {
                hess[i].resize(T::Function::JacobianCols, T::Function::JacobianCols);
            }
        }

        // Compute the Jacobian and Hessian of the compound function.
        autoj(x, y, j, hess, ij, ihess);
    }
};

// Compute Hessians using FiniteDiff
// Requires separate method because of additional template parameters
template <class T, Eigen::NumericalDiffMode mode>
struct HessianFullFinite : public T
{
    typedef Eigen::FiniteDiffChainHessian<typename T::Function, mode> diff;

    int operator()(const typename T::InputType &x, typename T::ValueType &y,
                   typename T::JacobianType &j, typename T::Diff::HessianType &hess)
    {
        typename T::Function f;
        diff autoj(f);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
        {
            y.resize(T::Function::Values, 1);
            hess.resize(T::Function::Values);
        }
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
        {
            j.resize(T::Function::Values, T::Function::JacobianCols);
            for (int i = 0; i < hess.rows(); ++i)
            {
                hess[i].resize(T::Function::JacobianCols, T::Function::JacobianCols);
            }
        }

        // Compute the Jacobian and Hessian of the compound function.
        return autoj(x, y, j, hess);
    }
};

// Compute Hessians using FiniteDiff given the Jacobian of the input variables
// Requires separate method because of additional template parameters
// compute_intermediate is used to provide update of the inputs
template <class T, class T2, Eigen::NumericalDiffMode mode>
struct HessianCompoundFinite : public T
{
    typedef Eigen::FiniteDiffChainHessian<typename T::Function, mode> diff;
    typedef typename T2::Function F2;

    int operator()(const typename diff::InputJacobianRowType &x, typename T::ValueType &y,
                   typename T::JacobianType &j, typename T::Diff::HessianType &hess)
    {
        typename T::Function f;

        auto compute_intermediate = [](const typename diff::InputJacobianRowType &jx, typename T::InputType &_x) {
            if (T::InputType::RowsAtCompileTime == Eigen::Dynamic) _x.resize(F2::Values);
            F2 f;
            // This is where the chaining of the functions happens
            f(jx, _x);
        };
        diff autoj(f, compute_intermediate);

        if (T::ValueType::RowsAtCompileTime == Eigen::Dynamic)
        {
            y.resize(T::Function::Values, 1);
            hess.resize(T::Function::Values);
        }
        if (T::JacobianType::RowsAtCompileTime == Eigen::Dynamic)
        {
            j.resize(T::Function::Values, T::Function::JacobianCols);
            for (int i = 0; i < hess.rows(); ++i)
            {
                hess[i].resize(T::Function::JacobianCols, T::Function::JacobianCols);
            }
        }

        // Compute the Jacobian and Hessian of the compound function.
        return autoj(x, y, j, hess);
    }
};

// Compile time matrix sizes
struct TestStaticTrait
{
    enum
    {
        Inputs1 = 4,
        Values1 = 4,
        JacobianCols1 = 4,
        Inputs2 = 4,
        Values2 = 12,
        JacobianCols2 = 4,
        Inputs3 = 12,
        Values3 = 4,
        JacobianCols3 = 4
    };
};

// Dynamic marix sizes
struct TestDynamicTrait
{
    enum
    {
        Inputs1 = Eigen::Dynamic,
        Values1 = Eigen::Dynamic,
        JacobianCols1 = Eigen::Dynamic,
        Inputs2 = Eigen::Dynamic,
        Values2 = Eigen::Dynamic,
        JacobianCols2 = Eigen::Dynamic,
        Inputs3 = Eigen::Dynamic,
        Values3 = Eigen::Dynamic,
        JacobianCols3 = Eigen::Dynamic
    };
};

// All tests are comparing Function1(x) == Function3(Function2(x))
template <template <typename> class DiffType, typename Dynamic, Eigen::NumericalDiffMode mode>
void TestJacobians()
{
    for (int i = 0; i < N; ++i)
    {
        // Setup functors
        typedef FunctorBase<double, Dynamic::Inputs1, Dynamic::Values1, Dynamic::JacobianCols1> MyFunctor1;
        typedef FunctorBase<double, Dynamic::Inputs2, Dynamic::Values2, Dynamic::JacobianCols2> MyFunctor2;
        typedef FunctorBase<double, Dynamic::Inputs3, Dynamic::Values3, Dynamic::JacobianCols3> MyFunctor3;
        // Define helper types
        typedef TestingBase<DiffType, MyFunctor1, Function1> Test1;
        typedef TestingBase<DiffType, MyFunctor2, Function2> Test2;
        typedef TestingBase<DiffType, MyFunctor3, Function3> Test3;

        typename Test1::InputType x1;
        if (Test1::InputType::RowsAtCompileTime == Eigen::Dynamic)
        {
            x1 = Test1::InputType::Random(4) * INPUT_VECTOR_SCALE;
        }
        else
        {
            x1 = Test1::InputType::Random() * INPUT_VECTOR_SCALE;
        }

        typename Test1::ValueType y1;
        typename Test1::JacobianType j1;

        typename Test2::ValueType y2;
        typename Test2::JacobianType j2;

        typename Test3::ValueType y3;
        typename Test3::JacobianType j3;

        // Compute Function1(x)'
        JacobianFull<Test1>()(x1, y1, j1);
        // Compute Function2(x)'
        JacobianFull<Test2>()(x1, y2, j2);
        // Compute Function3(Function2(x))'
        JacobianCompound<Test3>()(y2, y3, j3, j2);

        typename Test1::ValueType y1fd;
        typename Test1::JacobianType j1fd;
        typename Test3::ValueType y3fd;
        typename Test3::JacobianType j3fd;

        // Compute Function1(x)' using finite differences
        JacobianFullFinite<Test1, mode>()(x1, y1fd, j1fd);
        // Compute Function3(Function2(x))' using finite differences
        JacobianCompoundFinite<Test3, Test2, mode>()(x1, y3fd, j3fd);

        EXPECT_TRUE((y1 - y3).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j1 - j1fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((j3 - j3fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((y1fd - y3fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((y1 - y1fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
    }
}

// The sparse test is only different because FiniteDiff doesn't work with sparse matrices
template <template <typename> class DiffType, typename Dynamic>
void TestJacobiansSparse()
{
    for (int i = 0; i < N; ++i)
    {
        // Setup functors
        typedef FunctorBase<double, Dynamic::Inputs1, Dynamic::Values1, Dynamic::JacobianCols1> MyFunctor1;
        typedef FunctorBase<double, Dynamic::Inputs2, Dynamic::Values2, Dynamic::JacobianCols2> MyFunctor2;
        typedef FunctorBase<double, Dynamic::Inputs3, Dynamic::Values3, Dynamic::JacobianCols3> MyFunctor3;
        // Define sparse helper types
        typedef TestingBase<DiffType, MyFunctor1, Function1> Test1;
        typedef TestingBase<DiffType, MyFunctor2, Function2> Test2;
        typedef TestingBase<DiffType, MyFunctor3, Function3> Test3;

        typename Test1::InputType x1;
        if (Test1::InputType::RowsAtCompileTime == Eigen::Dynamic)
        {
            x1 = Test1::InputType::Random(4) * INPUT_VECTOR_SCALE;
        }
        else
        {
            x1 = Test1::InputType::Random() * INPUT_VECTOR_SCALE;
        }

        typename Test1::ValueType y1;
        typename Test1::JacobianType j1;

        typename Test2::ValueType y2;
        typename Test2::JacobianType j2;

        typename Test3::ValueType y3;
        typename Test3::JacobianType j3;

        // Compute sparse Function1(x)'
        JacobianFull<Test1>()(x1, y1, j1);
        // Compute sparse Function2(x)'
        JacobianFull<Test2>()(x1, y2, j2);
        // Compute sparse Function3(Function2(x))'
        JacobianCompound<Test3>()(y2, y3, j3, j2);

        // Define dense helper types
        typedef TestingBase<Eigen::AutoDiffChainJacobian, MyFunctor1, Function1> Test1D;
        typedef TestingBase<Eigen::AutoDiffChainJacobian, MyFunctor2, Function2> Test2D;
        typedef TestingBase<Eigen::AutoDiffChainJacobian, MyFunctor3, Function3> Test3D;
        typename Test1D::ValueType y1d;
        typename Test1D::JacobianType j1d;
        typename Test2D::ValueType y2d;
        typename Test2D::JacobianType j2d;
        typename Test3D::ValueType y3d;
        typename Test3D::JacobianType j3d;

        // Compute dense Function1(x)'
        JacobianFull<Test1D>()(x1, y1d, j1d);
        // Compute dense Function2(x)'
        JacobianFull<Test2D>()(x1, y2d, j2d);
        // Compute dense Function3(Function2(x))'
        JacobianCompound<Test3D>()(y2d, y3d, j3d, j2d);

        EXPECT_TRUE((y1 - y3).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j1 - j1d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j3 - j3d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((y1d - y3d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((y1 - y1d).norm() < THRESHOLD_ANALYTIC);
    }
}

// Hessian norm
// TODO: define a better norm
template <typename HessianType1, typename HessianType2>
double diffNorm(const HessianType1 &A, const HessianType2 &B)
{
    double ret = 0;
    double tmp;
    for (int i = 0; i < A.rows(); ++i)
    {
        tmp = (A[i] - B[i]).norm();
        ret += tmp * tmp;
    }
    return ret / static_cast<double>(A.rows());
}

template <template <typename> class DiffType, typename Dynamic, Eigen::NumericalDiffMode mode>
void TestHessians()
{
    for (int i = 0; i < N; ++i)
    {
        // Setup functors
        typedef FunctorBase<double, Dynamic::Inputs1, Dynamic::Values1, Dynamic::JacobianCols1> MyFunctor1;
        typedef FunctorBase<double, Dynamic::Inputs2, Dynamic::Values2, Dynamic::JacobianCols2> MyFunctor2;
        typedef FunctorBase<double, Dynamic::Inputs3, Dynamic::Values3, Dynamic::JacobianCols3> MyFunctor3;
        // Define helper types
        typedef TestingBase<DiffType, MyFunctor1, Function1> Test1;
        typedef TestingBase<DiffType, MyFunctor2, Function2> Test2;
        typedef TestingBase<DiffType, MyFunctor3, Function3> Test3;

        typename Test1::InputType x1;
        if (Test1::InputType::RowsAtCompileTime == Eigen::Dynamic)
        {
            x1 = Test1::InputType::Random(4) * INPUT_VECTOR_SCALE;
        }
        else
        {
            x1 = Test1::InputType::Random() * INPUT_VECTOR_SCALE;
        }

        typename Test1::ValueType y1;
        typename Test1::JacobianType j1;
        typename Test1::Diff::HessianType hess1;

        typename Test1::ValueType y1fd;
        typename Test1::JacobianType j1fd;
        typename Test1::Diff::HessianType hess1fd;

        typename Test2::ValueType y2;
        typename Test2::JacobianType j2;
        typename Test2::Diff::HessianType hess2;

        typename Test3::ValueType y3;
        typename Test3::JacobianType j3;
        typename Test3::Diff::HessianType hess3;

        typename Test3::ValueType y3fd;
        typename Test3::JacobianType j3fd;
        typename Test3::Diff::HessianType hess3fd;

        // Compute Function1(x)''
        HessianFull<Test1>()(x1, y1, j1, hess1);
        // Compute Function2(x)''
        HessianFull<Test2>()(x1, y2, j2, hess2);
        // Compute Function3(Function2(x))''
        HessianCompound<Test3>()(y2, y3, j3, hess3, j2, hess2);

        // Compute Function1(x)'' using finite differences
        HessianFullFinite<Test1, mode>()(x1, y1fd, j1fd, hess1fd);
        // Compute Function3(Function2(x))'' using finite differences
        HessianCompoundFinite<Test3, Test2, mode>()(x1, y3fd, j3fd, hess3fd);

        EXPECT_TRUE((y1 - y3).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j1 - j1fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((j3 - j3fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((y1fd - y3fd).norm() < THRESHOLD_FINITE_DIFFERENCES);
        EXPECT_TRUE((y1 - y1fd).norm() < THRESHOLD_FINITE_DIFFERENCES);

        EXPECT_TRUE(diffNorm(hess1, hess3) < THRESHOLD_ANALYTIC);
        EXPECT_TRUE(diffNorm(hess1, hess1fd) < THRESHOLD_FINITE_DIFFERENCES_HESSIAN);
        EXPECT_TRUE(diffNorm(hess3, hess3fd) < THRESHOLD_FINITE_DIFFERENCES_HESSIAN);
        EXPECT_TRUE(diffNorm(hess1, hess3fd) < THRESHOLD_FINITE_DIFFERENCES_HESSIAN);
    }
}

// The sparse test is only different because FiniteDiff doesn't work with sparse matrices
template <template <typename> class DiffType, typename Dynamic>
void TestHessiansSparse()
{
    for (int i = 0; i < N; ++i)
    {
        // Setup functors
        typedef FunctorBase<double, Dynamic::Inputs1, Dynamic::Values1, Dynamic::JacobianCols1> MyFunctor1;
        typedef FunctorBase<double, Dynamic::Inputs2, Dynamic::Values2, Dynamic::JacobianCols2> MyFunctor2;
        typedef FunctorBase<double, Dynamic::Inputs3, Dynamic::Values3, Dynamic::JacobianCols3> MyFunctor3;
        // Define sparse helper types
        typedef TestingBase<DiffType, MyFunctor1, Function1> Test1;
        typedef TestingBase<DiffType, MyFunctor2, Function2> Test2;
        typedef TestingBase<DiffType, MyFunctor3, Function3> Test3;

        typename Test1::InputType x1;
        if (Test1::InputType::RowsAtCompileTime == Eigen::Dynamic)
        {
            x1 = Test1::InputType::Random(4) * INPUT_VECTOR_SCALE;
        }
        else
        {
            x1 = Test1::InputType::Random() * INPUT_VECTOR_SCALE;
        }

        typename Test1::ValueType y1;
        typename Test1::JacobianType j1;
        typename Test1::Diff::HessianType hess1;

        typename Test2::ValueType y2;
        typename Test2::JacobianType j2;
        typename Test2::Diff::HessianType hess2;

        typename Test3::ValueType y3;
        typename Test3::JacobianType j3;
        typename Test3::Diff::HessianType hess3;

        // Compute sparse Function1(x)''
        HessianFull<Test1>()(x1, y1, j1, hess1);
        // Compute sparse Function2(x)''
        HessianFull<Test2>()(x1, y2, j2, hess2);
        // Compute sparse Function3(Function2(x))''
        HessianCompound<Test3>()(y2, y3, j3, hess3, j2, hess2);

        // Define dense helper types
        typedef TestingBase<Eigen::AutoDiffChainHessian, MyFunctor1, Function1> Test1D;
        typedef TestingBase<Eigen::AutoDiffChainHessian, MyFunctor2, Function2> Test2D;
        typedef TestingBase<Eigen::AutoDiffChainHessian, MyFunctor3, Function3> Test3D;

        typename Test1D::ValueType y1d;
        typename Test1D::JacobianType j1d;
        typename Test1D::Diff::HessianType hess1d;

        typename Test2D::ValueType y2d;
        typename Test2D::JacobianType j2d;
        typename Test2D::Diff::HessianType hess2d;

        typename Test3D::ValueType y3d;
        typename Test3D::JacobianType j3d;
        typename Test3D::Diff::HessianType hess3d;

        // Compute dense Function1(x)''
        HessianFull<Test1D>()(x1, y1d, j1d, hess1d);
        // Compute dense Function2(x)''
        HessianFull<Test2D>()(x1, y2d, j2d, hess2d);
        // Compute dense Function3(Function2(x))''
        HessianCompound<Test3D>()(y2d, y3d, j3d, hess3d, j2d, hess2d);

        EXPECT_TRUE((y1 - y3).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j1 - j1d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((j3 - j3d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((y1d - y3d).norm() < THRESHOLD_ANALYTIC);
        EXPECT_TRUE((y1 - y1d).norm() < THRESHOLD_ANALYTIC);

        EXPECT_TRUE(diffNorm(hess1, hess3) < THRESHOLD_ANALYTIC);
        EXPECT_TRUE(diffNorm(hess1, hess1d) < THRESHOLD_ANALYTIC);
        EXPECT_TRUE(diffNorm(hess3, hess3d) < THRESHOLD_ANALYTIC);
        EXPECT_TRUE(diffNorm(hess1, hess3d) < THRESHOLD_ANALYTIC);
    }
}

// Through magic of tempalting, test:
// 1. AutoDiffChainJacobian Jacobian computation against finite differences
// 2. AutoDiffChainHessian Jacobian computation against finite differences
// 3. AutoDiffChainHessian Hessian computation against finite differences
// 4. Run 1 with Central differencing
// 5. Run 2 with Central differencing
// 6. Run 3 with Central differencing
// 7. AutoDiffChainJacobianSparse Jacobian computation against AutoDiffChainJacobian
// 8. AutoDiffChainHessianSparse Hessian computation against AutoDiffChainHessian
// 9. Run 1-7 with matrix sizes fixed at compile time

TEST(AutoDiffJacobian, JacobianComputationDynamicMatrix)
{
    TestJacobians<Eigen::AutoDiffChainJacobian, TestDynamicTrait, Eigen::Forward>();
}

TEST(AutoDiffJacobian, JacobianComputationDynamicMatrixCentralDifferences)
{
    TestJacobians<Eigen::AutoDiffChainJacobian, TestDynamicTrait, Eigen::Central>();
}

TEST(AutoDiffJacobian, JacobianComputationTemplatedMatrix)
{
    TestJacobians<Eigen::AutoDiffChainJacobian, TestStaticTrait, Eigen::Forward>();
}

TEST(AutoDiffJacobian, JacobianComputationTemplatedMatrixCentralDifferences)
{
    TestJacobians<Eigen::AutoDiffChainJacobian, TestStaticTrait, Eigen::Central>();
}

TEST(AutoDiffHessian, JacobianComputationDynamicMatrix)
{
    TestJacobians<Eigen::AutoDiffChainHessian, TestDynamicTrait, Eigen::Forward>();
}

TEST(AutoDiffHessian, JacobianComputationDynamicMatrixCentralDifferences)
{
    TestJacobians<Eigen::AutoDiffChainHessian, TestDynamicTrait, Eigen::Central>();
}

TEST(AutoDiffHessian, JacobianComputationTemplatedMatrix)
{
    TestJacobians<Eigen::AutoDiffChainHessian, TestStaticTrait, Eigen::Forward>();
}

TEST(AutoDiffHessian, JacobianComputationTemplatedMatrixCentralDifferences)
{
    TestJacobians<Eigen::AutoDiffChainHessian, TestStaticTrait, Eigen::Central>();
}

TEST(AutoDiffHessian, HessianComputationDynamicMatrix)
{
    TestHessians<Eigen::AutoDiffChainHessian, TestDynamicTrait, Eigen::Forward>();
}

TEST(AutoDiffHessian, HessianComputationDynamicMatrixCentralDifferences)
{
    TestHessians<Eigen::AutoDiffChainHessian, TestDynamicTrait, Eigen::Central>();
}

TEST(AutoDiffHessian, HessianComputationTemplatedMatrix)
{
    TestHessians<Eigen::AutoDiffChainHessian, TestStaticTrait, Eigen::Forward>();
}

TEST(AutoDiffHessian, HessianComputationTemplatedMatrixCentralDifferences)
{
    TestHessians<Eigen::AutoDiffChainHessian, TestStaticTrait, Eigen::Central>();
}

TEST(AutoDiffJacobianSparse, JacobianComputationDynamicMatrix)
{
    TestJacobiansSparse<Eigen::AutoDiffChainJacobianSparse, TestDynamicTrait>();
}

TEST(AutoDiffJacobianSparse, JacobianComputationTemplatedMatrix)
{
    TestJacobiansSparse<Eigen::AutoDiffChainJacobianSparse, TestStaticTrait>();
}

TEST(AutoDiffHessianSparse, HessianComputationDynamicMatrix)
{
    TestHessiansSparse<Eigen::AutoDiffChainHessianSparse, TestDynamicTrait>();
}

TEST(AutoDiffHessianSparse, HessianComputationTemplatedMatrix)
{
    TestHessiansSparse<Eigen::AutoDiffChainHessianSparse, TestStaticTrait>();
}

#endif

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}