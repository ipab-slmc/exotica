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

#ifndef EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_
#define EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_

#include <exotica_ompl_solver/ompl_solver.h>

namespace exotica
{
class RRTSolver : public OMPLSolver<SamplingProblem>, Instantiable<RRTSolverInitializer>
{
public:
    RRTSolver();
    void Instantiate(const RRTSolverInitializer& init) override;
};

class RRTConnectSolver : public OMPLSolver<SamplingProblem>, Instantiable<RRTConnectSolverInitializer>
{
public:
    RRTConnectSolver();
    void Instantiate(const RRTConnectSolverInitializer& init) override;
    void SetRange(double range);
    double GetRange();
};

class ESTSolver : public OMPLSolver<SamplingProblem>, Instantiable<ESTSolverInitializer>
{
public:
    ESTSolver();
    void Instantiate(const ESTSolverInitializer& init) override;
};

class KPIECESolver : public OMPLSolver<SamplingProblem>, Instantiable<KPIECESolverInitializer>
{
public:
    KPIECESolver();
    void Instantiate(const KPIECESolverInitializer& init) override;
};

class BKPIECESolver : public OMPLSolver<SamplingProblem>, Instantiable<BKPIECESolverInitializer>
{
public:
    BKPIECESolver();
    void Instantiate(const BKPIECESolverInitializer& init) override;
};

class PRMSolver : public OMPLSolver<SamplingProblem>, Instantiable<PRMSolverInitializer>
{
public:
    PRMSolver();
    void Instantiate(const PRMSolverInitializer& init) override;
    void GrowRoadmap(double t);
    void ExpandRoadmap(double t);
    void Clear();
    void ClearQuery();
    void Setup();
    int EdgeCount();
    int MilestoneCount();
    bool IsMultiQuery() const;
    void SetMultiQuery(bool val);
};

class LazyPRMSolver : public OMPLSolver<SamplingProblem>, Instantiable<LazyPRMSolverInitializer>
{
public:
    LazyPRMSolver();
    void Instantiate(const LazyPRMSolverInitializer& init) override;
    void Clear();
    void ClearQuery();
    void Setup();
    int EdgeCount();
    int MilestoneCount();
    bool IsMultiQuery() const;
    void SetMultiQuery(bool val);
};

class RRTStarSolver : public OMPLSolver<SamplingProblem>, Instantiable<RRTStarSolverInitializer>
{
public:
    RRTStarSolver();
    void Instantiate(const RRTStarSolverInitializer& init) override;
};

class LBTRRTSolver : public OMPLSolver<SamplingProblem>, Instantiable<LBTRRTSolverInitializer>
{
public:
    LBTRRTSolver();
    void Instantiate(const LBTRRTSolverInitializer& init) override;
};
}

#endif  // EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_
