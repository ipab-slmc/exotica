/*
 *  Created on: 10 Oct 2017
 *      Author: Yiming Yang
 *
 * Copyright (c) 2017, University of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_
#define EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_

#include <exotica_ompl_solver/ompl_solver.h>

namespace exotica
{
class RRT : public OMPLSolver<SamplingProblem>, Instantiable<RRTInitializer>
{
public:
    RRT();
    void Instantiate(RRTInitializer& init) override;
};

class RRTConnect : public OMPLSolver<SamplingProblem>, Instantiable<RRTConnectInitializer>
{
public:
    RRTConnect();
    void Instantiate(RRTConnectInitializer& init) override;
    void SetRange(double range);
    double GetRange();
};

class EST : public OMPLSolver<SamplingProblem>, Instantiable<ESTInitializer>
{
public:
    EST();
    void Instantiate(ESTInitializer& init) override;
};

class KPIECE : public OMPLSolver<SamplingProblem>, Instantiable<KPIECEInitializer>
{
public:
    KPIECE();
    void Instantiate(KPIECEInitializer& init) override;
};

class BKPIECE : public OMPLSolver<SamplingProblem>, Instantiable<BKPIECEInitializer>
{
public:
    BKPIECE();
    void Instantiate(BKPIECEInitializer& init) override;
};

class PRM : public OMPLSolver<SamplingProblem>, Instantiable<PRMInitializer>
{
public:
    PRM();
    void Instantiate(PRMInitializer& init) override;
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

class LazyPRM : public OMPLSolver<SamplingProblem>, Instantiable<LazyPRMInitializer>
{
public:
    LazyPRM();
    void Instantiate(LazyPRMInitializer& init) override;
    void Clear();
    void ClearQuery();
    void Setup();
    int EdgeCount();
    int MilestoneCount();
    bool IsMultiQuery() const;
    void SetMultiQuery(bool val);
};

class RRTStar : public OMPLSolver<SamplingProblem>, Instantiable<RRTStarInitializer>
{
public:
    RRTStar();
    void Instantiate(RRTStarInitializer& init) override;
};

class LBTRRT : public OMPLSolver<SamplingProblem>, Instantiable<LBTRRTInitializer>
{
public:
    LBTRRT();
    void Instantiate(LBTRRTInitializer& init) override;
};
}

#endif /* EXOTICA_OMPL_SOLVER_OMPL_NATIVE_SOLVER_H_ */
