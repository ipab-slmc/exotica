/*
 *  Created on: 10 Oct 2017
 *      Author: Yiming Yang
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#ifndef INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_
#define INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_

#include <ompl_solver/ompl_solver.h>

namespace exotica
{
class RRT : public OMPLsolver, Instantiable<RRTInitializer>
{
public:
    RRT();
    virtual void Instantiate(RRTInitializer& init);
};

class RRTConnect : public OMPLsolver, Instantiable<RRTConnectInitializer>
{
public:
    RRTConnect();
    virtual void Instantiate(RRTConnectInitializer& init);
};

class PRM : public OMPLsolver, Instantiable<PRMInitializer>
{
public:
    PRM();
    virtual void Instantiate(PRMInitializer& init);
    void growRoadmap(double t);
    void expandRoadmap(double t);
    void clear();
    void clearQuery();
    void setup();
    int edgeCount();
    int milestoneCount();
    bool isMultiQuery();
    void setMultiQuery(bool val);
};

class LazyPRM : public OMPLsolver, Instantiable<LazyPRMInitializer>
{
public:
    LazyPRM();
    virtual void Instantiate(LazyPRMInitializer& init);
    void clear();
    void clearQuery();
    void setup();
    int edgeCount();
    int milestoneCount();
    bool isMultiQuery();
    void setMultiQuery(bool val);
};
}

#endif /* INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_ */
