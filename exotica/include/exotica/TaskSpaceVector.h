/*
 *      Author: Vladimir Ivan
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

#ifndef TASKSPACEVECTOR_H
#define TASKSPACEVECTOR_H

#include <exotica/Tools.h>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <vector>

namespace exotica
{
struct TaskVectorEntry
{
    RotationType type = RotationType::RPY;
    int inId = 0;

    TaskVectorEntry();
    TaskVectorEntry(int inId_, RotationType type_);
    static std::vector<TaskVectorEntry> reindex(const std::vector<TaskVectorEntry>& map, int oldStart, int newStart);
};

class TaskSpaceVector
{
public:
    TaskSpaceVector();
    ~TaskSpaceVector();
    TaskSpaceVector& operator=(std::initializer_list<double> other);
    Eigen::VectorXd operator-(const TaskSpaceVector& other);
    void setZero(int N);

    Eigen::VectorXd data;
    std::vector<TaskVectorEntry> map;
};
}

#endif  // TASKSPACEVECTOR_H
