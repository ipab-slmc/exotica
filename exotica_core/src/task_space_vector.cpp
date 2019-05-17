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

#include <exotica_core/task_space_vector.h>

namespace exotica
{
TaskVectorEntry::TaskVectorEntry(int _id, RotationType _type) : id(_id), type(_type)
{
}

TaskVectorEntry::TaskVectorEntry() = default;

TaskSpaceVector::TaskSpaceVector() = default;
TaskSpaceVector::~TaskSpaceVector() = default;

TaskSpaceVector& TaskSpaceVector::operator=(std::initializer_list<double> other)
{
    if (other.size() != data.rows()) ThrowPretty("Wrong initializer size: " << other.size() << " expecting " << data.rows());
    int i = 0;
    for (const double& val : other)
    {
        data(i++) = val;
    }
    return *this;
}

void TaskSpaceVector::SetZero(const int n)
{
    data = Eigen::VectorXd::Zero(n);
    for (const TaskVectorEntry& entry : map)
    {
        const int len = GetRotationTypeLength(entry.type);
        data.segment(entry.id, len) = SetRotation(KDL::Rotation(), entry.type);
    }
}

Eigen::VectorXd TaskSpaceVector::operator-(const TaskSpaceVector& other)
{
    if (data.rows() != other.data.rows()) ThrowPretty("Task space vector sizes do not match!");
    int entry_size = 0;
    for (const TaskVectorEntry& entry : map) entry_size += GetRotationTypeLength(entry.type);
    Eigen::VectorXd ret(data.rows() + map.size() * 3 - entry_size);
    int i_in = 0;
    int i_out = 0;
    for (const TaskVectorEntry& entry : map)
    {
        if (i_in < entry.id) ret.segment(i_out, entry.id - i_in) = data.segment(i_in, entry.id - i_in) - other.data.segment(i_in, entry.id - i_in);
        i_out += entry.id - i_in;
        i_in += entry.id - i_in;
        const int len = GetRotationTypeLength(entry.type);

        KDL::Rotation M1 = GetRotation(data.segment(entry.id, len), entry.type);
        KDL::Rotation M2 = GetRotation(other.data.segment(entry.id, len), entry.type);
        KDL::Rotation M = M2.Inverse() * M1;
        KDL::Vector rotvec = M1 * (M.GetRot());
        ret(i_out) = rotvec[0];
        ret(i_out + 1) = rotvec[1];
        ret(i_out + 2) = rotvec[2];
        i_out += 3;
        i_in += len;
    }
    if (i_in < data.rows()) ret.segment(i_out, data.rows() - i_in) = data.segment(i_in, data.rows() - i_in) - other.data.segment(i_in, data.rows() - i_in);
    return ret;
}

std::vector<TaskVectorEntry> TaskVectorEntry::reindex(const std::vector<TaskVectorEntry>& _map, int _old_start, int _new_start)
{
    std::vector<TaskVectorEntry> ret = _map;
    for (TaskVectorEntry& entry : ret)
    {
        entry.id = entry.id - _old_start + _new_start;
    }
    return ret;
}
}
