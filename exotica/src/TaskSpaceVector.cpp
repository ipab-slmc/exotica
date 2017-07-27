/*
 *      Author: Vladimir Ivan
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

#include <exotica/TaskSpaceVector.h>

namespace exotica
{

    TaskVectorEntry::TaskVectorEntry(int inId_, RotationType type_) :
        inId(inId_), type(type_)
    {

    }

    TaskVectorEntry::TaskVectorEntry() :
        inId(0), type(RotationType::RPY)
    {

    }

    TaskSpaceVector::TaskSpaceVector()
    {

    }

    TaskSpaceVector& TaskSpaceVector::operator=(std::initializer_list<double> other)
    {
        if(other.size()!=data.rows()) throw_pretty("Wrong initializer size: " << other.size() << " expecting " << data.rows());
        int i = 0;
        for(double val : other)
        {
            data(i) = val;
            i++;
        }
        return *this;
    }

    void TaskSpaceVector::setZero(int N)
    {

        data = Eigen::VectorXd::Zero(N);
        for(const TaskVectorEntry& id : map)
        {
            int len = getRotationTypeLength(id.type);
            data.segment(id.inId,len) = setRotation(KDL::Rotation(), id.type);
        }
    }

    Eigen::VectorXd TaskSpaceVector::operator-(const TaskSpaceVector& other)
    {
        if(data.rows() != other.data.rows()) throw_pretty("Task space vector sizes do not match!");
        int entrySize = 0;
        for(const TaskVectorEntry& id : map) entrySize += getRotationTypeLength(id.type);
        Eigen::VectorXd ret(data.rows()+map.size()*3-map.size()*entrySize);
        int iIn=0;
        int iOut=0;
        for(const TaskVectorEntry& id : map)
        {
            if(iIn<id.inId) ret.segment(iOut, id.inId-iIn) = data.segment(iIn, id.inId-iIn) - other.data.segment(iIn, id.inId-iIn);
            iOut += id.inId-iIn;
            iIn += id.inId-iIn;
            int len = getRotationTypeLength(id.type);

            KDL::Rotation M1 = getRotation(data.segment(id.inId, len), id.type);
            KDL::Rotation M2 = getRotation(other.data.segment(id.inId, len), id.type);
            KDL::Rotation M = M2.Inverse()*M1;
            KDL::Vector rotvec = M1*(M.GetRot());
            ret(iOut) = rotvec[0];
            ret(iOut+1) = rotvec[1];
            ret(iOut+2) = rotvec[2];
            iOut+=3;
            iIn+=len;
        }
        if(iIn<data.rows()) ret.segment(iOut, data.rows()-iIn) = data.segment(iIn, data.rows()-iIn) - other.data.segment(iIn, data.rows()-iIn);
        return ret;
    }
}
