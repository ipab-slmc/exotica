/*
 *      Author: Vladimir Ivan
 * 
 * Copyright (c) 2016, University Of Edinburgh 
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

#include "EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);

namespace exotica
{
    EffPosition::EffPosition()
    {
    }

    void EffPosition::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
    {
        if(phi.rows() != Kinematics.Phi.rows()*3) throw_named("Wrong size of phi!");
        for(int i=0;i<Kinematics.Phi.rows();i++)
        {
            phi(i*3) = Kinematics.Phi(i).p[0];
            phi(i*3+1) = Kinematics.Phi(i).p[1];
            phi(i*3+2) = Kinematics.Phi(i).p[2];
        }
    }

    void EffPosition::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
    {
        if(phi.rows() != Kinematics.Phi.rows()*3) throw_named("Wrong size of phi!");
        if(J.rows() != Kinematics.J.rows()*3 || J.cols() != Kinematics.J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics.J(0).data.cols());
        for(int i=0;i<Kinematics.Phi.rows();i++)
        {
            phi(i*3) = Kinematics.Phi(i).p[0];
            phi(i*3+1) = Kinematics.Phi(i).p[1];
            phi(i*3+2) = Kinematics.Phi(i).p[2];
            J.middleRows(i*3,3) = Kinematics.J[i].data.topRows(3);
        }
    }

    void EffPosition::taskSpaceDim(int & task_dim)
    {
        task_dim = Kinematics.Phi.rows()*3;
    }
}
