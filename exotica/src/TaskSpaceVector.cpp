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
            int len = getRotationTypeLength(id.type);
            while(iIn<id.inId)
            {
                ret(iOut) = data(iIn) - other.data(iIn);
                iIn++;
                iOut++;
            }
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
        while(iIn<data.rows())
        {
            ret(iOut) = data(iIn) - other.data(iIn);
            iIn++;
            iOut++;
        }
        return ret;
    }
}
