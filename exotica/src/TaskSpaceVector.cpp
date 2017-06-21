#include <exotica/TaskSpaceVector.h>

namespace exotica
{

    TaskVectorEntry::TaskVectorEntry(int inId_, int outId_, RotationType type_) :
        inId(inId_), outId(outId_), type(type_)
    {

    }

    TaskVectorEntry::TaskVectorEntry() :
        inId(0), outId(0), type(RotationType::RPY)
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

    Eigen::VectorXd TaskSpaceVector::operator-(const TaskSpaceVector& other)
    {
        Eigen::VectorXd ret = data-other.data;
        for(const TaskVectorEntry& id : map)
        {
            KDL::Rotation M1 = getRotation(data.segment(id.inId, getRotationTypeLength(id.type)), id.type);
            KDL::Rotation M2 = getRotation(other.data.segment(id.inId, getRotationTypeLength(id.type)), id.type);
            KDL::Vector rotvec = M1*(M2.Inverse()*M1).GetRot();
            ret(id.outId) = rotvec[0];
            ret(id.outId+1) = rotvec[1];
            ret(id.outId+2) = rotvec[2];
        }
        return ret;
    }
}
