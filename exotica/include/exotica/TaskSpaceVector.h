#ifndef TASKSPACEVECTOR_H
#define TASKSPACEVECTOR_H

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <vector>
#include <exotica/Tools.h>

namespace exotica
{

struct TaskVectorEntry
{
    RotationType type;
    int inId;
    int outId;

    TaskVectorEntry();
    TaskVectorEntry(int inId_, int outId_, RotationType type_);
};

class TaskSpaceVector
{
public:

    TaskSpaceVector();
    TaskSpaceVector& operator=(std::initializer_list<double> other);
    Eigen::VectorXd operator-(const TaskSpaceVector& other);

    Eigen::VectorXd data;
    std::vector<TaskVectorEntry> map;

};

}

#endif // TASKSPACEVECTOR_H
