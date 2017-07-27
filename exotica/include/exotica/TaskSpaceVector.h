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

    TaskVectorEntry();
    TaskVectorEntry(int inId_, RotationType type_);
};

class TaskSpaceVector
{
public:

    TaskSpaceVector();
    TaskSpaceVector& operator=(std::initializer_list<double> other);
    Eigen::VectorXd operator-(const TaskSpaceVector& other);
    void setZero(int N);

    Eigen::VectorXd data;
    std::vector<TaskVectorEntry> map;

};

}

#endif // TASKSPACEVECTOR_H
