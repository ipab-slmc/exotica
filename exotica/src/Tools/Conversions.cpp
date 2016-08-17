#include <exotica/Tools/Conversions.h>
#include <exotica/Tools/Exception.h>
#include <algorithm>

namespace Eigen
{
    Eigen::VectorXd VectorTransform(double px, double py, double pz, double qx, double qy, double qz, double qw)
    {
        Eigen::VectorXd ret((Eigen::VectorXd(7) << px,py,pz,qx,qy,qz,qw).finished());
        return ret;
    }

    Eigen::VectorXd IdentityTransform()
    {
        return VectorTransform();
    }
}

namespace exotica
{
    KDL::Frame getFrame(Eigen::VectorXdRefConst val)
    {
        if(val.rows()!=7) throw_pretty("Eigen vector has incorrect length!");
        return KDL::Frame(KDL::Rotation::Quaternion(val(3),val(4),val(5),val(6)),KDL::Vector(val(0),val(1),val(2)));
    }

    bool contains(std::string key, const std::vector<std::string>& vec)
    {

        if(std::find(vec.begin(),vec.end(),key)==vec.end()) return false;
        return true;
    }
}
