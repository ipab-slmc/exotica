
#include <exotica_core/tools/printable.h>

std::ostream& operator<<(std::ostream& os, const Printable& s)
{
    s.print(os);
    return os;
}

std::string toString(const KDL::Frame& s)
{
    double x, y, z, w;
    s.M.GetQuaternion(x, y, z, w);
    return "([" + std::to_string(s.p.data[0]) + " " + std::to_string(s.p.data[1]) + " " + std::to_string(s.p.data[2]) + "] [" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(w) + "])";
}

std::string toString(const Eigen::Isometry3d& s)
{
    Eigen::Quaterniond quat(s.linear());
    return "([" + std::to_string(s.translation().x()) + " " + std::to_string(s.translation().y()) + " " + std::to_string(s.translation().z()) + "] [" + std::to_string(quat.x()) + " " + std::to_string(quat.y()) + " " + std::to_string(quat.z()) + " " + std::to_string(quat.w()) + "])";
}

std::string toString(const Eigen::Affine3d& s)
{
    Eigen::Quaterniond quat(s.rotation());
    return "([" + std::to_string(s.translation().x()) + " " + std::to_string(s.translation().y()) + " " + std::to_string(s.translation().z()) + "] [" + std::to_string(quat.x()) + " " + std::to_string(quat.y()) + " " + std::to_string(quat.z()) + " " + std::to_string(quat.w()) + "])";
}
