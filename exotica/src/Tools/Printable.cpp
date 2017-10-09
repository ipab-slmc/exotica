
#include <exotica/Tools/Printable.h>

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
