
#include <exotica/Tools/Printable.h>

std::ostream& operator<< (std::ostream& os, const Printable& s)
{
  s.print(os);
  return os;
}
