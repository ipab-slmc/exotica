#ifndef VARIABLE_H
#define VARIABLE_H

#include <type_traits>
#include <exotica/Object.h>
#include <exotica/Tools/Uncopyable.h>

namespace exotica
{

    class Binary;
    class Integer;
    class Real;

    template<class C>
    class Variable : public Object
    {
    public:
        Variable()
        {
            static_assert(std::is_same<Binary, C>::value || std::is_same<Integer, C>::value || std::is_same<Real, C>::value,"Invalid variable type. Has to be Binary, Integer or Real!");
        }
    };

}

#endif // VARIABLE_H
