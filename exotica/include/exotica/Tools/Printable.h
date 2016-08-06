#ifndef PRINTABLE_H
#define PRINTABLE_H

#include <iostream>
#include <vector>
#include <map>

class Printable
{
public:
    virtual void print(std::ostream& os) const = 0;
};

std::ostream& operator<< (std::ostream& os, const Printable& s);

template<typename T>std::ostream& operator<< (std::ostream& os, const std::vector<T>& s)
{
    for(auto& p : s) os << p << "\n";
    return os;
}

template<typename I, typename T>std::ostream& operator<< (std::ostream& os, const std::map<I,T>& s)
{
    for(auto& p : s) os << p.first << ": "<<p.second<<"\n";
    return os;
}

#endif // PRINTABLE_H
