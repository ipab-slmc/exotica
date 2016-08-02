#ifndef PRINTABLE_H
#define PRINTABLE_H

#include <iostream>

class Printable
{
public:
    virtual void print(std::ostream& os) const = 0;
};

#endif // PRINTABLE_H
