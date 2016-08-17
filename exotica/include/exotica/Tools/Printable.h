#ifndef PRINTABLE_H
#define PRINTABLE_H

#include <iostream>
#include <vector>
#include <map>
#include <kdl/frames.hpp>

/**
 * \brief A set of debugging tools: basically these provide easy ways of checking code execution through std::cout prints
 */
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#ifdef EXOTICA_DEBUG_MODE
#define CHECK_EXECUTION         std::cout << "\033[1;32m[EXOTica]:\033[0m Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n"; //!< With endline
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)				std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";//!< With endline
#define ERROR(x)				std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)					std::clog << "\033[1;32m[EXOTica]:\033[0m Info in " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#else
#define CHECK_EXECUTION         // No operation
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)        std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";//!< With endline
#define ERROR(x)                std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)
#endif
#define HIGHLIGHT(x)			std::cout << "\033[1;32m[EXOTica]:\033[0m \033[36m" << x << "\033[0m\n";
#define HIGHLIGHT_NAMED(name, x)std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[36m" << x << "\033[0m\n";
#define WARNING_NAMED(name, x)	std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[33m" << x << "\033[0m\n";
#define INFO_NAMED(name, x)		std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m " << x << "\n";

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

std::string toString(const KDL::Frame& s);

#endif // PRINTABLE_H
