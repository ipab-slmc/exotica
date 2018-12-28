#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <sstream>

#define ThrowPretty(m)                                                              \
    {                                                                                \
        std::stringstream ss;                                                        \
        ss << m;                                                                     \
        throw exotica::Exception(ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__); \
    }
#define ThrowNamed(m)                                                                                   \
    {                                                                                                    \
        std::stringstream ss;                                                                            \
        ss << m;                                                                                         \
        throw exotica::Exception(ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__, this->object_name_); \
    }

namespace exotica
{
class Exception : public std::exception
{
public:
    enum ReportingType
    {
        Message = 1,
        FileName = 2,
        FunctionName = 4,
        LineNumber = 8,
        ObjectName = 16
    };

    explicit Exception(const std::string &msg, const char *file, const char *func, int line, const std::string &object = std::string());
    virtual const char *what() const noexcept;

    std::string msg_;

private:
    static ReportingType reporting_;
};

inline Exception::ReportingType operator|(Exception::ReportingType a, Exception::ReportingType b) noexcept
{
    return static_cast<Exception::ReportingType>(static_cast<int>(a) | static_cast<int>(b));
}

class SolveException : public Exception
{
    using Exception::Exception;
};
}

#endif  // EXCEPTION_H
