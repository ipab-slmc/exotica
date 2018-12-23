#include "exotica_core/tools/exception.h"

namespace exotica
{
Exception::ReportingType Exception::reporting_ = Exception::Message | Exception::FileName | Exception::FunctionName | Exception::LineNumber | Exception::ObjectName;

Exception::Exception(const std::string &msg, const char *file, const char *func, int line, const std::string &object)
{
    std::string tmp;
    if (Exception::reporting_ | FileName) tmp += "In " + std::string(file) + "\n";
    if (Exception::reporting_ | FunctionName) tmp += std::string(func) + " ";
    if (Exception::reporting_ | LineNumber) tmp += std::to_string(line) + "\n";
    if ((Exception::reporting_ | ObjectName) && !object.empty()) tmp += "Object: " + object + ": ";
    if (Exception::reporting_ | Message) tmp += msg;
    msg_ = tmp;
}

const char *Exception::what() const noexcept
{
    return msg_.c_str();
}
}
