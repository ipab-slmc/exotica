#include "exotica/Tools/Exception.h"

namespace exotica
{

Exception::ReportingType Exception::reporting_ = Exception::Message;

Exception::Exception()
{
}

Exception::Exception(const std::string &msg, const char *file, const char *func, int line) :
    msg_(msg), file_(file), func_(func), line_(std::to_string(line))
{

}

Exception::Exception(const std::string &msg, const char *file, const char *func, int line, const std::string& object) :
    msg_(msg), file_(file), func_(func), line_(std::to_string(line)), object_(object)
{

}

const char* Exception::what() const noexcept
{
    std::string tmp;
    if(Exception::reporting_ | FileName) tmp += "In "+file_+"\n";
    if(Exception::reporting_ | FunctionName) tmp += func_+" ";
    if(Exception::reporting_ | LineNumber) tmp += line_+"\n";
    if(Exception::reporting_ | ObjectName) tmp += "Object: "+object_+": ";
    if(Exception::reporting_ | Message) tmp += msg_;
    return tmp.c_str();
}

}
