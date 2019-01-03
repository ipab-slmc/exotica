//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_EXCEPTION_H_
#define EXOTICA_CORE_EXCEPTION_H_

#include <exception>
#include <sstream>

#define ThrowPretty(m)                                                               \
    {                                                                                \
        std::stringstream ss;                                                        \
        ss << m;                                                                     \
        throw exotica::Exception(ss.str(), __FILE__, __PRETTY_FUNCTION__, __LINE__); \
    }
#define ThrowNamed(m)                                                                                    \
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

#endif  // EXOTICA_CORE_EXCEPTION_H_
