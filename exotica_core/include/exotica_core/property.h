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

#ifndef EXOTICA_CORE_PROPERTY_H_
#define EXOTICA_CORE_PROPERTY_H_

#include <map>
#include <memory>
#include <string>

#include <boost/any.hpp>
#include <initializer_list>

#include <exotica_core/tools/conversions.h>

namespace exotica
{
class Property
{
public:
    boost::any Get() const;
    template <typename C>
    void Set(const C val)
    {
        value_ = val;
    }
    Property(const std::string& prop_name);
    Property(const std::string& prop_name, bool is_required);
    Property(const std::string& prop_name, bool is_required, boost::any val);
    Property(std::initializer_list<boost::any> val);
    bool IsRequired() const;
    bool IsSet() const;
    bool IsStringType() const;
    bool IsInitializerVectorType() const;
    const std::string& GetName() const;
    std::string GetType() const;

private:
    boost::any value_;
    bool required_;
    std::string name_;
};

class Initializer
{
public:
    Initializer();
    Initializer(const std::string& name);
    Initializer(const std::string& name, const std::map<std::string, boost::any>& properties);
    const std::string& GetName() const;
    void SetName(const std::string& name);
    void AddProperty(const Property& prop);
    boost::any GetProperty(const std::string& name) const;
    void SetProperty(const std::string& name, boost::any);
    bool HasProperty(const std::string& name) const;
    std::vector<std::string> GetPropertyNames() const;

    std::map<std::string, Property> properties_;
    std::string name_;
};

class InitializerBase
{
public:
    InitializerBase() = default;
    virtual ~InitializerBase() = default;
    virtual void Check(const Initializer& other) const = 0;
    virtual Initializer GetTemplate() const = 0;
    virtual std::vector<Initializer> GetAllTemplates() const = 0;
};

class InstantiableBase
{
public:
    InstantiableBase() = default;
    virtual ~InstantiableBase() = default;
    virtual Initializer GetInitializerTemplate() = 0;
    virtual void InstantiateInternal(const Initializer& init) = 0;
    virtual void InstantiateBase(const Initializer& init) {}
    virtual std::vector<Initializer> GetAllTemplates() const = 0;
};

template <class C, typename = typename std::enable_if<std::is_base_of<InitializerBase, C>::value, C>::type>
class Instantiable : public virtual InstantiableBase
{
public:
    void InstantiateInternal(const Initializer& init) override
    {
        InstantiateBase(init);
        const C tmp(init);
        tmp.Check(init);
        Instantiate(tmp);
    }

    Initializer GetInitializerTemplate() override
    {
        return C().GetTemplate();
    }

    std::vector<Initializer> GetAllTemplates() const override
    {
        return C().GetAllTemplates();
    }

    virtual void Instantiate(const C& init)
    {
        parameters_ = init;
    }

    const C& GetParameters() const { return parameters_; }

protected:
    C parameters_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_PROPERTY_H_
