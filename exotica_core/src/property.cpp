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

#include <typeinfo>
#include <vector>

#include <exotica_core/tools.h>
#include "exotica_core/property.h"

namespace exotica
{
//  * ///////////////////////////// Property //////////////////////////////////////

boost::any Property::Get() const { return value_; }
Property::Property(const std::string& prop_name) : name_(prop_name), required_(true) {}
Property::Property(const std::string& prop_name, bool is_required) : name_(prop_name), required_(is_required) {}
Property::Property(const std::string& prop_name, bool is_required, boost::any val) : name_(prop_name), required_(is_required) { value_ = val; }
bool Property::IsRequired() const { return required_; }
bool Property::IsSet() const { return !value_.empty(); }
bool Property::IsStringType() const { return value_.type() == typeid(std::string); }
bool Property::IsInitializerVectorType() const { return value_.type() == typeid(std::vector<exotica::Initializer>); }
std::string Property::GetName() const { return name_; }
std::string Property::GetType() const { return GetTypeName(value_.type()); }
Property::Property(std::initializer_list<boost::any> _val)
{
    std::vector<boost::any> val(_val);
    if (val.size() != 2 || val[0].type() != typeid(std::string)) ThrowPretty("Invalid property initialization!");
    name_ = boost::any_cast<std::string>(val[0]);
    value_ = val[1];
}

//  * ///////////////////////// InitializerBase ///////////////////////////////////
Initializer::Initializer()
{
}

Initializer::Initializer(const std::string& name) : name_(name)
{
}

Initializer::Initializer(const std::string& name, const std::map<std::string, boost::any>& properties) : name_(name)
{
    for (auto& prop : properties)
    {
        AddProperty(Property(prop.first, true, prop.second));
    }
}

std::string Initializer::GetName() const
{
    return name_;
}

void Initializer::AddProperty(const Property& prop)
{
    if (HasProperty(prop.GetName()))
    {
        WARNING("Property '" << prop.GetName() << "' already added - overriding.");
        SetProperty(prop.GetName(), prop.Get());
    }
    else
    {
        properties_.emplace(prop.GetName(), prop);
    }
}

bool Initializer::HasProperty(const std::string& name) const
{
    return properties_.find(name) != properties_.end();
}

boost::any Initializer::GetProperty(const std::string& name) const
{
    return properties_.at(name).Get();
}

void Initializer::SetProperty(const std::string& name, boost::any value)
{
    properties_.at(name).Set(value);
}

void Initializer::SetName(const std::string& name)
{
    name_ = name;
}

std::vector<std::string> Initializer::GetPropertyNames() const
{
    return GetKeys(properties_);
}
}  // namespace exotica
