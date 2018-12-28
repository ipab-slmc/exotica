#include "exotica_core/property.h"

namespace exotica
{
// //////////////////////////////// Property //////////////////////////////////////

boost::any Property::Get() const { return value_; }
Property::Property(std::string prop_name) : name_(prop_name), required_(true) {}
Property::Property(std::string prop_name, bool is_required) : name_(prop_name), required_(is_required) {}
Property::Property(std::string prop_name, bool is_required, boost::any val) : name_(prop_name), required_(is_required) { value_ = val; }
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

// //////////////////////////// InitializerBase ///////////////////////////////////
Initializer::Initializer()
{
}

Initializer::Initializer(std::string name) : name_(name)
{
}

Initializer::Initializer(std::string name, std::map<std::string, boost::any> properties) : name_(name)
{
    for (auto& prop : properties)
    {
        properties_.emplace(prop.first, Property(prop.first, true, prop.second));
    }
}

std::string Initializer::GetName() const
{
    return name_;
}

void Initializer::AddProperty(const Property& prop)
{
    properties_.emplace(prop.GetName(), prop);
}

bool Initializer::HasProperty(std::string name) const
{
    return properties_.find(name) != properties_.end();
}

boost::any Initializer::GetProperty(std::string name) const
{
    return properties_.at(name).Get();
}

void Initializer::SetProperty(std::string name, boost::any value)
{
    properties_.at(name).Set(value);
}

void Initializer::SetName(std::string name)
{
    name_ = name;
}

std::vector<std::string> Initializer::GetPropertyNames() const
{
    return getKeys(properties_);
}
}
