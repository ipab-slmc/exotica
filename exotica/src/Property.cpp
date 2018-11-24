#include "exotica/Property.h"

namespace exotica
{
// //////////////////////////////// Property //////////////////////////////////////

boost::any Property::get() const { return value; }
Property::Property(std::string prop_name) : name(prop_name), required(true) {}
Property::Property(std::string prop_name, bool isRequired) : name(prop_name), required(isRequired) {}
Property::Property(std::string prop_name, bool isRequired, boost::any val) : name(prop_name), required(isRequired) { value = val; }
bool Property::isRequired() const { return required; }
bool Property::isSet() const { return !value.empty(); }
bool Property::isStringType() const { return value.type() == typeid(std::string); }
bool Property::isInitializerVectorType() const { return value.type() == typeid(std::vector<exotica::Initializer>); }
std::string Property::getName() const { return name; }
std::string Property::getType() const { return getTypeName(value.type()); }
Property::Property(std::initializer_list<boost::any> val_)
{
    std::vector<boost::any> val(val_);
    if (val.size() != 2 || val[0].type() != typeid(std::string)) throw_pretty("Invalid property initialization!");
    name = boost::any_cast<std::string>(val[0]);
    value = val[1];
}

// //////////////////////////// InitializerBase ///////////////////////////////////
Initializer::Initializer()
{
}

Initializer::Initializer(std::string name_) : name(name_)
{
}

Initializer::Initializer(std::string name_, std::map<std::string, boost::any> properties_) : name(name_)
{
    for (auto& prop : properties_)
    {
        properties.emplace(prop.first, Property(prop.first, true, prop.second));
    }
}

std::string Initializer::getName() const
{
    return name;
}

void Initializer::addProperty(const Property& prop)
{
    properties.emplace(prop.getName(), prop);
}

bool Initializer::hasProperty(std::string name_) const
{
    return properties.find(name_) != properties.end();
}

boost::any Initializer::getProperty(std::string name_) const
{
    return properties.at(name_).get();
}

void Initializer::setProperty(std::string name_, boost::any value)
{
    properties.at(name_).set(value);
}

void Initializer::setName(std::string name_)
{
    name = name_;
}

std::vector<std::string> Initializer::getPropertyNames() const
{
    return getKeys(properties);
}
}
