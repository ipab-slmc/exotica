#include "exotica/Property.h"

namespace exotica
{

PropertyElement::PropertyElement() {}
PropertyElement::PropertyElement(bool isSet, bool isRequired,const std::string type, const std::string name)
    : isSet_(isSet),isRequired_(isRequired), type_(type), name_(name) {}
void PropertyElement::operator=(PropertyElement& other)
{
    isSet_=isSet_||other.isSet();
    copyValues(other);
}
void PropertyElement::operator=(const PropertyElement& other)
{
    isSet_=isSet_||other.isSet();
    copyValues(other);
}

std::string PropertyElement::getKnownType() const
{
    return type_;
}

void PropertyContainer::addProperty(boost::shared_ptr<PropertyElement> prop)
{
    if(properties_.find(prop->getName())==properties_.end())
    {
        managedProperties_.push_back(prop);
        properties_[prop->getName()] = prop.get();
    }
    else
    {
        throw_pretty("Property '"+prop->getName()+" already exists inside initializer '"+name_+"'!");
    }
}

void PropertyContainer::print(std::ostream& os) const
{
    os << "Container '" << name_ << "'\n";
}

PropertyContainer::PropertyContainer() : name_("") {}
PropertyContainer::PropertyContainer(const std::string& name) : name_(name) {}
const std::map<std::string, PropertyElement*>& PropertyContainer::getProperties() const {return properties_;}
std::map<std::string, PropertyElement*>& PropertyContainer::getProperties() {return properties_;}

std::string PropertyContainer::getName() const
{
    return name_;
}

void PropertyContainer::setName(const std::string& name)
{
    name_=name;
}

bool PropertyElement::isSet() const {return isSet_;}
bool PropertyElement::isRequired() const {return isRequired_;}
std::string PropertyElement::getType() const {return type_;}
std::string PropertyElement::getName() const {return name_;}
void PropertyElement::print(std::ostream& os) const
{
    os << "Property '" << name_ << "': type '" << type_ << "', " << (isSet_?"Set, ":"Unset, ") << (isRequired_?"Required":"Optional");
}

bool Containerable::isContainer() {return false;}
bool Containerable::isContainerVector() {return false;}
PropertyContainer& Containerable::getContainerTemplate() {return dummy_container_template_;}

}
