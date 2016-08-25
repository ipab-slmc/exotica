#include "exotica/Property.h"

namespace exotica
{

// //////////////////////////// InitializerGeneric ///////////////////////////////////

InitializerGeneric::InitializerGeneric() : InitializerBase("") {}
InitializerGeneric::InitializerGeneric(const std::string& name) : InitializerBase(name) {}

// //////////////////////////// InitializerBase ///////////////////////////////////


InitializerBase::InitializerBase() : name_("") {}
InitializerBase::InitializerBase(const std::string& name) : name_(name) {}

void InitializerBase::check() const
{
    for(auto& p : propertiesManaged_)
    {
        if(p.second->isRequired() && !p.second->isSet()) throw_pretty("Initializer "+name_+" requires property "+p.second->getName()+"to be set!");
    }
}

void InitializerBase::setName(const std::string& name)
{
    name_=name;
}

void InitializerBase::print(std::ostream& os) const
{
    os << "Container '" << name_ << "'\n";
}

std::string InitializerBase::getName() const
{
    return name_;
}

void InitializerBase::addProperty(const PropertyElement& prop)
{
    propertiesManaged_.emplace(prop.getName(), prop.getCopy());
}

void InitializerBase::addProperty(boost::shared_ptr<PropertyElement> prop)
{
    propertiesManaged_.emplace(prop->getName(), prop);
}

const std::map<std::string, boost::shared_ptr<PropertyElement>>& InitializerBase::getManagedProperties() const
{
    return propertiesManaged_;
}

std::map<std::string, boost::shared_ptr<PropertyElement>>& InitializerBase::getManagedProperties()
{
    return propertiesManaged_;
}

// //////////////////////////// PropertyElement ///////////////////////////////////

PropertyElement::PropertyElement()
{

}

PropertyElement::PropertyElement(bool isSet, bool isRequired,const std::string type, const std::string name)
    : isSet_(isSet),isRequired_(isRequired), type_(type), name_(name) {}

std::string PropertyElement::getKnownType() const
{
    return type_;
}

bool PropertyElement::isSet() const {return isSet_;}
bool PropertyElement::isRequired() const {return isRequired_;}
std::string PropertyElement::getType() const {return type_;}
std::string PropertyElement::getName() const {return name_;}

void PropertyElement::print(std::ostream& os) const
{
    os << "Property '" << name_ << "': type '" << type_ << "', " << (isSet_?"Set, ":"Unset, ") << (isRequired_?"Required":"Optional");
}

// //////////////////////////// Containerable ///////////////////////////////////

bool Containerable::isContainer() const {return false;}
bool Containerable::isContainerVector() const {return false;}
InitializerGeneric Containerable::getContainerTemplate() const {return InitializerGeneric();}

}


