#include "exotica/Property.h"

namespace exotica
{

// //////////////////////////// PropertyReferenceManaged ///////////////////////////////////
PropertyReferenceManaged::PropertyReferenceManaged() : managed_(nullptr)
{
}

PropertyReferenceManaged::PropertyReferenceManaged(boost::shared_ptr<PropertyElement> managed) : managed_(managed)
{
}

PropertyElement& PropertyReferenceManaged::get()
{
    if(!managed_) throw_pretty("Use of uninitialized property reference!");
    return *managed_;
}

const PropertyElement& PropertyReferenceManaged::get() const
{
    if(!managed_) throw_pretty("Use of uninitialized property reference!");
    return *managed_;
}

PropertyReferenceManaged::PropertyReferenceManaged(const PropertyReferenceMember& other)
{
    HIGHLIGHT_NAMED("ManagedProperty","copying '"<<other.get().getName()<<"'");
    managed_ = other.get().getCopy();
    HIGHLIGHT_NAMED("ManagedProperty","Copied '"<<other.get().getName()<<"' from "<<&other.get()<<" to "<<managed_);
}

// //////////////////////////// PropertyReferenceMember ///////////////////////////////////
PropertyReferenceMember::PropertyReferenceMember() : parent_(nullptr), member_(nullptr)
{
}

void PropertyReferenceMember::operator=(const PropertyReferenceManaged& other)
{
    if(!parent_) throw_pretty("Use of uninitialized property reference!");
    if(!member_) throw_pretty("Use of uninitialized property reference member!");
    *member_=*other.managed_;
    HIGHLIGHT("Copied managed to member");
}

void PropertyReferenceMember::operator=(const PropertyReferenceMember& other)
{
    HIGHLIGHT("Assigning member reference from: " << parent_ <<" "<<member_);
    if(!parent_ || !other.parent_) throw_pretty("Use of uninitialized property reference!");
    if(!member_ || !other.member_) throw_pretty("Use of uninitialized property reference member!");
    *member_=*other.member_;
}

PropertyReferenceMember::PropertyReferenceMember(const PropertyReferenceMember& other) : parent_(other.parent_), member_(other.member_)
{
    HIGHLIGHT("Creating member reference from: " << parent_ <<" "<<member_);
}

PropertyReferenceMember::PropertyReferenceMember(PropertyContainerBase* parent,PropertyElement* member) : parent_(parent), member_(member)
{
}

PropertyElement& PropertyReferenceMember::get()
{
    if(!parent_ || !member_)
    {
        throw_pretty("Invalid member property reference!");
    }
    return *member_;
}

const PropertyElement& PropertyReferenceMember::get() const
{
    if(!parent_ || !member_)
    {
        throw_pretty("Invalid member property reference!");
    }
    return *member_;
}

// //////////////////////////// PropertyContainerBase ///////////////////////////////////

PropertyContainerBase::PropertyContainerBase(const std::string& name) : name_(name)
{

}

void PropertyContainerBase::print(std::ostream& os) const
{
    os << "Container '" << name_ << "'\n";
}

PropertyContainerBase::PropertyContainerBase() : name_("")
{

}

std::string PropertyContainerBase::getName() const
{
    return name_;
}

void PropertyContainerBase::addProperty(const PropertyElement& prop)
{
    propertiesManaged_.emplace(prop.getName(), PropertyReferenceManaged(prop.getCopy()));
}

void PropertyContainerBase::addProperty(boost::shared_ptr<PropertyElement> prop)
{
    propertiesManaged_.emplace(prop->getName(), PropertyReferenceManaged(prop));
}

const std::map<std::string, PropertyReferenceManaged>& PropertyContainerBase::getManagedProperties() const
{
    return propertiesManaged_;
}

std::map<std::string, PropertyReferenceManaged>& PropertyContainerBase::getManagedProperties()
{
    return propertiesManaged_;
}

// //////////////////////////// PropertyElement ///////////////////////////////////

PropertyElement::PropertyElement()
{

}

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

bool PropertyElement::isSet() const {return isSet_;}
bool PropertyElement::isRequired() const {return isRequired_;}
std::string PropertyElement::getType() const {return type_;}
std::string PropertyElement::getName() const {return name_;}

void PropertyElement::print(std::ostream& os) const
{
    os << "Property '" << name_ << "': type '" << type_ << "', " << (isSet_?"Set, ":"Unset, ") << (isRequired_?"Required":"Optional");
}

// //////////////////////////// InitializerGeneric ///////////////////////////////////


InitializerGeneric::InitializerGeneric() {}
InitializerGeneric::InitializerGeneric(const std::string& name) : PropertyContainerBase(name) {}
InitializerGeneric::InitializerGeneric(const InitializerBase& other)
{
    name_ = other.name_;
    propertiesManaged_ = other.propertiesManaged_;
    for(auto& prop : other.properties_)
    {
        propertiesManaged_.emplace(prop.first, PropertyReferenceManaged(prop.second));
    }
}

InitializerGeneric::InitializerGeneric(InitializerBase& other)
{
    name_ = other.name_;
    propertiesManaged_ = other.propertiesManaged_;
    for(auto& prop : other.properties_)
    {
        HIGHLIGHT_NAMED("Creating generic",prop.first);
        HIGHLIGHT_NAMED("Creating generic",prop.second.get().getName());
        propertiesManaged_.emplace(prop.first, PropertyReferenceManaged(prop.second));
    }
}

void InitializerGeneric::setName(const std::string& name)
{
    name_=name;
}

// //////////////////////////// InitializerBase ///////////////////////////////////

InitializerBase::InitializerBase(const std::string& name) : PropertyContainerBase(name) {}

void InitializerBase::specialize(const InitializerGeneric& other)
{
    name_=other.name_;
    for(auto& prop : other.propertiesManaged_)
    {
        if(properties_.find(prop.first)==properties_.end())
        {
            propertiesManaged_.emplace(prop.first, PropertyReferenceManaged(prop.second));
        }
        else
        {
            properties_[prop.first] = prop.second;
        }
    }
}

const std::map<std::string, PropertyReferenceMember>& InitializerBase::getProperties() const
{
    return properties_;
}

std::map<std::string, PropertyReferenceMember>& InitializerBase::getProperties()
{
    return properties_;
}

// //////////////////////////// Containerable ///////////////////////////////////

bool Containerable::isContainer() const {return false;}
bool Containerable::isContainerVector() const {return false;}
InitializerGeneric Containerable::getContainerTemplate() const {return InitializerGeneric();}

}


