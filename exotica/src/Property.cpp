#include "exotica/Property.h"

std::ostream& operator<< (std::ostream& os, const Printable& s)
{
  s.print(os);
  return os;
}

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

bool PropertyElement::isSet() const {return isSet_;}
bool PropertyElement::isRequired() const {return isRequired_;}
std::string PropertyElement::getType() const {return type_;}
std::string PropertyElement::getName() const {return name_;}
void PropertyElement::print(std::ostream& os) const
{
    os << "Property '" << name_ << "': type '" << type_ << "', " << (isSet_?"Set, ":"Unset, ") << (isRequired_?"Required":"Optional");
}
