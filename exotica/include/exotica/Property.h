#ifndef PROPERTY_H
#define PROPERTY_H

#include <exotica/Tools/Printable.h>
#include <exotica/Tools/Exception.h>
#include <memory>
#include <map>
#include <vector>
#include <typeinfo>

class PropertyElement : public Printable
{
public:
    PropertyElement();
    PropertyElement(bool isSet, bool isRequired,const std::string type, const std::string name);
    void operator=(PropertyElement& other);
    void operator=(const PropertyElement& other);
    bool isSet() const;
    bool isRequired() const;
    std::string getType() const;
    std::string getName() const;
    virtual void print(std::ostream& os) const;
    virtual void copyValues(PropertyElement& other) = 0;
    virtual void copyValues(const PropertyElement& other) = 0;
protected:
    bool isSet_;
    bool isRequired_;
    std::string name_;
    std::string type_;
};
std::ostream& operator<< (std::ostream& os, const Printable& s);
template<typename T>std::ostream& operator<< (std::ostream& os, const std::vector<T>& s)
{
    for(auto& p : s) os << p << "\n";
    return os;
}

template<typename I, typename T>std::ostream& operator<< (std::ostream& os, const std::map<I,T>& s)
{
    for(auto& p : s) os << p.first << ": "<<p.second<<"\n";
    return os;
}

template<typename T>
class Property : public PropertyElement
{
public:
    // Various constructors
    Property(){}
    Property(const std::string& type, const std::string& name) : PropertyElement(false, true, type, name) {}
    Property(const std::string& type, const std::string& name, bool isRequired) : PropertyElement(false, isRequired, type, name) {}
    Property(const std::string& type, const std::string& name, T& value) : PropertyElement(true, true, type, name),value_(value) {}
    Property(const std::string& type, const std::string& name, const T& value) : PropertyElement(true, true, type, name),value_(value) {}
    Property(const std::string& type, const std::string& name, bool isRequired, T& value) : PropertyElement(true, isRequired, type, name),value_(value) {}
    Property(const std::string& type, const std::string& name, bool isRequired, const T& value) : PropertyElement(true, isRequired, type, name),value_(value) {}
    Property(Property& obj) : PropertyElement(obj.isSet_,obj.isRequired_, obj.type_, obj.name_), value_(obj.value_){}
    Property(const Property& obj) : PropertyElement(obj.isSet_,obj.isRequired_, obj.type_, obj.name_), value_(obj.value_){}

    virtual void copyValues(PropertyElement& other)
    {
        if(other.isSet()) value_=static_cast<Property<T>&>(other).value_;
    }

    virtual void copyValues(const PropertyElement& other)
    {
        if(other.isSet()) value_=static_cast<const Property<T>&>(other).value_;
    }

    // Construction from unique pointer as a copy constructor inside registerProperty (special case)
    Property(std::unique_ptr<Property<T>> obj) : PropertyElement(obj->isSet_,obj->isRequired_, obj->type_, obj->name_),value_(obj->value_) {}

    // Assign contained value
    // This makes it possible to do:
    // Property<T> prop = T();
    // Just like dealing with T directly.
    /*Property<T>& operator=(const T& val)
    {
        value_ = val;
        isSet_ = true;
        return *this;
    }*/

    void operator=(const T& val)
    {
        value_ = val;
        isSet_ = true;
    }

    void operator=(T& val)
    {
        value_ = val;
        isSet_ = true;
    }

    const T getValue() const {return value_;}

    // Return contained value
    // This makes it possible to do:
    // T val = prop;
    // Just like dealing with T directly.
    operator T() {return value_;}

    virtual void print(std::ostream& os) const
    {
        PropertyElement::print(os);
        os << ", Value: '" << value_<<"'";
    }

private:
    T value_;
};

template<typename T>
inline T operator+(T lhs, const Property<T>& rhs)
{
  return lhs + rhs.getValue();
}

inline std::string operator+(const char* lhs, const Property<std::string>& rhs)
{
  return std::string(lhs) + rhs.getValue();
}

template<typename T>
inline T operator-(T lhs, const Property<T>& rhs)
{
  return lhs - rhs.getValue();
}

template<typename T>
inline T operator*(T lhs, const Property<T>& rhs)
{
  return lhs * rhs.getValue();
}

template<typename T>
inline T operator/(T lhs, const Property<T>& rhs)
{
  return lhs / rhs.getValue();
}

// All user property containers must inherit from this.
// Use CMake tool to generate Initializers inheriting from this class
class PropertyContainer : public Printable
{
public:
    virtual void print(std::ostream& os) const
    {
        os << "Container '" << name_ << "'\n";
        for(auto& prop : propertiesOrdered_)
          os << "  " << *prop <<"\n";
    }

    PropertyContainer() : name_("") {}
    PropertyContainer(const std::string& name) : name_(name) {}
    std::string getName() {return name_;}

    const std::map<std::string, PropertyElement*>& getProperties() const {return properties_;}
    std::map<std::string, PropertyElement*>& getProperties() {return properties_;}
    const std::vector<PropertyElement*>& getPropertiesOrdered() const {return propertiesOrdered_;}
    std::vector<PropertyElement*>& getPropertiesOrdered() {return propertiesOrdered_;}

protected:
    std::string name_;
    std::map<std::string, PropertyElement*> properties_;
    std::vector<PropertyElement*> propertiesOrdered_;
};

class InstantiableBase
{
public:
    virtual const PropertyContainer& getInicializerTemplate() = 0;
    virtual void InstantiateInternal(const PropertyContainer& init) = 0;
};

template<class C>
class Instantiable : public virtual InstantiableBase
{
public:
    virtual void InstantiateInternal(const PropertyContainer& init)
    {
        if(const_cast<PropertyContainer&>(init).getName()==C::getContainerName())
        {
            Instantiate(static_cast<C&>(const_cast<PropertyContainer&>(init)));
        }
        else
        {
            C tmp;
            for(auto& param : init.getProperties())
            {
                if(tmp.getProperties().find(param.first)!= tmp.getProperties().end())
                {
                    // Copies over typeless PropertyElements using a virtual copyValue method
                    *tmp.getProperties()[param.first] = *param.second;
                }
                else
                {
                    //problem
                    throw_pretty("Combining incompatible initializers!");
                }
            }
            Instantiate(tmp);
        }
    }

    virtual const PropertyContainer& getInicializerTemplate()
    {
        return C();
    }

    virtual void Instantiate(C& init) = 0;
};


#endif // PROPERTY_H
