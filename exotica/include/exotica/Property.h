#ifndef PROPERTY_H
#define PROPERTY_H

#include <exotica/Tools/Printable.h>
#include <exotica/Tools/Exception.h>
#include <exotica/Tools/Conversions.h>
#include <memory>
#include <map>
#include <vector>
#include <typeinfo>

namespace exotica
{

class PropertyElement;

// All user property containers must inherit from this.
// Use CMake tool to generate Initializers inheriting from this class
class PropertyContainer : public Printable
{
public:
    virtual void print(std::ostream& os) const;

    PropertyContainer();
    PropertyContainer(const std::string& name);
    std::string getName() const;
    void setName(const std::string& name);

    const std::map<std::string, PropertyElement*>& getProperties() const;
    std::map<std::string, PropertyElement*>& getProperties();
    void addProperty(boost::shared_ptr<PropertyElement> prop);

protected:
    std::string name_;
    std::map<std::string, PropertyElement*> properties_;
    std::vector<boost::shared_ptr<PropertyElement>> managedProperties_;
};

class InstantiableBase
{
public:
    virtual PropertyContainer& getInitializerTemplate() = 0;
    virtual void InstantiateInternal(const PropertyContainer& init) = 0;
    virtual void InstantiateBase(const PropertyContainer& init) = 0;
};

class Containerable
{
public:
    Containerable& operator=(const Containerable&) = default;
    virtual bool isContainer();
    virtual bool isContainerVector();
    virtual PropertyContainer& getContainerTemplate();
protected:
    PropertyContainer dummy_container_template_;
};

class PropertyElement : public Printable, public virtual Containerable
{
public:
    PropertyElement();
    PropertyElement(bool isSet, bool isRequired,const std::string type, const std::string name);
    void operator=(PropertyElement& other);
    void operator=(const PropertyElement& other);
    bool isSet() const;
    bool isRequired() const;
    std::string getType() const;
    std::string getKnownType() const;
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

template<typename T>
class IsContainer : public virtual Containerable
{
};

template<typename T>
class Property : public PropertyElement, public IsContainer<T>
{
public:
    // Various constructors
    Property() = default;
    Property(const std::string& type, const std::string& name) : PropertyElement(false, true, type, name) {}
    Property(const std::string& type, const std::string& name, bool isRequired) : PropertyElement(false, isRequired, type, name) {}
    Property(const std::string& type, const std::string& name, bool isRequired, T& value) : PropertyElement(true, isRequired, type, name),value_(value) {}
    Property(const std::string& type, const std::string& name, bool isRequired, const T& value) : PropertyElement(true, isRequired, type, name),value_(value) {}

    void copyValues(PropertyElement& other){ if(other.isSet()) value_=static_cast<Property<T>&>(other).value_; }

    void copyValues(const PropertyElement& other)
    {
        if(other.isSet()) value_=static_cast<const Property<T>&>(other).value_;
    }

    // Assign contained value
    // This makes it possible to do:
    // Property<T> prop = T();
    // Just like dealing with T directly.
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
    T& getValue() {return value_;}

    // Return contained value
    // This makes it possible to do:
    // T val = prop;
    // Just like dealing with T directly.
    operator T() const {return value_;}

    virtual void print(std::ostream& os) const
    {
        PropertyElement::print(os);
    }

private:
    T value_;
};

template<typename C>
void getProperty(std::string Name, const PropertyContainer& init, C& ret)
{
    ret = static_cast<Property<C>*>(init.getProperties().at(Name))->getValue();
}

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

template<class C>
class Instantiable : public virtual InstantiableBase
{
public:
    C template_initializer_;

    virtual void InstantiateInternal(const PropertyContainer& init)
    {
        InstantiateBase(init);
        C tmp(init);
        Instantiate(tmp);
    }

    virtual PropertyContainer& getInitializerTemplate()
    {
        return template_initializer_;
    }

    virtual void Instantiate(C& init) = 0;
};

template<class C>
class InstantiableFinal : public virtual InstantiableBase
{
public:
    C template_initializer_;

    virtual void InstantiateInternal(const PropertyContainer& init)
    {
        C tmp(init);
        Instantiate(tmp);
    }

    virtual PropertyContainer& getInitializerTemplate()
    {
        return template_initializer_;
    }

    virtual void InstantiateBase(const PropertyContainer& init)
    {

    }

    virtual void Instantiate(C& init) = 0;
};

}

#endif // PROPERTY_H
