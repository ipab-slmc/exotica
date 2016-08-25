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
class InitializerGeneric;

// All user property containers must inherit from this.
// Use CMake tool to generate Initializers inheriting from this class
class InitializerBase : public Printable
{
public:
    virtual void print(std::ostream& os) const;
    InitializerBase();
    InitializerBase(const std::string& name);
    void setName(const std::string& name);
    std::string getName() const;
    void addProperty(const PropertyElement& prop);
    void addProperty(boost::shared_ptr<PropertyElement> prop);
    const std::map<std::string, boost::shared_ptr<PropertyElement>>& getManagedProperties() const;
    std::map<std::string, boost::shared_ptr<PropertyElement>>& getManagedProperties();
    virtual void check() const;
protected:
    std::string name_;
    std::map<std::string, boost::shared_ptr<PropertyElement>> propertiesManaged_;
};

class InitializerGeneric : public InitializerBase
{
public:

    InitializerGeneric();
    InitializerGeneric(const std::string& name);
};

class InstantiableBase
{
public:
    virtual InitializerGeneric getInitializerTemplate() = 0;
    virtual void InstantiateInternal(const InitializerGeneric& init) = 0;
    virtual void InstantiateBase(const InitializerGeneric& init) = 0;
};

class Containerable
{
public:
    Containerable& operator=(const Containerable&) = default;
    virtual bool isContainer() const;
    virtual bool isContainerVector() const;
    virtual InitializerGeneric getContainerTemplate() const;
};

class PropertyElement : public Printable, public virtual Containerable
{
public:
    PropertyElement();
    PropertyElement(bool isSet, bool isRequired,const std::string type, const std::string name);
    bool isSet() const;
    bool isRequired() const;
    std::string getType() const;
    std::string getKnownType() const;
    std::string getName() const;
    virtual void print(std::ostream& os) const;
    virtual boost::shared_ptr<PropertyElement> getCopy() const = 0;
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
    virtual boost::shared_ptr<PropertyElement> getCopy() const
    {
        boost::shared_ptr<Property<T>> ret(new Property<T>(type_,name_,isRequired_));
        ret->isSet_ = isSet_;
        if(isSet_) ret->value_ = value_;
        return ret;
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

    const T& getValue() const {return value_;}
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

template<typename C1>
class convertValues
{
public:
    static void get(Property<C1>& a, boost::shared_ptr<PropertyElement> b)
    {
        if(b->getType()==a.getType())
        {
            a=boost::static_pointer_cast<Property<C1>>(b)->getValue();
        }
        else
        {
            throw_pretty("Converting incompatible types: "+b->getType()+" to "+a.getType());
        }
    }
};

template<typename C>
void getProperty(std::string Name, const InitializerGeneric& init, C& ret)
{
    ret=boost::static_pointer_cast<const Property<C>>(init.getManagedProperties().at(Name))->getValue();
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

    virtual void InstantiateInternal(const InitializerGeneric& init)
    {
        InstantiateBase(init);
        C tmp = init;
        tmp.check();
        Instantiate(tmp);
    }

    virtual InitializerGeneric getInitializerTemplate()
    {
        return InitializerGeneric(C());
    }

    virtual void Instantiate(C& init) = 0;
};

template<class C>
class InstantiableFinal : public virtual InstantiableBase
{
public:
    virtual void InstantiateInternal(const InitializerGeneric& init)
    {
        C tmp = init;
        tmp.check();
        Instantiate(tmp);
    }

    virtual InitializerGeneric getInitializerTemplate()
    {
        return InitializerGeneric(C());
    }

    virtual void InstantiateBase(const InitializerGeneric& init)
    {

    }

    virtual void Instantiate(C& init) = 0;
};

}

#endif // PROPERTY_H
