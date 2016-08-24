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
class PropertyReferenceManaged;
class PropertyReferenceMember;
class PropertyContainerBase;
class InitializerGeneric;
class InitializerBase;


class PropertyReferenceManaged
{
public:
    PropertyReferenceManaged();
    PropertyReferenceManaged(const PropertyReferenceMember& other);
    PropertyReferenceManaged(boost::shared_ptr<PropertyElement> managed);
    PropertyElement& get();
    const PropertyElement& get() const;
    friend class PropertyReferenceMember;
protected:
    boost::shared_ptr<PropertyElement> managed_;
};

class PropertyReferenceMember
{
public:
    PropertyReferenceMember();
    PropertyReferenceMember(const PropertyReferenceMember& other);
    void operator=(const PropertyReferenceManaged& other);
    void operator=(const PropertyReferenceMember& other);
    PropertyReferenceMember(PropertyContainerBase* parent,PropertyElement* member);
    PropertyElement& get();
    const PropertyElement& get() const;
    friend class PropertyReferenceManaged;
protected:
    PropertyElement* member_;
    PropertyContainerBase* parent_;
};

// All user property containers must inherit from this.
// Use CMake tool to generate Initializers inheriting from this class
class PropertyContainerBase : public Printable
{
public:
    virtual void print(std::ostream& os) const;

    PropertyContainerBase();
    PropertyContainerBase(const std::string& name);
    std::string getName() const;
    void addProperty(const PropertyElement& prop);
    void addProperty(boost::shared_ptr<PropertyElement> prop);
    const std::map<std::string, PropertyReferenceManaged>& getManagedProperties() const;
    std::map<std::string, PropertyReferenceManaged>& getManagedProperties();

protected:
    std::string name_;
    std::map<std::string, PropertyReferenceManaged> propertiesManaged_;
};

class InitializerBase : public PropertyContainerBase
{
public:
    InitializerBase(const std::string& name);
    void specialize(const InitializerGeneric& other);
    virtual void RegisterParams() = 0;
    const std::map<std::string, PropertyReferenceMember>& getProperties() const;
    std::map<std::string, PropertyReferenceMember>& getProperties();
    friend class InitializerGeneric;
protected:
    std::map<std::string, PropertyReferenceMember> properties_;
};

class InitializerGeneric : public PropertyContainerBase
{
public:
    InitializerGeneric();
    InitializerGeneric(const std::string& name);
    void setName(const std::string& name);
    InitializerGeneric(const InitializerBase& other);
    InitializerGeneric(InitializerBase& other);
    friend class InitializerBase;
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
        ret->value_ = value_;
        return ret;
    }

    void copyValues(PropertyElement& other)
    {
        if(other.isSet()) value_=static_cast<Property<T>&>(other).value_;
    }

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
void getProperty(std::string Name, const InitializerGeneric& init, C& ret)
{
    ret = static_cast<const Property<C>&>(init.getManagedProperties().at(Name).get()).getValue();
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
        InstantiateBase((C&)init);
        Instantiate((C&)init);
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
        Instantiate((C&)init);
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
