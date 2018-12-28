#ifndef PROPERTY_H
#define PROPERTY_H

#include <boost/any.hpp>
#include <initializer_list>
#include <map>
#include <memory>
#include <typeinfo>
#include <vector>

#include <exotica_core/tools.h>
#include <exotica_core/tools/conversions.h>
#include <exotica_core/tools/exception.h>
#include <exotica_core/tools/printable.h>

namespace exotica
{
class Property
{
public:
    boost::any Get() const;
    template <typename C>
    void Set(const C val)
    {
        value_ = val;
    }
    Property(std::string prop_name);
    Property(std::string prop_name, bool is_required);
    Property(std::string prop_name, bool is_required, boost::any val);
    Property(std::initializer_list<boost::any> val);
    bool IsRequired() const;
    bool IsSet() const;
    bool IsStringType() const;
    bool IsInitializerVectorType() const;
    std::string GetName() const;
    std::string GetType() const;

private:
    boost::any value_;
    bool required_;
    std::string name_;
};

class Initializer
{
public:
    Initializer();
    Initializer(std::string name);
    Initializer(std::string name, std::map<std::string, boost::any> properties);
    std::string GetName() const;
    void SetName(std::string name);
    void AddProperty(const Property& prop);
    boost::any GetProperty(std::string name) const;
    void SetProperty(std::string name, boost::any);
    bool HasProperty(std::string name) const;
    std::vector<std::string> GetPropertyNames() const;

    std::map<std::string, Property> properties_;
    std::string name_;
};

class InitializerBase
{
public:
    virtual void Check(const Initializer& other) const = 0;
    virtual Initializer GetTemplate() const = 0;
    virtual std::vector<Initializer> GetAllTemplates() const = 0;
};

class InstantiableBase
{
public:
    virtual Initializer GetInitializerTemplate() = 0;
    virtual void InstantiateInternal(const Initializer& init) = 0;
    virtual void InstantiateBase(const Initializer& init) {}
    virtual std::vector<Initializer> GetAllTemplates() const = 0;
};

template <class C>
class Instantiable : public virtual InstantiableBase
{
public:
    virtual void InstantiateInternal(const Initializer& init)
    {
        InstantiateBase(init);
        C tmp(init);
        tmp.Check(init);
        Instantiate(tmp);
    }

    virtual Initializer GetInitializerTemplate()
    {
        return C().GetTemplate();
    }

    virtual std::vector<Initializer> GetAllTemplates() const
    {
        return C().GetAllTemplates();
    }

    virtual void Instantiate(C& init) = 0;
};
}

#endif  // PROPERTY_H
