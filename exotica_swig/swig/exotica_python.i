%include python/eigen.i
%include std_string.i
%include std_map.i
%include std_vector.i

%{
#include <ros/ros.h>
#include "exotica/Property.h"
#include "exotica/Setup.h"

using namespace exotica;

bool StructAsAnyMap(std::map<std::string,boost::any>& ret, const PyObject* val)
{
    return false;
}

void PrintInitializerInfo(const exotica::Initializer& init, std::ostringstream& oss, std::string prepend = "")
{
    oss << prepend << "Initializer '" << init.getName() << "':\n";
    prepend = prepend+"  ";
    for(auto& prop : init.properties)
    {
        if(prop.second.isStringType())
        {
            oss << prepend << "'" << prop.first << "' > '" << boost::any_cast<std::string>(prop.second.get()) <<  "'\n";
        }
        else if(prop.second.isInitializerVectorType())
        {
            oss << prepend << "'" << prop.first << "':\n";
            for(exotica::Initializer& init : boost::any_cast<std::vector<exotica::Initializer>>(prop.second.get()))
            {
                PrintInitializerInfo(init, oss, prepend+"  ");
            }
        }
        else
        {
            oss << "Property '" << prop.first << "' has unsupported type '" << prop.second.getType() << "'!\n";
            return;
        }
    }
}

%}

%include exotica.i

%extend exotica::Initializer
{
    Initializer(std::string name, const PyObject* val)
    {
        std::map<std::string,boost::any> tmp;
        if(!StructAsAnyMap(tmp, val))
        {
            return nullptr;
        }
        return new exotica::Initializer(name,tmp);
    }

    ~Initializer()
    {
        delete $self;
    }

    std::string __str__()
    {
        std::ostringstream oss;
        PrintInitializerInfo(*$self, oss);
        return std::string(oss.str());
    }
};

%{
class Core
{
public:
    Core()
    {
        int varc = 0;
        ros::init(varc, 0, "ExoticaPythonNode");
    }

    ~Core()
    {
        //exotica::Setup::Destroy();
    }
};
%}

class Core
{
public:
    Core()
    {
        int varc = 0;
        ros::init(varc, 0, "ExoticaMatlabNode");
    }

    ~Core()
    {
        exotica::Setup::Destroy();
    }
};
