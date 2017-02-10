%include matlab/eigen.i
%include std_string.i
%include std_map.i
%include std_vector.i

%{
#include <ros/ros.h>
#include "exotica/Property.h"
#include "exotica/Setup.h"

using namespace exotica;

bool StructAsAnyMap(std::map<std::string,boost::any>& ret, const mxArray* val)
{
    if(!mxIsStruct(val))
    {
        mexPrintf("Map can only be created from a struct!\n");
        return false;
    }
    int nn = mxGetN(val)*mxGetM(val);
    int n = mxGetNumberOfFields(val);
    if (!n)
    {
        mexPrintf("Struct with no fields!  (%d)\n",n);
        return false;
    }
    if (nn!=1)
    {
        mexPrintf("The (sub) struct can't be an array!  (%d)\n",nn);
        return false;
    }
    for(int i=0;i<n;i++)
    {
        const char* name_c = mxGetFieldNameByNumber(val,i);
        if(!name_c)
        {
            mexPrintf("Invalid field name! (%d)\n",i);
            return false;
        }
        std::string name(name_c);
        mxArray* fld = mxGetFieldByNumber(val,0,i);
        if(mxIsStruct(fld))
        {
            mexPrintf("Can't handle struct parameters. Please Use a cell array of structs instead!\n");
            return false;
        }
        else if(mxIsCell(fld))
        {
            int N = mxGetN(fld);
            int M = mxGetM(fld);
            if(N!=2)
            {
                mexPrintf("Initializer cell array must have two columns! (%dx%d)\n",M,N);
                return false;
            }
            std::vector<exotica::Initializer> vec;
            for(int j=0;j<M;j++)
            {
                std::string child_name = mxArrayToString(mxGetCell(fld, j));
                mxArray* child = mxGetCell(fld, j+M);
                std::map<std::string,boost::any> tmp;
                if(StructAsAnyMap(tmp,child))
                {
                    vec.push_back(exotica::Initializer(child_name,tmp));
                }
                else
                {
                    return false;
                }
            }
            ret[name] = vec;
        }
        else
        {
            const char* val_c = mxArrayToString(fld);
            if(!val_c)
            {
                mexPrintf("Invalid field value! (%s)\n",name_c);
                return false;
            }
            ret[name] = std::string(val_c);
        }
    }
    return true;
}

void PrintInitializerInfo(const exotica::Initializer& init, std::string prepend = "")
{
    mexPrintf("%sInitializer '%s':\n",prepend.c_str(),init.getName().c_str());
    prepend = prepend+"  ";
    for(auto& prop : init.properties)
    {
        if(prop.second.isStringType())
        {
            mexPrintf("%s'%s' > '%s'\n",prepend.c_str(),prop.first.c_str(),boost::any_cast<std::string>(prop.second.get()).c_str());
        }
        else if(prop.second.isInitializerVectorType())
        {
            mexPrintf("%s'%s':\n",prepend.c_str(),prop.first.c_str());
            for(exotica::Initializer& init : boost::any_cast<std::vector<exotica::Initializer>>(prop.second.get()))
            {
                PrintInitializerInfo(init, prepend+"  ");
            }
        }
        else
        {
            mexPrintf("Property '%s' has unsupported type '%s'!\n",prop.first.c_str(), prop.second.getType().c_str());
            return;
        }
    }
}

%}

%include exotica.i

%extend exotica::Initializer
{
    Initializer(std::string name, const mxArray* val)
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

    void print()
    {
        PrintInitializerInfo(*$self);
    }
};



%{
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
