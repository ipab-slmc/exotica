#include <exotica/Loaders/XMLLoader.h>
#include <tinyxml2/tinyxml2.h>

namespace exotica
{

    boost::shared_ptr<XMLLoader> XMLLoader::instance_ = nullptr;

    struct PropertyInfo
    {
        std::string Type;
        bool IsContainer;
        bool IsVectorContainer;
        PropertyInfo() = default;
        PropertyInfo(std::string Type_, bool IsContainer_, bool IsVectorContainer_) : IsContainer(IsContainer_), Type(Type_), IsVectorContainer(IsVectorContainer_) {}
    };

    XMLLoader::XMLLoader()
    {

    }

    inline bool IsContainerType(std::string type)
    {
        return type=="exotica::InitializerGeneric";
    }

    inline bool IsVectorType(std::string type)
    {
        return type.substr(0,11)=="std::vector";
    }

    inline bool IsVectorContainerType(std::string type)
    {
        return type=="std::vector<exotica::InitializerGeneric>";
    }

    bool parse(tinyxml2::XMLHandle& tag, InitializerGeneric& parent, std::map<std::string,std::map<std::string,PropertyInfo>>& info, const std::string& prefix);

    boost::shared_ptr<PropertyElement> parseValue(const std::string type, const std::string name, const std::string value)
    {
        boost::shared_ptr<PropertyElement> ret;
        if(type=="std::string")
        {
            ret.reset(new Property<std::string>(type,name,true,value));
        }
        else if(type=="std::vector<std::string>")
        {
            ret.reset(new Property<std::vector<std::string>>(type,name,true,parseList(value)));
        }
        else if(type=="int")
        {
            ret.reset(new Property<int>(type,name,true,parseInt(value)));
        }
        else if(type=="double")
        {
            ret.reset(new Property<double>(type,name,true,parseDouble(value)));
        }
        else if(type=="Eigen::VectorXd")
        {
            ret.reset(new Property<Eigen::VectorXd>(type,name,true,parseVector(value)));
        }
        else
        {
            throw_pretty("Unsupported type '"+type+"'!");
        }
        return ret;
    }

    void appendChild(InitializerGeneric& parent, std::string& name, PropertyInfo& prop, bool isAttribute, tinyxml2::XMLHandle& tag, std::map<std::string,std::map<std::string,PropertyInfo>>& info, const std::string& prefix)
    {
        if(isAttribute && (prop.IsContainer || prop.IsVectorContainer)) throw_pretty("Attributes can only be basic types! ("+parent.getName()+"/"+name+")");
        HIGHLIGHT(prefix<<"Adding "<<name<<" to " << parent.getName());
        if(prop.IsContainer)
        {
            HIGHLIGHT(prefix<<". Parsing container "<<name<<" (" << parent.getName()<<")");
            tinyxml2::XMLHandle child_tag(tag.FirstChildElement());
            InitializerGeneric tmp;
            if(parse(child_tag,tmp,info,prefix+"- "))
            {
                HIGHLIGHT(prefix<<". Adding parsed "<<name<<" to " << parent.getName());
                parent.addProperty(boost::shared_ptr<Property<InitializerGeneric>>(new Property<InitializerGeneric>(prop.Type,name,true,tmp)));
            }
            else
            {
                HIGHLIGHT(prefix<<". Failed to parse "<<name<<" (" << parent.getName()<<")");
            }
        }
        else if (prop.IsVectorContainer)
        {
            std::vector<InitializerGeneric> tmp_vec;
            tinyxml2::XMLHandle child_tag(tag.FirstChildElement());
            while(child_tag.ToElement())
            {
                tmp_vec.push_back(InitializerGeneric("New"+name));
                if(!parse(child_tag,tmp_vec[tmp_vec.size()-1],info,prefix+"- "))
                {
                    tmp_vec.pop_back();
                }
                else
                {
                    HIGHLIGHT(prefix<<". Adding parsed vector element "<<name<<" to " << parent.getName());
                }
                child_tag = child_tag.NextSiblingElement();
            }
            HIGHLIGHT(prefix<<". Adding parsed vector "<<name<<" to " << parent.getName());
            parent.addProperty(boost::shared_ptr<Property<std::vector<InitializerGeneric>>>(new Property<std::vector<InitializerGeneric>>(prop.Type,name,true,tmp_vec)));
        }
        else
        {
            HIGHLIGHT(prefix<<". Adding parsed value "<<name<<" to " << parent.getName());
            if(isAttribute)
            {
                std::string value = tag.ToElement()->Attribute(name.c_str());
                parent.addProperty(parseValue(prop.Type,name,value));
            }
            else
            {
                std::string value;
                if (!tag.ToElement()->GetText())
                {
                    throw_pretty("Can't get value! ("+name+")");
                }
                value = tag.ToElement()->GetText();
                parent.addProperty(parseValue(prop.Type,name,value));
            }

        }
    }

    bool parse(tinyxml2::XMLHandle& tag, InitializerGeneric& parent, std::map<std::string,std::map<std::string,PropertyInfo>>& info, const std::string& prefix)
    {
        std::string name = std::string(tag.ToElement()->Name());
        if(info.find(name)!=info.end())
        {
            HIGHLIGHT(prefix<<name<<" added");
            parent.setName("exotica/"+name);
            std::map<std::string,PropertyInfo>& members = info[name];
            tinyxml2::XMLAttribute* attr = const_cast<tinyxml2::XMLAttribute*>(tag.ToElement()->FirstAttribute());
            while(attr)
            {
                std::string member_name = attr->Name();
                if(members.find(member_name)!=members.end())
                {
                    appendChild(parent,member_name,members.at(member_name),true,tag,info,prefix+"- ");
                }
                else
                {
                    HIGHLIGHT("Found unknown attribute '"<<member_name<<"'' of '"<<name<<"', ignoring.");
                }
                attr = const_cast<tinyxml2::XMLAttribute*>(attr->Next());
            }

            tinyxml2::XMLHandle member_tag(tag.FirstChildElement());
            while(member_tag.ToElement())
            {
                std::string member_name = std::string(member_tag.ToElement()->Name());
                if(members.find(member_name)!=members.end())
                {
                    appendChild(parent,member_name,members.at(member_name),false,member_tag,info,prefix+"- ");
                }
                else
                {
                    HIGHLIGHT("Found unknown node '"<<member_name<<"'' as child of '"<<name<<"', ignoring.");
                }

                member_tag = member_tag.NextSiblingElement();
            }
            HIGHLIGHT(prefix<<name<<" finished parsing");
            return true;
        }
        else
        {
            HIGHLIGHT("Found unknown XML tag '"<<name<<"', ignoring.");
            return false;
        }
    }

    void XMLLoader::loadXML(std::string file_name, InitializerGeneric& solver, InitializerGeneric& problem, const std::string& solver_name, const std::string& problem_name)
    {
        std::map<std::string,std::vector<std::string>> inits = Initialiser::Instance()->getInitilizerTypes();
        std::map<std::string,std::map<std::string,PropertyInfo>> info;
        for(auto& prop : inits)
        {
            for(std::string& prop_name : prop.second)
            {
                int pos = prop_name.rfind('/');
                std::string name = prop_name.substr(8,pos-8);
                std::string member = prop_name.substr(pos+1);
                info[name][member]=PropertyInfo(prop.first,IsContainerType(prop.first),IsVectorContainerType(prop.first));
            }
        }

        for(auto& i : info)
        {
            HIGHLIGHT(i.first);
            for(auto& j : i.second)
            {
                HIGHLIGHT(" - "<<j.first<<", "<<j.second.Type<<" "<<j.second.IsContainer<<" "<<j.second.IsVectorContainer);
            }
        }

        tinyxml2::XMLDocument xml_file;
        if (xml_file.LoadFile(file_name.c_str()) != tinyxml2::XML_NO_ERROR)
        {
          throw_pretty("Can't load file!");
        }

        std::vector<InitializerGeneric> initializers;
        tinyxml2::XMLHandle root_tag(xml_file.RootElement()->FirstChildElement());
        while (root_tag.ToElement())
        {
            initializers.push_back(InitializerGeneric("TopLevel"));
            if(!parse(root_tag,initializers[initializers.size()-1], info,""))
            {
                initializers.pop_back();
            }
            root_tag = root_tag.NextSiblingElement();
        }
        bool foundSolver = false;
        bool foundProblem = false;
        for(InitializerGeneric& i : initializers)
        {
            std::string name;
            getProperty("Name",i,name);
            if(name==solver_name) {solver = i; foundSolver=true;}
            if(name==problem_name) {problem = i; foundProblem=true;}
        }
        if(!foundSolver) throw_pretty("Can't find solver '"+solver_name+"' in '"+file_name+"'!");
        if(!foundProblem) throw_pretty("Can't find problem '"+problem_name+"' in '"+file_name+"'!");
    }

}
