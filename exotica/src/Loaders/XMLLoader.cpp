#include <exotica/Loaders/XMLLoader.h>
#include <tinyxml2/tinyxml2.h>

namespace exotica
{

    boost::shared_ptr<XMLLoader> XMLLoader::instance_ = nullptr;

    XMLLoader::XMLLoader()
    {

    }

    bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix);

    void appendChildXML(Initializer& parent, std::string& name, bool isAttribute, tinyxml2::XMLHandle& tag, const std::string& prefix)
    {
        if(isAttribute)
        {
            // Attributes are always a regular property
            std::string value = tag.ToElement()->Attribute(name.c_str());
            parent.addProperty(Property(name,true,value));
        }
        else
        {
            int count = 0;
            for( tinyxml2::XMLHandle child = tag.FirstChild(); child.ToElement(); child = child.NextSibling() ) count++;
            if(count==0)
            {
                // No child tags = this is a regular property
                std::string value;
                if (!tag.ToElement()->GetText())
                {
                    throw_pretty("Can't get value! ("+name+")");
                }
                value = tag.ToElement()->GetText();
                parent.addProperty(Property(name,true,value));
            }
            else
            {
                // Child tags found = this is an initializer
                // All initializers are treated as a vector
                std::vector<Initializer> ret;
                tinyxml2::XMLHandle child_tag(tag.FirstChildElement());
                while(child_tag.ToElement())
                {
                    ret.push_back(Initializer("New"+name));
                    if(!parseXML(child_tag,ret[ret.size()-1],prefix+"- "))
                    {
                        ret.pop_back();
                    }
                    else
                    {
                        //HIGHLIGHT(prefix<<". Adding parsed vector element "<<name<<" to " << parent.getName());
                    }
                    child_tag = child_tag.NextSiblingElement();
                }
                parent.addProperty(Property(name,true,ret));
            }
        }
    }

    // Parses an Initializer parameters from ints attributes and child tags
    bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix)
    {
        // Get the name
        std::string name = std::string(tag.ToElement()->Name());
        //HIGHLIGHT(prefix<<name<<" added");
        parent.setName("exotica/"+name);

        // Parse values stored in attributes
        tinyxml2::XMLAttribute* attr = const_cast<tinyxml2::XMLAttribute*>(tag.ToElement()->FirstAttribute());
        while(attr)
        {
            std::string member_name = attr->Name();
            appendChildXML(parent,member_name,true,tag,prefix+"- ");
            attr = const_cast<tinyxml2::XMLAttribute*>(attr->Next());
        }

        // Parse values stored in tags
        tinyxml2::XMLHandle member_tag(tag.FirstChildElement());
        while(member_tag.ToElement())
        {
            std::string member_name = std::string(member_tag.ToElement()->Name());
            appendChildXML(parent,member_name,false,member_tag,prefix+"- ");
            member_tag = member_tag.NextSiblingElement();
        }
        //HIGHLIGHT(prefix<<name<<" finished parsing");
        return true;
    }

    void XMLLoader::loadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name, const std::string& problem_name)
    {
        tinyxml2::XMLDocument xml_file;
        if (xml_file.LoadFile(file_name.c_str()) != tinyxml2::XML_NO_ERROR)
        {
          throw_pretty("Can't load file!");
        }

        std::vector<Initializer> initializers;
        tinyxml2::XMLHandle root_tag(xml_file.RootElement()->FirstChildElement());
        while (root_tag.ToElement())
        {
            initializers.push_back(Initializer("TopLevel"));
            if(!parseXML(root_tag,initializers[initializers.size()-1], ""))
            {
                initializers.pop_back();
            }
            root_tag = root_tag.NextSiblingElement();
        }
        bool foundSolver = false;
        bool foundProblem = false;
        for(Initializer& i : initializers)
        {
            std::string name = boost::any_cast<std::string>(i.getProperty("Name"));
            if(name==solver_name) {solver = i; foundSolver=true;}
            if(name==problem_name) {problem = i; foundProblem=true;}
        }
        if(!foundSolver) throw_pretty("Can't find solver '"+solver_name+"' in '"+file_name+"'!");
        if(!foundProblem) throw_pretty("Can't find problem '"+problem_name+"' in '"+file_name+"'!");
    }

}
