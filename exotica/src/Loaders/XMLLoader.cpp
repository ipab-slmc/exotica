#include <exotica/Loaders/XMLLoader.h>
#include <exotica/Setup.h>
#include <tinyxml2/tinyxml2.h>

namespace exotica
{
std::shared_ptr<XMLLoader> XMLLoader::instance_ = nullptr;

XMLLoader::XMLLoader()
{
}

bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix);

void appendChildXML(Initializer& parent, std::string& name, bool isAttribute, tinyxml2::XMLHandle& tag, const std::string& prefix)
{
    if (isAttribute)
    {
        // Attributes are always a regular property
        std::string value = tag.ToElement()->Attribute(name.c_str());
        parent.addProperty(Property(name, true, value));
    }
    else
    {
        int count = 0;
        for (tinyxml2::XMLHandle child = tag.FirstChild(); child.ToNode(); child = child.NextSibling())
            if (child.ToElement() != nullptr) count++;
        if (count == 0)
        {
            if (tag.ToElement() == nullptr) return;

            if (!tag.ToElement()->GetText())
            {
                // No child tags = this is an empty vector of properties
                return;
            }
            std::string value = tag.ToElement()->GetText();
            parent.addProperty(Property(name, true, value));
        }
        else
        {
            // Child tags found = this is an initializer
            // All initializers are treated as a vector
            std::vector<Initializer> ret;
            tinyxml2::XMLHandle child_tag = tag.FirstChild();
            while (child_tag.ToNode())
            {
                if (child_tag.ToElement() == nullptr)
                {
                    child_tag = child_tag.NextSibling();
                    continue;
                }
                ret.push_back(Initializer("New" + name));
                if (!parseXML(child_tag, ret[ret.size() - 1], prefix + "- "))
                {
                    ret.pop_back();
                }
                else
                {
                    //HIGHLIGHT(prefix<<". Adding parsed vector element "<<name<<" to " << parent.getName());
                }
                child_tag = child_tag.NextSibling();
            }
            parent.addProperty(Property(name, true, ret));
        }
    }
}

// Parses an Initializer parameters from ints attributes and child tags
bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix)
{
    // Get the name
    std::string name = std::string(tag.ToElement()->Name());
    //HIGHLIGHT(prefix<<name<<" added");
    parent.setName("exotica/" + name);

    // Parse values stored in attributes
    tinyxml2::XMLAttribute* attr = const_cast<tinyxml2::XMLAttribute*>(tag.ToElement()->FirstAttribute());
    while (attr)
    {
        std::string member_name = attr->Name();
        appendChildXML(parent, member_name, true, tag, prefix + "- ");
        attr = const_cast<tinyxml2::XMLAttribute*>(attr->Next());
    }

    // Parse values stored in tags
    tinyxml2::XMLHandle member_tag = tag.FirstChild();
    while (member_tag.ToNode())
    {
        if (member_tag.ToElement() == nullptr)
        {
            member_tag = member_tag.NextSibling();
            continue;
        }
        std::string member_name = std::string(member_tag.ToElement()->Name());
        appendChildXML(parent, member_name, false, member_tag, prefix + "- ");
        member_tag = member_tag.NextSibling();
    }
    //HIGHLIGHT(prefix<<name<<" finished parsing");
    return true;
}

Initializer XMLLoader::loadXML(std::string file_name, bool parsePathAsXML)
{
    tinyxml2::XMLDocument xml_file;
    if (parsePathAsXML)
    {
        if (xml_file.Parse(file_name.c_str(), file_name.size()) == tinyxml2::XML_SUCCESS)
        {
            throw_pretty("Can't load file!\nFile: '" + file_name + "'");
        }
    }
    else
    {
        std::string xml = loadFile(file_name);
        if (xml_file.Parse(xml.c_str(), xml.size()) == tinyxml2::XML_SUCCESS)
        {
            throw_pretty("Can't load file!\nFile: '" + parsePath(file_name) + "'");
        }
    }

    Initializer ret("TopLevel");
    tinyxml2::XMLHandle root_tag(xml_file.RootElement()->FirstChildElement());
    if (!parseXML(root_tag, ret, ""))
    {
        throw_pretty("Can't parse XML!\nFile: '" + file_name + "'");
    }
    return ret;
}

void XMLLoader::loadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name, const std::string& problem_name, bool parsePathAsXML)
{
    tinyxml2::XMLDocument xml_file;
    if (parsePathAsXML)
    {
        if (xml_file.Parse(file_name.c_str(), file_name.size()) == tinyxml2::XML_SUCCESS)
        {
            throw_pretty("Can't load file!\nFile: '" + file_name + "'");
        }
    }
    else
    {
        std::string xml = loadFile(file_name);
        if (xml_file.Parse(xml.c_str(), xml.size()) == tinyxml2::XML_SUCCESS)
        {
            throw_pretty("Can't load file!\nFile: '" + parsePath(file_name) + "'");
        }
    }

    std::vector<Initializer> initializers;
    tinyxml2::XMLHandle root_tag = xml_file.RootElement()->FirstChild();
    while (root_tag.ToNode())
    {
        if (root_tag.ToElement() == nullptr)
        {
            root_tag = root_tag.NextSibling();
            continue;
        }
        initializers.push_back(Initializer("TopLevel"));
        if (!parseXML(root_tag, initializers[initializers.size() - 1], ""))
        {
            initializers.pop_back();
        }
        root_tag = root_tag.NextSibling();
    }
    bool foundSolver = false;
    bool foundProblem = false;
    if (solver_name == "" || solver_name == "")
    {
        for (Initializer& i : initializers)
        {
            std::string initializer_type = i.getName();
            if (!foundSolver)
            {
                for (std::string known_type : Setup::getSolvers())
                {
                    if (known_type == initializer_type)
                    {
                        solver = i;
                        foundSolver = true;
                        break;
                    }
                }
            }
            if (!foundProblem)
            {
                for (std::string known_type : Setup::getProblems())
                {
                    if (known_type == initializer_type)
                    {
                        problem = i;
                        foundProblem = true;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        for (Initializer& i : initializers)
        {
            std::string name = boost::any_cast<std::string>(i.getProperty("Name"));
            if (name == solver_name)
            {
                solver = i;
                foundSolver = true;
            }
            if (name == problem_name)
            {
                problem = i;
                foundProblem = true;
            }
        }
    }
    if (!foundSolver) throw_pretty("Can't find solver '" + solver_name + "' in '" + file_name + "'!");
    if (!foundProblem) throw_pretty("Can't find problem '" + problem_name + "' in '" + file_name + "'!");
}
}
