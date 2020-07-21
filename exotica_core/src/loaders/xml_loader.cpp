//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <tinyxml2.h>

#include <exotica_core/loaders/xml_loader.h>
#include <exotica_core/setup.h>

namespace exotica
{
std::shared_ptr<XMLLoader> XMLLoader::instance_ = nullptr;

XMLLoader::XMLLoader() = default;

bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix);

void appendChildXML(Initializer& parent, std::string& name, bool isAttribute, tinyxml2::XMLHandle& tag, const std::string& prefix)
{
    if (isAttribute)
    {
        // Attributes are always a regular property
        std::string value = tag.ToElement()->Attribute(name.c_str());
        parent.AddProperty(Property(name, true, value));
    }
    else
    {
        int count = 0;
        for (tinyxml2::XMLHandle child = tag.FirstChild(); child.ToNode(); child = child.NextSibling())
            if (child.ToElement() != nullptr) ++count;
        if (count == 0)
        {
            if (tag.ToElement() == nullptr) return;

            if (!tag.ToElement()->GetText())
            {
                // No child tags = this is an empty vector of properties
                return;
            }
            const std::string value = tag.ToElement()->GetText();
            parent.AddProperty(Property(name, true, value));
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
                    //HIGHLIGHT(prefix<<". Adding parsed vector element "<<name<<" to " << parent.GetName());
                }
                child_tag = child_tag.NextSibling();
            }
            parent.AddProperty(Property(name, true, ret));
        }
    }
}

// Parses an Initializer parameters from ints attributes and child tags
bool parseXML(tinyxml2::XMLHandle& tag, Initializer& parent, const std::string& prefix)
{
    // Get the name
    std::string name = std::string(tag.ToElement()->Name());
    //HIGHLIGHT(prefix<<name<<" added");
    parent.SetName("exotica/" + name);

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

Initializer XMLLoader::LoadXML(std::string file_name, bool parsePathAsXML)
{
    tinyxml2::XMLDocument xml_file;
    if (parsePathAsXML)
    {
        if (xml_file.Parse(file_name.c_str()) != tinyxml2::XML_SUCCESS)
        {
            ThrowPretty("Can't load file!\nFile: '" + file_name + "'");
        }
    }
    else
    {
        std::string xml = LoadFile(file_name);  // assume it is a null-terminated string
        if (xml_file.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
        {
            ThrowPretty("Can't load file!\nFile: '" + ParsePath(file_name) + "'");
        }
    }

    Initializer ret("TopLevel");
    tinyxml2::XMLHandle root_tag(xml_file.RootElement()->FirstChildElement());
    if (!parseXML(root_tag, ret, ""))
    {
        ThrowPretty("Can't parse XML!\nFile: '" + file_name + "'");
    }
    return ret;
}

void XMLLoader::LoadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name, const std::string& problem_name, bool parsePathAsXML)
{
    tinyxml2::XMLDocument xml_file;
    if (parsePathAsXML)
    {
        if (xml_file.Parse(file_name.c_str()) != tinyxml2::XML_SUCCESS)
        {
#ifdef TINYXML_HAS_ERROR_STR
            ThrowPretty("Can't load file!\n"
                        << xml_file.ErrorStr() << "\nFile: '" + file_name + "'");
#else
            ThrowPretty("Can't load file!"
                        << "\nFile: '" + file_name + "'");
#endif
        }
    }
    else
    {
        std::string xml = LoadFile(file_name);  // assume LoadFile returns a null-terminated string
        tinyxml2::XMLError return_code = xml_file.Parse(xml.c_str());
        if (xml_file.Error())
        {
#ifdef TINYXML_HAS_ERROR_STR
            ThrowPretty("Can't load file! Return code: " << return_code << "\n"
                                                         << xml_file.ErrorStr() << "\nFile: '" + ParsePath(file_name) + "'");
#else
            ThrowPretty("Can't load file! Return code: " << return_code << "\n"
                                                         << "File: '" + ParsePath(file_name) + "'");
#endif
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
    bool found_solver = false;
    bool found_problem = false;
    if (solver_name.empty() || solver_name == "")
    {
        for (Initializer& i : initializers)
        {
            std::string initializer_type = i.GetName();
            if (!found_solver)
            {
                for (std::string known_type : Setup::GetSolvers())
                {
                    if (known_type == initializer_type)
                    {
                        solver = i;
                        found_solver = true;
                        break;
                    }
                }
            }
            if (!found_problem)
            {
                for (std::string known_type : Setup::GetProblems())
                {
                    if (known_type == initializer_type)
                    {
                        problem = i;
                        found_problem = true;
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
            std::string name = boost::any_cast<std::string>(i.GetProperty("Name"));
            if (name == solver_name)
            {
                solver = i;
                found_solver = true;
            }
            if (name == problem_name)
            {
                problem = i;
                found_problem = true;
            }
        }
    }
    if (!found_solver) ThrowPretty("Can't find solver '" + solver_name + "' in '" + file_name + "'!");
    if (!found_problem) ThrowPretty("Can't find problem '" + problem_name + "' in '" + file_name + "'!");
}
}  // namespace exotica
