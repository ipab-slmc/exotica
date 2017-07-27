#ifndef XMLLOADER_H
#define XMLLOADER_H

#include <exotica/Property.h>

namespace exotica
{

class XMLLoader
{
public:
    static std::shared_ptr<XMLLoader> Instance()
    {
      if (!instance_) instance_.reset(new XMLLoader);
      return instance_;
    }

    ~XMLLoader() noexcept
    {
    }

    Initializer loadXML(std::string file_name, bool parsePathAsXML=false);
    void loadXML(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML=false);
    static void load(std::string file_name, Initializer& solver, Initializer& problem, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML=false)
    {
        Instance()->loadXML(file_name,solver,problem,solver_name,problem_name, parsePathAsXML);
    }

    static Initializer load(std::string file_name, bool parsePathAsXML=false)
    {
        return Instance()->loadXML(file_name, parsePathAsXML);
    }

private:
    XMLLoader();
    static std::shared_ptr<XMLLoader> instance_;
};

}

#endif // XMLLOADER_H
