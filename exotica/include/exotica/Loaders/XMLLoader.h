#ifndef XMLLOADER_H
#define XMLLOADER_H

#include <exotica/Property.h>
#include <exotica/Initialiser.h>

namespace exotica
{

class XMLLoader
{
public:
    static boost::shared_ptr<XMLLoader> Instance()
    {
      if (!instance_) instance_.reset(new XMLLoader);
      return instance_;
    }

    ~XMLLoader() noexcept
    {
    }

    void loadXML(std::string file_name, InitializerGeneric& solver, InitializerGeneric& problem, const std::string& solver_name, const std::string& problem_name);
    static void load(std::string file_name, InitializerGeneric& solver, InitializerGeneric& problem, const std::string& solver_name, const std::string& problem_name)
    {
        Instance()->loadXML(file_name,solver,problem,solver_name,problem_name);
    }

private:
    XMLLoader();
    static boost::shared_ptr<XMLLoader> instance_;
};

}

#endif // XMLLOADER_H
