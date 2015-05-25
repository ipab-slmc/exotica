#include "exotica/Initialiser.h"

using namespace rapidjson;

namespace exotica
{

	Initialiser::Initialiser()
	{
		//!< Empty constructor
	}

	EReturn Initialiser::initialise(const std::string & file_name, Server_ptr & server,
			MotionSolver_ptr & solver, PlanningProblem_ptr & problem)
	{
        std::vector<std::string> probs, sols;
        if(ok(Initialiser::listSolversAndProblems(file_name, probs, sols)))
        {
            if(probs.size()>0&&sols.size()>0)
            {
                return initialise(file_name, server, solver, problem, probs[0], sols[0]);
            }
            else
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
	}

    EReturn Initialiser::initialiseProblemJSON(PlanningProblem_ptr problem, const std::string& constraints)
    {
        {
            Document document;
            if (!document.Parse<0>(constraints.c_str()).HasParseError())
            {
                if(ok(problem->reinitialise(document,problem)))
                {
                    // Everythinh is fine
                }
                else
                {
                    INDICATE_FAILURE;
                    return FAILURE;
                }
            }
            else
            {
                ERROR("Can't parse constraints from JSON string!\n"<<document.GetParseError() <<"\n"<<constraints.substr(document.GetErrorOffset(),50));
                return FAILURE;
            }
        }
        return SUCCESS;
    }

	EReturn Initialiser::listSolversAndProblems(const std::string & file_name,
			std::vector<std::string>& problems, std::vector<std::string>& solvers)
	{
		EReturn ret_val = SUCCESS;
		EReturn aux_rtn;
		xml_file.Clear();
		if (xml_file.LoadFile(file_name.c_str()) != tinyxml2::XML_NO_ERROR)
		{
			INDICATE_FAILURE
			;
			xml_file.Clear();
			return PAR_ERR;
		}
		tinyxml2::XMLHandle root_handle(xml_file.RootElement());

		//!< First resolve any includes that exist
		std::string file_path = file_name;
		ret_val = resolveParent(file_path);
		if (!ok(ret_val))
		{
			INDICATE_FAILURE
			;
			xml_file.Clear();
			return ret_val;
		}
		file_path += "/";
		aux_rtn = parseIncludes(root_handle, file_path);
		if (aux_rtn)
		{
			ret_val = aux_rtn;
		}
		if (!ok(ret_val))
		{
			INDICATE_FAILURE
			;
			xml_file.Clear();
			return ret_val;
		}

		ret_val = SUCCESS;
		std::vector<std::string> registered_problems;
		PlanningProblem_fac::Instance().listImplementations(registered_problems);
		tinyxml2::XMLHandle problem_handle(root_handle.FirstChildElement());
		problems.clear();
		while (problem_handle.ToElement())
		{
			const char* atr = problem_handle.ToElement()->Attribute("name");
			const char* tp = problem_handle.ToElement()->Name();
			if (atr)
			{
				bool found = false;
				for (std::string& name : registered_problems)
				{
					if (std::string(tp).compare(name) == 0)
					{
						found = true;
						problems.push_back(std::string(atr));
						break;
					}
				}
				if (!found)
				{
					ERROR("Problem '"<<atr<<"' has unrecognized type '"<<tp<<"'");
				}
				problem_handle = problem_handle.NextSibling();
			}
			else
			{
				problem_handle = problem_handle.NextSibling();
				ERROR("Element name was not specified!");
				continue;
			}
		}

		std::vector<std::string> registered_solvers;
		MotionSolver_fac::Instance().listImplementations(registered_solvers);
		tinyxml2::XMLHandle solver_handle(root_handle.FirstChildElement());
		solvers.clear();
		while (solver_handle.ToElement())
		{
			const char* atr = solver_handle.ToElement()->Attribute("name");
			const char* tp = solver_handle.ToElement()->Name();
			if (atr)
			{
				bool found = false;
				for (std::string& name : registered_solvers)
				{
					if (std::string(tp).compare(name) == 0)
					{
						found = true;
						solvers.push_back(std::string(atr));
						break;
					}
				}
				if (!found)
				{
					ERROR("Solver '"<<atr<<"' has unrecognized type '"<<tp<<"'");
				}
				solver_handle = solver_handle.NextSibling();
			}
			else
			{
				solver_handle = solver_handle.NextSibling();
				ERROR("Element name was not specified!");
				continue;
			}
		}
		return ret_val;
	}

    EReturn Initialiser::initialise(tinyxml2::XMLHandle root_handle, Server_ptr & server)
    {
        server = Server::Instance();
        tinyxml2::XMLHandle server_handle(root_handle.FirstChildElement("Server"));
        if (server_handle.ToElement())
        {
            if (ok(server->initialise(server_handle)))
            {
                return SUCCESS;
            }
            else
            {
                INDICATE_FAILURE
                return FAILURE;
            }
        }
        else
        {
            ERROR("EXOTica Server element is missing in the xml");
            INDICATE_FAILURE
            return FAILURE;
        }
    }

    EReturn Initialiser::initialise(tinyxml2::XMLHandle root_handle, PlanningProblem_ptr & problem, const std::string & problem_name, Server_ptr & server)
    {
        tinyxml2::XMLHandle problem_handle(root_handle.FirstChildElement());
        while (problem_handle.ToElement())
        {
            const char* atr = problem_handle.ToElement()->Attribute("name");
            if (atr)
            {
                if (std::string(atr).compare(problem_name) == 0)
                {
                    if (ok(PlanningProblem_fac::Instance().createObject(problem, problem_handle, server)))
                    {
                        return SUCCESS;
                    }
                    else
                    {
                        INDICATE_FAILURE;
                        return FAILURE;
                    }
                }
                else
                {
                    problem_handle = problem_handle.NextSibling();
                    continue;
                }
            }
            else
            {
                problem_handle = problem_handle.NextSibling();
                ERROR("Element name was not specified!");
                continue;
            }
        }

        ERROR("File does not contain the '"<<problem_name<<"' problem definition.");
        return FAILURE;
    }

    EReturn Initialiser::initialise(tinyxml2::XMLHandle root_handle, MotionSolver_ptr & solver, const std::string & solver_name, Server_ptr & server)
    {
        tinyxml2::XMLHandle solver_handle(root_handle.FirstChildElement());
        while (solver_handle.ToElement())
        {
            if (!solver_name.empty())
            {
                const char* atr = solver_handle.ToElement()->Attribute("name");
                if (atr)
                {
                    if (std::string(atr).compare(solver_name) == 0)
                    {
                        if (ok(MotionSolver_fac::Instance().createObject(solver, solver_handle, server)))
                        {
                            return SUCCESS;
                        }
                        else
                        {
                            INDICATE_FAILURE;
                            return FAILURE;
                        }
                    }
                    else
                    {
                        solver_handle = solver_handle.NextSibling();
                        continue;
                    }

                }
                else
                {
                    solver_handle = solver_handle.NextSibling();
                    ERROR("Element name was not specified!");
                    continue;
                }
            }

        }

        ERROR("File does not contain the '"<<solver_name<<"' solver.");
        return FAILURE;
    }

	EReturn Initialiser::initialise(const std::string & file_name, Server_ptr & server,
			MotionSolver_ptr & solver, PlanningProblem_ptr & problem,
			const std::string & problem_name, const std::string & solver_name)
	{
        // /////////////////////////
        // Create the document
		xml_file.Clear();
		if (xml_file.LoadFile(file_name.c_str()) != tinyxml2::XML_NO_ERROR)
		{
            INDICATE_FAILURE;
			xml_file.Clear();
			return PAR_ERR;
		}
		tinyxml2::XMLHandle root_handle(xml_file.RootElement());

        // /////////////////////////
        // First resolve any includes that exist
		std::string file_path = file_name;
		if (!ok(resolveParent(file_path)))
		{
            INDICATE_FAILURE;
			xml_file.Clear();
            return FAILURE;
		}
		file_path += "/";
		if (!ok(parseIncludes(root_handle, file_path)))
		{
            INDICATE_FAILURE;
			xml_file.Clear();
            return FAILURE;
		}

        // /////////////////////////
        // Initialise server
        if(!ok(initialise(root_handle, server)))
        {
            INDICATE_FAILURE;
            return FAILURE;
        }

        // /////////////////////////
        // Initialise problem
        if(!ok(initialise(root_handle, problem, problem_name, server)))
        {
            INDICATE_FAILURE;
            return FAILURE;
        }

        // /////////////////////////
        // Initialise solver
        if(!ok(initialise(root_handle, solver, solver_name, server)))
        {
            INDICATE_FAILURE;
            return FAILURE;
        }

        return SUCCESS;
	}

    EReturn Initialiser::initialise(const std::string & file_name, Server_ptr & server, std::vector<MotionSolver_ptr> & solver, std::vector<PlanningProblem_ptr> & problem,std::vector<std::string> & problem_name, std::vector<std::string> & solver_name)
    {
        // /////////////////////////
        // Create the document
        xml_file.Clear();
        if (xml_file.LoadFile(file_name.c_str()) != tinyxml2::XML_NO_ERROR)
        {
            INDICATE_FAILURE;
            xml_file.Clear();
            return PAR_ERR;
        }
        tinyxml2::XMLHandle root_handle(xml_file.RootElement());

        // /////////////////////////
        // First resolve any includes that exist
        std::string file_path = file_name;
        if (!ok(resolveParent(file_path)))
        {
            INDICATE_FAILURE;
            xml_file.Clear();
            return FAILURE;
        }
        file_path += "/";
        if (!ok(parseIncludes(root_handle, file_path)))
        {
            INDICATE_FAILURE;
            xml_file.Clear();
            return FAILURE;
        }

        // /////////////////////////
        // Initialise server
        if(!ok(initialise(root_handle, server)))
        {
            INDICATE_FAILURE;
            return FAILURE;
        }

        // /////////////////////////
        // Initialise problem
        problem.resize(problem_name.size());
        for(int i=0;i<problem_name.size();i++)
        {
            if(!ok(initialise(root_handle, problem[i], problem_name[i], server)))
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }

        // /////////////////////////
        // Initialise solver
        solver.resize(solver_name.size());
        for(int i=0;i<solver_name.size();i++)
        {
            if(!ok(initialise(root_handle, solver[i], solver_name[i], server)))
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }

        return SUCCESS;
    }
}
