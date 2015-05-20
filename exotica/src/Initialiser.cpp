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
		return initialise(file_name, server, solver, problem, std::string(), std::string());
	}

	EReturn Initialiser::initialiseProblemJSON(PlanningProblem_ptr problem,
			const std::string& constraints, const std::string& poses)
	{
		{
			Document document;
			if (!document.Parse<0>(constraints.c_str()).HasParseError())
			{
				if (ok(problem->reinitialise(document)))
				{
					// Everythinh is fine
				}
				else
				{
					INDICATE_FAILURE
					;
					return FAILURE;
				}
			}
			else
			{
				ERROR("Can't parse constraints from JSON string!\n"<<document.GetParseError() <<"\n"<<constraints.substr(document.GetErrorOffset(),50));
				return FAILURE;
			}
		}
		{
			Document document;
			if (!document.Parse<0>(poses.c_str()).HasParseError())
			{
				Eigen::VectorXd tmp;
				if (ok(getJSON(document["reach_start"], tmp)))
				{
					int n = problem->getScenes().begin()->second->getNumJoints();
					if (tmp.rows() >= n)
					{
						problem->startState = tmp.tail(n);
					}
					else
					{
						INDICATE_FAILURE
						;
						return FAILURE;
					}
				}
				else
				{
					ERROR("Start pose not specified!");
					return FAILURE;
				}
			}
			else
			{
				ERROR("Can't parse constraints from JSON string!\n"<<document.GetParseError() <<"\n"<<poses.substr(document.GetErrorOffset(),50));
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

	EReturn Initialiser::initialise(const std::string & file_name, Server_ptr & server,
			MotionSolver_ptr & solver, PlanningProblem_ptr & problem,
			const std::string & problem_name, const std::string & solver_name)
	{
		EReturn ret_val = SUCCESS;
		EReturn aux_rtn;

		//!< Create the document
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
		if (!ok(resolveParent(file_path)))
		{
			INDICATE_FAILURE
			;
			xml_file.Clear();
			return ret_val;
		}
		file_path += "/";
		if (!ok(parseIncludes(root_handle, file_path)))
		{
			INDICATE_FAILURE
			;
			xml_file.Clear();
			return ret_val;
		}

		if (server != nullptr)
		{
			INFO("Using existing EXOTica server");
		}
		else
		{
			server.reset(new Server);
			tinyxml2::XMLHandle server_handle(root_handle.FirstChildElement("Server"));
			if (server_handle.ToElement())
			{
				if (!ok(server->initialise(server_handle)))
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

		if (problem_name.empty())
		{
			ERROR("File '"<<file_name<<"' does not contain any known problem definitions.");
			return FAILURE;
		}

		tinyxml2::XMLHandle problem_handle(root_handle.FirstChildElement());
		ret_val = FAILURE;
		while (problem_handle.ToElement())
		{
			const char* atr = problem_handle.ToElement()->Attribute("name");
			if (atr)
			{
				if (std::string(atr).compare(problem_name) == 0)
				{
					if (ok(PlanningProblem_fac::Instance().createObject(problem, problem_handle, server)))
					{
						ret_val = SUCCESS;
						break;
					}
					else
					{
						INDICATE_FAILURE
						;
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
		if (!ok(ret_val))
		{
			ERROR("File '"<<file_name<<"' does not contain the '"<<problem_name<<"' problem definition.");
			return FAILURE;
		}

		tinyxml2::XMLHandle solver_handle(root_handle.FirstChildElement());
		ret_val = FAILURE;
		while (solver_handle.ToElement())
		{
			if (!solver_name.empty())
			{
				const char* atr = solver_handle.ToElement()->Attribute("name");
//				INFO(atr);
				if (atr)
				{
					if (std::string(atr).compare(solver_name) != 0)
					{
						solver_handle = solver_handle.NextSibling();
						ret_val == WARNING;
						continue;
					}
				}
				else
				{
					solver_handle = solver_handle.NextSibling();
					ret_val == WARNING;
					ERROR("Element name was not specified!");
					continue;
				}
			}
			aux_rtn = MotionSolver_fac::Instance().createObject(solver, solver_handle, server);
			if (aux_rtn == WARNING)
			{
				solver_handle = solver_handle.NextSibling();
				ret_val = WARNING;
				continue;
			}
			else if (aux_rtn == SUCCESS)
			{
				return SUCCESS;
			}
			else
			{
				INDICATE_FAILURE
				;
				return aux_rtn;
			}

		}
		if (!ok(ret_val))
		{
			if (!solver_name.empty())
			{
				ERROR("File '"<<file_name<<"' does not contain the '"<<solver_name<<"' solver.");
			}
			else
			{
				ERROR("File '"<<file_name<<"' does not contain any known solvers.");
			}
			INDICATE_FAILURE
			return FAILURE;
		}

		return ret_val;
	}
}
