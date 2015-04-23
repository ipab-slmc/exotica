#include "exotica/Initialiser.h"

exotica::Initialiser::Initialiser()
{
	//!< Empty constructor
}

exotica::EReturn exotica::Initialiser::initialise(const std::string & file_name,
		Server_ptr & server, MotionSolver_ptr & solver, PlanningProblem_ptr & problem)
{
	return initialise(file_name, server, solver, problem, std::string(), std::string());
}

exotica::EReturn exotica::Initialiser::listSolversAndProblems(const std::string & file_name,
		std::vector<std::string>& problems, std::vector<std::string>& solvers)
{
	EReturn ret_val = SUCCESS;
	EReturn aux_rtn;
	tinyxml2::XMLDocument xml_file;
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

exotica::EReturn exotica::Initialiser::initialise(const std::string & file_name,
		Server_ptr & server, MotionSolver_ptr & solver, PlanningProblem_ptr & problem,
		const std::string & problem_name, const std::string & solver_name)
{
	EReturn ret_val = SUCCESS;
	EReturn aux_rtn;

	//!< Create the document
	tinyxml2::XMLDocument xml_file;
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

	tinyxml2::XMLHandle server_handle(root_handle.FirstChildElement("Server"));
	if (server_handle.ToElement())
	{
		ros::NodeHandlePtr nh;
		nh.reset(new ros::NodeHandle(server_handle.ToElement()->Attribute("name")));
		server.reset(new Server(nh));
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
	tinyxml2::XMLHandle problem_handle(root_handle.FirstChildElement());
	ret_val = FAILURE;
	while (problem_handle.ToElement())
	{
		if (!problem_name.empty())
		{
			const char* atr = problem_handle.ToElement()->Attribute("name");
			if (atr)
			{
				if (std::string(atr).compare(problem_name) != 0)
				{
					problem_handle = problem_handle.NextSibling();
					ret_val == WARNING;
					continue;
				}
			}
			else
			{
				problem_handle = problem_handle.NextSibling();
				ret_val == WARNING;
				ERROR("Element name was not specified!");
				continue;
			}
		}
		aux_rtn = PlanningProblem_fac::Instance().createObject(problem, problem_handle, server);
		if (aux_rtn == WARNING)
		{
			problem_handle = problem_handle.NextSibling();
			ret_val = WARNING;
			continue;
		}
		else if (aux_rtn == SUCCESS)
		{
			ret_val = SUCCESS;
			break;
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
		if (!problem_name.empty())
		{
			ERROR("File '"<<file_name<<"' does not contain the '"<<problem_name<<"' problem definition.");
		}
		else
		{
			ERROR("File '"<<file_name<<"' does not contain any known problem definitions.");
		}
		INDICATE_FAILURE
		return FAILURE;
	}

	tinyxml2::XMLHandle solver_handle(root_handle.FirstChildElement());
	ret_val = FAILURE;
	while (solver_handle.ToElement())
	{
		if (!solver_name.empty())
		{
			const char* atr = solver_handle.ToElement()->Attribute("name");
			INFO(atr);
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
			solver_handle = solver_handle.NextSibling();
			ret_val = SUCCESS;
			//solver->specifyProblem(problem);
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
