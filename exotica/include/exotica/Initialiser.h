/***********************************************************************\
|    This class wraps up all the functionality of initialising a new    |
|  EXOTica planning problem.                                            |
|                                                                       |
|           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
|                    Last Edited: 02 - April - 2014                     |
\***********************************************************************/

#ifndef EXOTICA_INITIALISER_H
#define EXOTICA_INITIALISER_H

#include "exotica/Tools.h"
#include "exotica/Object.h"
#include "exotica/Factory.h"
#include "exotica/MotionSolver.h"
#include "exotica/PlanningProblem.h"
#include "exotica/Server.h"
#include "rapidjson/document.h"

namespace exotica
{
  class Initialiser : public Object
  {
    public:
      /**
       * \brief Default Constructor
       * 
       *        Currently, is an empty constructor definition.
       */
      Initialiser();
    
      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Shared pointer to a Motion Solver
       * @param problem   Shared pointer to a Planning Problem
       * @return          Indication of Success: TODO
       */
      EReturn initialise(const std::string & file_name, Server_ptr & server, MotionSolver_ptr & solver, PlanningProblem_ptr & problem);

      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Shared pointer to a Motion Solver
       * @param problem   Shared pointer to a Planning Problem
       * @param problem_name Problem name
       * @param solver_name Solver name
       * @return          Indication of Success: TODO
       */
      EReturn initialise(const std::string & file_name, Server_ptr & server, MotionSolver_ptr & solver, PlanningProblem_ptr & problem, const std::string & problem_name, const std::string & solver_name);

      /**
       * \brief Creates a list of supported problems and solvers specified in a XML file
       * @param file_name File name
       * @param problems Return vector of prolem names
       * @param problems Return vector of solver names
       * @return Indication of success
       */
      EReturn listSolversAndProblems(const std::string & file_name, std::vector<std::string>& problems, std::vector<std::string>& solvers);

      EReturn initialiseProblemJSON(PlanningProblem_ptr problem, const std::string& constraints, const std::string& poses);

    private:

      /** Class Parameters **/
      tinyxml2::XMLDocument xml_file;
  };
}
#endif
