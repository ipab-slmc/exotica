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

      ///
      /// \brief initialise Initialises the server from XML handle
      /// \param root_handle XML handle
      /// \param server Returned server object
      /// \return Indication of success
      ///
      EReturn initialise(tinyxml2::XMLHandle root_handle, Server_ptr & server);

      ///
      /// \brief initialise Initialises the problem from XML handle
      /// \param root_handle XML handle
      /// \param problem Returned problem object
      /// \param problem_name Requested problem name
      /// \param server Server to use with this object
      /// \return Indication of success
      ///
      EReturn initialise(tinyxml2::XMLHandle root_handle, PlanningProblem_ptr & problem, const std::string & problem_name, Server_ptr & server);

      ///
      /// \brief initialise Initialises the solver from XML handle
      /// \param root_handle XML handle
      /// \param problem Returned solver object
      /// \param problem_name Requested solver name
      /// \param server Server to use with this object
      /// \return Indication of success
      ///
      EReturn initialise(tinyxml2::XMLHandle root_handle, MotionSolver_ptr & solver, const std::string & solver_name, Server_ptr & server);
    
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
       * @param solver    Returned vector of shared pointers to a motion solvers
       * @param problem   Returned vector of shared pointers to a problems
       * @param problem_name Vector of requested problem names
       * @param solver_name Vector of requested solver names
       * @return          Indication of Success
       */
      EReturn initialise(const std::string & file_name, Server_ptr & server, std::vector<MotionSolver_ptr> & solver, std::vector<PlanningProblem_ptr> & problem,std::vector<std::string> & problem_name, std::vector<std::string> & solver_name);

      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Shared pointer to a Motion Solver
       * @param problem   Shared pointer to a Planning Problem
       * @param problem_name Problem name
       * @param solver_name Solver name
       * @return          Indication of Success
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

      ///
      /// \brief initialiseProblemJSON Reinitialises the problem from JSON string
      /// \param problem Problem to be reinitialised
      /// \param constraints JSON string
      /// \return Indication of success
      ///
      EReturn initialiseProblemJSON(PlanningProblem_ptr problem, const std::string& constraints);

      ///
      /// \brief initialiseProblemMoveit Reinitialises the problem from moveit planning scene stored within the Scene object of the problem
      /// \param problem Problem to be reinitialised
      /// \return Indication of success
      ///
      EReturn initialiseProblemMoveit(PlanningProblem_ptr problem);
    private:

      /** Class Parameters **/
      tinyxml2::XMLDocument xml_file;
  };
}
#endif
