/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

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
  class Initialiser: public Object
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
      void initialise(tinyxml2::XMLHandle root_handle, Server_ptr & server);

      ///
      /// \brief initialise Initialises the problem from XML handle
      /// \param root_handle XML handle
      /// \param problem Returned problem object
      /// \param problem_name Requested problem name
      /// \param server Server to use with this object
      /// \return Indication of success
      ///
      void initialise(tinyxml2::XMLHandle root_handle,
          PlanningProblem_ptr & problem, const std::string & problem_name,
          Server_ptr & server);

      ///
      /// \brief initialise Initialises the solver from XML handle
      /// \param root_handle XML handle
      /// \param problem Returned solver object
      /// \param problem_name Requested solver name
      /// \param server Server to use with this object
      /// \return Indication of success
      ///
      void initialise(tinyxml2::XMLHandle root_handle,
          MotionSolver_ptr & solver, const std::string & solver_name,
          Server_ptr & server);

      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Shared pointer to a Motion Solver
       * @param problem   Shared pointer to a Planning Problem
       * @return          Indication of Success: TODO
       */
      void initialise(const std::string & file_name, Server_ptr & server,
          MotionSolver_ptr & solver, PlanningProblem_ptr & problem);

      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Returned vector of shared pointers to a motion solvers
       * @param problem   Returned vector of shared pointers to a problems
       * @param problem_name Vector of requested problem names
       * @param solver_name Vector of requested solver names
       * @return          Indication of Success
       */
      void initialise(const std::string & file_name, Server_ptr & server,
          std::vector<MotionSolver_ptr> & solver,
          std::vector<PlanningProblem_ptr> & problem,
          std::vector<std::string> & problem_name,
          std::vector<std::string> & solver_name);

      /**
       * \brief Initialiser function
       * @param file_name XML_file for initialising the problem with.
       * @param solver    Shared pointer to a Motion Solver
       * @param problem   Shared pointer to a Planning Problem
       * @param problem_name Problem name
       * @param solver_name Solver name
       * @return          Indication of Success
       */
      void initialise(const std::string & file_name, Server_ptr & server,
          MotionSolver_ptr & solver, PlanningProblem_ptr & problem,
          const std::string & problem_name, const std::string & solver_name);

      /**
       * \brief Creates a list of supported problems and solvers specified in a XML file
       * @param file_name File name
       * @param problems Return vector of prolem names
       * @param problems Return vector of solver names
       * @return Indication of success
       */
      void listSolversAndProblems(const std::string & file_name,
          std::vector<std::string>& problems,
          std::vector<std::string>& solvers);

      ///
      /// \brief initialiseProblemJSON Reinitialises the problem from JSON string
      /// \param problem Problem to be reinitialised
      /// \param constraints JSON string
      /// \return Indication of success
      ///
      void initialiseProblemJSON(PlanningProblem_ptr problem,
          const std::string& constraints);

      ///
      /// \brief initialiseProblemMoveit Reinitialises the problem from moveit planning scene stored within the Scene object of the problem
      /// \param problem Problem to be reinitialised
      /// \return Indication of success
      ///
      void initialiseProblemMoveit(PlanningProblem_ptr problem);
    private:

      /** Class Parameters **/
      tinyxml2::XMLDocument xml_file;
  };
}
#endif
