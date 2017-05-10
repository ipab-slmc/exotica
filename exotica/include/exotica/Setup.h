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
#include <exotica/Property.h>

#include <pluginlib/class_loader.h>

namespace exotica
{
  class Setup: public Object, Uncopyable
  {
    public:

      ~Setup() noexcept
      {
      }

      static boost::shared_ptr<Setup> Instance()
      {
        if (!singleton_initialiser_) singleton_initialiser_.reset(new Setup);
        return singleton_initialiser_;
      }

      static void Destroy()
      {
          if (singleton_initialiser_) singleton_initialiser_.reset();
      }

      static void printSupportedClasses();
      static boost::shared_ptr<exotica::MotionSolver> createSolver(const std::string & type) {return Instance()->solvers_.createInstance("exotica/"+type);}
      static boost::shared_ptr<exotica::TaskMap> createMap(const std::string & type) {return Instance()->maps_.createInstance("exotica/"+type);}
      static boost::shared_ptr<exotica::TaskDefinition> createDefinition(const std::string & type) {return Instance()->tasks_.createInstance("exotica/"+type);}
      static boost::shared_ptr<exotica::PlanningProblem> createProblem(const std::string & type) {return Instance()->problems_.createInstance("exotica/"+type);}
      static std::vector<std::string> getSolvers();
      static std::vector<std::string> getProblems();
      static std::vector<std::string> getMaps();
      static std::vector<std::string> getTasks();

      static boost::shared_ptr<exotica::MotionSolver> createSolver(const Initializer& init)
      {
          boost::shared_ptr<exotica::MotionSolver> ret = Instance()->solvers_.createInstance(init.getName());
          ret->InstantiateInternal(init);
          return ret;
      }
      static boost::shared_ptr<exotica::TaskMap> createMap(const Initializer& init)
      {
          boost::shared_ptr<exotica::TaskMap> ret = Instance()->maps_.createInstance(init.getName());
          ret->InstantiateInternal(init);
          return ret;
      }
      static boost::shared_ptr<exotica::TaskDefinition> createDefinition(const Initializer& init)
      {
          boost::shared_ptr<exotica::TaskDefinition> ret = Instance()->tasks_.createInstance(init.getName());
          ret->InstantiateInternal(init);
          return ret;
      }
      static boost::shared_ptr<exotica::PlanningProblem> createProblem(const Initializer& init)
      {
          boost::shared_ptr<exotica::PlanningProblem> ret = Instance()->problems_.createInstance(init.getName());
          ret->InstantiateInternal(init);
          return ret;
      }

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
      /// \brief initialiseProblemMoveit Reinitialises the problem from moveit planning scene stored within the Scene object of the problem
      /// \param problem Problem to be reinitialised
      /// \return Indication of success
      ///
      void initialiseProblemMoveit(PlanningProblem_ptr problem);
    private:

      /**
       * \brief Default Constructor
       *
       *        Currently, is an empty constructor definition.
       */
      Setup();
      static boost::shared_ptr<Setup> singleton_initialiser_;
      ///	\brief	Make sure the singleton does not get copied
      Setup(Setup const&) = delete;
      void operator=(Setup const&) = delete;

      pluginlib::ClassLoader<exotica::MotionSolver> solvers_;
      pluginlib::ClassLoader<exotica::TaskMap> maps_;
      PlanningProblem_fac problems_;
      TaskDefinition_fac tasks_;
  };

  typedef boost::shared_ptr<Setup> Setup_ptr;
}

#endif
