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

#include <exotica/EXOTica.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
  //!< Initialise ROS
  testing::InitGoogleTest(&argc, argv);           //!< Testing initialisation
  ros::init(argc, argv, "EXOTica_Test_Suite");    //!< name this node

  std::cout
      << "================================\n Starting ROS_Testing framework \n================================\n"
      << std::endl;

  std::vector<std::string> registered_taskMaps_;
  std::vector<std::string> registered_error_func_;

  if (exotica::TaskMap_fac::Instance().listImplementations(
      registered_taskMaps_))
  {
    std::cout << "OOPS! Could not find registry for "
        << exotica::TaskMap_fac::Instance().type() << std::endl;
    return -1;
  };

  std::cout << "Registered Task Map Types ["
      << exotica::TaskMap_fac::Instance().type()
      << "]: (t indicates testing requested)";
  for (int i = 0; i < registered_taskMaps_.size(); i++)
  {
    std::cout << "\n  " << i + 1 << ") " << registered_taskMaps_[i];
    std::string temp_string;
    if (exotica::XMLTester::Instance().getTest(registered_taskMaps_[i],
        temp_string) == exotica::SUCCESS)
    {
      std::cout << " (t)";
    }
  }
  std::cout << "\n---------------------\n" << std::endl;

  if (exotica::TaskDefinition_fac::Instance().listImplementations(
      registered_error_func_))
  {
    std::cout << "OOPS! Could not find registry for "
        << exotica::TaskDefinition_fac::Instance().type() << std::endl;
    return -1;
  };

  std::cout << "Registered Task Definition Types ["
      << exotica::TaskDefinition_fac::Instance().type()
      << "]: (t indicates testing requested)";
  for (int i = 0; i < registered_error_func_.size(); i++)
  {
    std::cout << "\n  " << i + 1 << ") " << registered_error_func_[i];
    std::string temp_string;
    if (exotica::XMLTester::Instance().getTest(registered_error_func_[i],
        temp_string) == exotica::SUCCESS)
    {
      std::cout << " (t)";
    }
  }
  std::cout << "\n---------------------\n" << std::endl;

  //!< Run Tests
  return RUN_ALL_TESTS();
}
