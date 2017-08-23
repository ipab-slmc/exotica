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

#include "exotica/Tools.h"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <random>
#include <iostream>
#include <typeinfo> //!< The RTTI Functionality of C++
#include <cxxabi.h> //!< The demangler for gcc... this makes this system dependent!
#include <regex>
#include <ros/package.h>

namespace exotica
{
    std_msgs::ColorRGBA randomColor()
    {
        std_msgs::ColorRGBA ret;
        ret.a = 1.0;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        ret.r = dis(gen);
        ret.g = dis(gen);
        ret.b = dis(gen);
        return ret;
    }

  void saveMatrix(std::string file_name,
      const Eigen::Ref<const Eigen::MatrixXd> mat)
  {
    std::ofstream myfile;
    myfile.open(file_name);
    if (myfile.good())
    {
      myfile << mat;
      myfile.close();
    }
    else
    {
      myfile.close();
      throw_pretty("Can't open file!");
    }
  }

  void loadOBJ(const std::string & data, Eigen::VectorXi& tri,
      Eigen::VectorXd& vert)
  {
    std::stringstream ss(data);
    std::string line;
    tri.resize(0, 1);
    vert.resize(0, 1);
    int vn = 0, tn = 0;
    double v[3];
    int vv[9];
    while (std::getline(ss, line))
    {
      if (line.compare(0, 2, "v ") == 0)
      {

        vert.conservativeResize((vn + 1) * 3);
        std::stringstream sss(line.substr(2));
        sss >> v[0] >> v[1] >> v[2];
        vert(vn * 3) = v[0];
        vert(vn * 3 + 1) = v[1];
        vert(vn * 3 + 2) = v[2];
        vn++;
      }
      else if (line.compare(0, 2, "f ") == 0)
      {
        std::stringstream sss(line.substr(2));
        int i;
        for (i = 0; i < 9 && sss >> vv[i]; i++)
        {
          while (sss.peek() == '/' || sss.peek() == ' ')
            sss.ignore();
        }
        if (i < 8)
        {
          throw_pretty("Invalid format!");
        }
        tri.conservativeResize((tn + 1) * 3);
        tri(tn * 3) = vv[0] - 1;
        tri(tn * 3 + 1) = vv[3] - 1;
        tri(tn * 3 + 2) = vv[6] - 1;
        tn++;
      }
    }
  }

  void getText(std::string& txt, KDL::Frame& ret)
  {
      std::vector<std::string> strs;
      boost::split(strs, txt, boost::is_any_of(" "));
      if(strs.size()!=7)
      {
          throw_pretty("Not a frame! " <<txt);
      }

      std::vector<double> doubleVector(strs.size());
      std::transform(strs.begin(), strs.end(), doubleVector.begin(), [](const std::string& val)
      {
          return std::stod(val);
      });

      ret.p = KDL::Vector(doubleVector[0],doubleVector[1],doubleVector[2]);
      ret.M = KDL::Rotation::Quaternion(doubleVector[4],doubleVector[5],doubleVector[6],doubleVector[3]);
  }

  std::string getTypeName(const std::type_info& type)
  {
      int status;
      std::string name;
      char * temp; //!< We need to store this to free the memory!

      temp = abi::__cxa_demangle(type.name(), 0, 0, &status);
      name = std::string(temp);
      free(temp);
      return name;
  }

  std::string parsePath(const std::string& path)
  {
      std::string ret = path;
      std::smatch matches;
      std::regex_search(ret, matches, std::regex("\\{([^\\}]+)\\}"));
      for(auto& match : matches)
      {
          std::string package = match.str();
          if(package[0]=='{') continue;
          std::string package_path = ros::package::getPath(package);
          if(package_path=="") throw_pretty("Unknown package '"<<package<<"'");
          try
          {
              ret = std::regex_replace(ret, std::regex("\\{"+package+"\\}"), package_path, std::regex_constants::match_any);
          }
          catch(const std::regex_error& e)
          {
              throw_pretty("Package name resolution failed (regex error "<<e.code()<<")");
          }
      }
      return ret;
  }

  std::string loadFile(const std::string& path)
  {
      std::string file_name = parsePath(path);
      std::ifstream fstream(file_name);
      if(!fstream) throw_pretty("File does not exist '" << file_name << "'");
      try
      {
          return std::string((std::istreambuf_iterator<char>(fstream)), std::istreambuf_iterator<char>());
      }
      catch (const std::ifstream::failure& e)
      {
          throw_pretty("Can't read file '"<<file_name<<"'");
      }
  }

  bool pathExists(const std::string& path)
  {
    std::ifstream file(parsePath(path));
    return (bool)file;
  }
}
