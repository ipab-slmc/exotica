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

namespace exotica
{

  void saveMatrix(std::string file_name,
      Eigen::MatrixXdRefConst& mat)
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

  void getMatrix(const tinyxml2::XMLElement & xml_matrix,
      Eigen::MatrixXd & eigen_matrix)
  {
    int dimension = 0;

    if (xml_matrix.QueryIntAttribute("dim", &dimension)
        != tinyxml2::XML_NO_ERROR)
    {
      throw_pretty("Missing dim!");
    }

    if (dimension < 1)
    {
      throw_pretty("Invalid dim!");
    }
    eigen_matrix.resize(dimension, dimension);

    if (!xml_matrix.GetText())
    {
      throw_pretty("Invalid value!");
    }

    std::istringstream text_parser(xml_matrix.GetText());
    double temp_entry;
    for (int i = 0; i < dimension; i++) //!< Note that this does not handle comments!
    {
      for (int j = 0; j < dimension; j++)
      {
        text_parser >> temp_entry;
        if (text_parser.fail() || text_parser.bad()) //!< If a failure other than end of file
        {
          throw_pretty("Can't parse value!");
        }
        else
        {
          eigen_matrix(i, j) = temp_entry;
        }
      }
    }
  }

  void getVector(const tinyxml2::XMLElement & xml_vector,
      Eigen::VectorXd & eigen_vector)
  {
    //!< Temporaries
    double temp_entry;
    int i = 0;

    if (!xml_vector.GetText())
    {
     throw_pretty("Can't get value!");
    }
    std::istringstream text_parser(xml_vector.GetText());

    //!< Initialise looping
    text_parser >> temp_entry;
    while (!(text_parser.fail() || text_parser.bad()))  //!< No commenting!
    {
      eigen_vector.conservativeResize(++i); //!< Allocate storage for this entry (by increasing i)
      eigen_vector(i - 1) = temp_entry;
      text_parser >> temp_entry;
    }
    if (i == 0) throw_pretty("Empty vector!");;
  }

  void getStdVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<double> & std_vector)
  {
    //!< Temporaries
    double temp_entry;

    std_vector.resize(0);
    if (!xml_vector.GetText())
    {
      throw_pretty("Can't get value!");
    }
    std::istringstream text_parser(xml_vector.GetText());

    //!< Initialise looping
    text_parser >> temp_entry;
    while (!(text_parser.fail() || text_parser.bad()))  //!< No commenting!
    {
      std_vector.push_back(temp_entry); //!< Allocate storage for this entry (by increasing i)
      text_parser >> temp_entry;
    }
    if (std_vector.size() == 0)
    {
      throw_pretty("Empy vector!");
    }
  }

  void getBool(const tinyxml2::XMLElement & xml_vector, bool & val)
  {
    std::vector<bool> tmp;
    getBoolVector(xml_vector, tmp);
    val = tmp[0];
  }
  void getBoolVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<bool> & bool_vector)
  {
    //!< Temporaries
    int temp_entry;

    bool_vector.resize(0);
    if (!xml_vector.GetText())
    {
      throw_pretty("Can't get value!");
    }
    std::istringstream text_parser(xml_vector.GetText());

    //!< Initialise looping
    text_parser >> temp_entry;
    while (!(text_parser.fail() || text_parser.bad()))  //!< No commenting!
    {
      if (temp_entry == 1)
        bool_vector.push_back(true);
      else
        bool_vector.push_back(false);
      text_parser >> temp_entry;
    }
    if (bool_vector.size() == 0)
    {
      throw_pretty("Empty vector!");
    }
  }

  void getDouble(const tinyxml2::XMLElement & xml_value, double & value)
  {
    if (!xml_value.GetText())
    {
      throw_pretty("Can't get value!");
    }
    std::istringstream text_parser(xml_value.GetText());

    text_parser >> value;
    if ((text_parser.fail() || text_parser.bad()))
    {
      throw_pretty("Can't parse value!");
    }
  }

  void getInt(const tinyxml2::XMLElement & xml_value, int & value)
  {
    if (!xml_value.GetText())
    {
      throw_pretty("Can't get value!");
    }
    std::istringstream text_parser(xml_value.GetText());

    text_parser >> value;
    if ((text_parser.fail() || text_parser.bad()))
    {
      throw_pretty("Can't parse value!");
    }
  }

  void getList(const tinyxml2::XMLElement & xml_value,
      std::vector<std::string> & value)
  {
    std::stringstream ss(xml_value.GetText());
    std::string item;
    while (std::getline(ss, item, ' '))
    {
      value.push_back(item);
    }
    if (value.size() == 0) throw_pretty("Empty vector!");;
  }

  void resolveParent(std::string & file_path)
  {
    size_t parent_dir = file_path.find_last_of('/'); //!< Find the last forward slash

    if (parent_dir == std::string::npos)
    {
      throw_pretty("Invalid parent!");
    }
    else
    {
      if (parent_dir == 0)
      {
        file_path = "/";  //!< Assign to just the root directory
        return;
      }
      else
      {
        file_path = file_path.substr(0, parent_dir); //!< Assign to the substring
      }
    }
  }

  void deepCopy(tinyxml2::XMLHandle & parent, tinyxml2::XMLHandle & child)
  {
    //!< First copy the child to the parent
    if (!parent.ToNode())
    {
      throw_pretty("Invalid parent!");
    }
    if (!child.ToNode())
    {
      throw_pretty("Invalid child!");
    }
    tinyxml2::XMLNode * node_ptr = parent.ToNode()->InsertEndChild(
        child.ToNode()->ShallowClone(parent.ToNode()->GetDocument()));
    if (node_ptr == 0)
    {
      throw_pretty("Invalid node!");
    }
    // Here we are first performing a shallow clone to assign the child node as a node of the parent's document and then moving that to be actually a child of the parent

    //!< Now iterate recursively on its children
    tinyxml2::XMLHandle grandchild(child.FirstChild());
    tinyxml2::XMLHandle new_child(node_ptr);
    while (grandchild.ToNode())
    {
      deepCopy(new_child, grandchild);
      grandchild = grandchild.NextSibling();
    }
  }

  void parseIncludes(tinyxml2::XMLHandle & handle, std::string directory)
  {
    //!< Temporaries
    std::string temp_path = directory;
    tinyxml2::XMLDocument doc;

    //!< First see if there are any includes at this level and resolve them:
    tinyxml2::XMLHandle include_handle(handle.FirstChildElement("include"));
    while (include_handle.ToElement())  //!< While a valid pointer
    {
      //!< First get the file attribute
      const char * file_path = include_handle.ToElement()->Attribute("file");
      if (file_path == nullptr)
      {
        throw_pretty("Empty file name!");
      }
      temp_path = directory.append(file_path); //!< Append to the current working directory

      //!< load the document
      if (doc.LoadFile(temp_path.c_str()) != tinyxml2::XML_NO_ERROR)
      {
        throw_pretty("Failed loading XML!");
      }
      if (!doc.RootElement())
      {
        throw_pretty("Invalid document!");
      }  //!< If no root element!

      //!< Change the scope (file-path) for this node which just got included
      resolveParent(temp_path);
      temp_path.append("/");

      doc.RootElement()->SetAttribute("current_path_scope", temp_path.c_str());

      //!< Now actually resolve the xml-structure
      tinyxml2::XMLHandle sub_tree(doc.RootElement());
      deepCopy(handle, sub_tree); //!< Note that the ordering is no longer maintained at this level (i.e. the included tag will go at the end)

      handle.ToNode()->DeleteChild(include_handle.ToNode()); //!< Delete the node handle;

      //!< Prepare for next <include> which now will be the first child element of the name
      include_handle = handle.FirstChildElement("include");
      doc.Clear();
    }

    //!< Now iterate recursively over the children
    tinyxml2::XMLHandle child_handle(handle.FirstChild());
    while (child_handle.ToElement()) //!< While a valid element (cannot be text etc...)
    {
      //!< Check if scope available, and if so use it
      const char * scope_path = child_handle.ToElement()->Attribute(
          "current_path_scope");
      if (scope_path != nullptr)
      {
        parseIncludes(child_handle, scope_path);
      }
      else
      {
        parseIncludes(child_handle, directory);
      }


      //!< Prepare for next iteration
      child_handle = child_handle.NextSibling();
    }
  }

  void loadOBJ(std::string & data, Eigen::VectorXi& tri,
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

  void getJSON(const rapidjson::Value& a, Eigen::VectorXd& ret)
  {
    if (a.IsArray())
    {
      ret.resize(a.Size());
      for (int i = 0; i < a.Size(); i++)
      {
        if (a[i].IsNumber())
        {
          ret(i) = a[i].GetDouble();
        }
        else
        {
          throw_pretty("NaN!");
        }
      }
    }
    else
    {
      throw_pretty("Not an array!");
    }
  }

  void getJSON(const rapidjson::Value& a, std::vector<std::string>& ret)
  {
    if (a.IsArray())
    {
      ret.resize(a.Size());
      for (int i = 0; i < a.Size(); i++)
      {
        ret[i] = a[i].GetString();
      }
    }
    else
    {
      throw_pretty("Not an array!");
    }
  }

  void getJSON(const rapidjson::Value& a, double& ret)
  {
    if (a.IsNumber())
    {
      ret = a.GetDouble();
    }
    else
    {
      throw_pretty("NaN!");
    }
  }

  void getJSON(const rapidjson::Value& a, int& ret)
  {
    if (a.IsInt())
    {
      ret = a.GetInt();
    }
    else
    {
      throw_pretty("Not an int!");
    }
  }

  void getJSON(const rapidjson::Value& a, bool& ret)
  {
    if (a.IsBool())
    {
      ret = a.GetBool();
    }
    else
    {
      throw_pretty("Not a bool!");
    }
  }

  void getJSON(const rapidjson::Value& a, std::string& ret)
  {
    ret = a.GetString();
  }

  void getJSON(const rapidjson::Value& a, KDL::Frame& ret)
  {
    if (a.IsObject())
    {
      Eigen::VectorXd pos(3), rot(4);
      getJSON(a["position"], pos);
      getJSON(a["quaternion"], rot);
      rot = rot / rot.norm();
      ret = KDL::Frame(
      KDL::Rotation::Quaternion(rot(1), rot(2), rot(3), rot(0)),
      KDL::Vector(pos(0), pos(1), pos(2)));
    }
    else
    {
      throw_pretty("Not an object!");
    }
  }

  void getJSONFrameNdArray(const rapidjson::Value& a, KDL::Frame& ret)
  {
    if (a.IsObject())
    {
        Eigen::VectorXd pos(3), rot(4);
        getJSON(a["position"]["__ndarray__"], pos);
        getJSON(a["quaternion"]["__ndarray__"], rot);
        rot = rot / rot.norm();
        ret = KDL::Frame(
        KDL::Rotation::Quaternion(rot(1), rot(2), rot(3), rot(0)),
        KDL::Vector(pos(0), pos(1), pos(2)));
    }
    else
    {
      throw_pretty("Not an object!");
    }
  }

  void vectorExoticaToEigen(const exotica::Vector & exotica,
      Eigen::VectorXd & eigen)
  {
    eigen.resize(exotica.data.size());
    for (int i = 0; i < exotica.data.size(); i++)
      eigen(i) = exotica.data[i];
  }

  void vectorEigenToExotica(Eigen::VectorXd eigen, exotica::Vector & exotica)
  {
    exotica.data.resize(eigen.rows());
    for (int i = 0; i < eigen.rows(); i++)
      exotica.data[i] = eigen(i);
  }

  void matrixExoticaToEigen(const exotica::Matrix & exotica,
      Eigen::MatrixXd & eigen)
  {
    if (exotica.col == 0 || exotica.row == 0 || exotica.data.size() == 0)
    {
      throw_pretty("Matrix conversion failed, no data in the matrix.");
    }
    if (exotica.col * exotica.row != exotica.data.size())
    {
      throw_pretty(
          "Matrix conversion failed, size mismatch."<<exotica.col<<" * "<<exotica.row<<" != "<<exotica.data.size());
    }
    eigen.resize(exotica.row, exotica.col);
    int cnt = 0;
    for (int r = 0; r < exotica.row; r++)
      for (int c = 0; c < exotica.col; c++)
      {
        eigen(r, c) = exotica.data[cnt];
        cnt++;
      }
  }

  void matrixEigenToExotica(const Eigen::MatrixXd & eigen,
      exotica::Matrix & exotica)
  {
    exotica.row = eigen.rows();
    exotica.col = eigen.cols();
    exotica.data.resize(exotica.col * exotica.row);
    int cnt = 0;
    for (int r = 0; r < exotica.row; r++)
      for (int c = 0; c < exotica.col; c++)
      {
        exotica.data[cnt] = eigen(r, c);
        cnt++;
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

}
