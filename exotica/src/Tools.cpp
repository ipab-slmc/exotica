#include "exotica/Tools.h"
#include <fstream>

namespace exotica
{

  EReturn saveMatrix(std::string file_name,
      const Eigen::Ref<const Eigen::MatrixXd> mat)
  {
    std::ofstream myfile;
    myfile.open(file_name);
    if (myfile.good())
    {
      myfile << mat;
      myfile.close();
      return SUCCESS;
    }
    else
    {
      myfile.close();
      return FAILURE;
    }
  }

  bool ok(const EReturn & value)
  {
    return (value == SUCCESS || value == WARNING || value == CANCELLED);
  }

  EReturn getMatrix(const tinyxml2::XMLElement & xml_matrix,
      Eigen::MatrixXd & eigen_matrix)
  {
    int dimension = 0;

    if (xml_matrix.QueryIntAttribute("dim", &dimension)
        != tinyxml2::XML_NO_ERROR)
    {
      INDICATE_FAILURE
      ;
      eigen_matrix = Eigen::MatrixXd(); //!< Null matrix again
      return PAR_ERR;
    }

    if (dimension < 1)
    {
      INDICATE_FAILURE
      ;
      eigen_matrix = Eigen::MatrixXd(); //!< Null matrix again
      return PAR_ERR;
    }
    eigen_matrix.resize(dimension, dimension);

    if (!xml_matrix.GetText())
    {
      INDICATE_FAILURE
      ;
      eigen_matrix = Eigen::MatrixXd(); //!< Null matrix again
      return PAR_ERR;
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
          INDICATE_FAILURE
          ;
          eigen_matrix.resize(0, 0);
          return PAR_ERR;
        }
        else
        {
          eigen_matrix(i, j) = temp_entry;
        }
      }
    }

    return SUCCESS;
  }

  EReturn getVector(const tinyxml2::XMLElement & xml_vector,
      Eigen::VectorXd & eigen_vector)
  {
    //!< Temporaries
    double temp_entry;
    int i = 0;

    if (!xml_vector.GetText())
    {
      INDICATE_FAILURE
      ;
      eigen_vector = Eigen::VectorXd(); //!< Null matrix again
      return PAR_ERR;
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
    return (i > 0) ? SUCCESS : PAR_ERR;
  }

  EReturn getStdVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<double> & std_vector)
  {
    //!< Temporaries
    double temp_entry;

    std_vector.resize(0);
    if (!xml_vector.GetText())
    {
      INDICATE_FAILURE
      return PAR_ERR;
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
      INDICATE_FAILURE
      return PAR_ERR;
    }
    return SUCCESS;
  }

  EReturn getBool(const tinyxml2::XMLElement & xml_vector, bool & val)
  {
    std::vector<bool> tmp;
    if (!ok(getBoolVector(xml_vector, tmp)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    val = tmp[0];
    return SUCCESS;
  }
  EReturn getBoolVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<bool> & bool_vector)
  {
    //!< Temporaries
    int temp_entry;

    bool_vector.resize(0);
    if (!xml_vector.GetText())
    {
      INDICATE_FAILURE
      return PAR_ERR;
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
      INDICATE_FAILURE
      return PAR_ERR;
    }
    return SUCCESS;
  }

  EReturn getDouble(const tinyxml2::XMLElement & xml_value, double & value)
  {
    if (!xml_value.GetText())
    {
      INDICATE_FAILURE
      ;
      return PAR_ERR;
    }
    std::istringstream text_parser(xml_value.GetText());

    text_parser >> value;
    if (!(text_parser.fail() || text_parser.bad()))
    {
      return SUCCESS;
    }
    else
    {
      return PAR_ERR;
    }
  }

  EReturn getInt(const tinyxml2::XMLElement & xml_value, int & value)
  {
    if (!xml_value.GetText())
    {
      INDICATE_FAILURE
      return PAR_ERR;
    }
    std::istringstream text_parser(xml_value.GetText());

    text_parser >> value;
    if (!(text_parser.fail() || text_parser.bad()))
    {
      return SUCCESS;
    }
    else
    {
      return PAR_ERR;
    }
  }

  EReturn getList(const tinyxml2::XMLElement & xml_value,
      std::vector<std::string> & value)
  {
    std::stringstream ss(xml_value.GetText());
    std::string item;
    while (std::getline(ss, item, ' '))
    {
      value.push_back(item);
    }
    if (value.size() == 0) return FAILURE;
    return SUCCESS;
  }
  EReturn resolveParent(std::string & file_path)
  {
    size_t parent_dir = file_path.find_last_of('/'); //!< Find the last forward slash

    if (parent_dir == std::string::npos)
    {
      return PAR_ERR;
    }
    else
    {
      if (parent_dir == 0)
      {
        file_path = "/";  //!< Assign to just the root directory
        return WARNING;
      }
      else
      {
        file_path = file_path.substr(0, parent_dir); //!< Assign to the substring
        return SUCCESS;
      }
    }
  }

  EReturn deepCopy(tinyxml2::XMLHandle & parent, tinyxml2::XMLHandle & child)
  {
    //!< First copy the child to the parent
    if (!parent.ToNode())
    {
      return WARNING;
    }
    if (!child.ToNode())
    {
      return WARNING;
    }
    tinyxml2::XMLNode * node_ptr = parent.ToNode()->InsertEndChild(
        child.ToNode()->ShallowClone(parent.ToNode()->GetDocument()));
    if (node_ptr == 0)
    {
      return FAILURE;
    }
    // Here we are first performing a shallow clone to assign the child node as a node of the parent's document and then moving that to be actually a child of the parent

    //!< Now iterate recursively on its children
    tinyxml2::XMLHandle grandchild(child.FirstChild());
    tinyxml2::XMLHandle new_child(node_ptr);
    EReturn ret_val = SUCCESS;
    while (grandchild.ToNode() and ok(ret_val))
    {
      EReturn aux_ret = deepCopy(new_child, grandchild);
      if (aux_ret)
      {
        ret_val = aux_ret;
      }

      grandchild = grandchild.NextSibling();
    }

    return ret_val;
  }

  EReturn parseIncludes(tinyxml2::XMLHandle & handle, std::string directory)
  {
    //!< Temporaries
    EReturn ret_val = SUCCESS;
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
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
      temp_path = directory.append(file_path); //!< Append to the current working directory

      //!< load the document
      if (doc.LoadFile(temp_path.c_str()) != tinyxml2::XML_NO_ERROR)
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
      if (!doc.RootElement())
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }  //!< If no root element!

      //!< Change the scope (file-path) for this node which just got included
      EReturn aux_rtn = resolveParent(temp_path);
      temp_path.append("/");
      if (aux_rtn)
      {
        ret_val = aux_rtn;
      }
      if (!ok(ret_val))
      {
        INDICATE_FAILURE
        ;
        return ret_val;
      }
      doc.RootElement()->SetAttribute("current_path_scope", temp_path.c_str());

      //!< Now actually resolve the xml-structure
      tinyxml2::XMLHandle sub_tree(doc.RootElement());
      aux_rtn = deepCopy(handle, sub_tree); //!< Note that the ordering is no longer maintained at this level (i.e. the included tag will go at the end)
      if (aux_rtn)
      {
        ret_val = aux_rtn;
      }
      if (!ok(ret_val))
      {
        INDICATE_FAILURE
        ;
        return ret_val;
      }
      handle.ToNode()->DeleteChild(include_handle.ToNode()); //!< Delete the node handle;

      //!< Prepare for next <include> which now will be the first child element of the name
      include_handle = handle.FirstChildElement("include");
      doc.Clear();
    }

    //!< Now iterate recursively over the children
    tinyxml2::XMLHandle child_handle(handle.FirstChild());
    while (child_handle.ToElement()) //!< While a valid element (cannot be text etc...)
    {
      //!< Temporary
      EReturn aux_rtn;

      //!< Check if scope available, and if so use it
      const char * scope_path = child_handle.ToElement()->Attribute(
          "current_path_scope");
      if (scope_path != nullptr)
      {
        aux_rtn = parseIncludes(child_handle, scope_path);
      }
      else
      {
        aux_rtn = parseIncludes(child_handle, directory);
      }

      //!< Error Checking
      if (aux_rtn)
      {
        ret_val = aux_rtn;
      }
      if (!ok(ret_val))
      {
        INDICATE_FAILURE
        ;
        return ret_val;
      }

      //!< Prepare for next iteration
      child_handle = child_handle.NextSibling();
    }

    return ret_val;
  }

  EReturn loadOBJ(std::string & data, Eigen::VectorXi& tri,
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
          INDICATE_FAILURE
          ;
          return PAR_ERR;
        }
        tri.conservativeResize((tn + 1) * 3);
        tri(tn * 3) = vv[0] - 1;
        tri(tn * 3 + 1) = vv[3] - 1;
        tri(tn * 3 + 2) = vv[6] - 1;
        tn++;
      }
    }

    return SUCCESS;
  }

  EReturn getJSON(const rapidjson::Value& a, Eigen::VectorXd& ret)
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
          INDICATE_FAILURE
          ;
          return FAILURE;
        }
      }
      return SUCCESS;
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn getJSON(const rapidjson::Value& a, std::vector<std::string>& ret)
  {
    if (a.IsArray())
    {
      ret.resize(a.Size());
      for (int i = 0; i < a.Size(); i++)
      {
        ret[i] = a[i].GetString();
      }
      return SUCCESS;
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn getJSON(const rapidjson::Value& a, double& ret)
  {
    if (a.IsNumber())
    {
      ret = a.GetDouble();
      return SUCCESS;
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn getJSON(const rapidjson::Value& a, int& ret)
  {
    if (a.IsInt())
    {
      ret = a.GetInt();
      return SUCCESS;
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn getJSON(const rapidjson::Value& a, std::string& ret)
  {
    ret = a.GetString();
    return SUCCESS;
  }

  EReturn getJSON(const rapidjson::Value& a, KDL::Frame& ret)
  {
    if (a.IsObject())
    {
      Eigen::VectorXd pos(3), rot(4);
      if (ok(getJSON(a["position"]["__ndarray__"], pos))
          && ok(getJSON(a["quaternion"]["__ndarray__"], rot)))
      {
        rot = rot / rot.norm();
        ret = KDL::Frame(
            KDL::Rotation::Quaternion(rot(1), rot(2), rot(3), rot(0)),
            KDL::Vector(pos(0), pos(1), pos(2)));
        return SUCCESS;
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
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn vectorExoticaToEigen(const exotica::Vector & exotica,
      Eigen::VectorXd & eigen)
  {
    eigen.resize(exotica.data.size());
    for (int i = 0; i < exotica.data.size(); i++)
      eigen(i) = exotica.data[i];
    return SUCCESS;
  }

  EReturn vectorEigenToExotica(Eigen::VectorXd eigen, exotica::Vector & exotica)
  {
    exotica.data.resize(eigen.rows());
    for (int i = 0; i < eigen.rows(); i++)
      exotica.data[i] = eigen(i);
    return SUCCESS;
  }

  EReturn matrixExoticaToEigen(const exotica::Matrix & exotica,
      Eigen::MatrixXd & eigen)
  {
    if (exotica.col == 0 || exotica.row == 0 || exotica.data.size() == 0)
    {
      ERROR("Matrix conversion failed, no data in the matrix.");
      return FAILURE;
    }
    if (exotica.col * exotica.row != exotica.data.size())
    {
      ERROR(
          "Matrix conversion failed, size mismatch."<<exotica.col<<" * "<<exotica.row<<" != "<<exotica.data.size());
      return FAILURE;
    }
    eigen.resize(exotica.row, exotica.col);
    int cnt = 0;
    for (int r = 0; r < exotica.row; r++)
      for (int c = 0; c < exotica.col; c++)
      {
        eigen(r, c) = exotica.data[cnt];
        cnt++;
      }
    return SUCCESS;
  }

  EReturn matrixEigenToExotica(const Eigen::MatrixXd & eigen,
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
    return SUCCESS;
  }

}
