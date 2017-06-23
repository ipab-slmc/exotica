#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <vector>
#include <exotica/Tools/Exception.h>
#include <exotica/MeshVertex.h>
#include <exotica/StringList.h>
#include <exotica/BoolList.h>
#include <exotica/Vector.h>
#include <exotica/Matrix.h>

namespace Eigen
{

  /// \brief Convenience wrapper for storing references to sub-matrices/vectors
  template<typename Derived>
  class Ref_ptr: public std::shared_ptr<Ref<Derived> >
  {
    public:
      inline Ref_ptr()
          : std::shared_ptr<Ref<Derived> >()
      {

      }

      inline Ref_ptr(const Eigen::Block<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Eigen::Block<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(const Eigen::VectorBlock<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Eigen::VectorBlock<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Derived& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(const Derived& other)
      {
        this->reset(new Ref<Derived>(other));
      }
  };

  /// \brief Reference to sub-vector.
  typedef Ref_ptr<VectorXd> VectorXdRef_ptr;
  /// \brief Reference to sub-Matrix.
  typedef Ref_ptr<MatrixXd> MatrixXdRef_ptr;

  typedef Ref<VectorXd> VectorXdRef;
  typedef const Ref<const VectorXd>& VectorXdRefConst;
  typedef Ref<MatrixXd> MatrixXdRef;
  typedef const Ref<const MatrixXd>& MatrixXdRefConst;

  Eigen::VectorXd VectorTransform(double px=0.0, double py=0.0, double pz=0.0, double qx=0.0, double qy=0.0, double qz=0.0, double qw=1.0);
  Eigen::VectorXd IdentityTransform();

}

namespace exotica
{
    typedef Eigen::Array<KDL::Frame,Eigen::Dynamic,1> ArrayFrame;
    typedef Eigen::Array<KDL::Twist,Eigen::Dynamic,1> ArrayTwist;
    typedef Eigen::Array<KDL::Jacobian,Eigen::Dynamic,1> ArrayJacobian;
    typedef Eigen::Ref<Eigen::Array<KDL::Frame,Eigen::Dynamic,1>> ArrayFrameRef;
    typedef Eigen::Ref<Eigen::Array<KDL::Twist,Eigen::Dynamic,1>> ArrayTwistRef;
    typedef Eigen::Ref<Eigen::Array<KDL::Jacobian,Eigen::Dynamic,1>> ArrayJacobianRef;
    typedef const Eigen::Ref<Eigen::Array<KDL::Frame,Eigen::Dynamic,1>>& ArrayFrameRefConst;
    typedef const Eigen::Ref<Eigen::Array<KDL::Twist,Eigen::Dynamic,1>>& ArrayTwistRefConst;
    typedef const Eigen::Ref<Eigen::Array<KDL::Jacobian,Eigen::Dynamic,1>>& ArrayJacobianRefConst;

    inline bool IsContainerType(std::string type)
    {
        return type=="exotica::Initializer";
    }

    inline bool IsVectorType(std::string type)
    {
        return type.substr(0,11)=="std::vector";
    }

    inline bool IsVectorContainerType(std::string type)
    {
        return type=="std::vector<exotica::Initializer>";
    }

    template <class Key, class Val>
    std::vector<Val> MapToVec( const  std::map<Key,Val> & map)
    {
        std::vector<Val> ret;
        for( auto& val : map)
        {
            ret.push_back( val.second );
        }
        return ret;
    }

    template <class Key, class Val>
    void appendMap(std::map<Key,Val>& orig, const std::map<Key,Val>& extra)
    {
        orig.insert(extra.begin(),extra.end());
    }

    template <class Val>
    void appendVector(std::vector<Val>& orig, const std::vector<Val>& extra)
    {
        orig.insert(orig.end(), extra.begin(),extra.end());
    }

    KDL::Frame getFrame(Eigen::VectorXdRefConst val);

    bool contains(std::string key, const std::vector<std::string>& vec);

    inline std::string trim(const std::string &s)
    {
       auto  wsfront=std::find_if_not(s.begin(),s.end(),[](int c){return std::isspace(c);});
       return std::string(wsfront,std::find_if_not(s.rbegin(),std::string::const_reverse_iterator(wsfront),[](int c){return std::isspace(c);}).base());
    }

    inline Eigen::VectorXd parseVector(const std::string value)
    {
        Eigen::VectorXd ret;
        double temp_entry;
        int i = 0;

        std::istringstream text_parser(value);

        text_parser >> temp_entry;
        while (!(text_parser.fail() || text_parser.bad()))
        {
            ret.conservativeResize(++i);
            ret(i - 1) = temp_entry;
            text_parser >> temp_entry;
        }
        if (i == 0) throw_pretty("Empty vector!");
        return ret;
    }

    inline bool parseBool(const std::string value)
    {
        bool ret;
        std::istringstream text_parser(value);
        text_parser >> ret;
        return ret;
    }

    inline double parseDouble(const std::string value)
    {
        double ret;
        std::istringstream text_parser(value);

        text_parser >> ret;
        if ((text_parser.fail() || text_parser.bad()))
        {
            throw_pretty("Can't parse value!");
        }
        return ret;
    }

    inline int parseInt(const std::string value)
    {
        int ret;
        std::istringstream text_parser(value);

        text_parser >> ret;
        if ((text_parser.fail() || text_parser.bad()))
        {
            throw_pretty("Can't parse value!");
        }
        return ret;
    }

    inline std::vector<std::string> parseList(const std::string value)
    {
        std::stringstream ss(value);
        std::string item;
        std::vector<std::string> ret;
        while (std::getline(ss, item, ','))
        {
            ret.push_back(trim(item));
        }
        if (ret.size() == 0) throw_pretty("Empty vector!");
        return ret;
    }

    inline std::vector<int> parseIntList(const std::string value)
    {
        std::istringstream text_parser(value);
        std::stringstream ss(value);
        std::string item;
        std::vector<int> ret;
        while (std::getline(ss, item, ','))
        {
            int tmp;
            text_parser >> tmp;
            if ((text_parser.fail() || text_parser.bad()))
            {
                throw_pretty("Can't parse value!");
            }
            ret.push_back(tmp);
        }
        if (ret.size() == 0) throw_pretty("Empty vector!");
        return ret;
    }

    inline std::vector<bool> parseBoolList(const std::string value)
    {
        std::istringstream text_parser(value);
        std::stringstream ss(value);
        std::string item;
        std::vector<bool> ret;
        while (std::getline(ss, item, ','))
        {
            bool tmp;
            text_parser >> tmp;
            if ((text_parser.fail() || text_parser.bad()))
            {
                throw_pretty("Can't parse value!");
            }
            ret.push_back(tmp);
        }
        if (ret.size() == 0) throw_pretty("Empty vector!");
        return ret;
    }

    void vectorExoticaToEigen(const exotica::Vector & exotica,
        Eigen::VectorXd & eigen);
    void vectorEigenToExotica(Eigen::VectorXd eigen,
        exotica::Vector & exotica);
    void matrixExoticaToEigen(const exotica::Matrix & exotica,
        Eigen::MatrixXd & eigen);
    void matrixEigenToExotica(const Eigen::MatrixXd & eigen,
        exotica::Matrix & exotica);
}

#endif // CONVERSIONS_H
