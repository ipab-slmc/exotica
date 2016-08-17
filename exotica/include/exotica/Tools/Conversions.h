#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace Eigen
{

  /// \brief Convenience wrapper for storing references to sub-matrices/vectors
  template<typename Derived>
  class Ref_ptr: public boost::shared_ptr<Ref<Derived> >
  {
    public:
      inline Ref_ptr()
          : boost::shared_ptr<Ref<Derived> >()
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
    KDL::Frame getFrame(Eigen::VectorXdRefConst val);

    bool contains(std::string key, const std::vector<std::string>& vec);
}

#endif // CONVERSIONS_H
