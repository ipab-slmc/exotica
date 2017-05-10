#include <exotica/Tools/Conversions.h>
#include <algorithm>

namespace Eigen
{
    Eigen::VectorXd VectorTransform(double px, double py, double pz, double qx, double qy, double qz, double qw)
    {
        Eigen::VectorXd ret((Eigen::VectorXd(7) << px,py,pz,qx,qy,qz,qw).finished());
        return ret;
    }

    Eigen::VectorXd IdentityTransform()
    {
        return VectorTransform();
    }
}

namespace exotica
{
    KDL::Frame getFrame(Eigen::VectorXdRefConst val)
    {
        if(val.rows()!=7) throw_pretty("Eigen vector has incorrect length! ("+std::to_string(val.rows())+")");
        return KDL::Frame(KDL::Rotation::Quaternion(val(3),val(4),val(5),val(6)),KDL::Vector(val(0),val(1),val(2)));
    }

    bool contains(std::string key, const std::vector<std::string>& vec)
    {

        if(std::find(vec.begin(),vec.end(),key)==vec.end()) return false;
        return true;
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
}
