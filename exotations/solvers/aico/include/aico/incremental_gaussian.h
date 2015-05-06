#ifndef SP_MEAN_COVARIANCE_HPP
#define SP_MEAN_COVARIANCE_HPP
 
#include <vector>
#include <Eigen/Dense>
 
struct sp_mean_covariance {
    int     D;
    int     D2;
    double  W;
    Eigen::VectorXd T;
    Eigen::VectorXd dX;
    Eigen::MatrixXd S;
 
    sp_mean_covariance() 
    {
        D = 0;
        D2 = 0;
        T.resize(0);
        dX.resize(0);
        S.resize(0,0);
    }
 
    sp_mean_covariance(int D_) 
    {
        resize( D_ );
    }
 
    void resize(int D_) 
    {
        D = D_;
        D2 = (D_*(D_+1)/2);
 
        T.resize(D_);
        dX.resize(D_);
        S.resize( D_,D_);
 
        clear();
    }
 
    void clear()
    {
        T.setZero();
        dX.setZero();
        S.setZero();
        W = 0.; 
    }
 
 
    void add(const Eigen::Ref<const Eigen::VectorXd> & x)
    {
        if (W == 0.)
        {
            W = 1.;
            T=x;
            S.setZero();
            return;
        }
        W += 1.;
        T+=x;
        double  f  = 1. / W / (W-1.);
        dX=W*x-T;
        for (int r=0; r<D; r++)
        {
            for (int c=0; c<D; c++)
            {
                S(r,c)+=f*dX(r)*dX(c);
            }
        }
    }
 
 
    inline void add(sp_mean_covariance& M)
    {
        add(M.W, M.T, M.S);
    }
 
    void add(double& W_, const Eigen::Ref<const Eigen::VectorXd> & T_, const Eigen::Ref<const Eigen::VectorXd> & S_)
    {
        if (W == 0.)
        {
            W = W_;
            T=T_;
            S=S_;
            return;
        }
        dX=T_/W_-T/W;

        double  f  = W * W_ / (W + W_);
        for (int r=0; r<D; r++)
        {
            for (int c=0; c<D; c++)
            {
                S(r,c)+=S_(r,c)+f*dX(r)*dX(c);
            }
        }
        T+=T_;
        W += W_;
    }
 
 
    inline void addw(double w, const Eigen::Ref<const Eigen::VectorXd> & x)
    {
        //--------------------------------
        // add( w, w*x, 0. );
        //--------------------------------
        if (W == 0.) {
            W = w;
            T = w*x;
            S.setZero();
            return;
        }
 
        dX = x - T/W;
 
        double  f  = W * w / (W + w);
        for (int r=0; r<D; r++)
        {
            for (int c=0; c<D; c++)
            {
                S(r,c)+= f * dX(r) * dX(c);
            }
        }
 
        T += w*x;
        W += w;
    }
 
    void mean(Eigen::VectorXd& mu) { mu=T/W; }
    void cov(Eigen::MatrixXd& sig) { sig=S/W; }
    void covp(Eigen::MatrixXd& sig) { sig=S/(W-1.); }
};
 
#endif
