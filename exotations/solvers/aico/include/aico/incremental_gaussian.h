#ifndef SP_MEAN_COVARIANCE_HPP
#define SP_MEAN_COVARIANCE_HPP
 
#include <vector>
 
struct sp_mean_covariance {
    int     D;
    int     D2;
    double  W;
    std::vector<double> T;
    std::vector<double> S;
    std::vector<double> dX;
 
    sp_mean_covariance() 
    {
        D = 0;
        D2 = 0;
        T.clear();
        dX.clear();
        S.clear();
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
        S.resize( D_*(D_+1)/2 );
 
        clear();
    }
 
    void clear()
    {
        for (int d=0; d<D; ++d) {
            T[d] = 0.;
            dX[d] = 0.;
        }
        for (int d=0; d<D2; ++d) {
            S[d] = 0.;
        }
        W = 0.;
 
    }
 
 
    void add(double* x)
    {
        if (W == 0.) {
            W = 1.;
            for (int i=0; i<D ; ++i) T[i] = x[i];
            for (int i=0; i<D2; ++i) S[i] = 0.;
            return;
        }
        W += 1.;
        for (int i=0; i<D; ++i)     
            T[i] += x[i];
 
        double* Si = &S[0];
        double  f  = 1. / W / (W-1.);
        for (int i=0; i<D; ++i) dX[i] = W*x[i]-T[i];
 
        for (int r=0; r<D; ++r) {
            for (int c=0; c<=r; ++c) {
                *Si += f * dX[r] * dX[c];
                Si++;
            }
        }
    }
 
 
    inline void add(sp_mean_covariance& M)
    {
        add(M.W, &M.T[0], &M.S[0]);
    }
 
    void add(double W_, double* T_, double* S_)
    {
 
        if (W == 0.) {
            W = W_;
            for (int i=0; i<D ; ++i) T[i] = T_[i];
            for (int i=0; i<D2; ++i) S[i] = S_[i];
            return;
        }
        for (int i=0; i<D ; ++i) 
            dX[i] = T_[i]/W_ - T[i]/W;
 
        double* Si = &S[0];
        double* Si_ = &S_[0];
        double  f  = W * W_ / (W + W_);
        for (int r=0; r<D; ++r) {
            for (int c=0; c<=r; ++c) {
                *Si += *Si_ + f * dX[r] * dX[c];
                Si++;
                Si_++;
            }
        }
 
        for (int i=0; i<D ; ++i)
            T[i] += T_[i];
        W += W_;
    }
 
 
    inline void addw(double w, double* x)
    {
        //--------------------------------
        // add( w, w*x, 0. );
        //--------------------------------
        if (W == 0.) {
            W = w;
            for (int i=0; i<D ; ++i) T[i] = w*x[i];
            for (int i=0; i<D2; ++i) S[i] = 0.;
            return;
        }
 
        for (int i=0; i<D ; ++i) 
            dX[i] = x[i] - T[i]/W;
 
        double* Si = &S[0];
        double  f  = W * w / (W + w);
        for (int r=0; r<D; ++r) {
            for (int c=0; c<=r; ++c) {
                *Si += f * dX[r] * dX[c];
                Si++;
            }
        }
 
        for (int i=0; i<D ; ++i) T[i] += w*x[i];
        W += w;
    }
 
    double mean(int i) const { return T[i]/W; }
    double cov(int i) const { return S[i]/W; } 
    double covp(int i) const { return S[i]/(W-1.); }
};
 
#endif
