#ifndef MISSILEFUNCAERO_H
#define MISSILEFUNCAERO_H

#include <math.h>
#include <vector>
#include <string>
#include "IAerodynamic.hpp"

class MissileFuncAerodynamic: public IAerodynamic{
    public:
        MissileFuncAerodynamic();
        ~MissileFuncAerodynamic();
        double get_cx(double M, std::vector<double> &alpha_beta) const override;
        double get_cy(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const override;
        double get_cz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const override;
        double get_mx(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
        double get_mStab() const;
        double get_my(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
        double get_mz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
    
    private:
        double rad2deg;
        double get_cy_a(double M) const;
        double get_cy_delta(double M, double alpha) const;
        double get_cz_b(double M) const;
        double get_cz_delta(double M, double beta) const;
        double get_mz_wz(double M) const;
        double get_mz_a(double M) const;
        double get_mz_delta(double M, double alpha) const;
        double get_my_wy(double M) const;
        double get_my_b(double M) const;
        double get_my_delta(double M, double beta) const;


};

#endif