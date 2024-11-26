#ifndef IAERO_H
#define IAERO_H

#include <math.h>
#include <vector>

class IAerodynamic{
    public:
        virtual ~IAerodynamic();
        virtual double get_cx(double M, std::vector<double> & alpha_beta) const = 0;
        virtual double get_cy(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const = 0;
        virtual double get_cz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const = 0;
        virtual double get_mx(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const = 0;
        virtual double get_my(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const = 0;
        virtual double get_mz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const = 0;
};


#endif