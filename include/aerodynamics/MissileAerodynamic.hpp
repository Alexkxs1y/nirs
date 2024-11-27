#ifndef MISSILEAERO_H
#define MISSILEAERO_H

#include <math.h>
#include <vector>
#include <string>
#include "IAerodynamic.hpp"
#include "../utils/AerodynamicTable.hpp"

class MissileAerodynamic: public IAerodynamic{
    public:
        MissileAerodynamic();
        ~MissileAerodynamic();
        bool init(  const std::string& Cx_a_filename, const std::string& Cx_don_filename, const std::string& Cy_a_filename,
                    const std::string& Cy_delta_filename, const std::string& Mz_a_filename, const std::string& Mz_delta_filename,
                    const std::string& Mz_wz_filename, const std::string& Mx_wx_filename, const std::string& Mx_delta_filename);
        double get_cx(double M, std::vector<double> &alpha_beta) const override;
        double get_cy(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const override;
        double get_cz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const override;
        double get_mx(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
        double get_my(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
        double get_mz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const override;
    
    private:
        DataTable Cx_a_dat;
        DataTable Cx_don_dat;
        DataTable Cy_a_dat;
        DataTable Cy_delta_dat;
        DataTable Mz_a_dat;
        DataTable Mz_delta_dat;
        DataTable Mz_wz_dat;
        DataTable Mx_wx_dat;
        DataTable Mx_delta_dat;
        double rad2deg;
};

#endif