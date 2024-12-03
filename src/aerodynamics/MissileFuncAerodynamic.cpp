#include "../../include/aerodynamics/MissileFuncAerodynamic.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#define _USE_MATH_DEFINES

using namespace std; 

MissileFuncAerodynamic::MissileFuncAerodynamic(){
    rad2deg = 1;// 180.0/M_PI;
}

MissileFuncAerodynamic::~MissileFuncAerodynamic(){}

double MissileFuncAerodynamic::get_cx(double M, std::vector<double> &alpha_beta) const{
    if (M < 2.16) {
		M = 2.16;
	}
	return (1 / (73.211 / exp(M) - 47.483 / M + 16.878));
}

double MissileFuncAerodynamic::get_cy_a(double M) const{
    double ds = 1.86 * (11.554 / exp(M) - 2.5191e-03 * M * M - 5.024 / M + 52.836e-03 * M + 4.112);
	if (ds >= 0) {
		return sqrt(ds);
	}
	else {
		return 1.86 * 1.039;
	}
}

double MissileFuncAerodynamic::get_cy_delta(double M, double alpha) const{
    double alpha_deg =  alpha * rad2deg;
	double p1 = 1 / (243.84e-03 / exp(alpha_deg) + 74.309e-03);
	double p2 = log(1.9773 * alpha_deg * alpha_deg - 25.587 * alpha_deg + 83.354);
	double p3 = 18.985 * alpha_deg * alpha_deg - 375.76 * alpha_deg + 1471;
	double p4 = -51.164e-03 * alpha_deg * alpha_deg + 805.52e-03 * alpha_deg + 1.8929;
	return (-p1 * 1e-06 * M * M + p2 * 1e-12 * exp(M) - p3 * 1e-06 * M - p4 * 1e-03) * 2;////////////////
}

double MissileFuncAerodynamic::get_cz_b(double M) const{
    return -get_cy_a(M);
}

double MissileFuncAerodynamic::get_cz_delta(double M, double beta) const{
    return -get_cy_delta(M, beta);
}

double MissileFuncAerodynamic::get_mz_wz(double M) const{
    return 1.89 * (146.79e-06 * M * M - 158.98e-03 / M - 7.639e-03 * M - 68.195e-03);
}

double MissileFuncAerodynamic::get_mz_a(double M) const{
    return -766.79e-03 / exp(M) + 438.74e-03 / M + 5.8822e-03 * M - 158.34e-03;
}

double MissileFuncAerodynamic::get_mz_delta(double M, double alpha) const{
    double alpha_deg = alpha * rad2deg;
    double k1 = exp(-19.488e-03 * alpha_deg * alpha_deg - 378.62e-03 * alpha_deg + 6.7518);
	double k2 = exp(-21.234e-03 * alpha_deg * alpha_deg - 635.84e-06 * exp(alpha_deg) - 98.296e-03 * alpha_deg + 2.5938);
	return 1.89 * sqrt(k1 * 1e-09 * M * M + k2 * 1e-06) ;
}

double MissileFuncAerodynamic::get_my_wy(double M) const{
    return get_mz_wz(M);
}

double MissileFuncAerodynamic::get_my_b(double M) const{
    return get_mz_a(M);
}

double MissileFuncAerodynamic::get_my_delta(double M, double beta) const{
    return get_mz_delta(M, beta);
}

double MissileFuncAerodynamic::get_cy(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const{
    return (get_cy_a(M) * alpha_beta[0] * rad2deg + get_cy_delta(M, alpha_beta[0])* deltas[2] * rad2deg);
}

double MissileFuncAerodynamic::get_cz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const{
    return (get_cz_b(M) * alpha_beta[1] * rad2deg + get_cz_delta(M, alpha_beta[1]) * deltas[1] * rad2deg);
}

double MissileFuncAerodynamic::get_mx(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    if(w[0] == 0){
        return 0;
    }
    return -0.005 * 0.6786 * l_div_Vabs * w[0]; 
}

double MissileFuncAerodynamic::get_my(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    return (get_my_wy(M) * w[1] * l_div_Vabs + get_my_b(M) * alpha_beta[1] * rad2deg + get_my_delta(M, alpha_beta[1]) * deltas[1] * rad2deg); 
}

double MissileFuncAerodynamic::get_mz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    return (get_mz_a(M) * alpha_beta[0] * rad2deg + get_mz_wz(M) * w[2] * l_div_Vabs + get_mz_delta(M, alpha_beta[0]) * deltas[2] * rad2deg);
}

double MissileFuncAerodynamic::get_mStab() const{
    return 100;
}