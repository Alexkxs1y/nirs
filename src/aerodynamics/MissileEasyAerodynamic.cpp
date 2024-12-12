#include "../../include/aerodynamics/MissileEasyAerodynamic.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#define _USE_MATH_DEFINES

using namespace std; 

MissileEasyAerodynamic::MissileEasyAerodynamic(){
    rad2deg = 180.0 / M_PI;
}

MissileEasyAerodynamic::~MissileEasyAerodynamic(){}

double MissileEasyAerodynamic::get_cx(double M, std::vector<double> &alpha_beta) const{
    if (M < 1.0) return 0.3 + 0.2 * M * M;
    if (M < 2.5) return 0.5 -  0.1 * (M - 1);
	return 0.35 + 0.05 * (M - 2.5); 
}

double MissileEasyAerodynamic::get_cy_a(double M) const{
    if(M < 0.8) return 3.5;
    if(M < 2.5) return 3.5 - 0.5 * (M - 0.8);
    return 2.65;
}

double MissileEasyAerodynamic::get_cy_delta(double M, double alpha) const{
    if(M < 1.0) return 1.5;
    if(M < 3.0) return 1.5 - 0.1 * (M - 1.0);
    return 1.3;
}

double MissileEasyAerodynamic::get_cz_b(double M) const{
    return -get_cy_a(M);
}

double MissileEasyAerodynamic::get_cz_delta(double M, double beta) const{
    return get_cy_delta(M, beta);
}

double MissileEasyAerodynamic::get_mz_wz(double M) const{
    return - 0.07 ;
}

double MissileEasyAerodynamic::get_mz_a(double M) const{
    if(M < 1.0) return -2.2;
    if(M < 2.5) return -2.2 - 0.2 * (M - 1.0);
    return -2.5;
}

double MissileEasyAerodynamic::get_mz_delta(double M, double alpha) const{
    if(M < 1.0) return -0.3;
    if(M < 2.5) return -0.3 - 0.2 * (M - 1.0) / 3.0;
    return - 0.4;
}

double MissileEasyAerodynamic::get_my_wy(double M) const{
    return get_mz_wz(M);
}

double MissileEasyAerodynamic::get_my_b(double M) const{
    return get_mz_a(M);
}

double MissileEasyAerodynamic::get_my_delta(double M, double beta) const{
    return -get_mz_delta(M, beta);
}

double MissileEasyAerodynamic::get_cy(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const{
    return (get_cy_a(M) * alpha_beta[0] + get_cy_delta(M, alpha_beta[0])* deltas[2]);
}

double MissileEasyAerodynamic::get_cz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas) const{
    return (get_cz_b(M) * alpha_beta[1] + get_cz_delta(M, alpha_beta[1]) * deltas[1]);
}

double MissileEasyAerodynamic::get_mx(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    return -0.0001 * w[0] * 0 + 0.0007 * deltas[0]; 
}

double MissileEasyAerodynamic::get_my(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    return (get_my_wy(M) * w[1] +   0.01 * get_my_b(M) * alpha_beta[1] + 0.005 * get_my_delta(M, alpha_beta[1]) * deltas[1]); 
}

double MissileEasyAerodynamic::get_mz(double M, std::vector<double> &alpha_beta, std::vector<double> &deltas, std::vector<double> &w, double l_div_Vabs) const{
    return ( 0.01 * get_mz_a(M) * alpha_beta[0] + get_mz_wz(M) * w[2] + 0.005 * get_mz_delta(M, alpha_beta[0]) * deltas[2]);
}

double MissileEasyAerodynamic::get_mStab() const{
    return 0.0;
}