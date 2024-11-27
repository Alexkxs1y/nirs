#include "../../include/aerodynamics/MissileAerodynamic.hpp"
#include <iostream>
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES

using namespace std; 

MissileAerodynamic::MissileAerodynamic(){}

MissileAerodynamic::~MissileAerodynamic(){}

bool MissileAerodynamic::init(  const string& Cx_a_filename, const string& Cx_don_filename, const string& Cy_a_filename,
                                const string& Cy_delta_filename, const string& Mz_a_filename, const string& Mz_delta_filename,
                                const string& Mz_wz_filename, const string& Mx_wx_filename, const string& Mx_delta_filename){

    if(!Cx_a_dat.loadFromFile(Cx_a_filename)){
        cout << "Ошибка при чтении файла Cx_a\n";
        return false;
    };

     if(!Cx_don_dat.loadFromFile(Cx_don_filename)){
        cout << "Ошибка при чтении файла Cx_don\n";
        return false;
    };

     if(!Cy_a_dat.loadFromFile(Cy_a_filename)){
        cout << "Ошибка при чтении файла Cy_a\n";
        return false;
    };

     if(!Cy_delta_dat.loadFromFile(Cy_delta_filename)){
        cout << "Ошибка при чтении файла Cy_delta\n";
        return false;
    };

     if(!Mz_a_dat.loadFromFile(Mz_a_filename)){
        cout << "Ошибка при чтении файла Mz_a\n";
        return false;
    };

     if(!Mz_delta_dat.loadFromFile(Mz_delta_filename)){
        cout << "Ошибка при чтении файла Mz_delta_a\n";
        return false;
    };

     if(!Mz_wz_dat.loadFromFile(Mz_wz_filename)){
        cout << "Ошибка при чтении файла Mz_wz\n";
        return false;
    };

    rad2deg = 180.0 / M_PI;
    return true;
}

double MissileAerodynamic::get_cx(double M, vector<double> &alpha_beta) const {
    return Cx_a_dat.interpolate(M, abs(alpha_beta[0])) * abs(alpha_beta[0]) + Cx_don_dat.interpolate(M, abs(alpha_beta[0]));   
}

double MissileAerodynamic::get_cy(double M, vector<double> &alpha_beta, vector<double>& deltas) const {
    return Cy_a_dat.interpolate(M, abs(alpha_beta[0])) * alpha_beta[0] + Cy_delta_dat.interpolate(M, abs(deltas[1])) * deltas[1]; 
}

double MissileAerodynamic::get_cz(double M, vector<double> &alpha_beta, vector<double>& deltas) const {
    return - Cy_a_dat.interpolate(M, abs(alpha_beta[1])) * alpha_beta[1] - Cy_delta_dat.interpolate(M, abs(deltas[2])) * deltas[2];    
}

double MissileAerodynamic::get_mx(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    return Mx_wx_dat.interpolate(M, abs(w[0])) * w[0] * l_div_Vabs + Mx_delta_dat.interpolate(M, abs(deltas[0])) * deltas[0];     
}

double MissileAerodynamic::get_my(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    return  Mz_a_dat.interpolate(M, abs(alpha_beta[1])) * alpha_beta[1] + Mz_delta_dat.interpolate(M, abs(deltas[1])) * deltas[1] + Mz_wz_dat.interpolate(M, abs(w[1])) * w[1] * l_div_Vabs;
}

double MissileAerodynamic::get_mz(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    return Mz_a_dat.interpolate(M, abs(alpha_beta[0])) * alpha_beta[0] + Mz_delta_dat.interpolate(M, abs(deltas[2])) * deltas[2] + Mz_wz_dat.interpolate(M, abs(w[2])) * w[2] * l_div_Vabs;
}