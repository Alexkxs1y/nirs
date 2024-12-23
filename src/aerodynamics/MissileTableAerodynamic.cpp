#include "../../include/aerodynamics/MissileTableAerodynamic.hpp"
#include <iostream>
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES

using namespace std; 

MissileTableAerodynamic::MissileTableAerodynamic(): isInit(false){}

MissileTableAerodynamic::~MissileTableAerodynamic(){}

bool MissileTableAerodynamic::init(  const string& Cx_a_filename, const string& Cx_don_filename, const string& Cy_a_filename,
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

    if(!Mx_wx_dat.loadFromFile(Mx_wx_filename)){
        cout << "Ошибка при чтении файла Mx_wx\n";
        return false;
    };

    if(!Mx_delta_dat.loadFromFile(Mx_delta_filename)){
        cout << "Ошибка при чтении файла Mx_delta\n";
        return false;
    };


    isInit = true;
    rad2deg = 180.0 / M_PI;
    return true;
}

double MissileTableAerodynamic::get_cx(double M, vector<double> &alpha_beta) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return Cx_a_dat.interpolate(M, abs(alpha_beta[0])) * abs(alpha_beta[0]) + Cx_don_dat.interpolate(M, abs(alpha_beta[0]));   
}

double MissileTableAerodynamic::get_cy(double M, vector<double> &alpha_beta, vector<double>& deltas) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return Cy_a_dat.interpolate(M, abs(alpha_beta[0])) * alpha_beta[0] + Cy_delta_dat.interpolate(M, abs(deltas[1])) * deltas[1]; 
}

double MissileTableAerodynamic::get_cz(double M, vector<double> &alpha_beta, vector<double>& deltas) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return - Cy_a_dat.interpolate(M, abs(alpha_beta[1])) * alpha_beta[1] - Cy_delta_dat.interpolate(M, abs(deltas[2])) * deltas[2];    
}

double MissileTableAerodynamic::get_mx(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return Mx_wx_dat.interpolate(M, abs(w[0])) * w[0] * l_div_Vabs + Mx_delta_dat.interpolate(M, abs(deltas[0])) * deltas[0];     
}

double MissileTableAerodynamic::get_my(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return  Mz_a_dat.interpolate(M, abs(alpha_beta[1])) * alpha_beta[1] + Mz_delta_dat.interpolate(M, abs(deltas[1])) * deltas[1] + Mz_wz_dat.interpolate(M, abs(w[1])) * w[1] * l_div_Vabs;
}

double MissileTableAerodynamic::get_mz(double M, vector<double> &alpha_beta, vector<double>& deltas, vector<double> &w, double l_div_Vabs) const {
    if(!isInit){
        cout << "Аэродинамика не задана!\n";
        return 0;
    }
    return Mz_a_dat.interpolate(M, abs(alpha_beta[0])) * alpha_beta[0] + Mz_delta_dat.interpolate(M, abs(deltas[2])) * deltas[2] + Mz_wz_dat.interpolate(M, abs(w[2])) * w[2] * l_div_Vabs;
}