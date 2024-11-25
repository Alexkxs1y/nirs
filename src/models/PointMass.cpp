#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/PointMass.hpp"

using namespace std;

PointMass::PointMass(): stateVector(vector<double>(6)), forces(vector<double>(3)), forcesUpToDate(false){
}

PointMass::~PointMass(){}

bool PointMass::init(double _m, vector<double> _stateVector){
    if(_stateVector.size() != stateVector.size()){
        cout << "Ошибка при инициализации вектора состояния. Размер задаваемого вектора состояния неправильный\n";
        return false;
    }
    m = _m;
    for(int i = 0; i < stateVector.size(); i++){
        stateVector[i] = _stateVector[i];
    }
    return true;
}

bool PointMass::STEP(double dt){
    if(dt <= 0){
        cout << "Ошибка при совершении шага. Отрицательное dt\n";
        return false; 
    }

    if(!forcesUpToDate){
        cout << "Ошибка при совершении шага. Силы не были обновлены\n";
        return false;
    }

    stateVector[0] += dx_dt() * dt;
    stateVector[1] += dy_dt() * dt;
    stateVector[2] += dz_dt() * dt;
    stateVector[3] += dVx_dt() * dt;
    stateVector[4] += dVy_dt() * dt;
    stateVector[5] += dVz_dt() * dt;

    forcesUpToDate = false;
    
    return true;
}

bool PointMass::set_forces(vector<double> _forces){
    if(_forces.size() != forces.size()){
        cout << "Ошибка при установке новых сил. Размер нового вектора сил не равен старому \n";
        return false;
    }

    if(forcesUpToDate){
        cout << "Ошибка. Обновление сил дважды подряд без совершения шага\n";
        return false;
    }

    for(int i = 0; i < forces.size(); i++){
        forces[i] = _forces[i];
    }

    forcesUpToDate = true;
    return true;
}

double PointMass::get_m(){
    return m;
}

double PointMass::get_x(){
    return stateVector[0];
}

double PointMass::get_y(){
    return stateVector[1];
}

double PointMass::get_z(){
    return stateVector[2];
}

double PointMass::get_Vx(){
    return stateVector[3];
}

double PointMass::get_Vy(){
    return stateVector[4];
}

double PointMass::get_Vz(){
    return stateVector[5];
}

vector<double> PointMass::get_r(){
    vector<double> _r(3);
    for(int i = 0; i < _r.size(); i++){
        _r[i] = stateVector[i];
    }
    return _r;
}

vector<double> PointMass::get_V(){
    vector<double> _V(3);
    for(int i = 0; i < _V.size(); i++){
        _V[i] = stateVector[3+i];
    }
    return _V;
}

vector<double> PointMass::get_forces(){
    return forces;
}

double PointMass::dx_dt(){
    return stateVector[3];
}

double PointMass::dy_dt(){
    return stateVector[4];
}

double PointMass::dz_dt(){
    return stateVector[5];
}

double PointMass::dVx_dt(){
    return forces[0] / m;
}

double PointMass::dVy_dt(){
    return forces[1] / m;
}

double PointMass::dVz_dt(){
    return forces[2] / m;
}