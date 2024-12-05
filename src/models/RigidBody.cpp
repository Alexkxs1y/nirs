#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/RigidBody.hpp"

using namespace std;

RigidBody::RigidBody(): orientationVector(vector<double>(6)), J(vector<double>(3)), torques(vector<double>(3)), torquesUpToDate(false){}


bool RigidBody::init(double _m, std::vector<double>& _stateVector, std::vector<double>& _J, std::vector<double>& _roll_yaw_pitch, std::vector<double>& _w){
    if(!PointMass::init(_m, _stateVector)){
        cout << "Ошибка произошла при инициализации тверого тела \n";
        return false;
    }

    if(J.size() != _J.size()){
        cout << "Ошибка при инициализации твердого тела. Размер иницилиазируемого вектора моментов инерции неверный \n";
        return false;
    }

     if(_roll_yaw_pitch.size() != 3){
        cout << "Ошибка при инициализации твердого тела. Размер иницилиазируемого вектора тангажа, рысканья и крена - неверный \n";
        return false;
    }

     if(_w.size() != 3){
        cout << "Ошибка при инициализации твердого тела. Размер иницилиазируемого вектора угловых скоростей неверный \n";
        return false;
    }

    for(int i = 0; i < J.size(); i++){
        J[i] = _J[i];
        orientationVector[i] = _roll_yaw_pitch[i];
        orientationVector[i + 3] = _w[i];
    }
    return true;
}

bool RigidBody::STEP(double dt){
    if(!PointMass::STEP(dt)){
        cout << "Ошибка произошла во время выполнения шага в твердом теле\n";
        return false;
    }

    if(!torquesUpToDate){
        cout << "Ошибка при совершении шага твердого тела. Моменты не были обновлены\n";
        return false;
    }

    orientationVector[0] += droll_dt() * dt;
    orientationVector[1] += dyaw_dt() * dt;
    orientationVector[2] += dpitch_dt() * dt;
    orientationVector[3] += dwx_dt() * dt;
    orientationVector[4] += dwy_dt() * dt;
    orientationVector[5] += dwz_dt() * dt;

    torquesUpToDate = false;

    return true;
}

RigidBody::~RigidBody(){}

bool RigidBody::set_torques(vector<double>& _torques){
    if(_torques.size() != torques.size()){
        cout << "Ошибка при установке новых моментов. Размер нового вектора моментов не равен старому \n";
        return false;
    }

    if(torquesUpToDate){
        cout << "Ошибка. Обновление моментов дважды подряд без совершения шага\n";
        return false;
    }

    for(int i = 0; i < torques.size(); i++){
        torques[i] = _torques[i];
    }

    torquesUpToDate = true;
    return true;
}

vector<double> RigidBody::get_J() const{
    return J;
}

double RigidBody::get_yaw() const{
    return orientationVector[1];
}

double RigidBody::get_pitch() const{
    return orientationVector[2];
}

double RigidBody::get_roll() const{
    return orientationVector[0];
}

double RigidBody::get_dyaw_dt() const{
    return dyaw_dt();
}

double RigidBody::get_dpitch_dt() const{
    return dpitch_dt();
}

double RigidBody::get_droll_dt() const{
    return droll_dt();
}

double RigidBody::get_wx() const{
    return orientationVector[3];
}

double RigidBody::get_wy() const{
    return orientationVector[4];
}

double RigidBody::get_wz() const{
    return orientationVector[5];
}

double RigidBody::get_alpha() const{
    vector<double> alpha_beta = get_alpha_beta();
    return alpha_beta[0];
}

double RigidBody::get_beta() const{
    vector<double> alpha_beta = get_alpha_beta();
    return alpha_beta[1];
}

vector<double> RigidBody::get_ypr() const{
    vector<double> _ypr(3);
    _ypr[0] = orientationVector[1];
    _ypr[1] = orientationVector[2];
    _ypr[2] = orientationVector[0];
    return _ypr;
}

vector<double> RigidBody::get_dypr_dt() const{
    vector<double> _d_ypr_dt(3);
    _d_ypr_dt[0] = dyaw_dt();
    _d_ypr_dt[1] = dpitch_dt();
    _d_ypr_dt[2] = droll_dt();
    return _d_ypr_dt;
}

vector<double> RigidBody::get_alpha_beta() const{
    vector<double> _alpha_beta(2);

    vector<double> ypr = get_ypr();
    vector<double> sin_ypr(3);
    vector<double> cos_ypr(3);
    for(int i = 0; i < ypr.size(); i++){
        sin_ypr[i] = sin(ypr[i]);
        cos_ypr[i] = cos(ypr[i]);
    }
    
    //Матрица перехода между НЗСК и СвСК
    vector<vector<double>> A(3, vector<double>(3));

    A[0] = {    cos_ypr[0] * cos_ypr[1], 
                sin_ypr[1],
              - sin_ypr[0] * cos_ypr[1] 
            };

    A[1] = {    sin_ypr[0] * sin_ypr[2] - cos_ypr[0] * sin_ypr[1] * cos_ypr[2],
                cos_ypr[1] * cos_ypr[2],
                cos_ypr[0] * sin_ypr[2] - sin_ypr[0] * sin_ypr[1] * cos_ypr[2]
            };

    A[2] = {    sin_ypr[0] * cos_ypr[2] - cos_ypr[0] * sin_ypr[1] * sin_ypr[2],
              - cos_ypr[1] * sin_ypr[2],
                cos_ypr[0] * cos_ypr[2] - sin_ypr[0] * sin_ypr[1] * sin_ypr[2]
            };
    vector<double> V_body = {0, 0, 0};
    vector<double> V = get_V();
    for(int i = 0; i < V_body.size(); i++){
        for(int j = 0; j < V.size(); j++){
            V_body[i] += V[j] * A[i][j];
        }
    }
    _alpha_beta[0] = - atan(V_body[1] / V_body[0]);
    _alpha_beta[1] = asin(V_body[2] / get_Vabs());
    return _alpha_beta;
}

vector<double> RigidBody::get_w() const{
    vector<double> _w(3);
    for(int i = 0; i < _w.size(); i++){
        _w[i] = orientationVector[3 + i];
    }
    return _w;
}

double RigidBody::droll_dt() const{
    return orientationVector[3] - (tan(orientationVector[2]) * (orientationVector[4] * cos(orientationVector[0]) - orientationVector[5] * sin(orientationVector[0])));

}

double RigidBody::dyaw_dt() const{
    return ((1 / cos(orientationVector[2])) * (orientationVector[4] * cos(orientationVector[0]) - orientationVector[5] * sin(orientationVector[0])));
}

double RigidBody::dpitch_dt() const{
    return orientationVector[4] * sin(orientationVector[0]) + orientationVector[5] * cos(orientationVector[0]);
}

double RigidBody::dwx_dt() const{
    return torques[0] / J[0] - (J[2] - J[1]) / J[0] * orientationVector[4] * orientationVector[5];
}

double RigidBody::dwy_dt() const{
    return torques[1] / J[1] - (J[0] - J[2]) / J[1] * orientationVector[3] * orientationVector[5];
}

double RigidBody::dwz_dt() const{
    return torques[2] / J[2] - (J[1] - J[0]) / J[2] * orientationVector[3] * orientationVector[4];
}

bool RigidBody::set_state(vector<double>& _stateVector, vector<double>& _roll_yaw_pitch, vector<double>& _w){
    if(!PointMass::set_state(_stateVector)) return false;
    if(_roll_yaw_pitch.size() + _w.size() != orientationVector.size() || _roll_yaw_pitch.size() != _w.size()){
        throw std::runtime_error(   "Exception in RigidBody::set_state(vector<double>& _stateVector, vector<double>& _roll_yaw_pitch, vector<double>& _w). Wrong _w or _roll_yaw_pitch size!\n");
        return false;
    }
    for(int i = 0; i < _w.size(); i++){
        orientationVector[i] = _roll_yaw_pitch[i];
        orientationVector[i + 3] = _w[i];
    }
    torquesUpToDate = false;
    return true;
}