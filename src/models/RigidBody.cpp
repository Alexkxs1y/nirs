#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/RigidBody.hpp"

using namespace std;

RigidBody::RigidBody(): orientationVector(vector<double>(6)), J(vector<double>(3)), torques(vector<double>(3)), torquesUpToDate(false){}


bool RigidBody::init(double _m, std::vector<double> _stateVector, std::vector<double> _J, std::vector<double> _roll_yaw_pitch, std::vector<double> _w){
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

bool RigidBody::set_torques(vector<double> _torques){
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
    return atan(this->get_Vy() / this->get_Vx());
}

double RigidBody::get_beta() const{
    return atan(this->get_Vz() / this->get_Vabs());
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
    _alpha_beta[0] = get_alpha();
    _alpha_beta[1] = get_beta();
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