#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/control/MissileStabilization.hpp"

using namespace std;

MissileStabilization::MissileStabilization(): K_stabilization(vector<double>(3)){}

MissileStabilization::~MissileStabilization(){}

vector<double> MissileStabilization::get_controlParams(RigidBody* missile, std::vector<double>& guidanceSignal){
    vector<double> _controlParams(3);
    _controlParams[0] = -( K_roll * missile->get_roll() + K_stabilization[0] * missile->get_droll_dt());
    _controlParams[1] = K_stabilization[1] * (guidanceSignal[1] - missile->get_dyaw_dt());
    _controlParams[2] = K_stabilization[2] * (guidanceSignal[0] - missile->get_dpitch_dt());
    return _controlParams;
}

bool MissileStabilization::init(vector<double>& _K_stabilization, double _K_roll){
    if(K_stabilization.size() != _K_stabilization.size()){
        cout <<"Ошибка при инициализации системы стабилизации ракеты. Размер вектора коэффициентов стабилизации неправильный\n";
        return false;
    }

    for(int i = 0; i < K_stabilization.size(); i++){
        K_stabilization[i] = _K_stabilization[i];
    }
    K_roll = _K_roll;

    return true;
}
