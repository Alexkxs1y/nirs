#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/Target.hpp"
#include "../../include/aerodynamics/Atmosphere_GOST_4401_81.hpp"

using namespace std;

Target::Target(): n_xyz(vector<double>(3)), targetGuidance(0), pursuers(1), nUpToDate(false){}

Target::~Target(){}

bool Target::init(double _m, vector<double>& _stateVector, double _n_max,
                     TargetGuidance* _targetGuidance, PointMass* _pursuer){

    if(!PointMass::init(_m, _stateVector)){
        cout << "Ошибка произошла при инициализации цели.\n";
        return false;
    }
    n_max = _n_max;
    targetGuidance = _targetGuidance;
    pursuers[0] = _pursuer;
    return true;
}

bool Target::set_controlParams(){
    if(nUpToDate){
        cout << "Ошибка. Повторная установка парметров управления целью\n";
        return false;
    }
    vector<double> guidanceSignal = targetGuidance->get_GuidanceSignal(this, pursuers);
    for(int i = 0; i < n_xyz.size(); i++){
        n_xyz[i] = guidanceSignal[i] * n_max;
    }
    nUpToDate = true;
    return true;
}

void Target::set_pursuer(PointMass* _pursuer){
    pursuers.resize(1);
    pursuers[0] = _pursuer;
}

void Target::set_pursuer(vector<PointMass*> _pursuers){
    pursuers.resize(_pursuers.size());
    pursuers = _pursuers;
}

bool Target::STEP(double dt){
    if(!nUpToDate){
        cout<<"Шаг цели производится без расчета управления\n";
        return false;
    }

    vector<double> _forces(3);
    for(int i = 0; i < _forces.size(); i++){
        _forces[i] = m * Atmosphere_GOST_4401_81<double>::get_g(this->get_y()) * n_xyz[i];
    }

    if(!this->set_forces(_forces)){
        cout << "Ошибка произошла при совершении шага цели\n";
        return false;
    }

    if(!PointMass::STEP(dt)){
        cout << "Ошибка возникла при совершении шага цели\n";
        return false;
    }

    nUpToDate = false;

    return true;
}

vector<double> Target::get_n_xyz() const{
    return n_xyz;
}

vector<PointMass*> Target::get_pursuers() const{
    return pursuers;
}

bool Target::set_actualForceAndTorques(){
    if(!nUpToDate){
        throw std::runtime_error("Try to set Forces and Torques without set target guidance. N's are not upToDate\n");
    }

    vector<double> _forces(3);
    for(int i = 0; i < _forces.size(); i++){
        _forces[i] = m * Atmosphere_GOST_4401_81<double>::get_g(this->get_y()) * n_xyz[i];
    }

    if(!this->set_forces(_forces)){
        throw std::runtime_error("Goes wrong while set actual forces and torques to target!\n");
    }

    nUpToDate = false;

    return true;
}


Target::Target(Target &_target){
    
}