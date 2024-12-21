#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/Missile.hpp"
#include "../../include/aerodynamics/Atmosphere_GOST_4401_81.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"

#define EFFECTIVE_RAD double(15)
#define TOLERANCE double(0.5)

using namespace std;

Missile::Missile(): deltas(vector<double>(3)), missileAerodynamic(0), missileStabilization(0),
                    workGuidance(0), propGuidance(0), crossGuidance(0), targets(1), deltaUpToDate(false){};

Missile::~Missile(){}

bool Missile::init( double _m, vector<double>& _stateVector, vector<double>& _J, vector<double>& _roll_yaw_pitch, vector<double>& _w,
                    double _l, double _d, double _delta_max, IAerodynamic* _missileAerodynamic,
                    MissileStabilization* _missileStabilization, IGuidance* _propGuidance, Target* _target
                    ){
    if(!RigidBody::init(_m, _stateVector, _J, _roll_yaw_pitch, _w)){
        cout<<"Ошибка произошла при инициализации ракеты\n";
        return false;
    }
    delta_max = _delta_max;
    missileAerodynamic = _missileAerodynamic;
    missileStabilization = _missileStabilization;
    propGuidance = _propGuidance;
    targets[0] = _target;
    l = _l;
    d = _d;
    return true;
}

bool Missile::set_controlParams(){
    if(deltaUpToDate){
        cout<<"Параметры управления уже были установлены на этом шаге\n";
        return false;
    }
    //КОСТЫЛИ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    /////////////////////////////////////////////////////
    vector<PointMass*> tags;
    solveControlConflict(tags);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    vector<double> _guidanceSignal = workGuidance->get_GuidanceSignal(this, tags);
    vector<double> _deltas = missileStabilization->get_controlParams(this, _guidanceSignal);
    for(int i = 0; i < deltas.size(); i++){
        deltas[i] = _deltas[i];
        if(abs(deltas[i]) > delta_max){
            deltas[i] = delta_max * deltas[i] / abs(deltas[i]);
        }
    }
    deltaUpToDate = true;
    return true; 
}

void Missile::set_target(Target* _target){
    targets.resize(1);
    targets[0] = _target;
}

void Missile::set_target(vector<Target*> _targets){
    targets.resize(_targets.size());
    targets = _targets;
}

bool Missile::STEP(double dt){
    if(!deltaUpToDate){
        cout<<"Шаг ракеты производится расчета систем наведения и стабилизации\n";
        return false;
    }
    if(!(calc_forces() && calc_torques())){
        cout << "Ошибка в расчёте сил или моментов при совершении шага ракеты\n";
        return false;
    }
    if(!RigidBody::STEP(dt)){
        cout << "Ошибка возникла при совершении шага ракеты\n";
        return false;
    }
    deltaUpToDate = false;
    return true;
}

bool Missile::set_actualForceAndTorques(){
    if(!deltaUpToDate){
        throw std::runtime_error("Try to set Forces and Torques without set deltas. Deltas not upToDate\n");
        return false;
    }
    if(!(calc_forces() && calc_torques())){
        throw std::runtime_error("While setting forces and torques in Missile class method.\n");
        return false;
    }

    deltaUpToDate = false;
    return true; 
}

vector<double> Missile::calc_bodyRelatedAeroForce(){
    double h = get_y();
    double V = get_Vabs();
    double M = V / Atmosphere_GOST_4401_81<double>::SoundSpeed(h);
    double q = Atmosphere_GOST_4401_81<double>::Density(h) * V * V * 0.5;
    double Sm = M_PI * d * d * 0.25;
    double F_max = 40 * m * Atmosphere_GOST_4401_81<double>::get_g(0); //Костыль====================================================
    vector<double> alpha_beta = get_alpha_beta();

    vector<double> aeroForces(3);
    aeroForces[0] = - missileAerodynamic->get_cx(M, alpha_beta) * q * Sm;
    aeroForces[1] = missileAerodynamic->get_cy(M, alpha_beta, deltas) * q * Sm;
    aeroForces[2] = missileAerodynamic->get_cz(M, alpha_beta, deltas) * q * Sm;
    double F_perpend = sqrt(aeroForces[1] * aeroForces[1] + aeroForces[2] * aeroForces[2]);
    if( F_perpend > F_max){
        aeroForces[1] *= F_max / F_perpend;
        aeroForces[2] *= F_max / F_perpend;
    }

    return aeroForces;
}

bool Missile::calc_forces(){
    vector<double> ypr = get_ypr();
    vector<double> sin_ypr(3);
    vector<double> cos_ypr(3);
    for(int i = 0; i < ypr.size(); i++){
        sin_ypr[i] = sin(ypr[i]);
        cos_ypr[i] = cos(ypr[i]);
    }
    
    //Матрица перехода между СвСК и НЗСК
    vector<vector<double>> A(3, vector<double>(3));

    A[0] = {    cos_ypr[1] * cos_ypr[0], 
              - sin_ypr[1] * cos_ypr[0] * cos_ypr[2] + sin_ypr[0] * sin_ypr[2],
                sin_ypr[1] * cos_ypr[0] * sin_ypr[2] + sin_ypr[0] * cos_ypr[2]
            };

    A[1] = {    sin_ypr[1],
                cos_ypr[1] * cos_ypr[2],
              - cos_ypr[1] * sin_ypr[2]
            };

    A[2] = {  - cos_ypr[1] * sin_ypr[0],
                sin_ypr[1] * sin_ypr[0] * cos_ypr[2] + cos_ypr[0] * sin_ypr[2],
              - sin_ypr[1] * sin_ypr[0] * sin_ypr[2] + cos_ypr[0] * cos_ypr[2]
            };
    
    vector<double> _forces = {0, 0, 0};
    //Вектор АД сил в СвСК
    vector<double> aeroBodyForces = calc_bodyRelatedAeroForce();
    
    //Перевод АД сил из СвСК в НЗСК
    for(int i = 0; i < aeroBodyForces.size(); i++){
        for(int j = 0; j < aeroBodyForces.size(); j++){
            _forces[i] += aeroBodyForces[j] * A[i][j];
        }       
    }

    //Добавление силы тяжесте
    //_forces[1] -= Atmosphere_GOST_4401_81<double>::get_g( get_y() ) * m; //Считаем, что сила тяжести мала
    
    //Установка действующих на ЛА сил в НЗСК
    if(!set_forces(_forces)){
        cout << "Ошибка в устанавливаемых силах ракеты\n";
        return false;
    }

    return true;
}

bool Missile::calc_torques(){
    double h = get_y();
    double V = get_Vabs();
    double M = V / Atmosphere_GOST_4401_81<double>::SoundSpeed(h);
    double q = Atmosphere_GOST_4401_81<double>::Density(h) * V * V * 0.5;
    double Sm = M_PI * d * d * 0.25;
    double l_div_V = l / V;
    vector<double> alpha_beta = get_alpha_beta();
    vector<double> w = get_w();

    //Подсчёт АД моментов
    vector<double> aeroTorques(3);
    aeroTorques[0] = missileAerodynamic->get_mx(M, alpha_beta, deltas, w, l_div_V) * q * Sm * l + missileAerodynamic->get_mStab() * deltas[0];
    aeroTorques[1] = missileAerodynamic->get_my(M, alpha_beta, deltas, w, l_div_V) * q * Sm * l;
    aeroTorques[2] = missileAerodynamic->get_mz(M, alpha_beta, deltas, w, l_div_V) * q * Sm * l;

    //Установка действующих на ЛА момента
    if(!set_torques(aeroTorques)){
        cout << "Ошибка при установке моментов, действующих на ракеты\n";
        return false;
    }

    return true;
}

vector<double> Missile::get_deltas(){
    return deltas;
}

vector<Target*> Missile::get_targets(){
    return targets;
}

void Missile::choose_Guidance(){
    if(targets.size() == 2){
        workGuidance = crossGuidance;
    } else {
        if(targets.size() == 1){
            workGuidance = propGuidance;
        } else{ 
            cout <<"РАЗМЕР ЦЕЛЕЙ: " <<  targets.size() << '\n';
            throw runtime_error("Wrong number of targets!");
        }
    }

    if(workGuidance == NULL) throw runtime_error("Flight without guidance!");
}

IGuidance* Missile::get_Guidance(){
    return workGuidance;
}

void Missile::solveControlConflict(vector<PointMass*>& _tags){
    choose_Guidance();
    if((workGuidance == crossGuidance) && workGuidance -> needToUpdateData()){
        vector< vector<double> > points = fairTrajectoryPoints(this, targets[0], targets[1], EFFECTIVE_RAD, TOLERANCE, 0.2, 1e-3);
        if(points[0][0] == -1){
            this -> set_target(targets[1]);
            choose_Guidance();
        } else {
            if(points[0][1] == -1){
                this -> set_target(targets[0]);
                choose_Guidance();
            } else {
                choose_Guidance();
                pair<vector<double>, vector<double>> coefs = fitCubicPolynomials3D(points);
                workGuidance -> updateData(coefs);
            }
        } 
    }
    _tags.resize(targets.size());
    for(size_t i = 0; i < _tags.size(); i++){
        _tags[i] = (PointMass*) targets[i];
    }
}

 void Missile::set_propGuidance(IGuidance* _propGuidance){
    propGuidance = _propGuidance;
 }

void Missile::set_crossGuidance(IGuidance* _crossGuidance){
    crossGuidance = _crossGuidance;
}

double Missile::get_l(){
    return l;
}

double Missile::get_d(){
    return d;
}

double Missile::get_delta_max(){
    return delta_max;
}

MissileStabilization* Missile::get_missileStab(){
    return missileStabilization;
}

IAerodynamic* Missile::get_missileAero(){
    return missileAerodynamic;
}

 Missile::Missile(Missile &_missile): RigidBody(_missile){
    missileAerodynamic = _missile.get_missileAero();
    missileStabilization = _missile.get_missileStab();
    workGuidance = 0;
    propGuidance = 0;
    crossGuidance = 0;
    targets = vector<Target*>(0);
    l = _missile.get_l();
    d = _missile.get_d();
    delta_max = _missile.get_delta_max();
    deltas = vector<double>(3);
    deltaUpToDate = false;
 }