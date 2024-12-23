#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/AperiodMissile.hpp"
#include "../../include/aerodynamics/Atmosphere_GOST_4401_81.hpp"
#include "../../include/models/Target.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"

#define DRAG_KOEF double(4e-5)
#define EFFECTIVE_RAD double(15)
#define TOLERANCE double(0.5)

using namespace std;

AperiodMissile::AperiodMissile():   workGuidance(0), propGuidance(0), crossGuidance(0), targets(vector<Target*>(0)), 
                                    n_xyz(vector<double>(3)), n_xyz_body(vector<double>(3)), nUpToDate(false),
                                    d_n_xyz_body(vector<double>(2)){}

AperiodMissile::~AperiodMissile(){}

AperiodMissile::AperiodMissile(AperiodMissile &_missile): PointMass(_missile){
    n_xyz = vector<double>(3);
    //n_xyz_body = vector<double>(3);
    n_xyz_body = _missile.get_n_xyz_body();
    //d_n_xyz_body = vector<double>(2);
    d_n_xyz_body = _missile.get_d_n_xyz_body();
    n_max = _missile.get_n_max();
    T_missisle = _missile.get_T_missile();
    nUpToDate = false;
}

bool AperiodMissile::init(double _m, std::vector<double>& _stateVector, double _n_max, double _T_missisle, IGuidance* _propGuidance, Target* _target ){
    if(!PointMass::init(_m, _stateVector)){
        cout<<"Ошибка произошла при инициализации ракеты\n";
        return false;
    }
    n_max = _n_max;
    n_xyz_body = {0, 0, 0};
    T_missisle = _T_missisle;
    propGuidance = _propGuidance;
    targets.resize(1);
    targets[0] = _target;
    nUpToDate = false;
    return true;
}

bool AperiodMissile::STEP(double dt) {
     if(!nUpToDate){
        cout<<"Шаг ракеты производится без расчета систем наведения\n";
        return false;
    }
    
    for(size_t i = 0; i < d_n_xyz_body.size(); i++){
        n_xyz_body[i + 1] += d_n_xyz_body[i] * dt;
    }
    double n_norm = sqrt(n_xyz_body[1] * n_xyz_body[1] + n_xyz_body[2] * n_xyz_body[2]);

    if (n_norm > n_max){
        n_xyz_body[1] *= n_max / n_norm;
        n_xyz_body[2] *= n_max / n_norm; 
    }

    if(!calc_forces()){
        cout << "Ошибка в расчёте сил при совершении шага ракеты\n";
        return false;
    }
    if(!PointMass::STEP(dt)){
        cout << "Ошибка возникла при совершении шага ракеты\n";
        return false;
    }
    nUpToDate = false;
    return true;
}

bool AperiodMissile::set_actualForceAndTorques(){
    if(!nUpToDate){
        throw std::runtime_error("Try to set Forces set controlParams. Overloads are not upToDate\n");
        return false;
    }
    if(!calc_forces()){
        throw std::runtime_error("While setting forces AperiodMissile class method.\n");
        return false;
    }

    nUpToDate = false;
    return true; 
}

bool AperiodMissile::set_controlParams(){
    if(nUpToDate){
        cout<<"Параметры управления уже были установлены на этом шаге\n";
        return false;
    }
    //КОСТЫЛИ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    /////////////////////////////////////////////////////
    vector<PointMass*> tags;
    solveControlConflict(tags);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    vector<double> _guidanceSignal = workGuidance->get_GuidanceSignal(this, tags);
    
    double Vabs = get_Vabs();
    double Theta = atan2(get_Vy(), get_Vx());
    double Psi = atan2(-get_Vz(), sqrt(get_Vx() * get_Vx() + get_Vy() * get_Vy()));

    vector<double> cos_tp = {cos(Theta), cos(Psi)};
    vector<double> sin_tp = {sin(Theta), sin(Psi)};

    vector<vector<double>> A_glob2loc(3, vector<double>(3));
    A_glob2loc[0] = {cos_tp[0] * cos_tp[1], sin_tp[0] * cos_tp[1], - sin_tp[1]};
    A_glob2loc[1] = {- sin_tp[0], cos_tp[0], 0};
    A_glob2loc[2] = {cos_tp[0] * sin_tp[1], sin_tp[0] * sin_tp[1], cos_tp[1]};
    
    vector<vector<double>> A_loc2glob(3, vector<double>(3));
    A_loc2glob[0] = {cos_tp[0] * cos_tp[1], -   sin_tp[0], cos_tp[0] * sin_tp[1]};
    A_loc2glob[1] = {sin_tp[0] * cos_tp[1],     cos_tp[0], sin_tp[0] * sin_tp[1]};
    A_loc2glob[2] = {-sin_tp[1], 0, cos_tp[1]};

    //double n_drag = DRAG_KOEF * Vabs; //Значение перегрузки от сопротивления
    
    n_xyz_body[0] = - DRAG_KOEF * Vabs * Vabs / Atmosphere_GOST_4401_81<double>::get_g(0); //Значение перегрузки от сопротивления

    d_n_xyz_body[0] = (_guidanceSignal[0] - n_xyz_body[1]) / T_missisle;
    d_n_xyz_body[1] = (_guidanceSignal[1] - n_xyz_body[2]) / T_missisle;

    //cout << n_xyz_body[1] << ' ' << n_xyz_body[2] << '\n'; 

    for(int i = 0; i < n_xyz.size(); i++){
        n_xyz[i] = 0;
        for(size_t j = 0; j < 3; j++){
            n_xyz[i] += n_xyz_body[j] * A_loc2glob[i][j];
        }
    }

    nUpToDate = true;
    return true; 
}

void AperiodMissile::set_target(Target* _target){
    targets.resize(1);
    targets[0] = _target;
}

void AperiodMissile::set_target(std::vector<Target*> &_targets){
    targets.resize(_targets.size());
    targets = _targets;
}

void AperiodMissile::set_propGuidance(IGuidance* _propGuidance){
    propGuidance = _propGuidance;
}

void AperiodMissile::set_crossGuidance(IGuidance* _crossGuidance){
    crossGuidance = _crossGuidance;
}

IGuidance* AperiodMissile::get_Guidance(){
    return workGuidance;
}

std::vector<Target*> AperiodMissile::get_targets(){
    return targets;
}

std::vector<double> AperiodMissile::get_n_xyz(){
    return n_xyz;
}

double AperiodMissile::get_T_missile(){
    return T_missisle;
}

double AperiodMissile::get_n_max(){
    return n_max;
}

void AperiodMissile::choose_Guidance(){
    if(targets.size() == 2){
        workGuidance = crossGuidance;
    } else {
        if(targets.size() < 2){
            workGuidance = propGuidance;
        } else{ 
            cout <<"РАЗМЕР ЦЕЛЕЙ: " <<  targets.size() << '\n';
            throw runtime_error("Wrong number of targets!");
        }
    }

    if(workGuidance == NULL) throw runtime_error("Flight without guidance!");
}

void AperiodMissile::solveControlConflict(std::vector<PointMass*>& _tags){
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

bool AperiodMissile::calc_forces(){
    vector<double> _forces = {0, 0, 0};
    
    for(int i = 0; i < _forces.size(); i++){
        _forces[i] = n_xyz[i] * m * Atmosphere_GOST_4401_81<double>::get_g(0);   
    }

    //Установка действующих на ЛА сил в НЗСК
    if(!set_forces(_forces)){
        cout << "Ошибка в устанавливаемых силах ракеты\n";
        return false;
    }

    return true;
}

vector<double> AperiodMissile::get_d_n_xyz_body(){
    return d_n_xyz_body;
}

bool AperiodMissile::STEP(double dt, std::vector<double>& dr_dt, std::vector<double>& dV_dt, std::vector<double>& d_n_xyz_body){
    PointMass::STEP(dt, dr_dt, dV_dt);
    for(size_t i = 0; i < d_n_xyz_body.size(); i++){
        n_xyz_body[i + 1] += d_n_xyz_body[i] * dt;
    }
    double n_norm = sqrt(n_xyz_body[1] * n_xyz_body[1] + n_xyz_body[2] * n_xyz_body[2]);

    if (n_norm > n_max){
        n_xyz_body[1] *= n_max / n_norm;
        n_xyz_body[2] *= n_max / n_norm; 
    }

    return true;
}

vector<double> AperiodMissile::get_n_xyz_body(){
    return n_xyz_body;
}

void AperiodMissile::set_state(vector<double>& _stateVector_missile, vector<double> & _n_xyz_body){
    PointMass::set_state(_stateVector_missile);
    for(size_t i = 0; i < _n_xyz_body.size(); i++){
        n_xyz_body[i] = _n_xyz_body[i];
    }
}