#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/control/TargetGuidance.hpp"

using namespace std;

TargetGuidance::TargetGuidance(): phi_hi(vector<double>(2)), V_rel(vector<double>(3)){}

TargetGuidance::~TargetGuidance(){}

vector<double> TargetGuidance::get_GuidanceSignal(PointMass* target, PointMass* missile){
    if(!updateInformation(target, missile)){
        return {0, 0};
    }
    vector<double> signal(2);
    signal = Transform_V_to_signal();
    return signal;
}

bool TargetGuidance::updateInformation(PointMass* target, PointMass* missile){
    if(missile == 0){
        return false;
    }
    r_rel = sqrt(   (target->get_x() - missile->get_x()) * (target->get_x() - missile->get_x()) + 
                    (target->get_y() - missile->get_y()) * (target->get_y() - missile->get_y()) +
                    (target->get_z() - missile->get_z()) * (target->get_z() - missile->get_z())
                    );
    
    
    phi_hi[0] = asin( (target->get_y() - missile->get_y()) / r_rel);
    phi_hi[1] = -atan( ( target->get_z() - missile->get_z() ) / ( target->get_x() - missile->get_x() ) );

    vector<double> V_tar = target->get_V();
    vector<double> V_mis = missile->get_V();
    for(int i = 0; i < V_tar.size(); i++){
        V_rel[i] = V_tar[i] - V_mis[i];
    }
    
    return true;
} 

vector<double> TargetGuidance::A_1() const{
    vector<double> _A_1(3);
    _A_1[0] = sin(phi_hi[0]) * cos(phi_hi[1]);
    _A_1[1] = cos(phi_hi[0]);
    _A_1[2] = - sin(phi_hi[0]) * sin(phi_hi[1]);
    return _A_1;
}

vector<double> TargetGuidance::A_2() const{
    vector<double> _A_2(3);
    _A_2[0] = sin(phi_hi[1]);
    _A_2[1] = 0;
    _A_2[2] = cos(phi_hi[1]);
    return _A_2;
}


vector<double> TargetGuidance::B_1() const{
    vector<double> _B_1(3);
    _B_1[0] = -sin(phi_hi[0]) * cos(phi_hi[1]);
    _B_1[1] = cos(phi_hi[0]);
    _B_1[2] = sin(phi_hi[0]) * sin(phi_hi[1]);
    return _B_1;
}

vector<double> TargetGuidance::B_2() const{
    vector<double> _B_2(3);
    _B_2[0] = sin(phi_hi[1]);
    _B_2[1] = 0;
    _B_2[2] = cos(phi_hi[1]);
    return _B_2;
}

vector<double> TargetGuidance::V_phi_hi() const{
    vector<double> _B_1 = B_1();
    vector<double> _B_2 = B_2();
    double V_phi = 0;
    double V_hi = 0;
    for(int i = 0; i < _B_1.size(); i++){
        V_phi += V_rel[i] * _B_1[i];
        V_hi += V_rel[i] * _B_2[i];
    }
    return {V_phi, V_hi};
}   

vector<double> TargetGuidance::Transform_V_to_signal() const{
    vector<double> _A_1 = A_1();
    vector<double> _A_2 = A_2();
    vector<double> signal = {0, 0};
    vector<double> _V_phi_hi = V_phi_hi();
    double V_norm = sqrt(_V_phi_hi[0] * _V_phi_hi[0] + _V_phi_hi[1] * _V_phi_hi[1]);
    for(int i = 0; i < _V_phi_hi.size(); i++){
        _V_phi_hi[i] /= V_norm;
    }

    vector<double> V_sphere = {0, _V_phi_hi[0], _V_phi_hi[1]}; //Для красоты записи вектор скоростей перпендикулярных относительному радиусу
    for(int i = 0; i < V_sphere.size(); i++){
        signal[0] += V_sphere[i] * _A_1[i];
        signal[1] += V_sphere[i] * _A_2[i];
    }
    if(_V_phi_hi[0] == 0 && _V_phi_hi[1] == 0){
        signal = {0, 1};
    }
    return signal;
}   