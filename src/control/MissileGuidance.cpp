#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/control/MissileGuidance.hpp"
#include "../../include/aerodynamics/Atmosphere_GOST_4401_81.hpp"

#define STOP_RANGE double(300)
using namespace std;

MissileGuidance::MissileGuidance(): K_guidance(vector<double>(2)), phi_hi(vector<double>(2)),
                                    V_rel(vector<double>(3)), last_signal(vector<double>(2)){}

MissileGuidance::~MissileGuidance(){}

vector<double> MissileGuidance::get_GuidanceSignal(PointMass* missile, vector<PointMass*> targets){
    if(!updateInformation(missile, targets)){
        return {0, 0};
    }
    double Vabs = missile -> get_Vabs();
    double g = Atmosphere_GOST_4401_81<double>::get_g(0);
    if(r_rel > STOP_RANGE)
        last_signal = d_eta_dt();
    return {last_signal[0] * K_guidance[0] * Vabs / g, last_signal[1] * K_guidance[1] * Vabs / g};
}

bool MissileGuidance::init(vector<double>& _K_guidance){
    if(K_guidance.size() != _K_guidance.size()){
        cout <<"Ошибка при инициализации системы наведения ракеты. Размер вектора коэффициентов наведения неправильный\n";
        return false;
    }

    for(int i = 0; i < K_guidance.size(); i++){
        K_guidance[i] = _K_guidance[i];
    }
    return true;
}

bool MissileGuidance::updateInformation(PointMass* missile, vector<PointMass*> targets){
    if(targets.size() > 1) throw runtime_error("Two or more targets on missile wich controled by proportional guidance");
    PointMass* target = targets[0];
    if(target == 0){
        return false;
    }
    r_rel = sqrt(   (target->get_x() - missile->get_x()) * (target->get_x() - missile->get_x()) + 
                    (target->get_y() - missile->get_y()) * (target->get_y() - missile->get_y()) +
                    (target->get_z() - missile->get_z()) * (target->get_z() - missile->get_z())
                    );
    
    
    phi_hi[0] = asin( (target->get_y() - missile->get_y()) / r_rel);
    /*phi_hi[0] = atan2(
        target->get_y() - missile->get_y(),
        sqrt(
                (target->get_x() - missile->get_x()) * (target->get_x() - missile->get_x()) +
                (target->get_z() - missile->get_z()) * (target->get_z() - missile->get_z())
            )
    );*/
    phi_hi[1] = -atan2(target->get_z() - missile->get_z(), target->get_x() - missile->get_x());
    if((abs(target->get_z() - missile->get_z()) < 0.001) && (target->get_x() - missile->get_x() < 0) ){
        phi_hi[1] = M_PI;
    }
    //phi_hi[1] = -atan( ( target->get_z() - missile->get_z() ) / ( target->get_x() - missile->get_x() ) );

    
    
    vector<double> V_tar = target->get_V();
    vector<double> V_mis = missile->get_V();
    for(int i = 0; i < V_tar.size(); i++){
        V_rel[i] = V_tar[i] - V_mis[i];
    }

    return true;
} 

vector<double> MissileGuidance::B_1() const{
    vector<double> _B_1(3);
    _B_1[0] = -sin(phi_hi[0]) * cos(phi_hi[1]);
    _B_1[1] = cos(phi_hi[0]);
    _B_1[2] = sin(phi_hi[0]) * sin(phi_hi[1]);
    return _B_1;
}

vector<double> MissileGuidance::B_2() const{
    vector<double> _B_2(3);
    _B_2[0] = sin(phi_hi[1]);
    _B_2[1] = 0;
    _B_2[2] = cos(phi_hi[1]);
    return _B_2;
}

vector<double> MissileGuidance::d_eta_dt() const{
    vector<double> _B_1 = B_1();
    vector<double> _B_2 = B_2();
    double V_phi = 0;
    double V_hi = 0;
    for(int i = 0; i < _B_1.size(); i++){
        V_phi += V_rel[i] * _B_1[i];
        V_hi += V_rel[i] * _B_2[i];
    }
    return {V_phi / r_rel, V_hi / r_rel / cos(phi_hi[0]) };
}

bool MissileGuidance::needToUpdateData(){
    return true;
}

void MissileGuidance::updateData(std::pair<std::vector<double>, std::vector<double>>& data){
 cout <<"Я ВНУТРИ ПРОПОРЦИОНАЛЬНОГО НАВЕДЕНИЯ"<< '\n';
}
