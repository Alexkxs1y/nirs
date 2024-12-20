#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/control/MissileCrossTargetGuidance.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"

using namespace std;

MissileCrossTargetGuidance::MissileCrossTargetGuidance():   K_guidance(vector<double>(2)), 
                                                            polynomCoefs(pair< vector<double>, vector<double> >
                                                            (vector<double>(4), vector<double>(4))){}

MissileCrossTargetGuidance::~MissileCrossTargetGuidance(){}

bool MissileCrossTargetGuidance::init(  vector<double>& _K_guidance, double _reGuidanceTime, double _dt){
    if(K_guidance.size() != _K_guidance.size()) 
        throw runtime_error("Wrong K_guidance vector size, while initialize CrossTargetGuidance");

    for(int i = 0; i < K_guidance.size(); i++){
        K_guidance[i] = _K_guidance[i];
    }
    time = 0;
    last_reGuidanceTime = numeric_limits<double>::min();
    reGuidanceTime = _reGuidanceTime;
    dt = _dt;
    needToUpdate = true;
    polynomCoefs = make_pair<vector<double>, vector<double>>({0,0,0,0}, {0,0,0,0});
    return true;
}

bool MissileCrossTargetGuidance::updateInformation(PointMass* missile, vector<PointMass*> targets){
    if(targets.size() != 2) 
        throw runtime_error("Try to use CrossTargetGuidance with targets number nonequal to 2!");
    time += dt;
    if(last_reGuidanceTime + reGuidanceTime < time){
        needToUpdate = true;
    }
    return true;
}

vector<double> MissileCrossTargetGuidance::currentMiss(PointMass* missile){
    vector<double> curr_r = missile -> get_r();
    double need_y = 0;
    double need_z = 0;
    for(size_t i = 0; i < polynomCoefs.first.size(); i++){
        need_y += pow(curr_r[0], i) * polynomCoefs.first[i];
        need_z += pow(curr_r[0], i) * polynomCoefs.second[i];
    }
    return {need_y - curr_r[1], need_z - curr_r[2]};
}

vector<double> MissileCrossTargetGuidance::get_GuidanceSignal(PointMass* missile, std::vector<PointMass*> targets){
    if(!updateInformation(missile, targets)) return {0, 0};
    vector<double> miss = currentMiss(missile);
    return {K_guidance[0] * miss[0], K_guidance[1] * miss[1]};
}

bool MissileCrossTargetGuidance::needToUpdateData(){
    return needToUpdate;
}

void MissileCrossTargetGuidance::updateData(pair< vector<double>, vector<double> >& data){
    cout <<"РАЗМЕР МАССИВА КОЭФФИЦИЕНТОВ: " << polynomCoefs.first.size() << '\n';
    for(size_t i = 0; i < polynomCoefs.first.size(); i ++){
        polynomCoefs.first[i] = data.first[i];
        polynomCoefs.second[i] = data.second[i];
    }
    last_reGuidanceTime = time;
    needToUpdate = false;
}