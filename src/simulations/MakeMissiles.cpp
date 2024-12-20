#include <math.h>
#include <vector>
#include <stdexcept>
#include "../../include/simulations/MakeMissiles.hpp"
#include "../../include/aerodynamics/MissileEasyAerodynamic.hpp"

using namespace std;

void makeOneMissile(Missile& mis, double _yaw, double _pitch, IAerodynamic & ma, MissileStabilization &ms, MissileGuidance &mg){
    double m = 420;
    double l = 7;
    double d = 0.5;
    double V = 1500;
    double delta_max = 15.0 * M_PI / 180.0;
    vector<double> w = {0.0, 0.0, 0.0};
    vector<double> J = {13, 875, 875};
    double pitch = _pitch;
    double yaw = _yaw; 
    vector<double> missile_stateVector = {0, 6000, 0, V * cos(pitch) * cos(yaw), V * sin(pitch) * cos(yaw), V * sin(yaw)};
    vector<double> roll_yaw_pitch = {0, yaw, pitch};




    vector<double> K_guidance = {235, 235};
    vector<double> K_stabilization = {5, 5, 5};
    double K_roll = 10;
    
  
    mg.init(K_guidance);
    ms.init(K_stabilization, K_roll);
    mis.init(m, missile_stateVector, J, roll_yaw_pitch, w, l, d, delta_max, &ma, &ms, &mg);
}

void makeMissiles(vector<Missile> & mis, vector<MissileGuidance> &mg, int n, vector<double>_yaw, vector<double>_pitch, IAerodynamic & ma, MissileStabilization &ms){
    if(_yaw.size() != _pitch.size() && n != _pitch.size()){
        throw runtime_error("While creating n missiles, size of _pitch and _yaw doesnt equal or doesnt equal n.");
    }
    double m = 420;
    double l = 7;
    double d = 0.5;
    double V = 1500;
    double delta_max = 15.0 * M_PI / 180.0;
    vector<double> w = {0.0, 0.0, 0.0};
    vector<double> J = {13, 875, 875};
    vector<double> K_guidance = {235, 235};
    vector<double> K_stabilization = {5, 5, 5};
    double K_roll = 10;
    ms.init(K_stabilization, K_roll);

    vector< vector<double> > missile_stateVector(n, vector<double>(6));  
    vector< vector<double> > roll_yaw_pitch(n, vector<double>(3));

    for(size_t i = 0; i < n; i++){
        roll_yaw_pitch[i] = {0, _yaw[i], _pitch[i]};
        missile_stateVector[i] = {0, 6000, 0, V * cos(_pitch[i]) * cos(_yaw[i]), V * sin(_pitch[i]) * cos(_yaw[i]), V * sin(_yaw[i])};
        mg[i].init(K_guidance);
        mis[i].init(m, missile_stateVector[i], J, roll_yaw_pitch[i], w, l, d, delta_max, &ma, &ms, &mg[i]);
    }
}