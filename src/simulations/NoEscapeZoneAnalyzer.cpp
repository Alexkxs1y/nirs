#include <math.h>
#include <vector>
#include <stdexcept>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/simulations/NoEscapeZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"
#include <iostream>

using namespace std;

vector<double> hitPointFinder(Missile* missile, Target* target, double effectiveRadius, double dt){
    
    bool inZone = false;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_w = missile -> get_w();
    vector<double> missile_ryp = missile -> get_ryp();
    vector<double> target_stateVector = target -> get_stateVector();
    vector<double> target_stateVector_initial = target_stateVector;

    vector<double> flight_res(5);

    double h = 250;
    int i_max = 50; //Тут рандом...................................................    
    int i = 0;
    double missDistance = 2 * effectiveRadius;
    double cos_y = cos(missile_ryp[1]);
    double sin_y = sin(missile_ryp[1]);
    vector<double> hitPoint = {0};
    
    while(i <= i_max){
        target_stateVector[0] = (10 + double(i) * h) * cos_y;
        target_stateVector[2] = (10 + double(i) * h) * sin_y;

        missile -> set_state(missile_stateVector, missile_ryp, missile_w);
        target -> set_state(target_stateVector);

        flight_res = oneMissileSimulation(missile, target, dt);
        if(flight_res[4] < 0){
            i ++;
            continue;
        }

        if((flight_res[0] < effectiveRadius) && (!inZone)){
            inZone = true;

            hitPoint.resize( (hitPoint[0] + 1) * 3 + 1);
            hitPoint[(hitPoint[0]) * 3 + 1] = target_stateVector[0];
            hitPoint[(hitPoint[0]) * 3 + 2] = target_stateVector[1];
            hitPoint[(hitPoint[0]) * 3 + 3] = target_stateVector[2];
            hitPoint[0] ++;
            i ++;
            continue;
        }

        if((flight_res[0] > effectiveRadius) && (inZone)){
            inZone = false;
            i ++;
            continue;
        }
        i ++;
    }
    
    missile -> set_state(missile_stateVector, missile_ryp, missile_w);
    target -> set_state(target_stateVector_initial);
    
    return  hitPoint;
}




vector< vector<double> > noEscapeSurface(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt, int numPoints){
    
    vector< vector<double> > noEscapeSurface(numPoints, vector<double>(2));
    vector<double> hitPoint = hitPointFinder(missile, target, effectiveRadius, dt);

    int numZones = hitPoint[0];
    if(numZones == 0){
        for(int i = 0; i < numPoints; i++){
            noEscapeSurface.assign(numPoints, {0, 0});
        }
        return noEscapeSurface;
    }

    vector<double> target_stateVector = target -> get_stateVector();
    vector<double> target_stateVector_initial = target_stateVector;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_w = missile -> get_w();
    vector<double> missile_ryp = missile -> get_ryp();
    
    vector<double> flightRes(5);
    int j = 1;
    double h = 1000;
    double missDistanse = 0;
    double searchAngle;
    double cos_i = 0;
    double sin_i = 0;
    bool isStepBack = false;
    

    for(int i = 0; i < numPoints; i++){
        
        searchAngle = missile_ryp[1] + i * 2 * M_PI / double(numPoints); 
        cos_i = cos( searchAngle );
        sin_i = sin( searchAngle );
        target_stateVector[0] = hitPoint[(numZones - 1) * 3 + 1];
        target_stateVector[2] = hitPoint[(numZones - 1)* 3 + 3];
        missDistanse = 0;
        h = 1000;
        isStepBack = false;

        while(abs(missDistanse - effectiveRadius) > tolerance){
            if(missDistanse < effectiveRadius){
                if(isStepBack){
                    h *= 0.5;
                }
                if(h < 0.5) break; //Тоже микрокостыль для ускорения******************************************************
                target_stateVector[0] += h * cos_i;
                target_stateVector[2] += h * sin_i;
            } else {
                h *= 0.5;
                target_stateVector[0] -= h * cos_i;
                target_stateVector[2] -= h * sin_i;
                isStepBack = true;                    
            }
            missile -> set_state(missile_stateVector, missile_ryp, missile_w);
            target -> set_state(target_stateVector);
            flightRes = oneMissileSimulation(missile, target, dt);
            missDistanse = flightRes[0];
            if(flightRes[4] < 0){
                missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
            }
        }
        if(i == 0){
            hitPoint[(numZones - 1) * 3 + 1] = 0.5 * (hitPoint[(numZones - 1) * 3 + 1] + target_stateVector[0]);
            hitPoint[(numZones - 1) * 3 + 3] = 0.5 * (hitPoint[(numZones - 1) * 3 + 3] + target_stateVector[2]);
        }
        noEscapeSurface[i] = {target_stateVector[0], target_stateVector[2]};
        cout << noEscapeSurface[i][0] << ' ' << noEscapeSurface[i][1] << '\n';
    }

    missile -> set_state(missile_stateVector, missile_ryp, missile_w);
    target -> set_state(target_stateVector_initial);

    return noEscapeSurface;
}



vector< vector< vector<double> > > noEscapeZone(Missile* missile, Target* target, double V_mis, double V_tar, double yaw_rel, double pitch_rel, double effectiveRadius, double tolerance, double dt, int numPoints){
    double y_mid = 6000; // Будем считать, что это высота, на которой находится ракета. Относительно неё будем варьировать высоту цели.
    vector<double> stateVector_tar_initial = target -> get_stateVector();
    vector<double> stateVector_mis_initial = missile -> get_stateVector();
    vector<double> ryp_mis_initial = missile -> get_ryp();
    vector<double> w_mis_initial = missile -> get_w();

    vector<double> stateVector_mis = {0, y_mid ,0, V_mis, 0, 0};
    vector<double> stateVector_tar = {0, y_mid , 0, V_tar * cos(yaw_rel) * cos(pitch_rel), V_tar * cos(yaw_rel) * sin(pitch_rel), V_tar * sin(yaw_rel)};
    vector<double> ryp = {0, 0, 0};
    vector<double> w = {0, 0, 0};
    double h_step = 100;
    bool outOfZone = false;

    vector< vector<double> > _noEscapeSurface(numPoints, vector<double>(2));
    vector< vector< vector<double> > > noEscapeZone;
    
    missile -> set_state(stateVector_mis, ryp, w);
    target -> set_state(stateVector_tar);

    while(!outOfZone){
        _noEscapeSurface = noEscapeSurface(missile, target, effectiveRadius, tolerance, dt, numPoints);
        if(_noEscapeSurface[0][0] == 0 && _noEscapeSurface[1][0] == 0){
            outOfZone = true;
        } else {
            noEscapeZone.push_back(_noEscapeSurface);
            stateVector_tar[1] += h_step;
            target -> set_state(stateVector_tar);
        }
    }

    missile -> set_state(stateVector_mis_initial, ryp_mis_initial, w_mis_initial);
    target -> set_state(stateVector_tar_initial);

    return noEscapeZone;
}