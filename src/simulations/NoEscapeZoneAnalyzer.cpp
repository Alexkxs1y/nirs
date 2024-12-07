#include <math.h>
#include <vector>
#include <stdexcept>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/simulations/NoEscapeZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"

using namespace std;

vector<double> hitPointFinder(Missile* missile, Target* target, double effectiveRadius, double dt){
    
    bool inZone = false;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_w = missile -> get_w();
    vector<double> missile_ryp = missile -> get_ryp();
    vector<double> target_stateVector = target -> get_stateVector();
    
    vector<double> flight_res(5);

    double h = 200;
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
    target -> set_state(target_stateVector);
    
    return  hitPoint;
}




vector< vector<double> > noEscapeSurface(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt, int numPoints){
    
    vector< vector<double> > noEscapeSurface(numPoints, vector<double>(2));
    vector<double> hitPoint = hitPointFinder(missile, target, effectiveRadius, dt);

    int numZones = hitPoint[0];
    if(numZones == 0){
        for(int i = 0; i < numPoints; i++){
            noEscapeSurface.assign(numPoints, {0, 0, 0});
        }
        return noEscapeSurface;
    }

    vector<double> target_stateVector = target -> get_stateVector();
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
    }
    return noEscapeSurface;
}