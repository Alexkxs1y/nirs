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
    
    double target_y = target -> get_y();
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_w = missile -> get_w();
    vector<double> missile_ryp = missile -> get_ryp();
    vector<double> target_stateVector = target -> get_stateVector();
    
    vector<double> flight_res(5);
    
    /*
    double t_inter = (missile_stateVector[1] - target_y) / (target_stateVector[4] - missile_stateVector[4]);
    double target_z = missile_stateVector[2] + ( missile_stateVector[5] - target_stateVector[5] )  * t_inter;
    double target_x = missile_stateVector[3] + ( missile_stateVector[3] - target_stateVector[3] )  * t_inter;

    target_stateVector[0] = target_x;
    target_stateVector[2] = target_z;

    target -> set_state(target_stateVector);
    flight_res = oneMissileSimulation(missile, target, dt);
    cout << "ВОООТ:" << flight_res[0] << '\n';
    if(flight_res[4] > 0 && flight_res[0] < effectiveRadius){
        return target_stateVector;
    }
    */

    double h = 100;
    int i_max = 70; //Тут рандом...................................................    
    int i = 0;
    double missDistance = 2 * effectiveRadius;
    double cos_y = cos(missile_ryp[1]);
    double sin_y = sin(missile_ryp[1]);;
    
    while((missDistance > effectiveRadius) && (i <= i_max)){
        target_stateVector[0] = (10 + double(i) * h) * cos_y;
        target_stateVector[2] = (10 + double(i) * h) * sin_y;

        missile -> set_state(missile_stateVector, missile_ryp, missile_w);
        target -> set_state(target_stateVector);

        flight_res = oneMissileSimulation(missile, target, dt);
        if(flight_res[4] < 0){
            return {-1, -1, -1};
        } else {
            missDistance = flight_res[0];
            cout << missDistance << ' ' << i << ' ' << h << '\n';
            i++; 
        }
    }
    if(i > i_max && missDistance > effectiveRadius){
        return {-1, -1, -1};
    }
    vector<double> hitPoint = {target_stateVector[0], target_stateVector[1], target_stateVector[2]};
    return  hitPoint;

}