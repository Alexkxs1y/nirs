#include <math.h>
#include <vector>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/utils/MyMath.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

#include "../../include/simulations/TwoTargetsMissileFlight.hpp"
#include "../../include/control/MissileCrossTargetGuidance.hpp"

using namespace std;

vector<double> twoTargetsOneMissileFlight(Missile& missile, std::vector<Target> &targets, double dt){

    ofstream out;          
    string name = "TRAJECTORY.dat";
    
    MissileCrossTargetGuidance mg;
    vector<double> K_guidance = {1, 1};
    double reGuidanceTime = 0.2;
    double t = 0;
    IGuidance* curr_Guidance = missile.get_Guidance();
    mg.init(K_guidance, reGuidanceTime, dt);
    vector<Target*> targetsPtr(targets.size());
    for(size_t i = 0; i < targets.size(); i++){
        targetsPtr[i] = &targets[i];
    }
    missile.set_crossGuidance(&mg);
    missile.set_target(targetsPtr);
    int i = 0;

    missile.set_controlParams();
    targets[0].set_controlParams();
    targets[1].set_controlParams();

    while (missile.get_Guidance() == &mg)
    {
        missile.STEP(dt);
        targets[0].STEP(dt);
        targets[1].STEP(dt);
        t += dt;
        if(i* 0.01 <= t ){
            out.open(name, ios::app);
            out << setprecision(10) << t << ' ' << missile.get_x() << ' ' << missile.get_y() << ' ' << missile.get_z() << ' ' << targets[0].get_x() << ' ' 
            << targets[0].get_y() << ' '  << targets[0].get_z() << ' '  << targets[1].get_x() << ' '  << targets[1].get_y() << ' '
            << targets[1].get_z() << '\n';  
            out.close(); 
            i++;
        }
        missile.set_controlParams();
        targets[0].set_controlParams();
        targets[1].set_controlParams();
    }

    Target* finalTarget = missile.get_targets()[0];
    vector<double> r_missile = missile.get_r();
    vector<double> r_target = finalTarget->get_r();
    double range_old = range(r_missile, r_target);
    double range_curr = range(r_missile, r_target);

    while(range_curr <= range_old){
        range_old = range_curr;
        //Проверка на достаточность скорости ракеты
        if(0.5 * missile.get_Vabs() < finalTarget->get_Vabs()){
            return {-1};
        }

        missile.STEP(dt);
        finalTarget -> STEP(dt);
        
        t += dt;
        r_missile = missile.get_r();
        r_target = finalTarget ->get_r();
        range_curr = range(r_missile, r_target);

        if(i* 0.01 <= t ){
            out.open(name, ios::app);
            out << setprecision(10) << t << ' ' << missile.get_x() << ' ' << missile.get_y() << ' ' << missile.get_z() << ' ' << targets[0].get_x() << ' ' 
            << targets[0].get_y() << ' '  << targets[0].get_z() << ' '  << targets[1].get_x() << ' '  << targets[1].get_y() << ' '
            << targets[1].get_z() << '\n';  
            out.close(); 
            i++;
        }

        missile.set_controlParams();
        finalTarget -> set_controlParams();
        
    }

    return {1};
}