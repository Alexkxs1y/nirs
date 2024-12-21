#include <math.h>
#include <vector>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/utils/MyMath.hpp"
#include <iostream>

using namespace std;

bool RK_STEP(Missile* missile, Target* target, double dt){
    vector<double> stateVector_missile = missile -> get_stateVector(); 
    vector<double> ryp_missile = missile -> get_ryp();
    vector<double> w_missile = missile -> get_w();
    vector<double> stateVector_target = target -> get_stateVector();
    
    //Первая итерация РК
    missile -> set_controlParams();
    target -> set_controlParams();
    missile -> set_actualForceAndTorques();
    target -> set_actualForceAndTorques();

    vector<double> dr1 = missile -> get_dr_dt();
    vector<double> dV1 = missile -> get_dV_dt();
    vector<double> dryp1 = missile -> get_dryp_dt();
    vector<double> dw1 = missile -> get_dw_dt();
    vector<double> dr1_tar = target -> get_dr_dt();
    vector<double> dV1_tar = target -> get_dV_dt();

    missile -> RigidBody::STEP(dt * 0.5, dr1, dV1, dryp1, dw1);
    target -> PointMass::STEP(dt * 0.5, dr1_tar, dV1_tar);
    
    //Вторая итерация РК
    missile -> set_controlParams();
    target -> set_controlParams();
    missile -> set_actualForceAndTorques();
    target -> set_actualForceAndTorques();

    vector<double> dr2 = missile -> get_dr_dt();
    vector<double> dV2 = missile -> get_dV_dt();
    vector<double> dryp2 = missile -> get_dryp_dt();
    vector<double> dw2 = missile -> get_dw_dt();
    vector<double> dr2_tar = target -> get_dr_dt();
    vector<double> dV2_tar = target -> get_dV_dt();

    //Возвращение Ракеты и цели в начальное состояние и шаг с производной со второго шага
    missile -> set_state(stateVector_missile, ryp_missile, w_missile);
    target -> set_state(stateVector_target);
    missile -> RigidBody::STEP(dt * 0.5, dr2, dV2, dryp2, dw2);
    target -> PointMass::STEP(dt * 0.5, dr2_tar, dV2_tar);

    //Третья итерация РК
    missile -> set_controlParams();
    target -> set_controlParams();
    missile -> set_actualForceAndTorques();
    target -> set_actualForceAndTorques();

    vector<double> dr3 = missile -> get_dr_dt();
    vector<double> dV3 = missile -> get_dV_dt();
    vector<double> dryp3 = missile -> get_dryp_dt();
    vector<double> dw3 = missile -> get_dw_dt();
    vector<double> dr3_tar = target -> get_dr_dt();
    vector<double> dV3_tar = target -> get_dV_dt();

    //Возвращение Ракеты и цели в начальное состояние и шаг с производной с третьего шага
    missile -> set_state(stateVector_missile, ryp_missile, w_missile);
    target -> set_state(stateVector_target);
    missile -> RigidBody::STEP(dt, dr3, dV3, dryp3, dw3);
    target -> PointMass::STEP(dt, dr3_tar, dV3_tar);

    //Четвертая итерация РК
    missile -> set_controlParams();
    target -> set_controlParams();
    missile -> set_actualForceAndTorques();
    target -> set_actualForceAndTorques();

    vector<double> dr4 = missile -> get_dr_dt();
    vector<double> dV4 = missile -> get_dV_dt();
    vector<double> dryp4 = missile -> get_dryp_dt();
    vector<double> dw4 = missile -> get_dw_dt();
    vector<double> dr4_tar = target -> get_dr_dt();
    vector<double> dV4_tar = target -> get_dV_dt();

    //Производные по методу РК
    vector<double> dr(3);
    vector<double> dV(3);
    vector<double> dryp(3);
    vector<double> dw(3);
    vector<double> dr_tar(3);
    vector<double> dV_tar(3);

    for(int i = 0; i < 3; i ++){
        dr[i] = (dr1[i] + 2 * dr2[i] + 2 * dr3[i] + dr4[i]) / 6;
        dV[i] = (dV1[i] + 2 * dV2[i] + 2 * dV3[i] + dV4[i]) / 6;
        dryp[i] = (dryp1[i] + 2 * dryp2[i] + 2 * dryp3[i] + dryp4[i]) / 6;
        dw[i] = (dw1[i] + 2 * dw2[i] + 2 * dw3[i] + dw4[i]) / 6;
        dr_tar[i] = (dr1_tar[i] + 2 * dr2_tar[i] + 2 * dr3_tar[i] + dr4_tar[i]) / 6;
        dV_tar[i] = (dV1_tar[i] + 2 * dV2_tar[i] + 2 * dV3_tar[i] + dV4_tar[i]) / 6;
    }

    //Возвращение Ракеты и цели в начальное состояние и шаг Методом РК
    missile -> set_state(stateVector_missile, ryp_missile, w_missile);
    target -> set_state(stateVector_target);
    missile -> RigidBody::STEP(dt, dr, dV, dryp, dw);
    target -> PointMass::STEP(dt, dr_tar, dV_tar);
    
    return true;
}

//Возвращает вектор (r, r_x, r_y, r_z, -1/1), r - велична промаха; r_x, r_y, r_z - её проекции на оси; -1/1 - проходим ли мы по критерию конечной скорости (-1 - не проходим) 
vector<double> oneMissileSimulation(Missile* missile, Target* target, double dt){
    
    vector<double> stateVector_missile = missile -> get_stateVector(); 
    vector<double> ryp_missile = missile -> get_ryp();
    vector<double> w_missile = missile -> get_w();
    IGuidance* initialMissileGuidance = missile -> get_Guidance();
    vector<double> stateVector_target = target -> get_stateVector();
    vector<Target*> initialMissilesTargets = missile -> get_targets();
    vector<PointMass*> initialTargetPursuers = target -> get_pursuers(); 

    missile -> set_target(target);
    target -> set_pursuer(missile);

    double t = 0;

    vector<double> r_missile = missile->get_r();
    vector<double> r_target = target->get_r();
    double range_old = range(r_missile, r_target);
    double range_curr = range(r_missile, r_target);
    
    while(range_curr <= range_old){
        range_old = range_curr;
        
        //Проверка на достаточность скорости ракеты
        if(0.5 * missile->get_Vabs() < target->get_Vabs()){
            missile -> set_state(stateVector_missile, ryp_missile, w_missile);
            missile -> set_target(initialMissilesTargets);
            target -> set_state(stateVector_target);
            target -> set_pursuer(initialTargetPursuers);
            return {0, 0, 0, 0, -1};
        }

        //Интегрирование Эйлером
        /*
        missile -> set_controlParams();
        target -> set_controlParams();
        missile -> STEP(dt);
        target -> STEP(dt);
        */

        //Интегрирование методом Рунге-Кутта
        RK_STEP(missile, target, dt);
        
        t += dt;
        r_missile = missile -> get_r();
        r_target = target -> get_r();
        range_curr = range(r_missile, r_target);
    }

    missile -> set_state(stateVector_missile, ryp_missile, w_missile);
    missile -> set_target(initialMissilesTargets);
   // missile ->choose_Guidance();
    target -> set_state(stateVector_target);
    target -> set_pursuer(initialTargetPursuers);
  
    vector<double> r_rel = sub(r_target, r_missile);
    return {range_curr, r_rel[0], r_rel[1], r_rel[2], 1};
}