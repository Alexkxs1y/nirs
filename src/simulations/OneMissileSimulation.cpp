#include <math.h>
#include <vector>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/utils/MyMath.hpp"

using namespace std;

//Возвращает вектор (r, r_x, r_y, r_z, -1/1), r - велична промаха; r_x, r_y, r_z - её проекции на оси; -1/1 - проходим ли мы по критерию конечной скорости (-1 - не проходим) 
vector<double> oneMissileSimulation(Missile* missile, Target* target, double dt){
    missile->set_target(target);
    target->set_pursuer(missile);

    double t = 0;

    vector<double> r_missile = missile->get_r();
    vector<double> r_target = target->get_r();
    double range_old = range(r_missile, r_target);
    double range_curr = range(r_missile, r_target);
    
    while(range_curr <= range_old){
        range_old = range_curr;
        
        //Проверка на достаточность скорости ракеты
        if(0.5 * missile->get_Vabs() < target->get_Vabs()){
            return {0, 0, 0, 0, -1};
        }

        missile -> set_controlParams();
        target -> set_controlParams();
        missile -> STEP(dt);
        target -> STEP(dt);
        
        t += dt;
        r_missile = missile -> get_r();
        r_target = target -> get_r();
        range_curr = range(r_missile, r_target);
    }
    vector<double> r_rel = sub(r_target, r_missile);
    return {range_curr, r_rel[0], r_rel[1], r_rel[2], 1};
}