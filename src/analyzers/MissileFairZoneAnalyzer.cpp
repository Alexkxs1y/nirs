#include <math.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <time.h>
#include "omp.h"
#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include "../../include/utils/MyMath.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"
#include <climits>

#define MISSILE_DIR_STEP double(11000) //Длина начального шага для определения границы по направлению 
#define NUM_OF_MNK_POINTS 100 //Число точек, используемых для аппроксимации траектории
#define NUM_FAIR_ZONE_POINTS 120  //Число которое показывает шаг угла при построении допустимой зоны ракеты dAnngle = 2 * M_PI / NUM_FAIR_ZONE_POINTS
#define NUM_SURFACE_POINT 24 //Число которое показываетшаг угла при построении допустимого положения ракеты в плоскости dAnngle = 2 * M_PI / NUM_SURFACE_POINT
#define MAX_Y double(30000)
#define MAX_X double(30000)
#define GRID_STEP double(1000)
#define SHAPE_GRID_W 101
#define SHAPE_GRID_H 101

using namespace std;


vector<double> directionBound(Missile* missile, Target* target, double _yaw, double _pitch, double effectiveRadius, double tolerance, double dt){
    
    double step = MISSILE_DIR_STEP;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_ryp_initial = missile -> get_ryp();
    vector<double> missile_w_initial = missile -> get_w();
    vector<double> missile_stateVector_initial = missile_stateVector;
    
    vector<double> flightRes(5);
    double missDistanse = 0;
    vector<double> cos_xyz = { cos(_pitch) * cos(_yaw), sin(_pitch), cos(_pitch) * sin(_yaw) };  
    bool isStepBack = false;
    bool inAir = true;
    bool beforeTarget = true;
    while(abs(missDistanse - effectiveRadius) > tolerance){
        if(missDistanse < effectiveRadius){
            if(isStepBack){
                step *= 0.5;
            }
            if(step < 10) break; //Если совсем маленький шаг уже, то останавливаемся.
            //Добавление шага по направляющим косинусам ко всем координатам начального положения            
            for(int i = 0; i < cos_xyz.size(); i++){
                missile_stateVector[i] += step * cos_xyz[i];
            }
            if(missile_stateVector[1] < 0){
                double _step = missile_stateVector[1] / cos_xyz[1];
                for(int i = 0; i < cos_xyz.size(); i++){
                    missile_stateVector[i] -= _step * cos_xyz[i];
                }
                inAir = false;   
            }
        } else {
            step *= 0.5;
            for(int i = 0; i < cos_xyz.size(); i++){
                missile_stateVector[i] -= step * cos_xyz[i];
            }
            isStepBack = true;                    
        }
        if(missile_stateVector[0] >= target -> get_stateVector()[0]){
            missile_stateVector[0] = target -> get_stateVector()[0] - 0.1;
            beforeTarget = false;
        }
        missile -> set_state(missile_stateVector, missile_ryp_initial, missile_w_initial);
        flightRes = oneMissileSimulation(missile, target, dt);
        missDistanse = flightRes[0];
        if(flightRes[4] < 0){
            missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
        }
        if(!beforeTarget && missDistanse < effectiveRadius) break;
        if(!inAir && missDistanse < effectiveRadius) break;
        inAir = true;
    }

    //cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_ryp_initial, missile_w_initial);

    vector<double> bound(3);
    for(size_t i = 0; i < bound.size(); i++){
        bound[i] = missile_stateVector[i];
    }

    return bound;
}


vector< vector<double> > missileFairZone(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt){
    int numPoints = NUM_FAIR_ZONE_POINTS;

    //Создание файла для записи плоскости не ухода
    ofstream out;          
    string name = "res.dat";  

    vector<double> flyghtRes = oneMissileSimulation(missile, target, dt);
    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes[0] > effectiveRadius || flyghtRes[4] < 0){    
        return { {-1, -1} };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound(3);
    vector< vector<double> > missileFairZone(0);

    bound = directionBound(missile, target, _yaw, 0 * M_PI * 0.5, effectiveRadius, tolerance, dt);
    missileFairZone.push_back(bound);
    out.open(name, ios::app);
    out << bound[0] << ' ' << bound[1] << ' ' << bound[2] << ' ' << '\n';
    out << '\n';
    out.close();
    for(int i = 1; i < int(numPoints * 0.5); i ++){
        for(int j = 0; j < numPoints; j ++){
            _yaw = 2 * double(j) * M_PI / double(numPoints);
            _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
            bound = directionBound(missile, target, _yaw, _pitch, effectiveRadius, tolerance, dt);
            missileFairZone.push_back(bound);
            out.open(name, ios::app);
            out << bound[0] << ' ' << bound[1] << ' ' << bound[2] << ' ' << '\n';
            out.close();
        }
        out.open(name, ios::app);
        out << '\n';
        out.close();
    }

    bound = directionBound(missile, target, 0, - M_PI * 0.5, effectiveRadius, tolerance, dt);
    missileFairZone.push_back(bound);
    out.open(name, ios::app);
    out << bound[0] << ' ' << bound[1] << ' ' << bound[2] << ' ' << '\n';
    out << '\n';
    out.close();

    return missileFairZone;
}


vector< vector<double> > crossTargetMissileFairZone(Missile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double dt){
    
    ofstream out;          
    string name = "crossZone_"+ to_string(int(missile->get_x())) + ".dat";  
    
    int numPoints = NUM_FAIR_ZONE_POINTS;

    vector<double> missileState = missile -> get_stateVector();
    vector<double> missileR = {missileState[0], missileState[1], missileState[2]};

    vector<double> flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
    vector<double> flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);

    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes_1[0] > effectiveRadius || flyghtRes_1[4] < 0){    
        return { {-1, 0 } };        
    }

    if(flyghtRes_2[0] > effectiveRadius || flyghtRes_2[4] < 0){    
        return { { 0 , -1 } };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound_1(3);
    vector<double> bound_2(3);
    vector< vector<double> > crossTargetMissileFairZone(0);
    cout << "ШАГ ПО ВРЕМЕНИ ПРИ АНАЛИЗУ: " << dt << '\n';
    bound_1 = directionBound(missile, target_1, _yaw, M_PI * 0.5, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, M_PI * 0.5, effectiveRadius, tolerance, dt);
    
    if(range(missileR, bound_1) < range(missileR, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
    }   
    omp_set_num_threads(omp_get_max_threads());
    double OMPtime = omp_get_wtime();
    #pragma omp parallel private(_yaw, _pitch, bound_1, bound_2)
    {   
        cout << omp_get_num_threads() << '\n';
        Missile missile_loc(*missile);
        Target target1_loc(*target_1);
        Target target2_loc(*target_2);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        target2_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        missile_loc.set_target({&target1_loc, &target2_loc});
        #pragma omp for schedule(dynamic)
        for(int i = 1; i < int(numPoints * 0.5); i ++){
            for(int j = 0; j <= int(numPoints * 0.5); j ++){
                _yaw =  M_PI * 0.5 - 2 * double(j) * M_PI / double(numPoints);
                _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
                bound_1 = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, effectiveRadius, tolerance, dt);
                bound_2 = directionBound(&missile_loc, &target2_loc, _yaw, _pitch, effectiveRadius, tolerance, dt);
                if(range(missileR, bound_1) < range(missileR, bound_2)){
                    #pragma omp critical
                        crossTargetMissileFairZone.push_back(bound_1);
                } else {
                    #pragma omp critical
                        crossTargetMissileFairZone.push_back(bound_2);
                }
            }
        }
    }
    cout << "ВРЕМЯ НА ЗОНУ" << omp_get_wtime() - OMPtime << '\n';

    bound_1 = directionBound(missile, target_1, _yaw, - M_PI * 0.5, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, - M_PI * 0.5, effectiveRadius, tolerance, dt);
    
    if(range(missileR, bound_1) < range(missileR, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
    }
    
    out.open(name, ios::app);
    for(size_t i = 0; i < crossTargetMissileFairZone.size(); i ++){
            out << crossTargetMissileFairZone[i][0] << ' ' << crossTargetMissileFairZone[i][1] << ' ' << crossTargetMissileFairZone[i][2] << ' ' << '\n';
    }
    out.close();

    return crossTargetMissileFairZone;     
}


//Определяет границу допустимой зоны, начинает в заданной точке и шагает в заданном направлении
vector<double> pointDirectionBound( Missile* missile, Target* target, double effectiveRadius,
                                    double tolerance, vector<double>& point ,vector<double>& searchDirection, double dt){
    double step = MISSILE_DIR_STEP;
    vector<double> missile_stateVector = missile -> get_stateVector();
    for(size_t i = 0; i < point.size(); i ++){
        missile_stateVector[i] = point[i];
    }
    vector<double> missile_ryp_initial = missile -> get_ryp();
    vector<double> missile_w_initial = missile -> get_w();
    vector<double> missile_stateVector_initial = missile -> get_stateVector();
    
    vector<double> flightRes(5);
    double missDistanse = 0; 
    bool isStepBack = false;
    bool inAir = true;
    bool beforeTarget = true;
    while(abs(missDistanse - effectiveRadius) > tolerance){
        if(missDistanse < effectiveRadius){
            if(isStepBack){
                step *= 0.5;
            }
            if(step < 10) break; //Если совсем маленький шаг уже, то останавливаемся.
            //Добавление шага по направляющим косинусам ко всем координатам начального положения            
            for(int i = 0; i < searchDirection.size(); i++){
                missile_stateVector[i] += step * searchDirection[i];
            }
            if(missile_stateVector[1] < 0){
                double _step = missile_stateVector[1] / searchDirection[1];
                for(int i = 0; i < searchDirection.size(); i++){
                    missile_stateVector[i] -= _step * searchDirection[i];
                }
                inAir = false;   
            }
        } else {
            step *= 0.5;
            for(int i = 0; i < searchDirection.size(); i++){
                missile_stateVector[i] -= step * searchDirection[i];
            }
            isStepBack = true;                    
        }
        if(missile_stateVector[0] >= target -> get_stateVector()[0]){
            missile_stateVector[0] = target -> get_stateVector()[0] - 0.1;
            beforeTarget = false;
        }
        missile -> set_state(missile_stateVector, missile_ryp_initial, missile_w_initial);
        flightRes = oneMissileSimulation(missile, target, dt);
        missDistanse = flightRes[0];
        if(flightRes[4] < 0){
            missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
        }
        if(!beforeTarget && missDistanse < effectiveRadius) break;
        if(!inAir && missDistanse < effectiveRadius) break;
        inAir = true;
    }

    //cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_ryp_initial, missile_w_initial);

    vector<double> bound(3);
    for(size_t i = 0; i < bound.size(); i++){
        bound[i] = missile_stateVector[i];
    }

    return bound;
}


vector< vector<double> > perpendToVectorFairSurface(    Missile* missile, Target* target_1, Target* target_2, double effectiveRadius,
                                                        double tolerance, vector<double>& direction, double step, double dt){
    ofstream out;          
    string name = "perpend_"  +  to_string(int(missile->get_x())) + "_" + to_string(int(step))  +".dat";

    int numPoints = NUM_SURFACE_POINT;

    vector< vector<double> > fairSurface(numPoints);
    vector<double> missileState = missile -> get_stateVector();
    vector<double> hitPoint(3);

    for(size_t i = 0; i < hitPoint.size(); i++){
        hitPoint[i] = missileState[i] +  step * direction[i];
    }

    //Вектор перпендикулярный к направлению шагов
    vector<double> normToDirection(3);
    if(direction[2] != 0){
        double z = -(direction[0] + direction[1]) / direction[2];
        normToDirection = {1, 1, z};
    } else {
        if(direction[1] != 0){
            double y = -(direction[0] + direction[2]) / direction[1];
            normToDirection = {1, y, 1};
        } else {
            double x = -(direction[1] + direction[2]) / direction[0];
            normToDirection = {x, 1, 1};
        }
    }
    
    normalize(normToDirection);

    double searchAngle = 0;
    vector<double> searchDirection(3);
    vector<double> bound_1(3);
    vector<double> bound_2(3);

    omp_set_num_threads(omp_get_max_threads());
    double OMPtime = omp_get_wtime();
    #pragma omp parallel private(bound_1, bound_2, searchAngle, searchDirection)
    {
        Missile missile_loc(*missile);
        Target target1_loc(*target_1);
        Target target2_loc(*target_2);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        target2_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        missile_loc.set_target({&target1_loc, &target2_loc});

        #pragma omp for schedule(dynamic)
        for(size_t i = 0; i < numPoints; i ++){
            searchAngle = 2 * M_PI * double(i) / double(numPoints);
            searchDirection = rotate(normToDirection, direction, searchAngle);
            bound_1 = pointDirectionBound(&missile_loc, &target1_loc, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
            bound_2 = pointDirectionBound(&missile_loc, &target2_loc, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
            if(range(hitPoint, bound_1) < range(hitPoint, bound_2)){
                #pragma omp critical
                    fairSurface[i] = bound_1; 
            } else {
                #pragma omp critical
                    fairSurface[i] = bound_2;
            }
        }
    }

    cout << "ВРЕМЯ НА ПЕРПЕНДИКУЛЯРНУЮ ПОВЕРХНОСТЬ: " << omp_get_wtime() - OMPtime << '\n';
    out.open(name, ios::app);
    for(size_t i = 0; i < numPoints; i++){
        out << fairSurface[i][0] << ' ' << fairSurface[i][1] << ' ' << fairSurface[i][2] << ' ' << '\n';
    }
    out.close();

    return fairSurface;
}


vector< vector<double> > fairTrajectoryPoints(Missile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt){
    ofstream out;          
    string name = "fairPoints_" + to_string(int(missile->get_x()))+ ".dat";
    
    vector< vector<double> > fairTrajectoryPoints(0);
    
    int nMNK = NUM_OF_MNK_POINTS; //Количество точек, используемых для прогноза траектории

    vector< vector<double> > crossTargetFairZone = crossTargetMissileFairZone(missile, target_1, target_2, effectiveRadius, tolerance, dt);
    
    if(crossTargetFairZone[0][0] == -1 ){
        return {{-1,0}};
    }

    if(crossTargetFairZone[0][1] == -1){
        return {{0,-1}};
    }

    //Определение ближайщей к целям точки области возможных положений
    vector<double> target_1R = target_1 -> get_stateVector();
    vector<double> target_2R = target_2 -> get_stateVector();
    target_1R.resize(3);
    target_2R.resize(3);
    vector<double> lastPoint = nearestPointFromSample(target_1R, target_2R, crossTargetFairZone);

    cout << "Ближайщая точка: " << lastPoint[0] << ' ' << lastPoint[1] << ' ' << lastPoint[2] << '\n';

    vector<double> missileState = missile -> get_stateVector();
    
    //Вектор от ракеты до ближайшей к целям точке, принадежащей поверхности допустимой зоны положения ракеты.
    vector<double> direction(3);
    for(size_t i = 0; i < direction.size(); i ++){
        direction[i] = lastPoint[i] - missileState[i];
    }
    normalize(direction);

    cout << "Направление построений плоскостей: " << direction[0] << ' ' << direction[1] << ' ' << direction[2] << '\n';

    //Определение максимальной дальности полётаза время постоянства траектории. 
    double maxLength = (missile -> get_Vabs()) * reGuidanceTime;
    double step = 0;

    cout << "Максимальная длина построения: " << maxLength << '\n';

    //Создание переменной, куда будет суваться плоскость поражения перпендикулярная направлению полёта
    vector< vector<double> > fairSurface(0);
    vector<double> fairPoint(3);
    
    for(size_t i = 0; i < nMNK; i ++){
        step = maxLength * double(i) / double(nMNK);
        fairSurface = perpendToVectorFairSurface(missile, target_1, target_2, effectiveRadius, tolerance, direction, step, dt);
        fairPoint = findFarthestPointInPlane(fairSurface, fairSurface[0], direction);
        fairTrajectoryPoints.push_back(fairPoint);
        out.open(name, ios::app);
        out << fairPoint[0] << ' ' << fairPoint[1] << ' ' << fairPoint[2] << ' ' << '\n';
        out.close();
    }
    return fairTrajectoryPoints;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////ПЕРЕГРУЗКА ДЛЯ APERIOD MISSILE///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*vector<double> directionBound(AperiodMissile* missile, Target* target, double _yaw, double _pitch, double effectiveRadius, double tolerance, double dt){
    
    double step = MISSILE_DIR_STEP;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    vector<double> missile_stateVector_initial = missile_stateVector;
    
    vector<double> flightRes(6);
    double missDistanse = 0;
    vector<double> cos_xyz = { cos(_pitch) * cos(_yaw),  sin(_pitch) * cos(_yaw), sin(_yaw) };  
    bool isStepBack = false;
    bool inAir = true;
    bool beforeTarget = true;
    while(abs(missDistanse - effectiveRadius) > tolerance){
        if(missDistanse < effectiveRadius){
            if(isStepBack){
                step *= 0.5;
            }
            if(step < 10) break; //Если совсем маленький шаг уже, то останавливаемся.
            //Добавление шага по направляющим косинусам ко всем координатам начального положения            
            for(int i = 0; i < cos_xyz.size(); i++){
                missile_stateVector[i] += step * cos_xyz[i];
            }
            if(missile_stateVector[1] < 0){
                double _step = missile_stateVector[1] / cos_xyz[1];
                for(int i = 0; i < cos_xyz.size(); i++){
                    missile_stateVector[i] -= _step * cos_xyz[i];
                }
                inAir = false;   
            }
        } else {
            step *= 0.5;
            for(int i = 0; i < cos_xyz.size(); i++){
                missile_stateVector[i] -= step * cos_xyz[i];
            }
            isStepBack = true;                    
        }
        if(missile_stateVector[0] >= target -> get_stateVector()[0]){
            missile_stateVector[0] = target -> get_stateVector()[0] - 0.1;
            beforeTarget = false;
        }
        missile -> set_state(missile_stateVector, missile_n_xyz_body);
        flightRes = oneMissileSimulation(missile, target, dt);
        missDistanse = flightRes[0];
        if(flightRes[4] < 0 || flightRes[5] < 0){
            missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
        }
        if(!beforeTarget && missDistanse < effectiveRadius) break;
        if(!inAir && missDistanse < effectiveRadius) break;
        inAir = true;
    }

    cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_n_xyz_body);

    vector<double> bound(3);
    for(size_t i = 0; i < bound.size(); i++){
        bound[i] = missile_stateVector[i];
    }

    return bound;
}*/


vector<double> directionBound(AperiodMissile* missile, Target* target, double _yaw, double _pitch, double top, double bottom, double effectiveRadius, double tolerance, double dt){
    
    double step = 200000;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    vector<double> missile_stateVector_initial = missile_stateVector;
    //_pitch = - 0.5 * M_PI;
    vector<double> flightRes(6);
    vector<double> cos_xyz = { cos(_pitch) * cos(_yaw),  sin(_pitch) * cos(_yaw), sin(_yaw) };  
    vector<double> left(3);
    vector<double> right(3);
    vector<double> med(3);
    
    double step_up;
    double step_down;
    if(top > numeric_limits<double>::max() - 1) step_up = -1;
    if(bottom == 0) step_down = -1;

    if(step_up != -1){
        step_up = (top - missile_stateVector[1]) / cos_xyz[1];
        if(step_up > 0){
            for(size_t i = 0; i < 3; i++){
                missile_stateVector[i] += step_up * cos_xyz[i];
            }
            missile -> set_state(missile_stateVector, missile_n_xyz_body);
            flightRes = oneMissileSimulation(missile, target, dt);
            if(flightRes[0] > effectiveRadius || flightRes[4] < 0 || flightRes[5] < 0) step = step_up;
        }
    }

    if(step_down != -1){
        step_down = (bottom - missile_stateVector[1]) / cos_xyz[1];
        if(step_down > 0){
            for(size_t i = 0; i < 3; i++){
                missile_stateVector[i] += step_down * cos_xyz[i];
            }
            missile -> set_state(missile_stateVector, missile_n_xyz_body);
            flightRes = oneMissileSimulation(missile, target, dt);
            if(flightRes[0] > effectiveRadius || flightRes[4] < 0 || flightRes[5] < 0) step = step_down;
        }
    }


    for(size_t i = 0; i < 3; i++){
        left[i] = missile_stateVector[i];
        right[i] = left[i] + step * cos_xyz[i];
    }
    while(range(left, right) > effectiveRadius){
        for(size_t i = 0; i < 3; i++){
            med[i] = 0.5 * (left[i] + right[i]);
            missile_stateVector[i] = med[i];
        }
        if(med[1] < 0){
            double back_step = med[1] / cos_xyz[1];
            for(size_t i = 0; i < 3; i++){
                med[i] -= back_step * cos_xyz[i];
                missile_stateVector[i] = med[i];
            }
        }
        missile -> set_state(missile_stateVector, missile_n_xyz_body);
        flightRes = oneMissileSimulation(missile, target, dt);
        if(flightRes[0] > effectiveRadius || flightRes[4] < 0 || flightRes[5] < 0){
            right = med;
        } else {
                if(abs(med[1]) < 1) break;
                left = med;
        }
    }

    cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_n_xyz_body);

    vector<double> bound(3);
    for(size_t i = 0; i < bound.size(); i++){
        bound[i] = missile_stateVector[i];
    }

    return bound;
}


vector< vector<double> > missileFairZone(AperiodMissile* missile, Target* target, double effectiveRadius, double tolerance, double dt){
    int numPoints = NUM_FAIR_ZONE_POINTS;

    //Создание файла для записи плоскости не ухода
    ofstream out;          
    string name = "target_new"+ to_string(int(target->get_x())) + ".dat";  

    vector<double> flyghtRes = oneMissileSimulation(missile, target, dt);
    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes[0] > effectiveRadius || flyghtRes[4] < 0){    
        return { {-1, -1} };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound(3);
    vector< vector<double> > missileFairZone(0);

    bound = directionBound(missile, target, _yaw, 0 * M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    missileFairZone.push_back(bound);
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel private(_yaw, _pitch, bound)
    {   
        cout << omp_get_num_threads() << '\n';
        AperiodMissile missile_loc(*missile);
        Target target1_loc(*target);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        vector<Target*> tags = {&target1_loc};
        missile_loc.set_target(tags);
        #pragma omp for schedule(dynamic)
        for(int i = 1; i < int(numPoints * 0.5); i ++){
            for(int j = 0; j < int(numPoints * 0.5); j ++){
                _yaw = M_PI * 0.5 - 2 * double(j) * M_PI / double(numPoints);
                _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
                bound = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
                #pragma critical
                    missileFairZone.push_back(bound);
            }
        }
    }

    bound = directionBound(missile, target, 0, - M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    missileFairZone.push_back(bound);
    out.open(name, ios::app);
    for(size_t i = 0; i < missileFairZone.size(); i++){
        out << missileFairZone[i][0] << ' ' << missileFairZone[i][1] << ' ' << missileFairZone[i][2] << ' ' << '\n';
    }
    out << '\n';
    out.close();

    return missileFairZone;
}


vector< vector<double> > crossTargetMissileFairZone(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double dt){
    
    ofstream out;          
    string name = "crossSurf_"+ to_string(int(missile->get_x())) + ".dat";  
    
    int numPoints = NUM_FAIR_ZONE_POINTS;

    vector<double> missileState = missile -> get_stateVector();
    vector<double> missileR = {missileState[0], missileState[1], missileState[2]};

    vector<double> flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
    vector<double> flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);

    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes_1[0] > effectiveRadius || flyghtRes_1[4] < 0){    
        return { {-1, 0 } };        
    }

    if(flyghtRes_2[0] > effectiveRadius || flyghtRes_2[4] < 0){    
        return { { 0 , -1 } };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound_1(3);
    vector<double> bound_2(3);
    vector< vector<double> > crossTargetMissileFairZone(0);
    cout << "ШАГ ПО ВРЕМЕНИ ПРИ АНАЛИЗУ: " << dt << '\n';
    bound_1 = directionBound(missile, target_1, _yaw, M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    
    if(range(missileR, bound_1) < range(missileR, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
    }   
    omp_set_num_threads(omp_get_max_threads());
    double OMPtime = omp_get_wtime();
    #pragma omp parallel private(_yaw, _pitch, bound_1, bound_2)
    {   
        cout << omp_get_num_threads() << '\n';
        AperiodMissile missile_loc(*missile);
        Target target1_loc(*target_1);
        Target target2_loc(*target_2);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        target2_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        vector<Target*> tags = {&target1_loc, &target2_loc};
        missile_loc.set_target(tags);
        #pragma omp for schedule(dynamic)
        for(int i = 1; i < int(numPoints * 0.5); i ++){
            for(int j = 0; j <= int(numPoints * 0.5); j ++){
                _yaw =  M_PI * 0.5 - 2 * double(j) * M_PI / double(numPoints);
                _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
                bound_1 = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
                bound_2 = directionBound(&missile_loc, &target2_loc, _yaw, _pitch, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
                if(range(missileR, bound_1) < range(missileR, bound_2)){
                    #pragma omp critical
                        crossTargetMissileFairZone.push_back(bound_1);
                } else {
                    #pragma omp critical
                        crossTargetMissileFairZone.push_back(bound_2);
                }
            }
        }
    }
    cout << "ВРЕМЯ НА ЗОНУ" << omp_get_wtime() - OMPtime << '\n';

    bound_1 = directionBound(missile, target_1, _yaw, - M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, - M_PI * 0.5, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
    
    if(range(missileR, bound_1) < range(missileR, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
    }
    
    out.open(name, ios::app);
    for(size_t i = 0; i < crossTargetMissileFairZone.size(); i ++){
            out << crossTargetMissileFairZone[i][0] << ' ' << crossTargetMissileFairZone[i][1] << ' ' << crossTargetMissileFairZone[i][2] << ' ' << '\n';
    }
    out.close();

    return crossTargetMissileFairZone;     
}


vector< vector<double> > crossTargetMissileFairSurf(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double dt){
    
    ofstream out;          
    string name = "crossSurf"+ to_string(int(missile->get_x())) + ".dat";  
    
    int numPoints = NUM_FAIR_ZONE_POINTS;

    vector<double> missileState = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    vector<double> missileR = {missileState[0], missileState[1], missileState[2]};

    vector<double> flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
    vector<double> flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);

    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes_1[0] > effectiveRadius || flyghtRes_1[4] < 0 || flyghtRes_1[5] < 0){    
        return { {-1, 0 } };        
    }

    if(flyghtRes_2[0] > effectiveRadius || flyghtRes_2[4] < 0 || flyghtRes_2[5] < 0){    
        return { { 0 , -1 } };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound_1(3);
    vector<double> bound_2(3);
    vector< vector<double> > crossTargetMissileFairZone(3 *int(numPoints * 0.5) - 1);
    vector<double> lowest(3);
    vector<double> highest(3);
    vector<double> denPoint(3);

    omp_set_num_threads(omp_get_max_threads());
    double OMPtime = omp_get_wtime();
    #pragma omp parallel private(_yaw, _pitch, bound_1, bound_2) 
    {   
        cout << omp_get_num_threads() << '\n';
        AperiodMissile missile_loc(*missile);
        Target target1_loc(*target_1);
        Target target2_loc(*target_2);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        target2_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        vector<Target*> tags = {&target1_loc, &target2_loc};
        missile_loc.set_target(tags);
        #pragma omp for schedule(dynamic)
        for(int i = 0; i <= int(numPoints * 0.5); i ++){
            _yaw =  0;
            _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
            bound_1 = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
            bound_2 = directionBound(&missile_loc, &target2_loc, _yaw, _pitch, numeric_limits<double>::max(), 0, effectiveRadius, tolerance, dt);
            if(range(missileR, bound_1) < range(missileR, bound_2)){
                #pragma omp critical
                    crossTargetMissileFairZone[i] = bound_1;
            } else {
                #pragma omp critical
                    crossTargetMissileFairZone[i] = bound_2;
            }
        }

        #pragma omp master
        {   
            denPoint = findDensestPoint(crossTargetMissileFairZone, int(numPoints * 0.5));
            cout << "ПЛОТНАЯ ТОЧКА: " << denPoint[1] << '\n';
            highest = crossTargetMissileFairZone[0];
            lowest = crossTargetMissileFairZone[int(numPoints * 0.5)];
            for(size_t i = 0; i < 3; i ++){
                missileState[i] = highest[i];
            }
            if(missileState[1] - 100 > missileR[1])
                missileState[1] -= 100;
        }
        #pragma omp barrier

        missile_loc.set_state(missileState, missile_n_xyz_body);

        #pragma omp for schedule(dynamic)
        for(int i = 1; i < int(numPoints * 0.5); i ++){
            _yaw =  0;
            _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
            bound_1 = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, numeric_limits<double>::max(), denPoint[1], effectiveRadius, tolerance, dt);
            bound_2 = directionBound(&missile_loc, &target2_loc, _yaw, _pitch, numeric_limits<double>::max(), denPoint[1], effectiveRadius, tolerance, dt);
            if(range(missileR, bound_1) < range(missileR, bound_2)){
                #pragma omp critical
                    crossTargetMissileFairZone[i + int(numPoints * 0.5)] = bound_1;
            } else {
                #pragma omp critical
                    crossTargetMissileFairZone[i + int(numPoints * 0.5)] = bound_2;
            }
        }

        #pragma omp master
        {
            for(size_t i = 0; i < 3; i ++){
                missileState[i] = lowest[i];
            }
            if(missileState[1] + 100 < missileR[1])
                missileState[1] += 100;
        }
        #pragma omp barrier

        missile_loc.set_state(missileState, missile_n_xyz_body);

        #pragma omp for schedule(dynamic)
        for(int i = 1; i < int(numPoints * 0.5); i ++){
            _yaw =  0;
            _pitch = - M_PI * 0.5 + 2 * double(i) * M_PI / double(numPoints);
            bound_1 = directionBound(&missile_loc, &target1_loc, _yaw, _pitch, denPoint[1], 0, effectiveRadius, tolerance, dt);
            bound_2 = directionBound(&missile_loc, &target2_loc, _yaw, _pitch, denPoint[1], 0, effectiveRadius, tolerance, dt);
            if(range(missileR, bound_1) < range(missileR, bound_2)){
                #pragma omp critical
                    crossTargetMissileFairZone[i + 2 * int(numPoints * 0.5) - 1] = bound_1;
            } else {
                #pragma omp critical
                    crossTargetMissileFairZone[i + 2 * int(numPoints * 0.5) - 1] = bound_2;
            }
        }

    }
    cout << "ВРЕМЯ НА ЗОНУ" << omp_get_wtime() - OMPtime << '\n';

    out.open(name, ios::app);
    for(size_t i = 0; i < 3 *( int(numPoints * 0.5) - 1); i ++){
            out << crossTargetMissileFairZone[i][0] << ' ' << crossTargetMissileFairZone[i][1] << ' ' << crossTargetMissileFairZone[i][2] << ' ' << '\n';
    }
    out.close();

    return crossTargetMissileFairZone;     
}


//Определяет границу допустимой зоны, начинает в заданной точке и шагает в заданном направлении
vector<double> pointDirectionBound( AperiodMissile* missile, Target* target, double effectiveRadius,
                                    double tolerance, vector<double>& point ,vector<double>& searchDirection, double dt){
    double step = MISSILE_DIR_STEP;
    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    for(size_t i = 0; i < point.size(); i ++){
        missile_stateVector[i] = point[i];
    }
    vector<double> missile_stateVector_initial = missile -> get_stateVector();
    
    vector<double> flightRes(5);
    double missDistanse = 0; 
    bool isStepBack = false;
    bool inAir = true;
    bool beforeTarget = true;
    while(abs(missDistanse - effectiveRadius) > tolerance){
        if(missDistanse < effectiveRadius){
            if(isStepBack){
                step *= 0.5;
            }
            if(step < 10) break; //Если совсем маленький шаг уже, то останавливаемся.
            //Добавление шага по направляющим косинусам ко всем координатам начального положения            
            for(int i = 0; i < searchDirection.size(); i++){
                missile_stateVector[i] += step * searchDirection[i];
            }
            if(missile_stateVector[1] < 0){
                double _step = missile_stateVector[1] / searchDirection[1];
                for(int i = 0; i < searchDirection.size(); i++){
                    missile_stateVector[i] -= _step * searchDirection[i];
                }
                inAir = false;   
            }
        } else {
            step *= 0.5;
            for(int i = 0; i < searchDirection.size(); i++){
                missile_stateVector[i] -= step * searchDirection[i];
            }
            isStepBack = true;                    
        }
        if(missile_stateVector[0] >= target -> get_stateVector()[0]){
            missile_stateVector[0] = target -> get_stateVector()[0] - 0.1;
            beforeTarget = false;
        }
        missile -> set_state(missile_stateVector, missile_n_xyz_body);
        flightRes = oneMissileSimulation(missile, target, dt);
        missDistanse = flightRes[0];
        if(flightRes[4] < 0){
            missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
        }
        if(!beforeTarget && missDistanse < effectiveRadius) break;
        if(!inAir && missDistanse < effectiveRadius) break;
        inAir = true;
    }

    //cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_n_xyz_body);

    vector<double> bound(3);
    for(size_t i = 0; i < bound.size(); i++){
        bound[i] = missile_stateVector[i];
    }

    return bound;
}


vector< vector<double> > perpendToVectorFairSurface(    AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius,
                                                        double tolerance, vector<double>& direction, double step, double dt){
    ofstream out;          
    string name = "perpend_"  +  to_string(int(missile->get_x())) + "_" + to_string(int(step))  +".dat";

    int numPoints = NUM_SURFACE_POINT;

    vector< vector<double> > fairSurface(numPoints);
    vector<double> missileState = missile -> get_stateVector();
    vector<double> hitPoint(3);

    for(size_t i = 0; i < hitPoint.size(); i++){
        hitPoint[i] = missileState[i] +  step * direction[i];
    }

    //Вектор перпендикулярный к направлению шагов
    vector<double> normToDirection(3);
    if(direction[2] != 0){
        double z = -(direction[0] + direction[1]) / direction[2];
        normToDirection = {1, 1, z};
    } else {
        if(direction[1] != 0){
            double y = -(direction[0] + direction[2]) / direction[1];
            normToDirection = {1, y, 1};
        } else {
            double x = -(direction[1] + direction[2]) / direction[0];
            normToDirection = {x, 1, 1};
        }
    }
    
    normalize(normToDirection);

    double searchAngle = 0;
    vector<double> searchDirection(3);
    vector<double> bound_1(3);
    vector<double> bound_2(3);

    omp_set_num_threads(omp_get_max_threads());
    double OMPtime = omp_get_wtime();
    #pragma omp parallel private(bound_1, bound_2, searchAngle, searchDirection)
    {
        AperiodMissile missile_loc(*missile);
        Target target1_loc(*target_1);
        Target target2_loc(*target_2);
        TargetGuidance tg;
        target1_loc.set_Guidance(&tg);
        target2_loc.set_Guidance(&tg);
        vector<double> K_guidance = {5, 5};
        MissileGuidance mg;
        mg.init(K_guidance);
        missile_loc.set_propGuidance(&mg);
        vector<Target*> tags = {&target1_loc, &target2_loc};
        missile_loc.set_target(tags);

        #pragma omp for schedule(dynamic)
        for(size_t i = 0; i < numPoints; i ++){
            searchAngle = 2 * M_PI * double(i) / double(numPoints);
            searchDirection = rotate(normToDirection, direction, searchAngle);
            bound_1 = pointDirectionBound(&missile_loc, &target1_loc, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
            bound_2 = pointDirectionBound(&missile_loc, &target2_loc, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
            if(range(hitPoint, bound_1) < range(hitPoint, bound_2)){
                #pragma omp critical
                    fairSurface[i] = bound_1; 
            } else {
                #pragma omp critical
                    fairSurface[i] = bound_2;
            }
        }
    }

    cout << "ВРЕМЯ НА ПЕРПЕНДИКУЛЯРНУЮ ПОВЕРХНОСТЬ: " << omp_get_wtime() - OMPtime << '\n';
    out.open(name, ios::app);
    for(size_t i = 0; i < numPoints; i++){
        out << fairSurface[i][0] << ' ' << fairSurface[i][1] << ' ' << fairSurface[i][2] << ' ' << '\n';
    }
    out.close();

    return fairSurface;
}


vector< vector<double> > perpendToVectorFairLine(    AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius,
                                                        double tolerance, vector<double>& direction, double step, double dt){
    ofstream out;          
    string name = "line_"  +  to_string(int(missile->get_x())) + "_" + to_string(int(step))  +".dat";

    int numPoints = NUM_SURFACE_POINT;

    vector< vector<double> > fairSurface(2);
    vector<double> missileState = missile -> get_stateVector();
    vector<double> hitPoint(3);

    for(size_t i = 0; i < hitPoint.size(); i++){
        hitPoint[i] = missileState[i] +  step * direction[i];
    }

    //Вектор перпендикулярный к направлению шагов
    vector<double> normToDirection = {-direction[1], direction[0], 0};
    
    normalize(normToDirection);

    vector<double> bound_1(3);
    vector<double> bound_2(3);

    bound_1 = pointDirectionBound(missile, target_1, effectiveRadius, tolerance, hitPoint, normToDirection, dt);
    bound_2 = pointDirectionBound(missile, target_2, effectiveRadius, tolerance, hitPoint, normToDirection, dt);
    if(range(hitPoint, bound_1) < range(hitPoint, bound_2)){
            fairSurface[0] = bound_1; 
    } else {
            fairSurface[0] = bound_2;
    }

    normToDirection = {direction[1], -direction[0], 0};
    
    normalize(normToDirection);

    bound_1 = pointDirectionBound(missile, target_1, effectiveRadius, tolerance, hitPoint, normToDirection, dt);
    bound_2 = pointDirectionBound(missile, target_2, effectiveRadius, tolerance, hitPoint, normToDirection, dt);
    if(range(hitPoint, bound_1) < range(hitPoint, bound_2)){
            fairSurface[1] = bound_1; 
    } else {
            fairSurface[1] = bound_2;
    }

    out.open(name, ios::app);
    for(size_t i = 0; i < fairSurface.size(); i++){
        out << fairSurface[i][0] << ' ' << fairSurface[i][1] << ' ' << fairSurface[i][2] << ' ' << '\n';
    }
    out.close();

    return fairSurface;
}


/*vector< vector<double> > fairTrajectoryPoints(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt){
    ofstream out;          
    string name = "fairPoints_" + to_string(int(missile->get_x()))+ ".dat";
    
    vector< vector<double> > fairTrajectoryPoints(0);
    
    int nMNK = NUM_OF_MNK_POINTS; //Количество точек, используемых для прогноза траектории

    vector< vector<double> > crossTargetFairZone = crossTargetMissileFairZone(missile, target_1, target_2, effectiveRadius, tolerance, dt);
    
    if(crossTargetFairZone[0][0] == -1 ){
        return {{-1,0}};
    }

    if(crossTargetFairZone[0][1] == -1){
        return {{0,-1}};
    }

    //Определение ближайщей к целям точки области возможных положений
    vector<double> target_1R = target_1 -> get_stateVector();
    vector<double> target_2R = target_2 -> get_stateVector();
    target_1R.resize(3);
    target_2R.resize(3);
    vector<double> lastPoint = nearestPointFromSample(target_1R, target_2R, crossTargetFairZone);

    cout << "Ближайщая точка: " << lastPoint[0] << ' ' << lastPoint[1] << ' ' << lastPoint[2] << '\n';

    vector<double> missileState = missile -> get_stateVector();
    
    //Вектор от ракеты до ближайшей к целям точке, принадежащей поверхности допустимой зоны положения ракеты.
    vector<double> direction(3);
    for(size_t i = 0; i < direction.size(); i ++){
        direction[i] = lastPoint[i] - missileState[i];
    }
    normalize(direction);

    cout << "Направление построений плоскостей: " << direction[0] << ' ' << direction[1] << ' ' << direction[2] << '\n';

    //Определение максимальной дальности полётаза время постоянства траектории. 
    double maxLength = (missile -> get_Vabs()) * reGuidanceTime;
    double step = 0;

    cout << "Максимальная длина построения: " << maxLength << '\n';

    //Создание переменной, куда будет суваться плоскость поражения перпендикулярная направлению полёта
    vector< vector<double> > fairSurface(0);
    vector<double> fairPoint(3);
    
    for(size_t i = 0; i < nMNK; i ++){
        step = maxLength * double(i) / double(nMNK);
        fairSurface = perpendToVectorFairSurface(missile, target_1, target_2, effectiveRadius, tolerance, direction, step, dt);
        fairPoint = findFarthestPointInPlane(fairSurface, fairSurface[0], direction);
        fairTrajectoryPoints.push_back(fairPoint);
        out.open(name, ios::app);
        out << fairPoint[0] << ' ' << fairPoint[1] << ' ' << fairPoint[2] << ' ' << '\n';
        out.close();
    }
    return fairTrajectoryPoints;
}*/

vector< vector<double> > fairTrajectoryPoints_surf(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt){
    ofstream out;          
    string name = "fairPointsOnSurf_" + to_string(int(missile->get_x()))+ ".dat";
    
    vector< vector<double> > fairTrajectoryPoints(0);
    
    int nMNK = NUM_OF_MNK_POINTS; //Количество точек, используемых для прогноза траектории

    vector< vector<double> > crossTargetFairZone = crossTargetMissileFairSurf(missile, target_1, target_2, effectiveRadius, tolerance, dt);
    
    if(crossTargetFairZone[0][0] == -1 ){
        return {{-1,0}};
    }

    if(crossTargetFairZone[0][1] == -1){
        return {{0,-1}};
    }

    //Определение ближайщей к целям точки области возможных положений
    vector<double> target_1R = target_1 -> get_stateVector();
    vector<double> target_2R = target_2 -> get_stateVector();
    target_1R.resize(3);
    target_2R.resize(3);
    vector<double> lastPoint = nearestPointFromSample(target_1R, target_2R, crossTargetFairZone);

    cout << "Ближайщая точка: " << lastPoint[0] << ' ' << lastPoint[1] << ' ' << lastPoint[2] << '\n';

    vector<double> missileState = missile -> get_stateVector();
    
    //Вектор от ракеты до ближайшей к целям точке, принадежащей поверхности допустимой зоны положения ракеты.
    vector<double> direction(3);
    for(size_t i = 0; i < direction.size(); i ++){
        direction[i] = lastPoint[i] - missileState[i];
    }
    normalize(direction);

    cout << "Направление построений плоскостей: " << direction[0] << ' ' << direction[1] << ' ' << direction[2] << '\n';

    //Определение максимальной дальности полётаза время постоянства траектории. 
    double maxLength = lastPoint[0];
    double step = 0;

    cout << "Максимальная длина построения: " << maxLength << '\n';

    //Создание переменной, куда будет суваться плоскость поражения перпендикулярная направлению полёта
    vector< vector<double> > fairSurface(0);
    vector<double> fairPoint(3);
    
    for(size_t i = 0; i < nMNK; i ++){
        step = maxLength * double(i) / double(nMNK);
        fairSurface = perpendToVectorFairLine(missile, target_1, target_2, effectiveRadius, tolerance, direction, step, dt);
        fairPoint = { (fairSurface[0][0] + fairSurface[1][0])*0.5, (fairSurface[0][1] + fairSurface[1][1])*0.5 };
        fairTrajectoryPoints.push_back(fairPoint);
        out.open(name, ios::app);
        out << fairPoint[0] << ' ' << fairPoint[1] << ' ' << fairPoint[2] << ' ' << '\n';
        out.close();
    }
    return fairTrajectoryPoints;
}


//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////РАБОТАЕМ С ЭТИМ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////Определение точек зоны/////////////////////////////////////////////

bool inZone(vector<double>& flightRes, double effectiveRadius){
    if(flightRes[0] < effectiveRadius && flightRes[4] >= 0 && flightRes[5] >= 0)
        return true;
    return false;
}

bool isInterested(vector<vector<int>>& zonePoints, size_t i, size_t j){
    int summ = zonePoints[i][j] + zonePoints[i][j + 1] + zonePoints[i + 1][j] + zonePoints[i + 1][j + 1];
    if(summ == 0 || summ == 4)
        return false;
    return true;
}

vector<double> checkPoint(vector<vector<int>>& zonePoints, size_t i, size_t j, double grid_step){
    int summ = zonePoints[i][j] + zonePoints[i][j + 1] + zonePoints[i + 1][j] + zonePoints[i + 1][j + 1];
    if(summ == 1 || summ == 2){
        if(zonePoints[i][j])
            return {double(i) * grid_step, double(j) * grid_step, 0};
        if(zonePoints[i][j + 1])
            return {double(i) * grid_step, double(j + 1) * grid_step, 3}; 
        if(zonePoints[i + 1][j])
            return {double(i + 1) * grid_step, double(j) * grid_step, 1};
        
        return {double(i + 1) * grid_step, double(j + 1) * grid_step, 2};    
    } else {
        if(!zonePoints[i][j])
            return {double(i + 1) * grid_step, double(j + 1) * grid_step, 2};
        if(!zonePoints[i][j + 1])
            return {double(i + 1) * grid_step, double(j) * grid_step, 1}; 
        if(!zonePoints[i + 1][j])
            return {double(i) * grid_step, double(j + 1) * grid_step, 3};
        
        return {double(i) * grid_step, double(j) * grid_step, 0};
    }
}

vector<vector<double>> checkPointBound(AperiodMissile* missile, Target* target_1, Target* target_2, vector<double> _checkPoint, double effectiveRadius, double dt){
    
    double step = sqrt(2) * GRID_STEP;
    vector< vector<double> > bounds;

    vector<double> missile_stateVector = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    vector<double> missile_stateVector_initial = missile_stateVector;
    vector<double> flyghtRes_1(6);
    vector<double> flyghtRes_2(6);
    //vector<double> cos_xyz = { cos(_pitch) * cos(_yaw),  sin(_pitch) * cos(_yaw), sin(_yaw) };  
    double pitch;

    vector<double> left(2);
    vector<double> right(2);
    vector<double> med(2);



    double numOfPoints = 9; ////////////////////////////
    for(size_t i = 0; i <= int(numOfPoints); i++){
        pitch = 0.5 * M_PI * _checkPoint[2] + double(i) * 0.5 * M_PI / numOfPoints;
        step = GRID_STEP / cos(fmod(double(i) * 0.5 * M_PI / numOfPoints, 0.25 * M_PI));

        left[0] = _checkPoint[0];
        right[0] = left[0] + step * cos(pitch);
        left[1] = _checkPoint[1];
        right[1] = left[1] + step * sin(pitch);
        for(size_t j = 0; j < 2; j++){
                missile_stateVector[j] = right[j];
        }
        missile -> set_state(missile_stateVector, missile_n_xyz_body);
        flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
        flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);
        
        if(inZone(flyghtRes_1, effectiveRadius) && inZone(flyghtRes_2, effectiveRadius))
        {   
            cout << "ЛОХ!!!\n";
            continue;
        }
        while(range(left, right) > effectiveRadius){
            for(size_t j = 0; j < 2; j++){
                med[j] = 0.5 * (left[j] + right[j]);
                missile_stateVector[j] = med[j];
            }
            if(med[1] < 0){
                double back_step = med[1] / sin(pitch);
                med[0] -= back_step * cos(pitch);
                med[1] -= back_step * sin(pitch);
                for(size_t j = 0; j < 2; j++){
                    missile_stateVector[j] = med[j];
                }
            }
            missile -> set_state(missile_stateVector, missile_n_xyz_body);
            flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
            flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);
            if(!(inZone(flyghtRes_1, effectiveRadius) && inZone(flyghtRes_2, effectiveRadius))){
                right = med;
            } else {
                    if(abs(med[1]) < 1) break;
                    left = med;
            }
        }
        bounds.push_back(med);
        cout << med[0] << ' ' << med[1] << '\n';
        
    }
    missile -> set_state(missile_stateVector_initial, missile_n_xyz_body);
    return bounds;
}

vector< vector<double> > crossGrid(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double dt) {
    
    vector<double> missileState = missile -> get_stateVector();
    vector<double> missile_n_xyz_body = missile -> get_n_xyz_body();
    vector<double> missileR = {missileState[0], missileState[1], missileState[2]};

    vector< vector<double> > res;

    vector<double> flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
    vector<double> flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);

    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes_1[0] > effectiveRadius || flyghtRes_1[4] < 0 || flyghtRes_1[5] < 0){    
        return { {-1, 0 } };        
    }

    if(flyghtRes_2[0] > effectiveRadius || flyghtRes_2[4] < 0 || flyghtRes_2[5] < 0){    
        return { { 0 , -1 } };        
    }

    double max_y = MAX_Y;
    double max_x = MAX_X;
    double grid_step = GRID_STEP;
    int x_steps = int(max_x / grid_step);
    int y_steps = int(max_y / grid_step);
    int ys = y_steps + 1;
    vector< vector<int> > zonePoints(int(x_steps + 1), vector<int>(ys));
    vector<double> _checkPoint(3); 

    for(size_t i = 0; i <= x_steps; i++){
        for(size_t j =0; j <= y_steps; j++){
            missileState[0] = double(i) * grid_step;
            missileState[1] = double(j) * grid_step;
            missile->set_state(missileState, missile_n_xyz_body);
            flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
            flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);
            if(inZone(flyghtRes_1, effectiveRadius) && inZone(flyghtRes_2, effectiveRadius)){
                zonePoints[i][j] = 1;
            } else {
                zonePoints[i][j] = 0;
            }
        }
    }

    missileState[0] = missileR[0];
    missileState[1] = missileR[1];
    missile->set_state(missileState, missile_n_xyz_body);

    ofstream out;          
    string name = "boundPoints" + to_string(int(missile->get_x())) + ".dat";  

    for(size_t i = 0; i < x_steps; i++){
        for(size_t j = 0; j < y_steps; j++){
            if(isInterested(zonePoints, i, j)){
                _checkPoint = checkPoint(zonePoints, i, j, grid_step);
                vector< vector<double> > bounds = checkPointBound(missile, target_1, target_2, _checkPoint, effectiveRadius, dt);
                for(size_t k = 0; k < bounds.size(); k++){
                    out.open(name, ios::app);
                    out << bounds[k][0] << ' ' << bounds[k][1] <<  '\n';
                    out.close();
                    res.push_back(bounds[k]);
                }
            }
        }   
    }
    return res;
}


////////////////////////////////////Определение траектории в зоне///////////////////////////////////////////

const std::vector<std::pair<int, int>> directions = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}
};

const std::vector<std::pair<int, int>> steps = {
    {1, 0},  // Right
    {0, -1}, // Up
    {0, 1},  // Down
    {1, -1}, // Diagonal Right Up
    {1, 1}   // Diagonal Right Down
};

bool is_within_grid(int x, int y, int grid_width, int grid_height) {
    return x >= 0 && x < grid_width && y >= 0 && y < grid_height;
}

int count_neighbor_zeros(const std::vector<std::vector<int>>& grid, int x, int y, int grid_width, int grid_height) {
    int count = 0;
    for (const auto& dir : directions) {
        int nx = x + dir.first;
        int ny = y + dir.second;
        if (is_within_grid(nx, ny, grid_width, grid_height) && grid[ny][nx] == 0) {
            count++;
        }
    }
    return count;
}

// Find the closest zero to the given isolated zero that has no neighbors or only one neighbor of zero
std::pair<int, int> find_closest_zero_with_isolation(const std::vector<std::vector<int>>& grid, int x, int y, int grid_width, int grid_height) {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, int> closest_zero = {-1, -1};

    // Search for the closest zero that has no more than one neighboring zero
    for (int i = 0; i < grid_height; ++i) {
        for (int j = 0; j < grid_width; ++j) {
            if (grid[i][j] == 0 && (i != y || j != x)) {
                // Count neighbors of the current zero
                int neighbor_zeros = count_neighbor_zeros(grid, j, i, grid_width, grid_height);
                
                // If the zero has no neighbors or only one neighbor, consider it
                if (neighbor_zeros <= 1) {
                    double distance = std::sqrt(std::pow((i - y) * 300, 2) + std::pow((j - x)*60, 2));
                    if (distance < min_distance) {
                        min_distance = distance;
                        if(distance < 1000 * sqrt(2))
                            closest_zero = {j, i};
                    }
                }
            }
        }
    }
    return closest_zero;
}

void draw_line(std::vector<std::vector<int>>& grid, int x1, int y1, int x2, int y2) {
    int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        grid[y1][x1] = 0;
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

vector<double> find_grid_step_xy(std::vector<std::vector<double> >& boundary_points){
    double min_x = std::numeric_limits<double>::max(), min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest(), max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : boundary_points) {
        min_x = std::min(min_x, point[0]);
        min_y = std::min(min_y, point[1]);
        max_x = std::max(max_x, point[0]);
        max_y = std::max(max_y, point[1]);
    }

    // Calculate grid dimensions based on the different step sizes
    size_t grid_width = SHAPE_GRID_W;
    size_t grid_height = SHAPE_GRID_H;

    double step_x = (max_x - min_x) / double(grid_width - 1);
    double step_y = (max_y - min_y) / double(grid_height - 1);

    return {step_x, step_y};
}

std::vector<std::vector<int>> shapeGrid( std::vector<std::vector<double> >& boundary_points ){

    ofstream out;          
    string name = "shapeGrid"+ to_string(int(boundary_points[0][0])) + ".dat";  


    double min_x = std::numeric_limits<double>::max(), min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest(), max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : boundary_points) {
        min_x = std::min(min_x, point[0]);
        min_y = std::min(min_y, point[1]);
        max_x = std::max(max_x, point[0]);
        max_y = std::max(max_y, point[1]);
    }

    // Calculate grid dimensions based on the different step sizes
    size_t grid_width = SHAPE_GRID_W;
    size_t grid_height = SHAPE_GRID_H;

    double step_x = (max_x - min_x) / double(grid_width - 1);
    double step_y = (max_y - min_y) / double(grid_height - 1);

    std::vector<std::vector<int>> grid(grid_height, std::vector<int>(grid_width, 1));

    for (const auto& point : boundary_points) {
        size_t grid_x = static_cast<size_t>((point[0] - min_x) / step_x);
        size_t grid_y = static_cast<size_t>((point[1] - min_y) / step_y);
        grid[grid_y][grid_x] = 0; 
    }

    std::vector<std::pair<int, int>> isolated_zeros;
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            if (grid[y][x] == 0) {
                int neighbor_zeros = 0;
                for (const auto& dir : directions) {
                    int nx = x + dir.first;
                    int ny = y + dir.second;
                    if (is_within_grid(nx, ny, grid_width, grid_height) && grid[ny][nx] == 0) {
                        neighbor_zeros++;
                    }
                }

                if (neighbor_zeros <= 1) {
                    isolated_zeros.push_back({x, y});
                }
            }
        }
    }

    // Step 3: Connect isolated zeros to their closest neighbor
    for (const auto& zero : isolated_zeros) {
        int x = zero.first, y = zero.second;
        auto closest_zero = find_closest_zero_with_isolation(grid, x, y, grid_width, grid_height);
        if (closest_zero.first != -1 && closest_zero.second != -1) {
            draw_line(grid, x, y, closest_zero.first, closest_zero.second);
        }
    }
    
    for(size_t i = 0; i < grid.size(); i ++){
        for(size_t j = 0; j < grid[0].size(); j++){
            if(grid[i][j] == 0){
                out.open(name, ios::app);
                out << min_x + step_x * j << ' ' << min_y + step_y * i << '\n';
                out.close();
            }
        }
    }
    return grid;
}

void fill_with_min_steps(std::vector<std::vector<int>>& grid) {
    int grid_width = SHAPE_GRID_W;
    int grid_height = SHAPE_GRID_H;

    std::queue<std::pair<int, int>> q;  // Queue for BFS
    std::vector<std::vector<int>> distance(grid_height, std::vector<int>(grid_width, std::numeric_limits<int>::max()));  // Distance grid

    // Step 1: Initialize the BFS with border cells (zeros)
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            if (grid[y][x] == 0) {
                q.push({x, y});
                distance[y][x] = 0;  // Border cells are at distance 0
            }
        }
    }

    // Step 2: Perform BFS to propagate the distances
    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        // Explore all 8 neighbors (diagonal, horizontal, and vertical)
        for (const auto& dir : directions) {
            int nx = x + dir.first;
            int ny = y + dir.second;
            if (is_within_grid(nx, ny, grid_width, grid_height) && distance[ny][nx] == std::numeric_limits<int>::max()) {
                // Update the distance for the neighboring cell
                distance[ny][nx] = distance[y][x] + 1;
                q.push({nx, ny});
            }
        }
    }

    // Step 3: Update the grid with the minimum distances
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            if (grid[y][x] != 0) {  // Skip the border cells (already set to 0)
                grid[y][x] = distance[y][x];
            }
        }
    }
}

std::pair<int, int> find_farthest_bound_point(const std::vector<std::vector<int>>& grid, int start_x, int start_y){
    int end_x = 0;
    int end_y = 0;
    int max_distance = 0;
    int distance;

    for (int y = 0; y < grid.size(); ++y) {
        for (int x = 0; x < grid[0].size(); ++x) {
            if (grid[y][x] == 0) {
                distance = (start_x -x) * (start_x - x) + (start_y - y) * (start_y - y);
                if(distance > max_distance){
                    max_distance = distance;
                    end_x = x;
                    end_y = y;
                }
            }
        }
    }
    return pair(end_x, end_y);
}


std::vector<std::pair<int, int>> find_single_path(
    const std::vector<std::vector<int>>& grid,
    int start_x, int start_y,
    int target_x, int target_y)
{
    int grid_width = grid[0].size();
    int grid_height = grid.size();
    
    // Find vertical constraints
    std::pair<int,int> upper_bound;
    std::pair<int,int> lower_bound;
    int count;

    for(size_t x = 0; x < grid[0].size(); x++){
        count = 0;
        for(size_t y = 0; y < grid.size(); y++){
            if(grid[y][x] == 0){
                if(count == 0){
                    lower_bound.first = x;
                    lower_bound.second = y;
                    count++;
                } else {
                    upper_bound.first = x;
                    upper_bound.second = y;
                    count ++;
                    break;
                }
            }
        }
        if (count == 2) break;    
    }

    auto is_right_side = [](int x, int y, std::pair<int, int> bound_point, bool is_upper) -> bool {
        if (is_upper) {
            // For upper bound: prevent going left and up around the point
            return !(x <= bound_point.first && y > bound_point.second);
        } else {
            // For lower bound: prevent going left and down around the point
            return !(x <= bound_point.first && y < bound_point.second);
        }
    };

    using State = std::tuple<int, int, int>;
    auto comparator = [](const State& a, const State& b) {
        return std::get<0>(a) < std::get<0>(b);
    };
    std::priority_queue<State, std::vector<State>, decltype(comparator)> pq(comparator);
    
    std::set<std::pair<int, int>> visited;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;
    
    pq.push({grid[start_y][start_x], start_x, start_y});
    visited.insert({start_x, start_y});
    parent[{start_x, start_y}] = {-1, -1};

    while (!pq.empty()) {
        auto [distance, x, y] = pq.top();
        pq.pop();

        if (x == target_x && y == target_y) {
            std::vector<std::pair<int, int>> path;
            for (auto p = std::make_pair(x, y); p != std::make_pair(-1, -1); p = parent[p]) {
                path.push_back(p);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& [dx, dy] : steps) {
            int nx = x + dx;
            int ny = y + dy;

            if (is_within_grid(nx, ny, grid_width, grid_height) &&
                visited.find({nx, ny}) == visited.end() &&
                is_right_side(nx, ny, upper_bound, true) &&
                is_right_side(nx, ny, lower_bound, false) &&
                (grid[ny][nx] > 0 || (nx == target_x && ny == target_y)))  // Allow zero only for target
            {
                visited.insert({nx, ny});
                parent[{nx, ny}] = {x, y};
                pq.push({grid[ny][nx], nx, ny});
            }
        }
    }

    return {};
}


std::vector<std::pair<int, int>> find_path_max_distance(
    const std::vector<std::vector<int>>& grid, 
    int start_x, int start_y, 
    int target_x, int target_y) 
{
    int grid_width = grid[0].size();
    int grid_height = grid.size();
    
    // Get all valid target points (target and adjacent zeros)
    std::vector<std::pair<int, int>> target_points;
    target_points.push_back({target_x, target_y});
    
    // Add all zero points within a reasonable radius of the target
    int search_radius = 5;  // Increase search radius
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            int nx = target_x + dx;
            int ny = target_y + dy;
            if (is_within_grid(nx, ny, grid_width, grid_height) && 
                grid[ny][nx] == 0 && 
                (nx != target_x || ny != target_y)) {
                target_points.push_back({nx, ny});
            }
        }
    }

    // Sort target points by distance from original target
    std::sort(target_points.begin(), target_points.end(),
        [target_x, target_y](const auto& a, const auto& b) {
            int dist_a = (a.first - target_x) * (a.first - target_x) + 
                        (a.second - target_y) * (a.second - target_y);
            int dist_b = (b.first - target_x) * (b.first - target_x) + 
                        (b.second - target_y) * (b.second - target_y);
            return dist_a < dist_b;
        });

    // Try paths to each target point
    for (const auto& target : target_points) {
        auto path = find_single_path(grid, start_x, start_y, target.first, target.second);
        if (!path.empty()) {
            return path;
        }
    }

    return {};
}




std::vector<std::vector<double> > readFromFile(std::string filename){
    std::vector<std::vector<double>> data(249,std::vector<double>(2));


    // Open the file
    std::ifstream infile(filename);

    // Read data from the file
    for (size_t i = 0; i < 249; ++i) {
        infile >> data[i][0];
        infile >> data[i][1];
    }

    infile.close();
    return data;
}


class ZeroConnector {
private:
    std::vector<std::vector<int>>& grid;
    int width;
    int height;

    void flood_fill(std::vector<std::vector<int>>& temp_grid, int x, int y, int component_id, std::set<std::pair<int, int>>& component) {
        std::queue<std::pair<int, int>> q;
        q.push({x, y});
        temp_grid[y][x] = component_id;

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();
            component.insert({cx, cy});

            for (const auto& [dx, dy] : directions) {
                int nx = cx + dx;
                int ny = cy + dy;
                if (is_within_grid(nx, ny, width, height) && temp_grid[ny][nx] == 0) {
                    temp_grid[ny][nx] = component_id;
                    q.push({nx, ny});
                }
            }
        }
    }

    std::vector<std::pair<int, int>> find_shortest_path(
        const std::set<std::pair<int, int>>& comp1,
        const std::set<std::pair<int, int>>& comp2) {
        
        std::queue<std::pair<int, int>> q;
        std::map<std::pair<int, int>, std::pair<int, int>> parent;
        std::set<std::pair<int, int>> visited;

        // Initialize distances with infinity
        std::vector<std::vector<int>> distance(height, std::vector<int>(width, INT_MAX));

        // Start from all points in comp1
        for (const auto& start : comp1) {
            q.push(start);
            parent[start] = {-1, -1};
            visited.insert(start);
            distance[start.second][start.first] = 0;
        }

        std::pair<int, int> target;
        bool found = false;

        while (!q.empty() && !found) {
            auto [cx, cy] = q.front();
            q.pop();

            for (const auto& [dx, dy] : directions) {
                int nx = cx + dx;
                int ny = cy + dy;
                auto next = std::make_pair(nx, ny);

                if (is_within_grid(nx, ny, width, height) && visited.find(next) == visited.end()) {
                    visited.insert(next);
                    parent[next] = {cx, cy};
                    distance[ny][nx] = distance[cy][cx] + 1;
                    q.push(next);

                    if (comp2.find(next) != comp2.end()) {
                        target = next;
                        found = true;
                        break;
                    }
                }
            }
        }

        if (found) {
            std::vector<std::pair<int, int>> path;
            auto current = target;
            while (current != std::make_pair(-1, -1)) {
                path.push_back(current);
                current = parent[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        return {};
    }

public:
    ZeroConnector(std::vector<std::vector<int>>& input_grid) 
        : grid(input_grid), width(input_grid[0].size()), height(input_grid.size()) {}

    void connect() {
        std::vector<std::vector<int>> temp_grid = grid;
        std::vector<std::set<std::pair<int, int>>> components;
        int component_id = 2;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (temp_grid[y][x] == 0) {
                    std::set<std::pair<int, int>> component;
                    flood_fill(temp_grid, x, y, component_id++, component);
                    if (!component.empty()) {
                        components.push_back(component);
                    }
                }
            }
        }

        // Connect components
        while (components.size() > 1) {
            int min_distance = width * height;
            int comp1_idx = -1, comp2_idx = -1;
            std::vector<std::pair<int, int>> best_path;

            for (size_t i = 0; i < components.size(); ++i) {
                for (size_t j = i + 1; j < components.size(); ++j) {
                    auto path = find_shortest_path(components[i], components[j]);
                    if (!path.empty() && path.size() < min_distance) {
                        min_distance = path.size();
                        comp1_idx = i;
                        comp2_idx = j;
                        best_path = std::move(path);
                    }
                }
            }

            if (comp1_idx != -1 && !best_path.empty()) {
                // Set zeros along the path in the original grid
                for (const auto& [x, y] : best_path) {
                    grid[y][x] = 0;
                }
                
                // Merge components
                components[comp1_idx].insert(
                    components[comp2_idx].begin(),
                    components[comp2_idx].end()
                );
                components.erase(components.begin() + comp2_idx);
            }
        }
    }
};


vector< vector<double> > fairTrajectoryPoints(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt){
    ofstream out;          
    string name = "fairPoints" + to_string(int(missile->get_x()))+ ".dat";
    
    vector< vector<double> > fairTrajectoryPoints(0);

    vector< vector<double> > boundPoints = crossGrid(missile, target_1, target_2, effectiveRadius, tolerance, dt);
    
    //vector< vector<double> > boundPoints = readFromFile("boundPoints.dat");

    if(boundPoints[0][0] == -1 ){
        return {{-1,0}};
    }

    if(boundPoints[0][1] == -1){
        return {{0,-1}};
    }

    double min_x = std::numeric_limits<double>::max(), min_y = std::numeric_limits<double>::max();

    for (const auto& point : boundPoints) {
        min_x = std::min(min_x, point[0]);
        min_y = std::min(min_y, point[1]);
    }

    vector<double> gridStep_xy = find_grid_step_xy(boundPoints);
    vector<double> missile_r = missile -> get_r();
    int start_x = int(missile_r[0] - min_x)/gridStep_xy[0];
    int start_y = int(missile_r[1] - min_y)/gridStep_xy[1];

    vector<vector<int>> _shapeGrid = shapeGrid(boundPoints);

    ZeroConnector connector(_shapeGrid);
    connector.connect();

    pair<int, int> endP = find_farthest_bound_point(_shapeGrid, start_x, start_y);

    fill_with_min_steps(_shapeGrid);

    vector<pair<int, int>> path = find_path_max_distance(   _shapeGrid, start_x, start_y, endP.first, endP.second);
    
    fairTrajectoryPoints.resize(path.size(), vector<double>(3));

    for(size_t i = 0; i < path.size(); i++){
        fairTrajectoryPoints[i][0] = min_x + double(path[i].first) * gridStep_xy[0];
        fairTrajectoryPoints[i][1] = min_y + double(path[i].second) * gridStep_xy[1];
        fairTrajectoryPoints[i][2] = 0;
        out.open(name, ios::app);
        out << fairTrajectoryPoints[i][0] << ' ' << fairTrajectoryPoints[i][1] << '\n';
        out.close();
    }

    return fairTrajectoryPoints;
}
