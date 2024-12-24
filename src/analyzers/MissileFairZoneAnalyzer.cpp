#include <math.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <time.h>
#include "omp.h"
#include <queue>
#include <set>

#include "../../include/utils/MyMath.hpp"
#include "../../include/analyzers/MissileFairZoneAnalyzer.hpp"

#define MISSILE_DIR_STEP double(11000) //Длина начального шага для определения границы по направлению 
#define NUM_OF_MNK_POINTS 100 //Число точек, используемых для аппроксимации траектории
#define NUM_FAIR_ZONE_POINTS 120  //Число которое показывает шаг угла при построении допустимой зоны ракеты dAnngle = 2 * M_PI / NUM_FAIR_ZONE_POINTS
#define NUM_SURFACE_POINT 24 //Число которое показываетшаг угла при построении допустимого положения ракеты в плоскости dAnngle = 2 * M_PI / NUM_SURFACE_POINT
#define MAX_Y double(30000)
#define MAX_X double(30000)
#define GRID_STEP double(1000)

using namespace std;

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


vector< vector<double> > fairTrajectoryPoints(AperiodMissile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt){
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
        missile -> set_state(missile_stateVector_initial, missile_n_xyz_body);
    }

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

    ofstream out;          
    string name = "gridInt.dat";  
    out.open(name, ios::app);
    for(size_t i = 0; i < res.size(); i ++){
            out << res[i][0] << ' ' << res[i][1] <<  '\n';
    }
    out.close();

    for(size_t i = 0; i < x_steps; i++){
        for(size_t j = 0; j < y_steps; j++){
            if(isInterested(zonePoints, i, j)){
                out.open(name, ios::app);
                out << i * grid_step << ' ' << j * grid_step <<  '\n';
                out << i  * grid_step << ' ' << (j + 1) * grid_step <<  '\n';
                out << (i + 1) * grid_step << ' ' << (j + 1) * grid_step <<  '\n';
                out << (i + 1) * grid_step << ' ' << j * grid_step <<  '\n';
                out.close();
                _checkPoint = checkPoint(zonePoints, i, j, grid_step);
                vector< vector<double> > bounds = checkPointBound(missile, target_1, target_2, _checkPoint, effectiveRadius, dt);
                for(size_t k = 0; k < bounds.size(); k++){
                    res.push_back(bounds[k]);
                }
            }
        }   
    }
    return res;
}