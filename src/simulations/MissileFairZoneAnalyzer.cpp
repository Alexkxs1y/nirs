#include <math.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <time.h>

#include "../../include/utils/MyMath.hpp"
#include "../../include/simulations/MissileFairZoneAnalyzer.hpp"

#define MISSILE_DIR_STEP double(8000) //Длина начального шага для определения границы по направлению 

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


vector< vector<double> > missileFairZone(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt, int numPoints){

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


vector< vector<double> > crossTargetMissileFairZone(Missile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double dt, int numPoints){
    
    ofstream out;          
    string name = "crossZone.dat";  
    
    vector<double> missileState = missile -> get_stateVector();

    vector<double> flyghtRes_1 = oneMissileSimulation(missile, target_1, dt);
    vector<double> flyghtRes_2 = oneMissileSimulation(missile, target_2, dt);

    //Если из текущий точки поражение совершить невозможно
    //Функция прекращает работы и делает вывод пары {-1, {{-1,-1}}}
    if(flyghtRes_1[0] > effectiveRadius || flyghtRes_1[4] < 0){    
        return { {-1, -1} };        
    }

    if(flyghtRes_2[0] > effectiveRadius || flyghtRes_2[4] < 0){    
        return { {-1, -1} };        
    }

    double _yaw = 0, _pitch = 0;
    vector<double> bound_1(3);
    vector<double> bound_2(3);
    vector< vector<double> > crossTargetMissileFairZone(0);

    bound_1 = directionBound(missile, target_1, _yaw, M_PI * 0.5, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, M_PI * 0.5, effectiveRadius, tolerance, dt);
    
    if(range(missileState, bound_1) < range(missileState, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
        out.open(name, ios::app);
        out << bound_1[0] << ' ' << bound_1[1] << ' ' << bound_1[2] << ' ' << '\n';
        out << '\n';
        out.close();
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
        out.open(name, ios::app);
        out << bound_2[0] << ' ' << bound_2[1] << ' ' << bound_2[2] << ' ' << '\n';
        out << '\n';
        out.close();
    }

    for(int i = 1; i < int(numPoints * 0.5); i ++){
        for(int j = 0; j < numPoints; j ++){
            _yaw = 2 * double(j) * M_PI / double(numPoints);
            _pitch = M_PI * 0.5 - 2 * double(i) * M_PI / double(numPoints);
            bound_1 = directionBound(missile, target_1, _yaw, _pitch, effectiveRadius, tolerance, dt);
            bound_2 = directionBound(missile, target_2, _yaw, _pitch, effectiveRadius, tolerance, dt);
            if(range(missileState, bound_1) < range(missileState, bound_2)){
                crossTargetMissileFairZone.push_back(bound_1);
                out.open(name, ios::app);
                out << bound_1[0] << ' ' << bound_1[1] << ' ' << bound_1[2] << ' ' << '\n';
                out.close();
            } else {
                crossTargetMissileFairZone.push_back(bound_2);
                out.open(name, ios::app);
                out << bound_2[0] << ' ' << bound_2[1] << ' ' << bound_2[2] << ' ' << '\n';
                out.close();
            }
        }
        out.open(name, ios::app);
        out << '\n';
        out.close();
    }

    bound_1 = directionBound(missile, target_1, _yaw, - M_PI * 0.5, effectiveRadius, tolerance, dt);
    bound_2 = directionBound(missile, target_2, _yaw, - M_PI * 0.5, effectiveRadius, tolerance, dt);
    
    if(range(missileState, bound_1) < range(missileState, bound_2)){
        crossTargetMissileFairZone.push_back(bound_1);
        out.open(name, ios::app);
        out << bound_1[0] << ' ' << bound_1[1] << ' ' << bound_1[2] << ' ' << '\n';
        out << '\n';
        out.close();
    } else {
        crossTargetMissileFairZone.push_back(bound_2);
        out.open(name, ios::app);
        out << bound_2[0] << ' ' << bound_2[1] << ' ' << bound_2[2] << ' ' << '\n';
        out << '\n';
        out.close();
    }

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
                                                        double tolerance, vector<double>& direction, double step, double dt, int numPoints){
    
    vector< vector<double> > fairSurface(0);
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
    
    for(size_t i = 0; i < numPoints; i ++){
        searchAngle = 2 * M_PI * double(i) / double(numPoints);
        searchDirection = rotate(normToDirection, direction, searchAngle);
        bound_1 = pointDirectionBound(missile, target_1, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
        bound_2 = pointDirectionBound(missile, target_2, effectiveRadius, tolerance, hitPoint, searchDirection, dt);
        if(range(hitPoint, bound_1) < range(hitPoint, bound_2)){
            fairSurface.push_back(bound_1);
            cout << bound_1[0] << ' ' << bound_1[1] << ' ' << bound_1[2] << '\n';
        } else {
            fairSurface.push_back(bound_2);
            cout << bound_2[0] << ' ' << bound_2[1] << ' ' << bound_2[2] << '\n';
        }
    }
    return fairSurface;
}

/*vector< vector<double> > fairTrajectoryDots(Missile* missile, Target* target_1, Target* target_2, double effectiveRadius, double tolerance, double reGuidanceTime, double dt, int numPoints){
    vector< vector<double> > fairTrajectoryDots(0);
    
    int nMNK = 10; //Количество точек, используемых для прогноза траектории

    vector< vector<double> > crossTargetFairZone = crossTargetMissileFairZone(missile, target_1, target_2, effectiveRadius, tolerance, dt, numPoints);
    vector<double> lastPoint = nearesPointFromSample(target_1 -> get_stateVector(), target_2 -> get_stateVector(), crossTargetFairZone);

    vector<double> missileState = missile -> get_stateVector();
    
    //Вектор от ракеты до ближайшей к целям точке, принадежащей поверхности допустимой зоны положения ракеты.
    vector<double> direction(3);
    for(size_t i = 0; i < direction.size(); i ++){
        direction[i] = lastPoint[i] - missileState[i];
    }
    normalize(direction);

    //Определение максимальной дальности полётаза время постоянства траектории. 
    double maxLength = missile -> get_Vabs() * reGuidanceTime;
    double step = 0;

    for(size_t i = 1; i <= nMNK; i ++){
        step = maxLength * double(i) / double(nMNK);

    }
}*/