#include <math.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <time.h>

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
        missile -> set_state(missile_stateVector, missile_ryp_initial, missile_w_initial);
        flightRes = oneMissileSimulation(missile, target, dt);
        missDistanse = flightRes[0];
        if(flightRes[4] < 0){
            missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
        }
        if(!inAir && missDistanse < effectiveRadius) break;
        inAir = true;
    }

    //cout << missile_stateVector[0] << ' ' << missile_stateVector[1] << ' ' << missile_stateVector[2] << '\n';

    missile -> set_state(missile_stateVector_initial, missile_ryp_initial, missile_w_initial);

    return missile_stateVector;
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

    bound = directionBound(missile, target, _yaw, M_PI * 0.5, effectiveRadius, tolerance, dt);
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
