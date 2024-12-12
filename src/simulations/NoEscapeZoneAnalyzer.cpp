#include <math.h>
#include <vector>
#include <stdexcept>
#include "../../include/models/Target.hpp"
#include "../../include/models/Missile.hpp"
#include "../../include/simulations/NoEscapeZoneAnalyzer.hpp"
#include "../../include/utils/MyMath.hpp"
#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;

vector<double> luckyHitFinder(Missile* missile, Target* target, double effectiveRadius, double approxPointX, double approxPointZ, double dt){
    double h = 500;
    int i_max = 10;
    int i = 1;

    vector<double> missile_ryp = missile -> get_ryp();
    vector<double> target_stateVector = target -> get_stateVector();
    vector<double> target_stateVector_initial = target_stateVector;

    vector<double> flight_res = {0, 0, 0, 0, 0};

    double missDistance = 2 * effectiveRadius;
    double cos_y = cos(missile_ryp[1]);
    double sin_y = sin(missile_ryp[1]);
    
    while(i <= i_max){
        target_stateVector[0] = approxPointX + double(i) * h * cos_y;
        target_stateVector[2] = approxPointZ + double(i) * h * sin_y;

        target -> set_state(target_stateVector);

        flight_res = oneMissileSimulation(missile, target, dt);

        if(flight_res[4] < 0){
            i ++;
            continue;
        }
        
        if(flight_res[0] < effectiveRadius){
            return {1, target_stateVector[0], target_stateVector[1], target_stateVector[2]};
        }
        i ++;
    }
    
    target -> set_state(target_stateVector_initial);
    
    return  {0};

}

vector<double> hitPointFinder(Missile* missile, Target* target, double effectiveRadius, double approxPointX, double approxPointZ, double dt){
    
    if(approxPointX != 0 || approxPointZ != 0){
        vector<double> luckyHitPoint = luckyHitFinder(missile, target, effectiveRadius, approxPointX, approxPointZ, dt);
        return luckyHitPoint;
    }

    bool inZone = false;
    vector<double> missile_ryp = missile -> get_ryp();
    vector<double> target_stateVector = target -> get_stateVector();
    vector<double> target_stateVector_initial = target_stateVector;

    vector<double> flight_res(5);

    double h = 500;
    double salt = 10; //Значение, чтобы сделать ненулевое начальное положение цели
    int i_max = 40; //Тут рандом...................................................    
    int i = 0;
    double missDistance = 2 * effectiveRadius;
    double cos_y = cos(missile_ryp[1]);
    double sin_y = sin(missile_ryp[1]);
    vector<double> hitPoint = {0};
    
    while(i <= i_max){
        target_stateVector[0] = (salt + double(i) * h) * cos_y;
        target_stateVector[2] = (salt + double(i) * h) * sin_y;

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
    
    target -> set_state(target_stateVector_initial);
    
    return  hitPoint;
}




vector< vector<double> > noEscapeSurface(Missile* missile, Target* target, double effectiveRadius, double tolerance, double approxPointX, double approxPointZ, double dt, int numPoints){

    ofstream out;          
    int hhh = int(target->get_y());
    string name = "res_" + to_string(hhh) + ".dat"; 
          

    vector< vector<double> > noEscapeSurface(numPoints, vector<double>(2));
    vector<double> hitPoint = hitPointFinder(missile, target, effectiveRadius, approxPointX, approxPointZ, dt);

    int numZones = hitPoint[0];
    if(numZones == 0){
        for(int i = 0; i < numPoints; i++){
            noEscapeSurface.assign(numPoints, {0, 0});
        }
        return noEscapeSurface;
    }

    vector<double> target_stateVector = target -> get_stateVector();
    vector<double> target_stateVector_initial = target_stateVector;
    vector<double> missile_ryp = missile -> get_ryp();
    
    vector<double> flightRes(5);
    int j = 1;
    double h = 8000;
    double missDistanse = 0;
    double searchAngle;
    double cos_i = 0;
    double sin_i = 0;
    bool isStepBack = false;

    int hit_x = (numZones - 1) * 3 + 1;
    int hit_z = (numZones - 1) * 3 + 3;
    

    for(int i = 0; i < numPoints; i++){
        
        searchAngle = missile_ryp[1] + double(i) * 2.0 * M_PI / double(numPoints); 
        cos_i = cos( searchAngle );
        sin_i = sin( searchAngle );
        target_stateVector[0] = hitPoint[hit_x];
        target_stateVector[2] = hitPoint[hit_z];
        missDistanse = 0;
        h = 8000;
        isStepBack = false;

        while(abs(missDistanse - effectiveRadius) > tolerance){
            if(missDistanse < effectiveRadius){
                if(isStepBack){
                    h *= 0.5;
                }
                if(h < 10) break; //Тоже микрокостыль для ускорения******************************************************
                target_stateVector[0] += h * cos_i;
                target_stateVector[2] += h * sin_i;
            } else {
                h *= 0.5;
                target_stateVector[0] -= h * cos_i;
                target_stateVector[2] -= h * sin_i;
                isStepBack = true;                    
            }
            target -> set_state(target_stateVector);
            flightRes = oneMissileSimulation(missile, target, dt);
            missDistanse = flightRes[0];
            if(flightRes[4] < 0){
                missDistanse = 2 * effectiveRadius; //Костыль при нехватке скорости.......................
            }
        }
        if(i == 0){
            hitPoint[hit_x] = 0.5 * (hitPoint[hit_x] + target_stateVector[0]);
            hitPoint[hit_z] = 0.5 * (hitPoint[hit_z] + target_stateVector[2]);
        }
        noEscapeSurface[i] = {target_stateVector[0], target_stateVector[2]};
        
        out.open(name, ios::app);
        out << noEscapeSurface[i][0] << ' ' << noEscapeSurface[i][1] << '\n';
        out.close(); 
        cout << noEscapeSurface[i][0] << ' ' << noEscapeSurface[i][1] << '\n';
    }

    target -> set_state(target_stateVector_initial);

    return noEscapeSurface;
}



vector< pair< double, vector< vector<double> > > > noEscapeZone(Missile* missile, Target* target, double V_mis, double V_tar, double yaw_rel, double pitch_rel, double effectiveRadius, double tolerance, double dt, int numPoints){
    double y_mid = 6000; // Будем считать, что это высота, на которой находится ракета. Относительно неё будем варьировать высоту цели.
    
    //НУ для движения цели и ракеты
    vector<double> stateVector_tar_initial = target -> get_stateVector();
    vector<double> stateVector_mis_initial = missile -> get_stateVector();
    vector<double> ryp_mis_initial = missile -> get_ryp();
    vector<double> w_mis_initial = missile -> get_w();

    //Мы считаем, что ракета маневренная и g не учитываем. 
    //Поэтому при варьировании конфугурации системы цель-ракета, можно задавать только угол их относительной скорости 
    //и считать, что скорость ракеты всегда направлена по оси Ox НЗСК. Так же считаем, что в начальный момент ракета стабилизирована
    vector<double> stateVector_mis = {0, y_mid ,0, V_mis, 0, 0};
    vector<double> stateVector_tar = {0, y_mid , 0, V_tar * cos(yaw_rel) * cos(pitch_rel), V_tar * cos(yaw_rel) * sin(pitch_rel), V_tar * sin(yaw_rel)};
    vector<double> ryp = {0, 0, 0};
    vector<double> w = {0, 0, 0};
    missile -> set_state(stateVector_mis, ryp, w);
    target -> set_state(stateVector_tar);

    double h_step = 500;
    bool outOfZone = false;

    vector< vector<double> > _noEscapeSurface(numPoints, vector<double>(2));
    vector< pair< double, vector< vector<double> > > > noEscapeZone;
    
    clock_t tStart = clock();

    double approxPointX = 0;
    double approxPointZ = 0;

    //От плоскости ракеты делаем шаги вверх
    while(!outOfZone){
        _noEscapeSurface = noEscapeSurface(missile, target, effectiveRadius, tolerance, approxPointX, approxPointZ, dt, numPoints);
        
        //Проверка, что на рассматриваемой высоте есть попадания, иначе считаем, что вышли из зоны
        if(_noEscapeSurface[0][0] == 0 && _noEscapeSurface[1][0] == 0){
            outOfZone = true;
            approxPointX = 0;
            approxPointZ = 0;
        } else {
            noEscapeZone.push_back( make_pair( stateVector_tar[1], _noEscapeSurface ) );
            approxPointX = _noEscapeSurface[int(numPoints * 0.5)][0];
            approxPointZ = _noEscapeSurface[int(numPoints * 0.5)][1];
            cout<< "Высота: " << stateVector_tar[1] << ". За время: " << (double)(clock() - tStart)/CLOCKS_PER_SEC << '\n';
            stateVector_tar[1] += h_step;
            target -> set_state(stateVector_tar);
        }
    }

    outOfZone = false;

        stateVector_tar[1] = stateVector_mis[1];
    //От плоскости ракеты делаем шаги вниз
    while(!outOfZone){
        stateVector_tar[1] -= h_step;
        target -> set_state(stateVector_tar);
        _noEscapeSurface = noEscapeSurface(missile, target, effectiveRadius, tolerance, approxPointX, approxPointZ, dt, numPoints);
        
        //Проверка, что на рассматриваемой высоте есть попадания, иначе считаем, что вышли из зоны
        if(_noEscapeSurface[0][0] == 0 && _noEscapeSurface[1][0] == 0){
            outOfZone = true;
            approxPointX = 0;
            approxPointZ = 0;

        } else {
            noEscapeZone.push_back( make_pair( stateVector_tar[1], _noEscapeSurface ) );
            approxPointX = _noEscapeSurface[int(numPoints * 0.5)][0];
            approxPointZ = _noEscapeSurface[int(numPoints * 0.5)][1];
            cout<< "Высота: " << stateVector_tar[1] << ". За время: " << (double)(clock() - tStart)/CLOCKS_PER_SEC << '\n';
            stateVector_tar[1] -= h_step;
            target -> set_state(stateVector_tar);
        }
    }



    //Возвращение цели и ракеты в начальное состояние, как перед использованием этой функции
    missile -> set_state(stateVector_mis_initial, ryp_mis_initial, w_mis_initial);
    target -> set_state(stateVector_tar_initial);

    return noEscapeZone;
}