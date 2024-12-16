#ifndef _MISSILE_FAIR_ZONE_ANALYZER_H
#define _MISSILE_FAIR_ZONE_ANALYZER_H

#include <math.h>
#include <vector>
#include "OneMissileSimulation.hpp"

//Поиск границы зоны допустимых мест пусков по направлению относительно НЗСК, где начало - положение ракеты,
//а направление задают два угла _yaw, _pitch (!!!это не углы рысканья и тангажа, они так названы, потому что похожи по смылсу)
std::vector<double> directionBound(Missile* missile, Target* target, double _yaw, double _pitch, double effectiveRadius, double tolerance, double dt);

//Построитель зоны допустимых мест пусков ракет. 
//Это значит допустимых для поражения начальных положений ракеты относительно цели в текущий момент времени для текущей конфигурации цель-ракета,
//то есть для текущего направления и величины скорости ракеты, а также скорости цели
//Возвращает вектор пар: (высота, список точек зон ОВП)
std::vector< std::vector<double> > missileFairZone(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt, int numPoints = 36);

#endif