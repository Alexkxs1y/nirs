#ifndef _NO_ESCAPE_ZONE_ANALYZER_H
#define _NO_ESCAPE_ZONE_ANALYZER_H

#include <math.h>
#include <vector>
#include "../simulations/OneMissileSimulation.hpp"

//Находит точку принадлежащую плоскости достижимости на высоте цели
std::vector<double> hitPointFinder(Missile* missile, Target* target, double effectiveRadius, double approxPointX, double approxPointZ, double dt);

//Попытка найти попадающую точку по предыдущему слою
std::vector<double> luckyHitFinder(Missile* missile, Target* target, double effectiveRadius, double approxPointX, double approxPointZ, double dt);

//Построитель плоскости достижимости. Варьируется только положение XZ цели 
std::vector< std::vector<double> > noEscapeSurface(Missile* missile, Target* target, double effectiveRadius, double tolerance, double approxPointX, double approxPointZ, double dt, int numPoints = 36);

//Построитель зоны неухода. Для текущей конфигурации цель-ракета
//При прямолинейном движении цели зона зависит от скорости цели, скорости ракеты, направления относительной скорости. (Относительную скорость будем рассматривать относительно ракеты).
//Возвращает вектор пар: (высота, список точек зон ОВП)
std::vector< std::pair< double, std::vector< std::vector<double> > > > noEscapeZone(Missile* missile, Target* target, double V_mis, double V_tar, double yaw_rel, double pitch_rel, double effectiveRadius, double tolerance, double dt, int numPoints = 36);

#endif