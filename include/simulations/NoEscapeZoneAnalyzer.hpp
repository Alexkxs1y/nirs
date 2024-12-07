#ifndef _NO_ESCAPE_ZONE_ANALYZER_H
#define _NO_ESCAPE_ZONE_ANALYZER_H

#include <math.h>
#include <vector>
#include "OneMissileSimulation.hpp"

//Находит точку принадлежащую плоскости достижимости на высоте цели
std::vector<double> hitPointFinder(Missile* missile, Target* target, double effectiveRadius, double dt);

//Построитель плоскости достижимости. Варьируется только положение XZ цели 
std::vector< std::vector<double> > noEscapeSurface(Missile* missile, Target* target, double effectiveRadius, double tolerance, double dt, int numPoints = 36);

#endif