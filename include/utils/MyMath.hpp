#ifndef _MY_MATH_H
#define _MY_MATH_H

#include <math.h>
#include <vector>
#include <string>


std::vector<double> sub(const std::vector<double> &a, const std::vector<double> &b);

double range(const std::vector<double>& a, const std::vector<double>& b);

double squareDistance(const std::vector<double> &a, const std::vector<double> &b);

//Возвращает ближайшую (по сумме растояний до двух точек) точку из набора точек
std::vector<double> nearestPointFromSample(const std::vector<double> &a, const std::vector<double> &b, std::vector< std::vector<double> >& sample);

void normalize(std::vector<double> &a);

double absVector(const std::vector<double> &a);

std::vector<double> summVector(const std::vector<double> &a, const std::vector<double> &b);

std::vector<double> crossProduct(const std::vector<double>& a, const std::vector<double>& b);

double dotProduct(const std::vector<double>& a, const std::vector<double>& b);

//Поворот вектора a относительно вектора b на угол Theta 
std::vector<double> rotate(const std::vector<double>& a, const std::vector<double>& b, double Theta);

//Возвращает центр масс многоугольника
std::vector<double> geometricCentroid(const std::vector< std::vector<double> > &polygon, const std::vector<double>& normToPolygon);

//Расстояние от точки p до отрезка ab (где a - радуис-вектор точки a, b - радуис-вектор точки b) внутри плоскости многоугольника
//a,b - вершины многоугольник
//p - исследуемая точка 
double pointToSegmentDistance(const std::vector<double>& point, const std::vector<double>& a, const std::vector<double>& b);

//Создание ортогонального базиса (векторов u, v) в плоскости перпендикулярной вектору normal
//normal - вектор задающий перпендикуляр к плоскости, в которой хотим создать базис
//u, v - вектора в которые будет записан базис
void createPlaneBasis(const std::vector<double>& normal, std::vector<double>& u, std::vector<double>& v);

//Преобразование координат точки из глобальной сисетмы к системе координат в плоскости многоугольника
//point - точка в глобальной 3Д системе координат
//planePoint - известная точка плоскости
//u, v - базис в плоскости, на которую проецируем
std::vector<double> toLocalPlane(const std::vector<double>& point, const std::vector<double>& planePoint, const std::vector<double>& u, const std::vector<double>& v);

//Преобразование из системы координат в плоскости многоугольника к глобольной
//localPoint - точка в 2D системе координат
//planePoint - точка, принадлежащая этой 2D СК
//u,v - базис данной плоскости
std::vector<double> toGlobal3D(const std::vector<double>& localPoint, const std::vector<double>& planePoint, const std::vector<double>& u, const std::vector<double>& v);

//Проверка, что точка в плоскости многоугольника находится внутри многоугольника
//point - точка в плоскости многоугольника
//polygon - набор вершин многоугольника (координаты в представлены в СК, сделанной в его плоскости)
bool isPointInsidePolygon2D(const std::vector<double>& point, const std::vector< std::vector<double> >& polygon);

//Определение максимально удаленной от границы многоугольника точки
//polygon - набор вершин многоугольника в глобальной 3-D СК
//planePoint - известная точка, принадлежащая плоскости многоугольника
//normal - нормаль к плоскости многоугольника
std::vector<double> findFarthestPointInPlane(   const std::vector< std::vector<double> >& polygon, const std::vector<double>& planePoint, 
                                                const std::vector<double>& normal);

#endif