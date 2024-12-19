#include <stdexcept>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include "../../include/utils/MyMath.hpp"

#define GRIDSIZE 1000

using namespace std;

vector<double> sub(const vector<double>& a, const vector<double>& b){
    if(a.size() != b.size()){
        throw std::runtime_error("Exception in MyMath. Calc sub between differen size vectors!\n");
    }
    vector<double> res(a.size());

    for(int i = 0; i < a.size(); i++){
        res[i] = a[i] - b[i];
    }
    return res;
}

double range(const vector<double> &a, const vector<double>& b){
    if(a.size() != b.size()){
        throw std::runtime_error("Exception in MyMath. Calc range between differen size vectors!\n");
    }
    double range = 0;
    for(int i = 0; i < a.size(); i++){
        range += (b[i] - a[i]) * (b[i] - a[i]);
    }
    return sqrt(range);
}

double squareDistance(const vector<double> &a, const vector<double> &b){
    return (a[0] - b[0]) * (a[0] - b[0]) + 
           (a[1] - b[1]) * (a[1] - b[1]) + 
           (a[2] - b[2]) * (a[2] - b[2]);
}

vector<double> nearestPointFromSample(const vector<double> &a, const vector<double> &b, vector< vector<double> >& sample){
    int nearestID = -1;
    double minDistance = numeric_limits<double>::max();
    double distance = 0;
    for(size_t i = 0; i < sample.size(); i ++){
        distance = squareDistance(a, sample[i]) + squareDistance(b, sample[i]);
        if(distance < minDistance){
            nearestID = i;
            minDistance = distance;
        }
        //distance = 0;
    }
    return sample[nearestID];
}

void normalize(vector<double> &a){
    double norm = 0;
    for(size_t i = 0; i < a.size(); i ++){
        norm += a[i] * a[i];
    }
    norm = sqrt(norm);
    for(size_t i = 0; i < a.size(); i ++){
        a[i] /= norm;
    }
    return;
}

double absVector(const vector<double> &a){
    double res = 0;
    for(size_t i = 0; i < a.size(); i++){
        res += a[i] * a[i];
    }
    return sqrt(res);
}

vector<double> summVector(const vector<double> &a, const vector<double> &b){
    if(a.size() != b.size()){
        throw std::runtime_error("Exception in MyMath. Summ of different size vectors!\n");
    }
    vector<double> res(a.size());
    for(size_t i = 0; i < a.size(); i++){
        res[i] = a[i] + b[i];
    }
    return res;
}

vector<double> crossProduct(const vector<double>& a, const vector<double>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

double dotProduct(const vector<double>& a, const vector<double>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

vector<double> rotate(const vector<double>& a, const vector<double>& b, double Theta){
    vector<double> res(a.size());
    double a_Dot_b = dotProduct(a, b);
    double _cos = cos(Theta);
    double _sin = sin(Theta);
    vector<double> b_Cross_a = crossProduct(b, a);
    for(size_t i = 0; i < res.size(); i++){
        res[i] = a[i] * _cos +  b_Cross_a[i] * _sin + b[i] * a_Dot_b * (1 - _cos);
    }
    return res;
}

//Тут проблема со знаком используемой нормали. Знак нормали и направления обхода точек должны быть согласованы
vector<double> geometricCentroid(const vector< vector<double> > &polygon, const vector<double>& normToPolygon){
    double A = 0; //площадь многоугольника
    
    //Подсчёт площади многоугольника
    vector<double> tmpSquareVec = {0, 0, 0};
    for(size_t i = 0; i < polygon.size() - 1; i++){
        tmpSquareVec = summVector(tmpSquareVec, crossProduct(polygon[i], polygon[i + 1]));
    }
    tmpSquareVec = summVector(tmpSquareVec, crossProduct(polygon[polygon.size() - 1 ], polygon[0]));
    A = 0.5 * absVector(tmpSquareVec);

    cout << A << '\n';
    vector<double> C = {0, 0, 0};
    double tmp_sectionMoment = 0;
    for(size_t i = 0; i < polygon.size() - 1; i ++){
        tmp_sectionMoment =  dotProduct(crossProduct(polygon[i], polygon[i + 1]), normToPolygon);
        for(size_t j = 0; j < C.size(); j++){
            C[j] += (polygon[i][j] + polygon[i + 1][j]) * tmp_sectionMoment; 
       }
    }
    tmp_sectionMoment =  dotProduct(crossProduct(polygon[polygon.size() - 1], polygon[0]), normToPolygon);
    for(size_t j = 0; j < C.size(); j++){
        C[j] += (polygon[polygon.size() - 1][j] + polygon[0][j]) * tmp_sectionMoment; 
        C[j] /= 6 * A; 
    }
    return C;
}

double pointToSegmentDistance(const vector<double>& point, const vector<double>& a, const vector<double>& b) {
    vector<double> ab = sub(b, a);
    vector<double> ap = sub(point, a);

    double t = dotProduct(ap, ab) / dotProduct(ab, ab);
    if (t < 0) {
        return absVector(sub(point, a)); // Closest to vertex `a`
    } else if (t > 1) {
        return absVector(sub(point, b)); // Closest to vertex `b`
    } else {
        // Closest point lies on the segment
        vector<double> closestPoint = summVector(a, {t * ab[0], t * ab[1]});
        return absVector(sub(point, closestPoint));
    }
}
 
void createPlaneBasis(const vector<double>& normal, vector<double>& u, vector<double>& v) {
    if (abs(normal[0]) > abs(normal[1])) {
        u = {-normal[2], 0, normal[0]};
    } else {
        u = {0, -normal[2], normal[1]};
    }
    normalize(u);
    v = crossProduct(normal, u);
    normalize(v);
}

vector<double> toLocalPlane(const vector<double>& point, const vector<double>& planePoint, const vector<double>& u, const vector<double>& v) {
    vector<double> relative = sub(point, planePoint);
    return {dotProduct(relative, u), dotProduct(relative, v)};
}

vector<double> toGlobal3D(const vector<double>& localPoint, const vector<double>& planePoint, const vector<double>& u, const vector<double>& v) {
    return summVector(planePoint, summVector({localPoint[0] * u[0], localPoint[0] * u[1], localPoint[0] * u[2]},
                                             {localPoint[1] * v[0], localPoint[1] * v[1], localPoint[1] * v[2]}));
}

bool isPointInsidePolygon2D(const vector<double>& point, const vector<vector<double>>& polygon) {
    int crossings = 0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % n];

        if ((a[1] > point[1]) != (b[1] > point[1])) {
            double intersectX = a[0] + (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if (point[0] < intersectX) {
                ++crossings;
            }
        }
    }

    return crossings % 2 == 1;
}

vector<double> findFarthestPointInPlane(const vector<vector<double>>& polygon, const vector<double>& planePoint, const vector<double>& normal) {
    int gridSize = GRIDSIZE;

    // Создаем СК в плоскости многоугольника
    vector<double> u, v;
    createPlaneBasis(normal, u, v);

    // Узнаём координаты вершин многоугольника в его СК
    vector<vector<double>> localPolygon;
    for (const auto& vertex : polygon) {
        localPolygon.push_back(toLocalPlane(vertex, planePoint, u, v));
    }

    // Определяем максимальную и минимальную координату многоугольника
    double xMin = numeric_limits<double>::max();
    double xMax = numeric_limits<double>::lowest();
    double yMin = numeric_limits<double>::max();
    double yMax = numeric_limits<double>::lowest();
    for (const auto& p : localPolygon) {
        xMin = min(xMin, p[0]);
        xMax = max(xMax, p[0]);
        yMin = min(yMin, p[1]);
        yMax = max(yMax, p[1]);
    }

    vector<double> farthestPointLocal;
    double maxDistance = -1.0;

    // Производим поиск максимально удалённой точки, проходя по вершинам сетка, наложенной на многоугольник
    for (int i = 0; i <= gridSize; ++i) {
        for (int j = 0; j <= gridSize; ++j) {
            //Очередная вершина сетки
            vector<double> testPointLocal = {xMin + (xMax - xMin) * i / gridSize, yMin + (yMax - yMin) * j / gridSize};
            //Проверяем принадлежит ли вершина сетка многоугольнику
            if (isPointInsidePolygon2D(testPointLocal, localPolygon)) {
                //Расчитываем расстояние от вершины до границы многоугольника
                double minDistance = numeric_limits<double>::max();
                for (size_t k = 0; k < localPolygon.size(); ++k) {
                    const auto& a = localPolygon[k];
                    const auto& b = localPolygon[(k + 1) % localPolygon.size()];
                    minDistance = min(minDistance, pointToSegmentDistance(testPointLocal, a, b));
                }

                // Проверяем самая ли это дальняя точка
                if (minDistance > maxDistance) {
                    maxDistance = minDistance;
                    farthestPointLocal = testPointLocal;
                }
            }
        }
    }

    // Преобразуем результат обратно в глобальную СК
    return toGlobal3D(farthestPointLocal, planePoint, u, v);
}

vector<double> solveLinearSystem(const vector<vector<double>>& A, const vector<double>& b) {
    int n = A.size();
    vector<vector<double>> mat(A);
    vector<double> rhs(b);        

    for (int i = 0; i < n; ++i) {
        int maxRow = i;
        for (int k = i + 1; k < n; ++k) {
            if (abs(mat[k][i]) > abs(mat[maxRow][i])) {
                maxRow = k;
            }
        }
        swap(mat[i], mat[maxRow]);
        swap(rhs[i], rhs[maxRow]);

        if (abs(mat[i][i]) < 1e-10) {
            throw runtime_error("Matrix is singular or nearly singular!");
        }

        for (int k = i + 1; k < n; ++k) {
            double factor = mat[k][i] / mat[i][i];
            for (int j = i; j < n; ++j) {
                mat[k][j] -= factor * mat[i][j];
            }
            rhs[k] -= factor * rhs[i];
        }
    }

    vector<double> x(n, 0);
    for (int i = n - 1; i >= 0; --i) {
        x[i] = rhs[i];
        for (int j = i + 1; j < n; ++j) {
            x[i] -= mat[i][j] * x[j];
        }
        x[i] /= mat[i][i];
    }

    return x;
}

vector<double> fitCubicPolynomial(const vector<vector<double>>& points, int xIndex, int yIndex) {
    int n = points.size();
    if (n < 4) {
        throw runtime_error("At least 4 points are needed to fit a cubic polynomial.");
    }

    double Sx = 0, Sx2 = 0, Sx3 = 0, Sx4 = 0, Sx5 = 0, Sx6 = 0;
    double Sy = 0, Sxy = 0, Sx2y = 0, Sx3y = 0;

    for (const auto& point : points) {
        double xi = point[xIndex];
        double yi = point[yIndex];
        double xi2 = xi * xi;
        double xi3 = xi2 * xi;
        double xi4 = xi3 * xi;
        double xi5 = xi4 * xi;
        double xi6 = xi5 * xi;

        Sx += xi;
        Sx2 += xi2;
        Sx3 += xi3;
        Sx4 += xi4;
        Sx5 += xi5;
        Sx6 += xi6;

        Sy += yi;
        Sxy += xi * yi;
        Sx2y += xi2 * yi;
        Sx3y += xi3 * yi;
    }

    vector<vector<double>> A = {
        {static_cast<double>(n), Sx, Sx2, Sx3},
        {Sx, Sx2, Sx3, Sx4},
        {Sx2, Sx3, Sx4, Sx5},
        {Sx3, Sx4, Sx5, Sx6}
    };

    vector<double> b = {Sy, Sxy, Sx2y, Sx3y};

    return solveLinearSystem(A, b);
}

pair<vector<double>, vector<double>> fitCubicPolynomials3D(const vector<vector<double>>& points) {
    if (points.empty() || points[0].size() < 3) {
        throw runtime_error("Input points must be non-empty and contain at least three coordinates per point (x, y, z).");
    }

    // Fit cubic polynomials for y(x) and z(x)
    vector<double> yCoeffs = fitCubicPolynomial(points, 0, 1); // y as a function of x
    vector<double> zCoeffs = fitCubicPolynomial(points, 0, 2); // z as a function of x

    return {yCoeffs, zCoeffs};
}