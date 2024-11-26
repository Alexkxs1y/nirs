#include "../../include/utils/AerodynamicTable.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std; 

bool DataTable::loadFromFile(const string& filename) {
    
    ifstream file(filename);
    
    if (!file.is_open()) {
        return false;
    }

    string line;
    bool isHeader = true;

    while (getline(file, line)) {

        istringstream stream(line);
        vector<double> row;
        double value;

        while (stream >> value) {
            row.push_back(value);
        }

        if (row.empty()) continue;

        if (isHeader) {
            //Запись первой строки чисел Маха
            M_list = vector<double>(row.begin() + 1, row.end()); // Пропуск первой колонны - углов атак
            isHeader = false;
        } else {
            // Первая колонна - углы атаки
            alpha_list.push_back(row[0]);
            values.emplace_back(row.begin() + 1, row.end());
        }
    }

    file.close();

    return !(M_list.empty() || alpha_list.empty() || values.empty());
}

double DataTable::interpolate(double M, double alpha) const {
    
    if (M >= M_list.back()) {
        M = M_list.back();
    }
    
    if (alpha >= alpha_list.back()) {
        alpha = alpha_list.back();
    }

    // Определение между какими числами Маха находимся
    int mIdx1 = 0, mIdx2 = 0;
    for (size_t i = 0; i < M_list.size() - 1; ++i) {
        if (M >= M_list[i] && M <= M_list[i + 1]) {
            mIdx1 = i;
            mIdx2 = i + 1;
            break;
        }
    }

    // Определение между какими углами атаки находимся
    int aIdx1 = 0, aIdx2 = 0;
    for (size_t i = 0; i < alpha_list.size() - 1; ++i) {
        if (alpha >= alpha_list[i] && alpha <= alpha_list[i + 1]) {
            aIdx1 = i;
            aIdx2 = i + 1;
            break;
        }
    }

    //Интерполяция
    double m1 = M_list[mIdx1];
    double m2 = M_list[mIdx2];
    double a1 = alpha_list[aIdx1];
    double a2 = alpha_list[aIdx2];

    double Q11 = values[aIdx1][mIdx1];
    double Q12 = values[aIdx1][mIdx2];
    double Q21 = values[aIdx2][mIdx1];
    double Q22 = values[aIdx2][mIdx2];

    double R1 = Q11 + (Q12 - Q11) * (M - m1) / (m2 - m1);
    double R2 = Q21 + (Q22 - Q21) * (M - m1) / (m2 - m1);

    return R1 + (R2 - R1) * (alpha - a1) / (a2 - a1);
}
