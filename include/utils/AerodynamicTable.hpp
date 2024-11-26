#ifndef DATA_TABLE_H
#define DATA_TABLE_H

#include <vector>
#include <string>

class DataTable {
private:
    std::vector<double> M_list;  //Строка чисел Маха
    std::vector<double> alpha_list; //Колонна углов атаки
    std::vector<std::vector<double>> values; //Таблица из экспресса

public:
    
    bool loadFromFile(const std::string& filename); //Превращение .txt в таблицу

    double interpolate(double mach, double alpha); //Интерполяция
};

#endif // DATA_TABLE_H
