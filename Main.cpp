#include <iostream>
#include <fstream>
#include <iomanip>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"

int main() {
    // ==========================================
    // ПАРАМЕТРИ СИМУЛЯЦІЇ
    // ==========================================
    const double t_start = 0.0;      // Початковий час
    const double t_end = 10.0;       // Кінцевий час моделювання
    const double h = 0.01;           // Крок інтегрування (чим менше, тим точніше)

    // ==========================================
    // ПОЧАТКОВІ УМОВИ
    // Початковий стан: двигун стоїть, всі похідні = 0
    // state = {x, x', x'', x'''}
    // ==========================================
    State state = {0.0, 0.0, 0.0, 0.0}; 

    // ==========================================
    // ВІДКРИТТЯ ФАЙЛУ для запису результатів
    // ==========================================
    std::ofstream file("simulation_results.csv");
    if (!file.is_open()) {
        std::cerr << "Помилка відкриття файлу!" << std::endl;
        return 1;
    }

    // ==========================================
    // ЗАГОЛОВКИ CSV
    // t      - час
    // x      - кількість обертів (вихід системи)
    // x_d    - швидкість зміни обертів (x')
    // x_dd   - прискорення (x'')
    // x_ddd  - третя похідна (x''')
    // x_dddd - четверта похідна (x⁽⁴⁾)
    // F      - збурення F(t)
    // ==========================================
    file << "t;x;x_d;x_dd;x_ddd;x_dddd;F\n"; 

    // Виведення параметрів системи
    std::cout << "=== РОЗВ'ЯЗАННЯ РІВНЯННЯ (1) ===" << std::endl;
    std::cout << "Метод: Рунге-Кутта 4-го порядку" << std::endl;
    std::cout << "Інтервал часу: [" << t_start << ", " << t_end << "]" << std::endl;
    std::cout << "Крок інтегрування: h = " << h << std::endl;
    std::cout << std::endl;

    EngineModel::printParameters();
    
    std::cout << std::endl << "Розрахунок..." << std::endl;

    // Налаштування точності виводу
    file << std::fixed << std::setprecision(6);

    // ==========================================
    // ОСНОВНИЙ ЦИКЛ ІНТЕГРУВАННЯ
    // ==========================================
    int step_count = 0;
    for (double t = t_start; t <= t_end; t += h) {
        
        // Отримуємо похідні в поточній точці
        State derivatives = EngineModel::computeDerivatives(t, state);
        
        // Поточний стан
        double x_val     = state[0];        // x
        double x_d_val   = state[1];        // x'
        double x_dd_val  = state[2];        // x''
        double x_ddd_val = state[3];        // x'''
        double x_dddd_val = derivatives[3]; // x⁽⁴⁾
        
        // Збурення
        double F_val = EngineModel::F(t);

        // Запис у файл
        file << t << ";" 
             << x_val << ";" 
             << x_d_val << ";" 
             << x_dd_val << ";" 
             << x_ddd_val << ";"
             << x_dddd_val << ";"
             << F_val << "\n";

        // Крок інтегрування методом Рунге-Кутта 4-го порядку
        state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);
        
        step_count++;
    }

    file.close();

    std::cout << "✓ Розрахунок завершено!" << std::endl;
    std::cout << "Кількість кроків: " << step_count << std::endl;
    std::cout << "Результати збережено у файл: simulation_results.csv" << std::endl;
    std::cout << std::endl;
    std::cout << "Для аналізу результатів можна:" << std::endl;
    std::cout << "1. Відкрити CSV файл у Excel/LibreOffice" << std::endl;
    std::cout << "2. Побудувати графіки x(t), x'(t), x''(t)" << std::endl;
    std::cout << "3. Перевірити поведінку системи при збуренні F(t)" << std::endl;

    return 0;
}
