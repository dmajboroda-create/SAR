/**
 * @file Main.cpp
 * @brief Головна програма моделювання системи автоматичного регулювання авіаційного двигуна.
 *
 * Реалізує консольний інтерфейс для введення параметрів системи,
 * цикл чисельного інтегрування методом Рунге-Кутта 4-го порядку
 * та експорт результатів у CSV-формат з автоматичною візуалізацією.
 *
 * ## Порядок роботи програми:
 * 1. Введення параметрів системи (T, r, k1, k2, k3, F0, alpha)
 * 2. Введення параметрів симуляції (t_start, t_end, h)
 * 3. Чисельне інтегрування методом RK4
 * 4. Збереження результатів у simulation_results.csv
 * 5. Автоматичний запуск visualize.py для побудови графіків
 *
 * @author Мажборода Д. М.
 * @date 2025
 * @see EngineModel, RKSolver
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <limits>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"

/**
 * @brief Зчитує дійсне число з введення користувача з підтримкою значення за замовчуванням.
 *
 * Якщо користувач натискає Enter без введення значення — повертає defaultValue.
 * Якщо введене значення виходить за межі [minValue, maxValue] — також повертає defaultValue.
 * Якщо введення не є числом — повертає defaultValue.
 *
 * @param prompt       Рядок запиту, що виводиться користувачу
 * @param defaultValue Значення за замовчуванням (при порожньому введенні)
 * @param minValue     Мінімально допустиме значення (за замовчуванням -inf)
 * @param maxValue     Максимально допустиме значення (за замовчуванням +inf)
 * @return             Введене користувачем значення або defaultValue
 *
 * @note Функція не кидає виняток — при будь-якій помилці повертає defaultValue.
 */
double readDoubleWithDefault(const std::string& prompt, double defaultValue,
                             double minValue = -std::numeric_limits<double>::infinity(),
                             double maxValue =  std::numeric_limits<double>::infinity()) {
    std::string input;
    std::cout << prompt;
    std::getline(std::cin, input);

    if (input.empty()) {
        return defaultValue;
    }

    std::stringstream ss(input);
    double value = NAN;

    if (ss >> value && ss.eof()) {
        if (value >= minValue && value <= maxValue) {
            return value;
        }
    }

    return defaultValue;
}

/**
 * @brief Точка входу програми моделювання CAP авіаційного двигуна.
 *
 * Виконує повний цикл моделювання:
 * - зчитує параметри системи та симуляції від користувача;
 * - запускає чисельне інтегрування методом RK4;
 * - зберігає результати (t, x, x', x'', x''', x^(4), F) у CSV;
 * - запускає Python-скрипт visualize.py для побудови графіків.
 *
 * @return 0 при успішному завершенні, 1 при помилці відкриття файлу.
 */
int main() {
    std::cout << "=== AIRCRAFT ENGINE CONTROL SYSTEM SIMULATION ===" << '\n';
    std::cout << "Method: 4th Order Runge-Kutta" << '\n';
    std::cout << '\n';

    // ==========================================
    // INPUT SYSTEM PARAMETERS
    // ==========================================
    std::cout << "Enter system parameters (press Enter for default values):" << '\n';
    std::cout << '\n';

    EngineModel::T = readDoubleWithDefault(
        "Time constant T (seconds) [default: 0.1]: ",
        0.1, 1e-6, 1000.0);

    EngineModel::r = readDoubleWithDefault(
        "Feedback coefficient r [default: 1.5]: ",
        1.5, -100.0, 100.0);

    EngineModel::k1 = readDoubleWithDefault(
        "Transfer coefficient k1 [default: 2.0]: ",
        2.0, -1000.0, 1000.0);

    EngineModel::k2 = readDoubleWithDefault(
        "Transfer coefficient k2 [default: 1.0]: ",
        1.0, -1000.0, 1000.0);

    EngineModel::k3 = readDoubleWithDefault(
        "Transfer coefficient k3 [default: 0.5]: ",
        0.5, -1000.0, 1000.0);

    std::cout << '\n';
    std::cout << "Disturbance parameters F(t) = F0*exp(-alpha*t):" << '\n';

    EngineModel::F0 = readDoubleWithDefault(
        "Initial disturbance F0 [default: 10.0]: ",
        10.0, 0.0, 1e6);

    EngineModel::alpha = readDoubleWithDefault(
        "Decay coefficient alpha (1/s) [default: 0.3]: ",
        0.3, 0.0, 1000.0);

    std::cout << '\n';

    // ==========================================
    // INPUT SIMULATION PARAMETERS
    // ==========================================
    std::cout << "Enter simulation parameters:" << '\n';
    std::cout << '\n';

    double t_start = readDoubleWithDefault(
        "Start time t_start (s) [default: 0.0]: ",
        0.0, 0.0, 1e6);

    double t_end = readDoubleWithDefault(
        "End time t_end (s) [default: 10.0]: ",
        10.0, t_start + 0.001, 1e6);

    double h = readDoubleWithDefault(
        "Integration step h (s) [default: 0.01]: ",
        0.01, 1e-6, (t_end - t_start) / 10.0);

    std::cout << '\n';

    // ==========================================
    // INITIAL CONDITIONS
    // ==========================================
    State state = {0.0, 0.0, 0.0, 0.0};

    // ==========================================
    // OPEN OUTPUT FILE
    // ==========================================
    std::ofstream file("simulation_results.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create output file 'simulation_results.csv'" << '\n';
        return 1;
    }

    file << "t;x;x_d;x_dd;x_ddd;x_dddd;F\n";

    std::cout << "Time interval: [" << t_start << ", " << t_end << "] s" << '\n';
    std::cout << "Integration step: h = " << h << " s" << '\n';
    std::cout << '\n';

    EngineModel::printParameters();
    std::cout << '\n';
    file << std::fixed << std::setprecision(6);

    // ==========================================
    // MAIN INTEGRATION LOOP
    // ==========================================
    int step_count = 0;
    for (double t = t_start; t <= t_end; t += h) {

        State derivatives = EngineModel::computeDerivatives(t, state);

        double x_val      = state[0];
        double x_d_val    = state[1];
        double x_dd_val   = state[2];
        double x_ddd_val  = state[3];
        double x_dddd_val = derivatives[3];
        double F_val      = EngineModel::F(t);

        file << t         << ";"
             << x_val     << ";"
             << x_d_val   << ";"
             << x_dd_val  << ";"
             << x_ddd_val << ";"
             << x_dddd_val << ";"
             << F_val     << "\n";

        state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);
        step_count++;
    }

    file.close();

    std::cout << "Simulation completed successfully!" << '\n';
    std::cout << "  Number of steps: " << step_count << '\n';
    std::cout << "  Results saved to file: simulation_results.csv" << '\n';
    std::cout << '\n';
    std::cout << "Final system state (t = " << t_end << " s):" << '\n';
    std::cout << "  x   = " << std::setprecision(4) << state[0] << " rev/s" << '\n';
    std::cout << "  x'  = " << std::setprecision(4) << state[1] << " rev/s^2" << '\n';
    std::cout << "  F(t) = " << std::setprecision(4) << EngineModel::F(t_end) << '\n';
    std::cout << '\n';

    std::cout << "Starting results visualization..." << '\n';
    int viz_result = system("python visualize.py");

    if (viz_result == 0) {
        std::cout << "Visualization completed. Plots saved to file: simulation_results.png" << '\n';
    } else {
        std::cout << "Could not start visualization automatically." << '\n';
        std::cout << "  Run manually: python3 visualize.py" << '\n';
    }

    return 0;
}