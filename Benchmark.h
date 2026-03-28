#pragma once

/**
 * @file Benchmark.h
 * @brief Утиліти для вимірювання продуктивності компонентів системи SAR.
 *
 * Реалізує клас Timer для точного вимірювання часу виконання
 * та клас BenchmarkRunner для запуску серій вимірювань.
 *
 * @author Мажборода Д. М.
 * @date 2025
 */

#include <chrono>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

/**
 * @brief Клас для точного вимірювання часу виконання коду.
 *
 * Використовує std::chrono::high_resolution_clock для точності
 * до наносекунд.
 */
class Timer {
public:
    /** @brief Запускає таймер. */
    void start() {
        m_start = std::chrono::high_resolution_clock::now();
    }

    /** @brief Зупиняє таймер. */
    void stop() {
        m_end = std::chrono::high_resolution_clock::now();
    }

    /** @brief Повертає час виконання у мілісекундах. */
    double elapsedMs() const {
        return std::chrono::duration<double, std::milli>(m_end - m_start).count();
    }

    /** @brief Повертає час виконання у мікросекундах. */
    double elapsedUs() const {
        return std::chrono::duration<double, std::micro>(m_end - m_start).count();
    }

private:
    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_end;
};

/**
 * @brief Результат одного бенчмарку.
 */
struct BenchmarkResult {
    std::string name;       ///< Назва тесту
    int steps;              ///< Кількість кроків інтегрування
    double h;               ///< Крок інтегрування
    double t_end;           ///< Кінець інтервалу
    double total_ms;        ///< Загальний час (мс)
    double per_step_us;     ///< Час на крок (мкс)
    double rk4_ms;          ///< Час RK4 (мс)
    double deriv_ms;        ///< Час computeDerivatives (мс)
    double exp_ms;          ///< Час F(t)/exp (мс)
    double io_ms;           ///< Час запису CSV (мс)
};

/**
 * @brief Виводить результати бенчмарку у форматований звіт.
 */
inline void printResults(const std::vector<BenchmarkResult>& results) {
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "         SAR PERFORMANCE BENCHMARK RESULTS                    \n";
    std::cout << "==============================================================\n\n";

    std::cout << std::left
              << std::setw(12) << "Scenario"
              << std::setw(8)  << "Steps"
              << std::setw(8)  << "h (s)"
              << std::setw(12) << "Total (ms)"
              << std::setw(14) << "Per step (us)"
              << std::setw(12) << "RK4 (ms)"
              << std::setw(14) << "Derivat (ms)"
              << std::setw(10) << "I/O (ms)"
              << "\n";
    std::cout << std::string(90, '-') << "\n";

    for (const auto& r : results) {
        std::cout << std::left << std::fixed << std::setprecision(3)
                  << std::setw(12) << r.name
                  << std::setw(8)  << r.steps
                  << std::setw(8)  << r.h
                  << std::setw(12) << r.total_ms
                  << std::setw(14) << r.per_step_us
                  << std::setw(12) << r.rk4_ms
                  << std::setw(14) << r.deriv_ms
                  << std::setw(10) << r.io_ms
                  << "\n";
    }
    std::cout << "\n";
}

/**
 * @brief Зберігає результати бенчмарку у CSV-файл.
 */
inline void saveResultsCsv(const std::vector<BenchmarkResult>& results,
                           const std::string& filename = "benchmark_results.csv") {
    std::ofstream f(filename);
    f << "name;steps;h;total_ms;per_step_us;rk4_ms;deriv_ms;exp_ms;io_ms\n";
    for (const auto& r : results) {
        f << std::fixed << std::setprecision(6)
          << r.name << ";"
          << r.steps << ";"
          << r.h << ";"
          << r.total_ms << ";"
          << r.per_step_us << ";"
          << r.rk4_ms << ";"
          << r.deriv_ms << ";"
          << r.exp_ms << ";"
          << r.io_ms << "\n";
    }
    std::cout << "Results saved to " << filename << "\n";
}