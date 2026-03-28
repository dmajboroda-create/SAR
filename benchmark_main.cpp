/**
 * @file benchmark_main.cpp
 * @brief Програма профілювання продуктивності системи SAR.
 *
 * Запускає три тестових сценарії:
 * - Малий:    h=0.1,   t=[0,10]  —    100 кроків
 * - Середній: h=0.01,  t=[0,10]  —   1000 кроків
 * - Великий:  h=0.001, t=[0,100] — 100000 кроків
 *
 * Вимірює час:
 * - загальний час моделювання
 * - час RKSolver::step()
 * - час EngineModel::computeDerivatives()
 * - час EngineModel::F() (std::exp)
 * - час запису CSV (I/O)
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"
#include "Benchmark.h"

/**
 * @brief Ініціалізує параметри моделі за замовчуванням.
 */
void initDefaultParams() {
    EngineModel::T     = 0.1;
    EngineModel::r     = 1.5;
    EngineModel::k1    = 2.0;
    EngineModel::k2    = 1.0;
    EngineModel::k3    = 0.5;
    EngineModel::F0    = 10.0;
    EngineModel::alpha = 0.3;
}

/**
 * @brief Запускає один сценарій профілювання.
 *
 * @param name   Назва сценарію
 * @param t_end  Кінець інтервалу моделювання
 * @param h      Крок інтегрування
 * @return BenchmarkResult з результатами вимірювань
 */
BenchmarkResult runScenario(const std::string& name, double t_end, double h) {
    initDefaultParams();

    BenchmarkResult result;
    result.name  = name;
    result.h     = h;
    result.t_end = t_end;
    result.steps = static_cast<int>((t_end - 0.0) / h);

    State state = {0.0, 0.0, 0.0, 0.0};
    Timer totalTimer, rk4Timer, derivTimer, expTimer, ioTimer;

    // Тимчасовий CSV файл
    std::string csvFile = "bench_" + name + ".csv";
    std::ofstream file(csvFile);
    file << "t;x;F\n";
    file << std::fixed << std::setprecision(6);

    double deriv_total = 0.0;
    double rk4_total   = 0.0;
    double exp_total   = 0.0;
    double io_total    = 0.0;

    totalTimer.start();

    for (double t = 0.0; t <= t_end; t += h) {

        // Вимірюємо computeDerivatives окремо
        derivTimer.start();
        State derivatives = EngineModel::computeDerivatives(t, state);
        derivTimer.stop();
        deriv_total += derivTimer.elapsedUs();

        // Вимірюємо F(t) / std::exp окремо
        expTimer.start();
        double F_val = EngineModel::F(t);
        expTimer.stop();
        exp_total += expTimer.elapsedUs();

        // Вимірюємо RK4 step
        rk4Timer.start();
        state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);
        rk4Timer.stop();
        rk4_total += rk4Timer.elapsedUs();

        // Вимірюємо I/O
        ioTimer.start();
        file << t << ";" << state[0] << ";" << F_val << "\n";
        ioTimer.stop();
        io_total += ioTimer.elapsedUs();
    }

    totalTimer.stop();
    file.close();

    result.total_ms    = totalTimer.elapsedMs();
    result.per_step_us = (result.steps > 0) ? (result.total_ms * 1000.0 / result.steps) : 0.0;
    result.rk4_ms      = rk4_total   / 1000.0;
    result.deriv_ms    = deriv_total / 1000.0;
    result.exp_ms      = exp_total   / 1000.0;
    result.io_ms       = io_total    / 1000.0;

    return result;
}

int main() {
    std::cout << "=== SAR Performance Benchmark ===\n";
    std::cout << "Parameters: T=0.1, r=1.5, k1=2.0, k2=1.0, k3=0.5, F0=10.0, alpha=0.3\n\n";

    std::vector<BenchmarkResult> results;

    // ==========================================
    // СЦЕНАРІЙ 1: МАЛИЙ (100 кроків)
    // ==========================================
    std::cout << "Running scenario: Small (100 steps)...\n";
    results.push_back(runScenario("Small", 10.0, 0.1));

    // ==========================================
    // СЦЕНАРІЙ 2: СЕРЕДНІЙ (1000 кроків)
    // ==========================================
    std::cout << "Running scenario: Medium (1000 steps)...\n";
    results.push_back(runScenario("Medium", 10.0, 0.01));

    // ==========================================
    // СЦЕНАРІЙ 3: ВЕЛИКИЙ (100000 кроків)
    // ==========================================
    std::cout << "Running scenario: Large (100000 steps)...\n";
    results.push_back(runScenario("Large", 100.0, 0.001));

    // ==========================================
    // ВИВЕДЕННЯ РЕЗУЛЬТАТІВ
    // ==========================================
    printResults(results);
    saveResultsCsv(results);

    // ==========================================
    // АНАЛІЗ ГАРЯЧИХ ТОЧОК
    // ==========================================
    std::cout << "=== HOT SPOTS ANALYSIS ===\n\n";
    for (const auto& r : results) {
        double total_us = r.total_ms * 1000.0;
        std::cout << "Scenario: " << r.name << "\n";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "  RK4 step():              "
                  << (r.rk4_ms * 1000.0 / total_us * 100.0) << "% of total\n";
        std::cout << "  computeDerivatives():    "
                  << (r.deriv_ms * 1000.0 / total_us * 100.0) << "% of total\n";
        std::cout << "  F(t) / std::exp():       "
                  << (r.exp_ms * 1000.0 / total_us * 100.0) << "% of total\n";
        std::cout << "  I/O (CSV write):         "
                  << (r.io_ms * 1000.0 / total_us * 100.0) << "% of total\n\n";
    }

    return 0;
}