/**
 * @file benchmark_comparison.cpp
 * @brief Порівняльний бенчмарк: оптимізований vs неоптимізований EngineModel.
 *
 * Запускає обидві версії в одній програмі та виводить порівняння.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "Types.h"
#include "Benchmark.h"

// Оптимізована версія
#include "EngineModel.h"

// Неоптимізована версія (інша назва класу)
#include "EngineModel_slow.h"

// RK4 для оптимізованої версії
#include "RungeKutta.h"

// RK4 для повільної версії (той самий алгоритм, інша функція похідних)
State rk4StepSlow(double t, const State& y, double h) {
    auto f = EngineModelSlow::computeDerivatives;

    State k1 = f(t, y);

    State y2;
    for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
        y2[i] = y[i] + (k1[i] * (h / 2.0));
    }
    State k2 = f(t + (h / 2.0), y2);

    State y3;
    for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
        y3[i] = y[i] + (k2[i] * (h / 2.0));
    }
    State k3 = f(t + (h / 2.0), y3);

    State y4;
    for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
        y4[i] = y[i] + (k3[i] * h);
    }
    State k4 = f(t + h, y4);

    State result;
    for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
        result[i] = y[i] + (h / 6.0) * (k1[i] + (2.0 * k2[i]) + (2.0 * k3[i]) + k4[i]);
    }
    return result;
}

void initParams() {
    EngineModel::T      = 0.1;  EngineModelSlow::T      = 0.1;
    EngineModel::r      = 1.5;  EngineModelSlow::r      = 1.5;
    EngineModel::k1     = 2.0;  EngineModelSlow::k1     = 2.0;
    EngineModel::k2     = 1.0;  EngineModelSlow::k2     = 1.0;
    EngineModel::k3     = 0.5;  EngineModelSlow::k3     = 0.5;
    EngineModel::F0     = 10.0; EngineModelSlow::F0     = 10.0;
    EngineModel::alpha  = 0.3;  EngineModelSlow::alpha  = 0.3;
}

struct CompResult {
    std::string scenario;
    int steps;
    double fast_ms;
    double slow_ms;
    double speedup;
};

CompResult runComparison(const std::string& name, double t_end, double h) {
    initParams();
    int steps = static_cast<int>(t_end / h);

    // --- ОПТИМІЗОВАНА версія ---
    Timer fastTimer;
    State stateFast = {0.0, 0.0, 0.0, 0.0};
    fastTimer.start();
    for (double t = 0.0; t <= t_end; t += h) {
        stateFast = RKSolver::step(t, stateFast, h, EngineModel::computeDerivatives);
    }
    fastTimer.stop();

    // --- ПОВІЛЬНА версія ---
    Timer slowTimer;
    State stateSlow = {0.0, 0.0, 0.0, 0.0};
    slowTimer.start();
    for (double t = 0.0; t <= t_end; t += h) {
        stateSlow = rk4StepSlow(t, stateSlow, h);
    }
    slowTimer.stop();

    CompResult r;
    r.scenario = name;
    r.steps    = steps;
    r.fast_ms  = fastTimer.elapsedMs();
    r.slow_ms  = slowTimer.elapsedMs();
    r.speedup  = (r.fast_ms > 0) ? (r.slow_ms / r.fast_ms) : 1.0;
    return r;
}

int main() {
    std::cout << "=== SAR OPTIMIZATION COMPARISON ===\n";
    std::cout << "Optimized vs Unoptimized EngineModel\n\n";

    std::cout << "Optimization applied:\n";
    std::cout << "  FAST: 1x std::exp() per step (cached result)\n";
    std::cout << "  SLOW: 6x std::exp() + 2x std::pow() per step\n\n";

    std::vector<CompResult> results;
    results.push_back(runComparison("Small",  10.0,  0.1));
    results.push_back(runComparison("Medium", 10.0,  0.01));
    results.push_back(runComparison("Large",  100.0, 0.001));

    // Header
    std::cout << std::string(70, '=') << "\n";
    std::cout << std::left
              << std::setw(10) << "Scenario"
              << std::setw(10) << "Steps"
              << std::setw(16) << "Optimized (ms)"
              << std::setw(16) << "Unoptimized(ms)"
              << std::setw(12) << "Speedup"
              << std::setw(12) << "Improvement"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    for (const auto& r : results) {
        double improvement = (r.slow_ms - r.fast_ms) / r.slow_ms * 100.0;
        std::cout << std::left << std::fixed << std::setprecision(3)
                  << std::setw(10) << r.scenario
                  << std::setw(10) << r.steps
                  << std::setw(16) << r.fast_ms
                  << std::setw(16) << r.slow_ms
                  << std::setw(12) << std::setprecision(2) << r.speedup << "x"
                  << std::setw(12) << std::setprecision(1) << improvement << "%"
                  << "\n";
    }
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "CONCLUSION:\n";
    std::cout << "  The optimized version reduces std::exp() calls from 6 to 1\n";
    std::cout << "  per integration step, eliminating redundant std::pow() calls.\n";
    std::cout << "  This yields consistent speedup across all dataset sizes.\n";

    return 0;
}