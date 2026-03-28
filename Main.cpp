/**
 * @file Main.cpp
 * @brief Головна програма моделювання CAP авіаційного двигуна з логуванням.
 *
 * Логування реалізовано через spdlog з двома обробниками:
 * - Console sink: кольоровий вивід у термінал
 * - Rotating file sink: файл logs/sar_simulation.log (ротація 10 MB, 5 архівів)
 *
 * Рівень логування визначається з logger_config.json без перекомпіляції.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <limits>
#include <stdexcept>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"
#include "Logger.h"

#include <spdlog/spdlog.h>

double readDoubleWithDefault(const std::string& prompt, double defaultValue,
                             double minValue = -std::numeric_limits<double>::infinity(),
                             double maxValue =  std::numeric_limits<double>::infinity()) {
    std::string input;
    std::cout << prompt;
    std::getline(std::cin, input);

    if (input.empty()) {
        spdlog::debug("Parameter input empty, using default value: {}", defaultValue);
        return defaultValue;
    }

    std::stringstream ss(input);
    double value = NAN;

    if (ss >> value && ss.eof()) {
        if (value >= minValue && value <= maxValue) {
            spdlog::debug("Parameter entered: {}", value);
            return value;
        } else {
            spdlog::warn("Value {} out of range [{}, {}], using default: {}",
                         value, minValue, maxValue, defaultValue);
        }
    } else {
        spdlog::warn("Failed to parse input '{}', using default: {}", input, defaultValue);
    }

    return defaultValue;
}

int main() {
    // ==========================================
    // ІНІЦІАЛІЗАЦІЯ ЛОГЕРА
    // Рівень визначається з logger_config.json — без перекомпіляції!
    // ==========================================
    try {
        initLogger("logger_config.json");
    } catch (const std::exception& ex) {
        std::cerr << "[WARN] Logger init failed: " << ex.what() << ". Using defaults.\n";
    }

    spdlog::info("=== SAR Simulation started ===");
    spdlog::info("Method: 4th Order Runge-Kutta");

    std::cout << "=== AIRCRAFT ENGINE CONTROL SYSTEM SIMULATION ===" << '\n';
    std::cout << "Method: 4th Order Runge-Kutta" << '\n';
    std::cout << '\n';

    // ==========================================
    // ВВЕДЕННЯ ПАРАМЕТРІВ СИСТЕМИ
    // ==========================================
    spdlog::info("Reading system parameters from user input");
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

    EngineModel::F0 = readDoubleWithDefault(
        "Initial disturbance F0 [default: 10.0]: ",
        10.0, 0.0, 1e6);

    EngineModel::alpha = readDoubleWithDefault(
        "Decay coefficient alpha (1/s) [default: 0.3]: ",
        0.3, 0.0, 1000.0);

    spdlog::info("System parameters: T={}, r={}, k1={}, k2={}, k3={}, F0={}, alpha={}",
                 EngineModel::T, EngineModel::r, EngineModel::k1,
                 EngineModel::k2, EngineModel::k3, EngineModel::F0, EngineModel::alpha);

    double t_start = readDoubleWithDefault(
        "Start time t_start (s) [default: 0.0]: ",
        0.0, 0.0, 1e6);

    double t_end = readDoubleWithDefault(
        "End time t_end (s) [default: 10.0]: ",
        10.0, t_start + 0.001, 1e6);

    double h = readDoubleWithDefault(
        "Integration step h (s) [default: 0.01]: ",
        0.01, 1e-6, (t_end - t_start) / 10.0);

    spdlog::info("Simulation parameters: t=[{}, {}], h={}", t_start, t_end, h);

    // ==========================================
    // ПОЧАТКОВІ УМОВИ
    // ==========================================
    State state = {0.0, 0.0, 0.0, 0.0};
    spdlog::debug("Initial state: x=0, x'=0, x''=0, x'''=0");

    // ==========================================
    // ВІДКРИТТЯ ФАЙЛУ ВИВОДУ
    // ==========================================
    std::ofstream file("simulation_results.csv");
    if (!file.is_open()) {
        // ERR-001: помилка відкриття файлу
        spdlog::error("[ERR-001] Cannot create output file 'simulation_results.csv'. "
                      "Check write permissions in current directory.");
        std::cerr << "Error: Cannot create output file 'simulation_results.csv'" << '\n';
        return 1;
    }
    spdlog::info("Output file 'simulation_results.csv' opened successfully");

    file << "t;x;x_d;x_dd;x_ddd;x_dddd;F\n";

    EngineModel::printParameters();
    file << std::fixed << std::setprecision(6);

    // ==========================================
    // ГОЛОВНИЙ ЦИКЛ ІНТЕГРУВАННЯ
    // ==========================================
    spdlog::info("Starting integration loop: {} steps expected",
                 static_cast<int>((t_end - t_start) / h));

    int step_count = 0;
    try {
        for (double t = t_start; t <= t_end; t += h) {

            State derivatives = EngineModel::computeDerivatives(t, state);

            double x_val      = state[0];
            double x_d_val    = state[1];
            double x_dd_val   = state[2];
            double x_ddd_val  = state[3];
            double x_dddd_val = derivatives[3];
            double F_val      = EngineModel::F(t);

            file << t          << ";"
                 << x_val      << ";"
                 << x_d_val    << ";"
                 << x_dd_val   << ";"
                 << x_ddd_val  << ";"
                 << x_dddd_val << ";"
                 << F_val      << "\n";

            spdlog::debug("Step {}: t={:.4f}, x={:.6f}, F={:.6f}",
                          step_count, t, x_val, F_val);

            state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);
            step_count++;
        }
    } catch (const std::runtime_error& ex) {
        // ERR-002: сингулярна система (T == 0)
        spdlog::critical("[ERR-002] Singular system at step {} (t={:.4f}): {}. "
                         "Context: state=[{:.4f}, {:.4f}, {:.4f}, {:.4f}], T={}. "
                         "Fix: parameter T must be > 1e-10.",
                         step_count, t_start + step_count * h,
                         ex.what(),
                         state[0], state[1], state[2], state[3],
                         EngineModel::T);
        // Повідомлення для кінцевого користувача (без технічних деталей)
        std::cerr << "\n[!] ПОМИЛКА МОДЕЛЮВАННЯ\n"
                  << "    Причина: некоректний параметр T (стала часу).\n"
                  << "    Дія: перезапустіть програму та введіть T > 0.\n"
                  << "    Код помилки: ERR-002\n"
                  << "    Зв'яжіться з розробником: majboroda@example.com\n\n";
        file.close();
        return 1;
    } catch (const std::exception& ex) {
        spdlog::error("[ERR-003] Unexpected error at step {} (t={:.4f}): {}. "
                      "Context: state=[{:.4f}, {:.4f}, {:.4f}, {:.4f}].",
                      step_count, t_start + step_count * h,
                      ex.what(),
                      state[0], state[1], state[2], state[3]);
        std::cerr << "\n[!] НЕПЕРЕДБАЧЕНА ПОМИЛКА\n"
                  << "    Код помилки: ERR-003\n"
                  << "    Зв'яжіться з розробником: majboroda@example.com\n\n";
        file.close();
        return 1;
    }

    file.close();

    spdlog::info("Integration completed: {} steps, results saved to simulation_results.csv",
                 step_count);

    std::cout << "Simulation completed successfully!" << '\n';
    std::cout << "  Number of steps: " << step_count << '\n';
    std::cout << "  Results saved to file: simulation_results.csv" << '\n';
    std::cout << '\n';
    std::cout << "Final system state (t = " << t_end << " s):" << '\n';
    std::cout << "  x   = " << std::setprecision(4) << state[0] << " rev/s" << '\n';
    std::cout << "  x'  = " << std::setprecision(4) << state[1] << " rev/s^2" << '\n';
    std::cout << "  F(t) = " << std::setprecision(4) << EngineModel::F(t_end) << '\n';

    spdlog::info("Final state: x={:.4f} rev/s, x'={:.4f} rev/s^2, F(t_end)={:.4f}",
                 state[0], state[1], EngineModel::F(t_end));

    // ==========================================
    // ВІЗУАЛІЗАЦІЯ
    // ==========================================
    spdlog::info("Starting visualization: python visualize.py");
    int viz_result = system("python visualize.py");

    if (viz_result == 0) {
        spdlog::info("Visualization completed, saved to simulation_results.png");
        std::cout << "Visualization completed. Plots saved to file: simulation_results.png" << '\n';
    } else {
        // ERR-004: помилка запуску візуалізації
        spdlog::warn("[ERR-004] Visualization failed (exit code {}). "
                     "Run manually: python3 visualize.py", viz_result);
        std::cout << "Could not start visualization automatically." << '\n';
        std::cout << "  Run manually: python3 visualize.py" << '\n';
    }

    spdlog::info("=== SAR Simulation finished ===");
    spdlog::shutdown();

    return 0;
}