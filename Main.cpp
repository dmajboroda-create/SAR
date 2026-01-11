#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <limits>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"

// Function to read a double value with default option
double readDoubleWithDefault(const std::string& prompt, double defaultValue, double minValue = -std::numeric_limits<double>::infinity(), double maxValue = std::numeric_limits<double>::infinity()) {
    std::string input;
    std::cout << prompt;
    std::getline(std::cin, input);

    // If empty input, return default value
    if (input.empty()) {
        return defaultValue;
    }

    // Try to parse the input
    std::stringstream ss(input);
    double value;

    if (ss >> value && ss.eof()) {
        // Check if value is within allowed range
        if (value >= minValue && value <= maxValue) {
            return value;
        }
    }

    // If parsing failed or value out of range, return default
    return defaultValue;
}

int main() {
    std::cout << "=== AIRCRAFT ENGINE CONTROL SYSTEM SIMULATION ===" << std::endl;
    std::cout << "Method: 4th Order Runge-Kutta" << std::endl;
    std::cout << std::endl;

    // ==========================================
    // INPUT SYSTEM PARAMETERS
    // ==========================================
    std::cout << "Enter system parameters (press Enter for default values):" << std::endl;
    std::cout << std::endl;

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

    std::cout << std::endl;
    std::cout << "Disturbance parameters F(t) = F0*exp(-alpha*t):" << std::endl;

    EngineModel::F0 = readDoubleWithDefault(
        "Initial disturbance F0 [default: 10.0]: ",
        10.0, 0.0, 1e6);

    EngineModel::alpha = readDoubleWithDefault(
        "Decay coefficient alpha (1/s) [default: 0.3]: ",
        0.3, 0.0, 1000.0);

    std::cout << std::endl;

    // ==========================================
    // INPUT SIMULATION PARAMETERS
    // ==========================================
    std::cout << "Enter simulation parameters:" << std::endl;
    std::cout << std::endl;

    double t_start = readDoubleWithDefault(
        "Start time t_start (s) [default: 0.0]: ",
        0.0, 0.0, 1e6);

    double t_end = readDoubleWithDefault(
        "End time t_end (s) [default: 10.0]: ",
        10.0, t_start + 0.001, 1e6);

    double h = readDoubleWithDefault(
        "Integration step h (s) [default: 0.01]: ",
        0.01, 1e-6, (t_end - t_start) / 10.0);

    std::cout << std::endl;

    // ==========================================
    // INITIAL CONDITIONS
    // Initial state: engine at rest, all derivatives = 0
    // state = {x, x', x'', x'''}
    // ==========================================
    State state = {0.0, 0.0, 0.0, 0.0};

    // ==========================================
    // OPEN OUTPUT FILE for saving results
    // ==========================================
    std::ofstream file("simulation_results.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create output file 'simulation_results.csv'" << std::endl;
        return 1;
    }

    // CSV headers
    file << "t;x;x_d;x_dd;x_ddd;x_dddd;F\n";

    std::cout << "Time interval: [" << t_start << ", " << t_end << "] s" << std::endl;
    std::cout << "Integration step: h = " << h << " s" << std::endl;
    std::cout << std::endl;

    EngineModel::printParameters();
    std::cout << std::endl;
    file << std::fixed << std::setprecision(6);

    // ==========================================
    // MAIN INTEGRATION LOOP
    // ==========================================
    int step_count = 0;
    for (double t = t_start; t <= t_end; t += h) {

        // Get derivatives at current point
        State derivatives = EngineModel::computeDerivatives(t, state);

        // Current system state
        double x_val     = state[0];        // x - revolutions
        double x_d_val   = state[1];        // x' - rate of change
        double x_dd_val  = state[2];        // x'' - acceleration
        double x_ddd_val = state[3];        // x''' - third derivative
        double x_dddd_val = derivatives[3]; // x(4) - fourth derivative

        // Disturbance value
        double F_val = EngineModel::F(t);

        // Write to file
        file << t << ";"
             << x_val << ";"
             << x_d_val << ";"
             << x_dd_val << ";"
             << x_ddd_val << ";"
             << x_dddd_val << ";"
             << F_val << "\n";

        // Integration step using 4th order Runge-Kutta method
        state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);

        step_count++;
    }

    file.close();

    std::cout << "Simulation completed successfully!" << std::endl;
    std::cout << "  Number of steps: " << step_count << std::endl;
    std::cout << "  Results saved to file: simulation_results.csv" << std::endl;
    std::cout << std::endl;
    std::cout << "Final system state (t = " << t_end << " s):" << std::endl;
    std::cout << "  x   = " << std::setprecision(4) << state[0] << " rev/s" << std::endl;
    std::cout << "  x'  = " << std::setprecision(4) << state[1] << " rev/s^2" << std::endl;
    std::cout << "  F(t) = " << std::setprecision(4) << EngineModel::F(t_end) << std::endl;
    std::cout << std::endl;

    std::cout << "Starting results visualization..." << std::endl;
    int viz_result = system("python visualize.py");

    if (viz_result == 0) {
        std::cout << "Visualization completed. Plots saved to file: simulation_results.png" << std::endl;
    } else {
        std::cout << "Could not start visualization automatically." << std::endl;
        std::cout << "  Run manually: python3 visualize.py" << std::endl;
    }

    return 0;
}