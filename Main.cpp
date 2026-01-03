#include <iostream>
#include <fstream>
#include <iomanip>

#include "Types.h"
#include "EngineModel.h"
#include "RungeKutta.h"

int main() {
    // ==========================================
    // SIMULATION PARAMETERS
    // ==========================================
    const double t_start = 0.0;      // Start time
    const double t_end = 10.0;       // End time of simulation
    const double h = 0.01;           // Integration step (smaller step = higher accuracy)

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
    // t      - time (s)
    // x      - revolutions (rev/s) - system output
    // x_d    - rate of change of revolutions (x') - (rev/s²)
    // x_dd   - acceleration (x'') - (rev/s³)
    // x_ddd  - third derivative (x''') - (rev/s⁴)
    // x_dddd - fourth derivative (x⁽⁴⁾) - (rev/s⁵)
    // F      - disturbance F(t)
    file << "t;x;x_d;x_dd;x_ddd;x_dddd;F\n";

    std::cout << "=== AIRCRAFT ENGINE CONTROL SYSTEM SIMULATION ===" << std::endl;
    std::cout << "Method: 4th Order Runge-Kutta" << std::endl;
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
        double x_dddd_val = derivatives[3]; // x⁽⁴⁾ - fourth derivative

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
    std::cout << "  x      = " << std::setprecision(4) << state[0] << " rev/s" << std::endl;
    std::cout << "  x'     = " << std::setprecision(4) << state[1] << " rev/s²" << std::endl;
    std::cout << "  F(t)   = " << std::setprecision(4) << EngineModel::F(t_end) << std::endl;
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