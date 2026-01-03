#pragma once
#include "Types.h"
#include <cmath>
#include <stdexcept>

class EngineModel {
public:
    // ==========================================
    // SYSTEM PARAMETERS from equation (1)
    //
    // Equation (1):
    // T·d⁴x/dt⁴ + (1 + r·T·k₂)·d³x/dt³ + T·k₁·k₂·k₃·d²x/dt² =
    //     = k₁·T·d³F/dt³ + (k₁ + r·T·k₂)·d²F/dt²
    // ==========================================

    // Time constant (seconds)
    static constexpr double T = 0.1;

    // Feedback coefficient (dimensionless)
    static constexpr double r = 1.5;

    // Transfer coefficients (dimensionless)
    static constexpr double k1 = 2.0;
    static constexpr double k2 = 1.0;
    static constexpr double k3 = 0.5;

    // ==========================================
    // DISTURBANCE FUNCTION F(t) - ALWAYS POSITIVE!
    //
    // Physical interpretation: fuel supply to engine
    // F(t) > 0 always (cannot be negative!)
    //
    // Exponential decay pulse:
    // F(t) = F₀·exp(-α·t)
    // ==========================================

    static constexpr double F0 = 10.0;    // Initial fuel supply
    static constexpr double alpha = 0.3;  // Decay rate (1/s)

    static double F(double t) {
        if (t < 0) return 0.0;
        // Exponential decay - always positive
        return F0 * std::exp(-alpha * t);
    }

    // ==========================================
    // DERIVATIVES OF DISTURBANCE F(t)
    //
    // For F(t) = F₀·e^(-α·t):
    //
    // F'(t)   = -α·F₀·e^(-α·t)     = -α·F(t)
    // F''(t)  = α²·F₀·e^(-α·t)     = α²·F(t)
    // F'''(t) = -α³·F₀·e^(-α·t)    = -α³·F(t)
    // ==========================================

    static double F_first_derivative(double t) {
        return -alpha * F(t);
    }

    static double F_second_derivative(double t) {
        return alpha * alpha * F(t);
    }

    static double F_third_derivative(double t) {
        return -alpha * alpha * alpha * F(t);
    }

    // ==========================================
    // SYSTEM OF DIFFERENTIAL EQUATIONS
    //
    // Reducing 4th order equation to 1st order system:
    //
    // y₀ = x        →  dy₀/dt = y₁       (kinematics)
    // y₁ = x'       →  dy₁/dt = y₂       (kinematics)
    // y₂ = x''      →  dy₂/dt = y₃       (kinematics)
    // y₃ = x'''     →  dy₃/dt = x⁽⁴⁾     (dynamics)
    //
    // From equation (1), expressing x⁽⁴⁾:
    //
    // T·x⁽⁴⁾ = k₁·T·F⁽³⁾ + (k₁ + r·T·k₂)·F⁽²⁾
    //          - (1 + r·T·k₂)·x⁽³⁾ - T·k₁·k₂·k₃·x⁽²⁾
    //
    // x⁽⁴⁾ = [k₁·T·F⁽³⁾ + (k₁ + r·T·k₂)·F⁽²⁾
    //         - (1 + r·T·k₂)·x⁽³⁾ - T·k₁·k₂·k₃·x⁽²⁾] / T
    // ==========================================
    static State computeDerivatives(double t, const State& current_state) {
        State dydt;

        // Unpacking state vector
        // current_state[0] = x      (position)
        // current_state[1] = x'     (velocity)
        // current_state[2] = x''    (acceleration)
        // current_state[3] = x'''   (third derivative)

        double x_double  = current_state[2];  // x''  (second derivative)
        double x_triple  = current_state[3];  // x''' (third derivative)

        // Kinematic relations (order reduction)
        dydt[0] = current_state[1];  // dx/dt = x'
        dydt[1] = current_state[2];  // d(x')/dt = x''
        dydt[2] = current_state[3];  // d(x'')/dt = x'''

        // Dynamics (from equation 1)
        // Get disturbance derivatives at current time
        double F_dd = F_second_derivative(t);   // d²F/dt²
        double F_ddd = F_third_derivative(t);   // d³F/dt³

        // Calculate coefficients from equation (1)
        double coef_x_triple = (1.0 + r * T * k2);     // Coefficient at x'''
        double coef_x_double = T * k1 * k2 * k3;        // Coefficient at x''
        double coef_F_dd = (k1 + r * T * k2);           // Coefficient at F''
        double coef_F_ddd = k1 * T;                     // Coefficient at F'''

        // Check for singularity (T cannot be zero)
        if (std::abs(T) < 1e-10) {
            throw std::runtime_error("T cannot be zero (singular system).");
        }

        // Calculate x⁽⁴⁾ from equation (1)
        // x⁽⁴⁾ = [k₁·T·F⁽³⁾ + (k₁ + r·T·k₂)·F⁽²⁾ - (1 + r·T·k₂)·x⁽³⁾ - T·k₁·k₂·k₃·x⁽²⁾] / T
        double x_fourth = (coef_F_ddd * F_ddd + coef_F_dd * F_dd
                          - coef_x_triple * x_triple
                          - coef_x_double * x_double) / T;

        dydt[3] = x_fourth;  // d(x''')/dt = x⁽⁴⁾

        return dydt;
    }

    // ==========================================
    // HELPER FUNCTIONS for analysis
    // ==========================================

    // Calculate coefficients from notation (2) for diagnostics
    static double compute_C1() {
        return T * k1 * k2 * k3;
    }

    static double compute_C2() {
        return (1.0 + r * T * k2);
    }

    static double compute_C3() {
        return T;
    }

    // Print system parameters
    static void printParameters() {
        std::cout << "=== System Parameters ===" << std::endl;
        std::cout << "T  = " << T << " (time constant, s)" << std::endl;
        std::cout << "r  = " << r << " (feedback coefficient)" << std::endl;
        std::cout << "k1 = " << k1 << " (transfer coefficient 1)" << std::endl;
        std::cout << "k2 = " << k2 << " (transfer coefficient 2)" << std::endl;
        std::cout << "k3 = " << k3 << " (transfer coefficient 3)" << std::endl;
        std::cout << std::endl;
        std::cout << "Disturbance F(t):" << std::endl;
        std::cout << "F₀ = " << F0 << " (initial value)" << std::endl;
        std::cout << "α  = " << alpha << " (decay coefficient, 1/s)" << std::endl;
        std::cout << std::endl;
        std::cout << "Coefficients in notation (2):" << std::endl;
        std::cout << "C1 = " << compute_C1() << std::endl;
        std::cout << "C2 = " << compute_C2() << std::endl;
        std::cout << "C3 = " << compute_C3() << std::endl;
        std::cout << "=========================" << std::endl;
    }
};