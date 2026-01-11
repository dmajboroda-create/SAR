#pragma once
#include "Types.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

class EngineModel {
public:
    // ==========================================
    // SYSTEM PARAMETERS from equation (1)
    //
    // Equation (1):
    // T*d^4x/dt^4 + (1 + r*T*k2)*d^3x/dt^3 + T*k1*k2*k3*d^2x/dt^2 =
    //     = k1*T*d^3F/dt^3 + (k1 + r*T*k2)*d^2F/dt^2
    // ==========================================

    // Time constant (seconds)
    static double T;

    // Feedback coefficient (dimensionless)
    static double r;

    // Transfer coefficients (dimensionless)
    static double k1;
    static double k2;
    static double k3;

    // ==========================================
    // DISTURBANCE FUNCTION F(t) - ALWAYS POSITIVE!
    //
    // Physical interpretation: fuel supply to engine
    // F(t) > 0 always (cannot be negative!)
    //
    // Exponential decay pulse:
    // F(t) = F0*exp(-alpha*t)
    // ==========================================

    static double F0;    // Initial fuel supply
    static double alpha;  // Decay rate (1/s)

    static double F(double t) {
        if (t < 0) return 0.0;
        // Exponential decay - always positive
        return F0 * std::exp(-alpha * t);
    }

    // ==========================================
    // DERIVATIVES OF DISTURBANCE F(t)
    //
    // For F(t) = F0*e^(-alpha*t):
    //
    // F'(t)   = -alpha*F0*e^(-alpha*t)     = -alpha*F(t)
    // F''(t)  = alpha^2*F0*e^(-alpha*t)    = alpha^2*F(t)
    // F'''(t) = -alpha^3*F0*e^(-alpha*t)   = -alpha^3*F(t)
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
    // y0 = x        ->  dy0/dt = y1       (kinematics)
    // y1 = x'       ->  dy1/dt = y2       (kinematics)
    // y2 = x''      ->  dy2/dt = y3       (kinematics)
    // y3 = x'''     ->  dy3/dt = x(4)     (dynamics)
    //
    // From equation (1), expressing x(4):
    //
    // T*x(4) = k1*T*F''' + (k1 + r*T*k2)*F''
    //          - (1 + r*T*k2)*x''' - T*k1*k2*k3*x''
    //
    // x(4) = [k1*T*F''' + (k1 + r*T*k2)*F''
    //         - (1 + r*T*k2)*x''' - T*k1*k2*k3*x''] / T
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
        double F_dd = F_second_derivative(t);   // d^2F/dt^2
        double F_ddd = F_third_derivative(t);   // d^3F/dt^3

        // Calculate coefficients from equation (1)
        double coef_x_triple = (1.0 + r * T * k2);     // Coefficient at x'''
        double coef_x_double = T * k1 * k2 * k3;        // Coefficient at x''
        double coef_F_dd = (k1 + r * T * k2);           // Coefficient at F''
        double coef_F_ddd = k1 * T;                     // Coefficient at F'''

        // Check for singularity (T cannot be zero)
        if (std::abs(T) < 1e-10) {
            throw std::runtime_error("T cannot be zero (singular system).");
        }

        // Calculate x(4) from equation (1)
        // x(4) = [k1*T*F''' + (k1 + r*T*k2)*F'' - (1 + r*T*k2)*x''' - T*k1*k2*k3*x''] / T
        double x_fourth = (coef_F_ddd * F_ddd + coef_F_dd * F_dd
                          - coef_x_triple * x_triple
                          - coef_x_double * x_double) / T;

        dydt[3] = x_fourth;  // d(x''')/dt = x(4)

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
        std::cout << "F0    = " << F0 << " (initial value)" << std::endl;
        std::cout << "alpha = " << alpha << " (decay coefficient, 1/s)" << std::endl;
        std::cout << std::endl;
        std::cout << "Coefficients in notation (2):" << std::endl;
        std::cout << "C1 = " << compute_C1() << std::endl;
        std::cout << "C2 = " << compute_C2() << std::endl;
        std::cout << "C3 = " << compute_C3() << std::endl;
        std::cout << "=========================" << std::endl;
    }
};

// Initialize static members
double EngineModel::T = 0.0;
double EngineModel::r = 0.0;
double EngineModel::k1 = 0.0;
double EngineModel::k2 = 0.0;
double EngineModel::k3 = 0.0;
double EngineModel::F0 = 0.0;
double EngineModel::alpha = 0.0;