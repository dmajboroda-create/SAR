#pragma once
#include "Types.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

/**
 * @file EngineModel_slow.h
 * @brief Навмисно неоптимізована версія EngineModel для порівняння продуктивності.
 *
 * Відмінності від оптимізованої версії:
 * 1. std::exp() викликається 6 разів на крок замість 1
 * 2. std::pow() замість простого множення для alpha^2, alpha^3
 * 3. Зайва перевірка std::abs(T) всередині циклу
 * 4. Проміжні змінні обчислюються двічі
 */
class EngineModelSlow {
public:
    static double T;
    static double r;
    static double k1;
    static double k2;
    static double k3;
    static double F0;
    static double alpha;

    // ПОГАНО: окремий виклик exp() в кожній функції
    static double F(double t) {
        if (t < 0) { return 0.0; }
        return F0 * std::exp(-alpha * t);  // exp #1
    }

    static double F_first_derivative(double t) {
        // ПОГАНО: викликає F(t) → ще один exp
        return -alpha * F(t);  // exp #2
    }

    static double F_second_derivative(double t) {
        // ПОГАНО: std::pow замість alpha*alpha
        // ПОГАНО: викликає F(t) → ще один exp
        return std::pow(alpha, 2) * F(t);  // exp #3 + pow
    }

    static double F_third_derivative(double t) {
        // ПОГАНО: std::pow замість alpha*alpha*alpha
        // ПОГАНО: викликає F(t) → ще один exp
        return -std::pow(alpha, 3) * F(t);  // exp #4 + pow
    }

    static State computeDerivatives(double t, const State& current_state) {
        State dydt;

        double x_double = current_state[2];
        double x_triple = current_state[3];

        dydt[0] = current_state[1];
        dydt[1] = current_state[2];
        dydt[2] = current_state[3];

        // ПОГАНО: три окремі виклики exp() через F_second і F_third
        double F_dd  = F_second_derivative(t);   // exp #5
        double F_ddd = F_third_derivative(t);    // exp #6

        // ПОГАНО: коефіцієнти обчислюються з дублюванням
        double coef_x_triple = 1.0 + r * T * k2;
        double coef_x_double = T * k1 * k2 * k3;
        double coef_F_dd     = k1 + r * T * k2;  // r*T*k2 обчислюється знову
        double coef_F_ddd    = k1 * T;

        // ПОГАНО: перевірка T всередині функції що викликається мільйони разів
        if (std::abs(T) < 1e-10) {
            throw std::runtime_error("T cannot be zero (singular system).");
        }

        double x_fourth = (coef_F_ddd * F_ddd + coef_F_dd * F_dd
                          - coef_x_triple * x_triple
                          - coef_x_double * x_double) / T;

        dydt[3] = x_fourth;
        return dydt;
    }

    static double compute_C1() { return T * k1 * k2 * k3; }
    static double compute_C2() { return (1.0 + r * T * k2); }
    static double compute_C3() { return T; }
};

double EngineModelSlow::T     = 0.0;
double EngineModelSlow::r     = 0.0;
double EngineModelSlow::k1    = 0.0;
double EngineModelSlow::k2    = 0.0;
double EngineModelSlow::k3    = 0.0;
double EngineModelSlow::F0    = 0.0;
double EngineModelSlow::alpha = 0.0;