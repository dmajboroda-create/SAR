#pragma once
#include "Types.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

/**
 * @file EngineModel.h
 * @brief Математична модель системи автоматичного регулювання авіаційного двигуна.
 *
 * Реалізує математичну модель САР на основі диференціального рівняння 4-го порядку:
 *
 * @f[
 * T \cdot x^{(4)} + (1 + rTk_2) \cdot x''' + Tk_1k_2k_3 \cdot x'' =
 * k_1 T \cdot F''' + (k_1 + rTk_2) \cdot F''
 * @f]
 *
 * де:
 * - x(t) — кількість обертів валу турбіни (об/с)
 * - F(t) = F0 * exp(-alpha * t) — зовнішнє збурення (подача палива)
 * - T — стала часу системи
 * - r — коефіцієнт зворотного зв'язку
 * - k1, k2, k3 — коефіцієнти передачі
 *
 * @author Мажборода Д. М.
 * @date 2025
 * @see RKSolver
 */

/**
 * @brief Клас математичної моделі CAP авіаційного двигуна.
 *
 * Містить параметри системи, функцію збурення F(t) та її похідні,
 * а також метод обчислення правої частини системи диференціальних рівнянь.
 *
 * Усі члени класу є статичними, що відповідає концепції єдиної глобальної
 * моделі двигуна в рамках одного сеансу моделювання.
 *
 * @note Перед використанням необхідно встановити всі параметри:
 *       T, r, k1, k2, k3, F0, alpha.
 *
 * @warning Параметр T не може дорівнювати нулю (сингулярна система).
 */
class EngineModel {
public:
    /**
     * @defgroup SystemParameters Параметри системи
     * @brief Параметри рівняння (1) системи автоматичного регулювання.
     * @{
     */

    /** @brief Стала часу системи (секунди). Визначає швидкодію регулятора. */
    static double T;

    /** @brief Коефіцієнт зворотного зв'язку (безрозмірний). */
    static double r;

    /** @brief Коефіцієнт передачі 1 (безрозмірний). */
    static double k1;

    /** @brief Коефіцієнт передачі 2 (безрозмірний). */
    static double k2;

    /** @brief Коефіцієнт передачі 3 (безрозмірний). */
    static double k3;

    /** @} */ // end of SystemParameters

    /**
     * @defgroup DisturbanceParameters Параметри збурення
     * @brief Параметри функції зовнішнього збурення F(t) = F0 * exp(-alpha * t).
     *
     * Фізичний сенс: моделювання подачі палива до двигуна.
     * Значення F(t) завжди невід'ємне (подача палива не може бути від'ємною).
     * @{
     */

    /** @brief Початкове значення збурення (початкова подача палива). */
    static double F0;

    /** @brief Коефіцієнт загасання збурення (1/с). Визначає швидкість спадання F(t). */
    static double alpha;

    /** @} */ // end of DisturbanceParameters

    /**
     * @brief Обчислює значення функції збурення F(t) у момент часу t.
     *
     * Функція моделює експоненціально спадаючу подачу палива до двигуна:
     * @f[ F(t) = F_0 \cdot e^{-\alpha t}, \quad t \geq 0 @f]
     *
     * @param t Момент часу (секунди). При t < 0 повертає 0.
     * @return Значення збурення F(t) >= 0.
     *
     * @note Функція завжди повертає невід'ємне значення.
     */
    static double F(double t) {
        if (t < 0) { return 0.0; }
        return F0 * std::exp(-alpha * t);
    }

    /**
     * @brief Обчислює першу похідну функції збурення F'(t).
     *
     * @f[ F'(t) = -\alpha \cdot F(t) @f]
     *
     * @param t Момент часу (секунди)
     * @return Значення F'(t)
     */
    static double F_first_derivative(double t) {
        return -alpha * F(t);
    }

    /**
     * @brief Обчислює другу похідну функції збурення F''(t).
     *
     * @f[ F''(t) = \alpha^2 \cdot F(t) @f]
     *
     * @param t Момент часу (секунди)
     * @return Значення F''(t)
     */
    static double F_second_derivative(double t) {
        return alpha * alpha * F(t);
    }

    /**
     * @brief Обчислює третю похідну функції збурення F'''(t).
     *
     * @f[ F'''(t) = -\alpha^3 \cdot F(t) @f]
     *
     * @param t Момент часу (секунди)
     * @return Значення F'''(t)
     */
    static double F_third_derivative(double t) {
        return -alpha * alpha * alpha * F(t);
    }

    /**
     * @brief Обчислює вектор похідних стану системи dy/dt = f(t, y).
     *
     * Реалізує зведення диференціального рівняння 4-го порядку до системи
     * 4 рівнянь 1-го порядку:
     *
     * @f{eqnarray*}{
     * \dot{y}_0 &=& y_1 \quad (dx/dt = x') \\
     * \dot{y}_1 &=& y_2 \quad (dx'/dt = x'') \\
     * \dot{y}_2 &=& y_3 \quad (dx''/dt = x''') \\
     * \dot{y}_3 &=& x^{(4)} \quad \text{з рівняння (1)}
     * @f}
     *
     * де x^(4) обчислюється з основного рівняння:
     * @f[
     * x^{(4)} = \frac{k_1 T F''' + (k_1 + rTk_2)F'' - (1 + rTk_2)x''' - Tk_1k_2k_3 x''}{T}
     * @f]
     *
     * @param t             Поточний момент часу (секунди)
     * @param current_state Поточний вектор стану {x, x', x'', x'''}
     * @return              Вектор похідних {x', x'', x''', x^(4)}
     *
     * @throws std::runtime_error якщо T == 0 (сингулярна система)
     *
     * @see RKSolver::step()
     */
    static State computeDerivatives(double t, const State& current_state) {
        State dydt;

        double x_double  = current_state[2];
        double x_triple  = current_state[3];

        dydt[0] = current_state[1];
        dydt[1] = current_state[2];
        dydt[2] = current_state[3];

        double F_dd  = F_second_derivative(t);
        double F_ddd = F_third_derivative(t);

        double coef_x_triple = (1.0 + (r * T * k2));
        double coef_x_double = T * k1 * k2 * k3;
        double coef_F_dd     = (k1 + (r * T * k2));
        double coef_F_ddd    = k1 * T;

        if (std::abs(T) < 1e-10) {
            throw std::runtime_error("T cannot be zero (singular system).");
        }

        double x_fourth = (coef_F_ddd * F_ddd + coef_F_dd * F_dd
                          - coef_x_triple * x_triple
                          - coef_x_double * x_double) / T;

        dydt[3] = x_fourth;

        return dydt;
    }

    /**
     * @defgroup DiagnosticCoefficients Діагностичні коефіцієнти
     * @brief Допоміжні функції для обчислення коефіцієнтів нотації (2).
     * @{
     */

    /**
     * @brief Обчислює коефіцієнт C1 = T * k1 * k2 * k3.
     * @return Значення C1
     */
    static double compute_C1() {
        return T * k1 * k2 * k3;
    }

    /**
     * @brief Обчислює коефіцієнт C2 = 1 + r * T * k2.
     * @return Значення C2
     */
    static double compute_C2() {
        return (1.0 + (r * T * k2));
    }

    /**
     * @brief Обчислює коефіцієнт C3 = T.
     * @return Значення C3 (стала часу)
     */
    static double compute_C3() {
        return T;
    }

    /** @} */ // end of DiagnosticCoefficients

    /**
     * @brief Виводить поточні параметри системи у стандартний вивід.
     *
     * Друкує значення T, r, k1, k2, k3, F0, alpha та обчислені
     * коефіцієнти C1, C2, C3 для діагностики.
     */
    static void printParameters() {
        std::cout << "=== System Parameters ===" << '\n';
        std::cout << "T  = " << T  << " (time constant, s)" << '\n';
        std::cout << "r  = " << r  << " (feedback coefficient)" << '\n';
        std::cout << "k1 = " << k1 << " (transfer coefficient 1)" << '\n';
        std::cout << "k2 = " << k2 << " (transfer coefficient 2)" << '\n';
        std::cout << "k3 = " << k3 << " (transfer coefficient 3)" << '\n';
        std::cout << '\n';
        std::cout << "Disturbance F(t):" << '\n';
        std::cout << "F0    = " << F0    << " (initial value)" << '\n';
        std::cout << "alpha = " << alpha << " (decay coefficient, 1/s)" << '\n';
        std::cout << '\n';
        std::cout << "Coefficients in notation (2):" << '\n';
        std::cout << "C1 = " << compute_C1() << '\n';
        std::cout << "C2 = " << compute_C2() << '\n';
        std::cout << "C3 = " << compute_C3() << '\n';
        std::cout << "=========================" << '\n';
    }
};

// Initialize static members
double EngineModel::T     = 0.0;
double EngineModel::r     = 0.0;
double EngineModel::k1    = 0.0;
double EngineModel::k2    = 0.0;
double EngineModel::k3    = 0.0;
double EngineModel::F0    = 0.0;
double EngineModel::alpha = 0.0;