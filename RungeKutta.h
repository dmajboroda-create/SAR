#pragma once
#include "Types.h"

/**
 * @file RungeKutta.h
 * @brief Реалізація методу Рунге-Кутта 4-го порядку.
 *
 * Надає клас RKSolver для чисельного інтегрування системи
 * диференціальних рівнянь з точністю O(h^4).
 *
 * @author Мажборода Д. М.
 * @date 2025
 */

/**
 * @brief Клас для чисельного інтегрування методом Рунге-Кутта 4-го порядку.
 *
 * Реалізує класичний RK4-алгоритм для розв'язання систем звичайних
 * диференціальних рівнянь вигляду dy/dt = f(t, y).
 *
 * Формула RK4:
 * @code
 * k1 = f(t, y)
 * k2 = f(t + h/2, y + h*k1/2)
 * k3 = f(t + h/2, y + h*k2/2)
 * k4 = f(t + h, y + h*k3)
 * y_{n+1} = y_n + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
 * @endcode
 *
 * @see EngineModel::computeDerivatives()
 */
class RKSolver {
public:
    /**
     * @brief Виконує один крок інтегрування методом Рунге-Кутта 4-го порядку.
     *
     * Обчислює наближене значення вектора стану у момент часу t + h,
     * використовуючи поточний стан y та функцію правої частини f.
     *
     * Похибка методу має порядок O(h^4) на кроці та O(h^4) глобально.
     *
     * @param t  Поточний момент часу (секунди)
     * @param y  Поточний вектор стану {x, x', x'', x'''}
     * @param h  Крок інтегрування (секунди), має бути > 0
     * @param f  Вказівник на функцію правої частини dy/dt = f(t, y)
     * @return   Вектор стану у момент часу t + h
     *
     * @note Точність O(h^4) досягається при достатньо малому кроці h.
     *       Для задачі CAP авіаційного двигуна рекомендується h <= 0.01 с.
     *
     * @example
     * @code
     * State y = {0.0, 0.0, 0.0, 0.0};
     * double t = 0.0, h = 0.01;
     * State y_next = RKSolver::step(t, y, h, EngineModel::computeDerivatives);
     * @endcode
     */
    static State step(double t, const State& y, double h, DerivativeFunc f) {
        // Calculate k1
        State k1 = f(t, y);

        // Calculate k2
        State y2;
        for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y2[i] = y[i] + (k1[i] * (h / 2.0));
        }
        State k2 = f(t + (h / 2.0), y2);

        // Calculate k3
        State y3;
        for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y3[i] = y[i] + (k2[i] * (h / 2.0));
        }
        State k3 = f(t + (h / 2.0), y3);

        // Calculate k4
        State y4;
        for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y4[i] = y[i] + (k3[i] * h);
        }
        State k4 = f(t + h, y4);

        // Final: y_{n+1} = y_n + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
        State result;
        for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
            result[i] = y[i] + (h / 6.0) * (k1[i] + (2.0 * k2[i]) + (2.0 * k3[i]) + k4[i]);
        }

        return result;
    }
};