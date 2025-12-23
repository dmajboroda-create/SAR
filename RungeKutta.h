#pragma once
#include "Types.h"

class RKSolver {
public:
    // Один крок методу Рунге-Кутта 4-го порядку
    // 
    // Формула РК-4:
    // k₁ = f(t, y)
    // k₂ = f(t + h/2, y + h·k₁/2)
    // k₃ = f(t + h/2, y + h·k₂/2)
    // k₄ = f(t + h, y + h·k₃)
    // y_{n+1} = y_n + (h/6)·(k₁ + 2k₂ + 2k₃ + k₄)
    //
    static State step(double t, const State& y, double h, DerivativeFunc f) {
        // Обчислення k₁
        State k1 = f(t, y);
        
        // Обчислення k₂
        State y2;
        for(size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y2[i] = y[i] + k1[i] * (h / 2.0);
        }
        State k2 = f(t + h / 2.0, y2);

        // Обчислення k₃
        State y3;
        for(size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y3[i] = y[i] + k2[i] * (h / 2.0);
        }
        State k3 = f(t + h / 2.0, y3);

        // Обчислення k₄
        State y4;
        for(size_t i = 0; i < SYSTEM_ORDER; ++i) {
            y4[i] = y[i] + k3[i] * h;
        }
        State k4 = f(t + h, y4);

        // Фінальне обчислення: y_{n+1} = y_n + (h/6)·(k₁ + 2k₂ + 2k₃ + k₄)
        State result;
        for (size_t i = 0; i < SYSTEM_ORDER; ++i) {
            result[i] = y[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        
        return result;
    }
};
