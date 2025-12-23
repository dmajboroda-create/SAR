#pragma once
#include <array>

// Розмірність системи (порядок диф. рівняння)
// Рівняння (1) має 4-й порядок, тому система має 4 змінні
constexpr size_t SYSTEM_ORDER = 4; 

// State - це {x, x', x'', x'''} у поточний момент часу
using State = std::array<double, SYSTEM_ORDER>;

// Функція правої частини рівняння: dy/dt = f(t, y)
using DerivativeFunc = State (*)(double t, const State& y);
