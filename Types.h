#pragma once
#include <array>

// System dimension (order of differential equation)
// Equation (1) is 4th order, so system has 4 variables
constexpr size_t SYSTEM_ORDER = 4;

// State is {x, x', x'', x'''} at current time
using State = std::array<double, SYSTEM_ORDER>;

// Right-hand side function of equation: dy/dt = f(t, y)
using DerivativeFunc = State (*)(double t, const State& y);
