#include <gtest/gtest.h>
#include <cmath>
#include "EngineModel.h"
#include "RungeKutta.h"
#include "Types.h"

// ==========================================
// ТЕСТИ ФУНКЦІЇ ЗБУРЕННЯ F(t)
// ==========================================

// Тест 1.1: F(t) для від'ємного часу має повертати 0
TEST(DisturbanceFunctionTests, NegativeTimeReturnsZero) {
    double result = EngineModel::F(-1.0);
    EXPECT_DOUBLE_EQ(result, 0.0) << "F(t) для t < 0 має дорівнювати 0";
}

// Тест 1.2: F(t) для t=0 має повертати F₀ = 10.0
TEST(DisturbanceFunctionTests, ZeroTimeReturnsInitialValue) {
    double result = EngineModel::F(0.0);
    EXPECT_NEAR(result, 10.0, 1e-6) << "F(0) має дорівнювати F₀ = 10.0";
}

// Тест 1.3: F(t) завжди невід'ємна для t ≥ 0
TEST(DisturbanceFunctionTests, AlwaysNonNegative) {
    for (double t = 0.0; t <= 10.0; t += 0.5) {
        double result = EngineModel::F(t);
        EXPECT_GE(result, 0.0) << "F(" << t << ") має бути невід'ємною";
    }
}

// Тест 1.4: F(t) спадає з часом (експоненціальне загасання)
TEST(DisturbanceFunctionTests, DecreasingOverTime) {
    double f1 = EngineModel::F(1.0);
    double f2 = EngineModel::F(2.0);
    double f3 = EngineModel::F(5.0);
    
    EXPECT_GT(f1, f2) << "F(1) має бути більшою за F(2)";
    EXPECT_GT(f2, f3) << "F(2) має бути більшою за F(5)";
}

// Тест 1.5: F(t) прямує до нуля при великих t
TEST(DisturbanceFunctionTests, ApproachesZeroAtLargeTime) {
    double result = EngineModel::F(50.0);
    EXPECT_LT(result, 0.001) << "F(50) має бути близькою до 0";
}

// Тест 1.6: Перевірка формули F(t) = F₀·exp(-α·t)
TEST(DisturbanceFunctionTests, MatchesExpectedFormula) {
    double t = 2.0;
    double F0 = 10.0;
    double alpha = 0.3;
    double expected = F0 * std::exp(-alpha * t);
    double result = EngineModel::F(t);
    
    EXPECT_NEAR(result, expected, 1e-9) << "F(t) не відповідає формулі F₀·exp(-α·t)";
}

// ==========================================
// ТЕСТИ ПОХІДНИХ ФУНКЦІЇ ЗБУРЕННЯ
// ==========================================

// Тест 2.1: F'(0) має дорівнювати -α·F₀ = -3.0
TEST(DerivativeTests, FirstDerivativeAtZero) {
    double result = EngineModel::F_first_derivative(0.0);
    double expected = -0.3 * 10.0; // -α·F₀
    EXPECT_NEAR(result, expected, 1e-6) << "F'(0) має дорівнювати -3.0";
}

// Тест 2.2: F'(t) завжди від'ємна для t > 0 (спадна функція)
TEST(DerivativeTests, FirstDerivativeIsNegative) {
    for (double t = 0.1; t <= 10.0; t += 1.0) {
        double result = EngineModel::F_first_derivative(t);
        EXPECT_LT(result, 0.0) << "F'(" << t << ") має бути від'ємною";
    }
}

// Тест 2.3: F'(t) = -α·F(t)
TEST(DerivativeTests, FirstDerivativeMatchesFormula) {
    double t = 3.0;
    double alpha = 0.3;
    double expected = -alpha * EngineModel::F(t);
    double result = EngineModel::F_first_derivative(t);
    
    EXPECT_NEAR(result, expected, 1e-9) << "F'(t) не відповідає формулі -α·F(t)";
}

// Тест 2.4: F''(t) = α²·F(t)
TEST(DerivativeTests, SecondDerivativeMatchesFormula) {
    double t = 2.5;
    double alpha = 0.3;
    double expected = alpha * alpha * EngineModel::F(t);
    double result = EngineModel::F_second_derivative(t);
    
    EXPECT_NEAR(result, expected, 1e-9) << "F''(t) не відповідає формулі α²·F(t)";
}

// Тест 2.5: F'''(t) = -α³·F(t)
TEST(DerivativeTests, ThirdDerivativeMatchesFormula) {
    double t = 1.5;
    double alpha = 0.3;
    double expected = -alpha * alpha * alpha * EngineModel::F(t);
    double result = EngineModel::F_third_derivative(t);
    
    EXPECT_NEAR(result, expected, 1e-9) << "F'''(t) не відповідає формулі -α³·F(t)";
}

// ==========================================
// ТЕСТИ computeDerivatives()
// ==========================================

// Тест 3.1: Кінематичні зв'язки для нульового стану
TEST(ComputeDerivativesTests, KinematicRelationsAtZero) {
    State state = {0.0, 0.0, 0.0, 0.0};
    State derivatives = EngineModel::computeDerivatives(0.0, state);
    
    // dydt[0] = state[1] = 0
    EXPECT_DOUBLE_EQ(derivatives[0], 0.0);
    // dydt[1] = state[2] = 0
    EXPECT_DOUBLE_EQ(derivatives[1], 0.0);
    // dydt[2] = state[3] = 0
    EXPECT_DOUBLE_EQ(derivatives[2], 0.0);
}

// Тест 3.2: Кінематичні зв'язки для ненульового стану
TEST(ComputeDerivativesTests, KinematicRelationsNonZero) {
    State state = {1.0, 2.0, 3.0, 4.0};
    State derivatives = EngineModel::computeDerivatives(1.0, state);
    
    EXPECT_DOUBLE_EQ(derivatives[0], state[1]) << "dydt[0] має дорівнювати state[1]";
    EXPECT_DOUBLE_EQ(derivatives[1], state[2]) << "dydt[1] має дорівнювати state[2]";
    EXPECT_DOUBLE_EQ(derivatives[2], state[3]) << "dydt[2] має дорівнювати state[3]";
}

// Тест 3.3: Четверта похідна обчислюється коректно
TEST(ComputeDerivativesTests, FourthDerivativeComputation) {
    State state = {0.0, 0.0, 0.0, 0.0};
    double t = 1.0;
    State derivatives = EngineModel::computeDerivatives(t, state);
    
    // Для нульового стану x⁽⁴⁾ залежить тільки від F(t)
    // x⁽⁴⁾ = (k₁·T·F''' + (k₁ + r·T·k₂)·F'') / T
    double T = 0.1, r = 1.5, k1 = 2.0, k2 = 1.0;
    double F_dd = EngineModel::F_second_derivative(t);
    double F_ddd = EngineModel::F_third_derivative(t);
    
    double expected = (k1 * T * F_ddd + (k1 + r * T * k2) * F_dd) / T;
    
    EXPECT_NEAR(derivatives[3], expected, 1e-6) 
        << "x⁽⁴⁾ не відповідає диференціальному рівнянню";
}

// Тест 3.4: Розмірність вектора похідних
TEST(ComputeDerivativesTests, OutputDimensionCorrect) {
    State state = {1.0, 2.0, 3.0, 4.0};
    State derivatives = EngineModel::computeDerivatives(1.0, state);
    
    EXPECT_EQ(derivatives.size(), SYSTEM_ORDER) 
        << "Розмірність вектора похідних має дорівнювати " << SYSTEM_ORDER;
}

// ==========================================
// ТЕСТИ МЕТОДУ РУНГЕ-КУТТА
// ==========================================

// Проста функція для тестування: dy/dt = -y
State simple_derivative(double t, const State& y) {
    State dydt;
    dydt[0] = -y[0];
    return dydt;
}

// Тест 4.1: Перевірка точності методу RK4 для простої задачі
// dy/dt = -y, y(0) = 1, точний розв'язок: y(t) = exp(-t)
TEST(RungeKuttaTests, AccuracyForSimpleProblem) {
    State y = {1.0, 0.0, 0.0, 0.0}; // y(0) = 1
    double t = 0.0;
    double h = 0.1;
    
    // Один крок RK4
    State y_next = RKSolver::step(t, y, h, simple_derivative);
    
    // Точний розв'язок: y(0.1) = exp(-0.1)
    double expected = std::exp(-0.1);
    
    // Похибка має бути менше 10^-6 (вимога R2.3)
    EXPECT_NEAR(y_next[0], expected, 1e-6) 
        << "Похибка методу RK4 перевищує допустиму 10^-6";
}

// Тест 4.2: Перевірка для декількох кроків
TEST(RungeKuttaTests, MultipleStepsAccuracy) {
    State y = {1.0, 0.0, 0.0, 0.0};
    double t = 0.0;
    double h = 0.01;
    int steps = 100; // до t = 1.0
    
    for (int i = 0; i < steps; ++i) {
        y = RKSolver::step(t, y, h, simple_derivative);
        t += h;
    }
    
    double expected = std::exp(-1.0);
    EXPECT_NEAR(y[0], expected, 1e-4) 
        << "Накопичена похибка після " << steps << " кроків занадто велика";
}

// Тест 4.3: Розмірність вихідного вектора
TEST(RungeKuttaTests, OutputDimensionCorrect) {
    State y = {1.0, 2.0, 3.0, 4.0};
    State y_next = RKSolver::step(0.0, y, 0.1, EngineModel::computeDerivatives);
    
    EXPECT_EQ(y_next.size(), SYSTEM_ORDER) 
        << "Розмірність вихідного вектора має дорівнювати " << SYSTEM_ORDER;
}

// Тест 4.4: Перевірка стабільності для реальної системи
TEST(RungeKuttaTests, RealSystemStability) {
    State state = {0.0, 0.0, 0.0, 0.0};
    double t = 0.0;
    double h = 0.01;
    
    // Виконуємо 100 кроків
    for (int i = 0; i < 100; ++i) {
        state = RKSolver::step(t, state, h, EngineModel::computeDerivatives);
        t += h;
        
        // Перевірка, що значення не стають нескінченними
        for (size_t j = 0; j < SYSTEM_ORDER; ++j) {
            EXPECT_FALSE(std::isinf(state[j])) 
                << "Значення стало нескінченним на кроці " << i;
            EXPECT_FALSE(std::isnan(state[j])) 
                << "Значення стало NaN на кроці " << i;
        }
    }
}

// Тест 4.5: Перевірка, що крок вперед змінює стан
TEST(RungeKuttaTests, StateChangesAfterStep) {
    State y_initial = {1.0, 0.0, 0.0, 0.0};
    State y_next = RKSolver::step(0.0, y_initial, 0.1, simple_derivative);
    
    EXPECT_NE(y_next[0], y_initial[0]) 
        << "Стан має змінитися після кроку інтегрування";
}

// ==========================================
// ТЕСТИ ОБЧИСЛЕННЯ КОЕФІЦІЄНТІВ СИСТЕМИ
// ==========================================

// Тест 5.1: C₁ = T·k₁·k₂·k₃
TEST(CoefficientsTests, C1Computation) {
    double expected = 0.1 * 2.0 * 1.0 * 0.5; // T·k₁·k₂·k₃
    double result = EngineModel::compute_C1();
    
    EXPECT_NEAR(result, expected, 1e-9) 
        << "C₁ має дорівнювати T·k₁·k₂·k₃ = " << expected;
}

// Тест 5.2: C₂ = 1 + r·T·k₂
TEST(CoefficientsTests, C2Computation) {
    double expected = 1.0 + 1.5 * 0.1 * 1.0; // 1 + r·T·k₂
    double result = EngineModel::compute_C2();
    
    EXPECT_NEAR(result, expected, 1e-9) 
        << "C₂ має дорівнювати 1 + r·T·k₂ = " << expected;
}

// Тест 5.3: C₃ = T
TEST(CoefficientsTests, C3Computation) {
    double expected = 0.1; // T
    double result = EngineModel::compute_C3();
    
    EXPECT_NEAR(result, expected, 1e-9) 
        << "C₃ має дорівнювати T = " << expected;
}

// Тест 5.4: Всі коефіцієнти позитивні
TEST(CoefficientsTests, AllCoefficientsPositive) {
    EXPECT_GT(EngineModel::compute_C1(), 0.0) << "C₁ має бути позитивним";
    EXPECT_GT(EngineModel::compute_C2(), 0.0) << "C₂ має бути позитивним";
    EXPECT_GT(EngineModel::compute_C3(), 0.0) << "C₃ має бути позитивним";
}

// ==========================================
// ГОЛОВНА ФУНКЦІЯ
// ==========================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
