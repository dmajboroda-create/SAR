# Linting Guide

## Обраний лінтер: clang-tidy

clang-tidy обрано як стандартний інструмент статичного аналізу для C++.
Інтегрований в LLVM/Clang інфраструктуру, підтримує сотні перевірок.

## Запуск лінтера
```bash
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -B build -G "MinGW Makefiles"
clang-tidy -p build Main.cpp EngineModel.h RungeKutta.h Types.h test_engine.cpp
```

## Базові правила

- `readability-braces-around-statements` — фігурні дужки обов'язкові
- `readability-math-missing-parentheses` — явний порядок операцій
- `performance-*` — перевірки продуктивності
```
