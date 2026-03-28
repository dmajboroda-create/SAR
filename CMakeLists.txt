cmake_minimum_required(VERSION 3.14)
project(SAR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==========================================
# SPDLOG
# ==========================================
find_package(spdlog REQUIRED)

# ==========================================
# ОСНОВНА ПРОГРАМА
# ==========================================
add_executable(engine_simulation
        Main.cpp
        EngineModel.h
        RungeKutta.h
        Types.h
        Logger.h
        error_messages.h
)
target_link_libraries(engine_simulation spdlog::spdlog)

# ==========================================
# BENCHMARK
# ==========================================
add_executable(engine_benchmark
        benchmark_main.cpp
        EngineModel.h
        RungeKutta.h
        Types.h
        Benchmark.h
)

# ==========================================
# COMPARISON BENCHMARK
# ==========================================
add_executable(engine_comparison
        benchmark_comparison.cpp
        EngineModel.h
        EngineModel_slow.h
        RungeKutta.h
        Types.h
        Benchmark.h
)

# ==========================================
# GOOGLE TEST
# ==========================================
include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

add_executable(engine_tests
        test_engine.cpp
        EngineModel.h
        RungeKutta.h
        Types.h
)
target_link_libraries(engine_tests GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(engine_tests)