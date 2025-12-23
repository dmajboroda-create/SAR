cmake_minimum_required(VERSION 3.14)
project(SAR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================
# ОСНОВНА ПРОГРАМА МОДЕЛЮВАННЯ
# ==========================================
add_executable(engine_simulation
        Main.cpp
        EngineModel.h
        RungeKutta.h
        Types.h
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

# ==========================================
# ТЕСТИ (окрема програма)
# ==========================================
add_executable(engine_tests
        test_engine.cpp
        EngineModel.h
        RungeKutta.h
        Types.h
)

target_link_libraries(engine_tests
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(engine_tests)